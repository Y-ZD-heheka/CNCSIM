// CNCSIM vs 原生 OpenVDB：同样的切削工作量，谁快、快多少。
//
// 口径与 `crates/voxel/tests/cut_perf.rs` 完全对齐：
//   · 毛坯 30×30×12 mm（300×300×120 体素，h=0.1），实心
//   · 刀具 φ6 平底刀（r=3，长 20），刀尖 z=11.5（切深 0.5）
//   · 每刀沿 +X 走 len mm，y 各不相同（互不重叠）
//   · 窄带 4 体素 = 0.4 mm
//
// 两个口径都量：
//   A. **端到端**：构造刀具扫掠体 level set + csgDifference
//      —— 这是与 `cut_segment` 对等的比较（CNCSIM 每刀重新解析求值）
//   B. **纯 CSG**：只算 csgDifference（刀具已就绪）
//      —— OpenVDB 的优势区：流水线里刀具形状复用时就是这个成本
//
// OpenVDB 没有"解析扫掠 SDF"这种东西，所以刀具体必须自己逐体素写。
// 那一步的成本与 CNCSIM 的求值同量级，差别只在 CSG 本身。

#include <openvdb/openvdb.h>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/MeshToVolume.h>

#include <algorithm>
#include <array>
#include <unordered_map>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using Clock = std::chrono::steady_clock;
static double ms_since(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// 盒 SDF（与 CNCSIM 的 DesignModel::Box 同式）
static inline double box_sdf(double x, double y, double z,
                             const double lo[3], const double hi[3]) {
    const double dx = std::max(lo[0] - x, x - hi[0]);
    const double dy = std::max(lo[1] - y, y - hi[1]);
    const double dz = std::max(lo[2] - z, z - hi[2]);
    return std::max(std::max(dx, dy), dz);
}

// 平底刀水平扫掠体的解析 SDF——
// 与 CNCSIM 优化后的 `sd_flat_prism` 是**同一个公式**（2D 胶囊 × 轴向区间的笛卡尔积），
// 这样"刀具体构造"这一步两边算法等价，比较才有意义。
static inline float flat_prism_sdf(double px, double py, double pz,
                                   double x0, double y, double tip_z,
                                   double len, double r, double L) {
    double t = (px - x0) / len;
    t = std::clamp(t, 0.0, 1.0);
    const double cx = x0 + t * len;
    const double dr = std::hypot(px - cx, py - y) - r;
    const double z0 = pz - tip_z;
    const double dz = std::max(-z0, z0 - L);
    const double a = std::max(dr, 0.0), b = std::max(dz, 0.0);
    return static_cast<float>(std::sqrt(a * a + b * b) + std::min(std::max(dr, dz), 0.0));
}


// ── 场景二：50x50x100 毛坯被 phi6 立铣刀**分层铣光** ──
//
// 与 CNCSIM 的 `crates/voxel/tests/millout.rs` 逐项对齐：
//   h=0.2（250x250x500 体素）、phi6 平底刀**刃长 6**（不是总长 60）、
//   切深 5 mm（< 刃长）、每层 10 刀等距铺满 y、20 层 = 200 刀。
void mill_out_scenario() {
    const double h = 0.2;
    const int NW = 4;
    const double band = NW * h;
    const double SX = 50, SY = 50, SZ = 100;
    const double R = 3.0, LEN = 6.0;      // 刃长 6
    const int NX = 250, NY = 250, NZ = 500;

    auto xform = openvdb::math::Transform::createLinearTransform(h);

    auto t_stock = Clock::now();
    auto stock = openvdb::FloatGrid::create(static_cast<float>(band));
    stock->setTransform(xform);
    stock->setGridClass(openvdb::GRID_LEVEL_SET);
    stock->fill(openvdb::CoordBBox(openvdb::Coord(0, 0, 0), openvdb::Coord(NX - 1, NY - 1, NZ - 1)),
                static_cast<float>(-band), false);
    {
        const int PAD = static_cast<int>(std::ceil(band / h)) + 2;
        auto acc = stock->getAccessor();
        for (int k = -PAD; k < NZ + PAD; ++k) {
            for (int j = -PAD; j < NY + PAD; ++j) {
                for (int i = -PAD; i < NX + PAD; ++i) {
                    const double x = (i + 0.5) * h, y = (j + 0.5) * h, z = (k + 0.5) * h;
                    const double dx = std::max(-x, x - SX);
                    const double dy = std::max(-y, y - SY);
                    const double dz = std::max(-z, z - SZ);
                    const double d = std::max(std::max(dx, dy), dz);
                    if (std::abs(d) <= band) {
                        acc.setValue(openvdb::Coord(i, j, k), static_cast<float>(d));
                    }
                }
            }
        }
    }
    const double stock_ms = ms_since(t_stock);

    const int passes = 10, layers = 20;
    const double doc = SZ / layers;          // 5 mm
    double tool_ms_total = 0, csg_ms_total = 0;
    int cuts = 0;

    auto t_all = Clock::now();
    for (int li = 0; li < layers; ++li) {
        const double tip_z = SZ - (li + 1) * doc;
        for (int pi = 0; pi < passes; ++pi) {
            const double y = R + (SY - 2 * R) * pi / (passes - 1);
            const double x0 = (pi % 2 == 0) ? -R : SX + R;
            const double x1 = (pi % 2 == 0) ? SX + R : -R;

            auto t0 = Clock::now();
            auto tool = openvdb::FloatGrid::create(static_cast<float>(band));
            tool->setTransform(xform);
            tool->setGridClass(openvdb::GRID_LEVEL_SET);
            const int i0 = static_cast<int>(std::floor((std::min(x0, x1) - R - band) / h)) - 1;
            const int i1 = static_cast<int>(std::ceil((std::max(x0, x1) + R + band) / h)) + 1;
            const int j0 = static_cast<int>(std::floor((y - R - band) / h)) - 1;
            const int j1 = static_cast<int>(std::ceil((y + R + band) / h)) + 1;
            const int k0 = static_cast<int>(std::floor((tip_z - band) / h)) - 1;
            const int k1 = static_cast<int>(std::ceil((tip_z + LEN + band) / h)) + 1;
            tool->fill(openvdb::CoordBBox(openvdb::Coord(i0, j0, k0), openvdb::Coord(i1, j1, k1)),
                       static_cast<float>(-band), false);
            {
                auto acc = tool->getAccessor();
                for (int k = k0; k <= k1; ++k) {
                    for (int j = j0; j <= j1; ++j) {
                        for (int i = i0; i <= i1; ++i) {
                            const double x = (i + 0.5) * h, yw = (j + 0.5) * h, z = (k + 0.5) * h;
                            const float d = flat_prism_sdf(x, yw, z, x0, y, tip_z, x1 - x0, R, LEN);
                            if (std::abs(d) <= band * 1.5) {
                                acc.setValue(openvdb::Coord(i, j, k), d);
                            }
                        }
                    }
                }
            }
            const double t_tool = ms_since(t0);

            auto t1 = Clock::now();
            openvdb::tools::csgDifference(*stock, *tool);
            const double t_csg = ms_since(t1);

            tool_ms_total += t_tool;
            csg_ms_total += t_csg;
            ++cuts;
        }
    }
    const double total_ms = ms_since(t_all);

    std::printf("\n=== MILLOUT 50x50x100 (h=%.1f, phi6 flute 6, doc 5, %d cuts) ===\n", h, cuts);
    std::printf("  stock build: %.0f ms (one-off), %lld active voxels\n",
                stock_ms, static_cast<long long>(stock->activeVoxelCount()));
    std::printf("  tool build:  %.0f ms total (%.2f ms/cut)\n", tool_ms_total, tool_ms_total / cuts);
    std::printf("  csg:         %.0f ms total (%.2f ms/cut)\n", csg_ms_total, csg_ms_total / cuts);
    std::printf("  TOTAL:       %.0f ms (%.2f ms/cut)\n", total_ms, total_ms / cuts);
}

// ── 水平集 → 网格（与 CNCSIM 的 extract_surface 对齐）──
//
// 场景：200×200×100 mm 毛坯 @ h=0.1（2000×2000×1000 体素），与
// `crates/voxel/tests/extract.rs::perf_full_stock_extraction` 完全一致。
//
// 口径：**只计 volumeToMesh 本身**，不含建场。OpenVDB 的 volumeToMesh 是
// Marching Cubes（不是对偶法），输出 points + polygons（四边形，退化时用重复
// 索引表示三角形）。
//
// adaptivity = 0 等价于 CNCSIM 的 k=1 细格。adaptivity > 0 是**自适应简化**
// （按曲率决定简化程度），与 CNCSIM 的"均匀粗格重采样 k"不是同一个口径，
// 所以只作为参考，不能直接比。
static void bench_extract() {
    const double h = 0.1;
    const int NW = 4;
    const double band = NW * h;
    auto xform = openvdb::math::Transform::createLinearTransform(h);

    const int NX = 2000, NY = 2000, NZ = 1000;
    const double lo[3] = {0.0, 0.0, 0.0};
    const double hi[3] = {NX * h, NY * h, NZ * h};

    auto t_build = Clock::now();
    auto g = openvdb::FloatGrid::create(static_cast<float>(band));
    g->setTransform(xform);
    g->fill(openvdb::CoordBBox(openvdb::Coord(0, 0, 0),
                               openvdb::Coord(NX - 1, NY - 1, NZ - 1)),
            static_cast<float>(-band), false);
    {
        auto acc = g->getAccessor();
        auto put = [&](int i, int j, int k) {
            const double x = (i + 0.5) * h, y = (j + 0.5) * h, z = (k + 0.5) * h;
            const double d = box_sdf(x, y, z, lo, hi);
            if (std::abs(d) < band) {
                acc.setValue(openvdb::Coord(i, j, k), static_cast<float>(d));
            }
        };
        // 只写最外层一圈：够 Marching Cubes 找到过零点，且建场成本可控
        for (int i = 0; i < NX; ++i)
            for (int j = 0; j < NY; ++j) { put(i, j, 0); put(i, j, NZ - 1); }
        for (int i = 0; i < NX; ++i)
            for (int k = 0; k < NZ; ++k) { put(i, 0, k); put(i, NY - 1, k); }
        for (int j = 0; j < NY; ++j)
            for (int k = 0; k < NZ; ++k) { put(0, j, k); put(NX - 1, j, k); }
    }
    const double build_ms = ms_since(t_build);

    std::printf("\n=== EXTRACT 200x200x100 mm @ h=%.1f (level set -> mesh) ===\n", h);
    std::printf("  build:       %.0f ms (one-off), %lld active voxels\n",
                build_ms, static_cast<long long>(g->activeVoxelCount()));

    for (double adapt : {0.0, 0.5, 1.0}) {
        std::vector<openvdb::Vec3s> pts;
        std::vector<openvdb::Vec3I> tris;
        std::vector<openvdb::Vec4I> quads;
        auto t0 = Clock::now();
        openvdb::tools::volumeToMesh(*g, pts, tris, quads, 0.0, adapt, true);
        const double ms = ms_since(t0);
        const size_t ntris = tris.size() + quads.size() * 2;
        std::printf("  adapt=%.1f  points %zu  tris %zu  (quads %zu)  %.0f ms\n",
                    adapt, pts.size(), ntris, quads.size(), ms);
    }
}

// ── 外部 STL → level set → mesh（与 CNCSIM 的 mesh_to_field + extract_surface 对齐）──
//
// 口径完全对齐：
//   · 同一份 STL，二进制解析（每三角形 3 个独立顶点，不建顶点表）
//   · halfWidth = 4 体素 = 4·h（与 CNCSIM 的 band_voxels = 4 一致）
//   · 体素中心约定 (i+0.5)h（`createLinearTransform(h)` 默认就是）
//   · 只计 `meshToLevelSet` 与 `volumeToMesh` 本身，不含读盘
static void bench_stl(const char* path, double h, uint32_t max_tris = 0) {
    // 读 STL
    FILE* f = std::fopen(path, "rb");
    if (!f) { std::printf("打不开 %s\n", path); return; }
    std::fseek(f, 0, SEEK_END);
    const long sz = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::vector<char> buf(static_cast<size_t>(sz));
    if (std::fread(buf.data(), 1, buf.size(), f) != buf.size()) { std::fclose(f); return; }
    std::fclose(f);

    const uint32_t n_all = *reinterpret_cast<const uint32_t*>(&buf[80]);
    if (static_cast<long>(84 + 50ull * n_all) != sz) {
        std::printf("不是二进制 STL（声明 %u 个三角形，尺寸对不上）\n", n_all);
        return;
    }
    // 只测前 n 个三角形（模型不闭合，符号会错，但**时间**有效）
    uint32_t n = n_all;
    if (max_tris && n > max_tris) {
        n = max_tris;
    }
    // **顶点焊接**。STL 每个三角形带 3 个独立顶点，190 万三角形 = 570 万个点，
    // 其中 2/3 是重复的。`meshToLevelSet` 内部要为每个点建"点 → 叶"的索引，
    // 不焊接的话那一步就是 570 万条记录——实测 h=1.0 跑 7 分钟都没完。
    // 焊接容差 1e-4 mm（0.1 µm），远小于 STL 的 f32 精度（87 mm 时 ULP 7.6e-6 mm）。
    struct K3 {
        int32_t a, b, c;
        bool operator==(const K3& o) const { return a == o.a && b == o.b && c == o.c; }
    };
    struct K3Hash {
        size_t operator()(const K3& k) const {
            return (static_cast<size_t>(k.a) * 73856093u) ^ (static_cast<size_t>(k.b) * 19349663u)
                 ^ (static_cast<size_t>(k.c) * 83492791u);
        }
    };
    auto quant = [](float x) { return static_cast<int32_t>(std::lround(x * 10000.0)); };

    const auto t_weld0 = Clock::now();
    std::vector<openvdb::Vec3s> pts;
    std::vector<openvdb::Vec3I> tris;
    pts.reserve(static_cast<size_t>(n) / 2);
    tris.reserve(n);
    std::unordered_map<K3, uint32_t, K3Hash> weld;
    weld.reserve(static_cast<size_t>(n));
    const float* base = reinterpret_cast<const float*>(&buf[84]);
    for (uint32_t i = 0; i < n; ++i) {
        const float* v = base + static_cast<size_t>(i) * 12 + 3; // 跳过法向
        openvdb::Vec3I tri;
        for (int k = 0; k < 3; ++k) {
            const openvdb::Vec3s p(v[k * 3], v[k * 3 + 1], v[k * 3 + 2]);
            const K3 key{quant(p.x()), quant(p.y()), quant(p.z())};
            auto it = weld.find(key);
            if (it == weld.end()) {
                tri[k] = static_cast<int>(pts.size());
                weld.emplace(key, static_cast<uint32_t>(pts.size()));
                pts.push_back(p);
            } else {
                tri[k] = static_cast<int>(it->second);
            }
        }
        tris.push_back(tri);
    }
    const double t_weld = ms_since(t_weld0);
    std::printf("  weld:           %.0f ms, %zu points (was %u)\n", t_weld, pts.size(), n * 3);
    std::printf("\n=== STL ROUNDTRIP (h=%.3f, %u tris) ===\n", h, n);

    auto xform = openvdb::math::Transform::createLinearTransform(h);
    const float halfWidth = static_cast<float>(4.0 * h);

    auto t0 = Clock::now();
    auto grid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(*xform, pts, tris, halfWidth);
    const double ms_ls = ms_since(t0);
    std::printf("  meshToLevelSet: %.0f ms, %lld active voxels\n",
                ms_ls, static_cast<long long>(grid->activeVoxelCount()));

    std::vector<openvdb::Vec3s> out_pts;
    std::vector<openvdb::Vec3I> out_tris;
    std::vector<openvdb::Vec4I> out_quads;
    auto t1 = Clock::now();
    openvdb::tools::volumeToMesh(*grid, out_pts, out_tris, out_quads, 0.0, 0.0, true);
    const double ms_mesh = ms_since(t1);
    std::printf("  volumeToMesh:   %.0f ms, %zu points, %zu tris\n",
                ms_mesh, out_pts.size(), out_tris.size() + out_quads.size() * 2);
    std::printf("  TOTAL:          %.0f ms\n", ms_ls + ms_mesh);
}

int main(int argc, char** argv) {
    // 行缓冲：跑长任务时要能看到进度（否则被 kill 时输出全丢）
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    openvdb::initialize();
    if (argc >= 2) {
        const double h = (argc >= 3) ? std::atof(argv[2]) : 0.1;
        const uint32_t mx = (argc >= 4) ? static_cast<uint32_t>(std::atoi(argv[3])) : 0;
        bench_stl(argv[1], h, mx);
        return 0;
    }
    bench_extract();

    const double h = 0.1;
    const int NW = 4;                        // 窄带 4 体素
    const double band = NW * h;              // 0.4 mm
    const double R = 3.0, LEN = 20.0;        // φ6 平底刀
    const double TIP_Z = 11.5;               // 切深 0.5

    // 体素中心约定 (i+0.5)h —— 与 CNCSIM 的 SparseGrid 一致
    auto xform = openvdb::math::Transform::createLinearTransform(h);

    // ── 毛坯：只建一次（不计入每刀）──
    auto t_stock = Clock::now();
    auto stock = openvdb::FloatGrid::create(static_cast<float>(band));
    stock->setTransform(xform);
    stock->setGridClass(openvdb::GRID_LEVEL_SET);

    // 内部先铺成 −band 的 inactive tile，再只把窄带写成精确值。
    // 这正是 OpenVDB level set 的标准形态（内部饱和）。
    stock->fill(openvdb::CoordBBox(openvdb::Coord(0, 0, 0), openvdb::Coord(299, 299, 119)),
                static_cast<float>(-band), /*active=*/false);
    {
        const double lo[3] = {0.0, 0.0, 0.0};
        const double hi[3] = {30.0, 30.0, 12.0};
        auto acc = stock->getAccessor();
        // **窄带要覆盖盒外一个 band**：只写盒内的话外侧是背景值（+band），
        // 与内侧的精确值不对称，插值出来的过零点会偏（实测偏 0.04 mm）。
        const int PAD = static_cast<int>(std::ceil(band / h)) + 2;
        for (int k = -PAD; k < 120 + PAD; ++k) {
            for (int j = -PAD; j < 300 + PAD; ++j) {
                for (int i = -PAD; i < 300 + PAD; ++i) {
                    const double x = (i + 0.5) * h, y = (j + 0.5) * h, z = (k + 0.5) * h;
                    const double d = box_sdf(x, y, z, lo, hi);
                    if (std::abs(d) <= band) {
                        acc.setValue(openvdb::Coord(i, j, k), static_cast<float>(d));
                    }
                }
            }
        }
    }
    const double stock_ms = ms_since(t_stock);
    std::printf("毛坯：%lld 个激活体素，%zu 个叶，建场 %.1f ms（一次性，不计入每刀）\n",
                static_cast<long long>(stock->activeVoxelCount()),
                stock->tree().leafCount(), stock_ms);

    std::printf("\n%-8s %10s %10s %10s %10s\n",
                "刀长", "刀具体ms", "CSG_ms", "端到端ms", "程序时间");
    std::printf("%s\n", std::string(56, '-').c_str());

    struct Case { double len, x0, y; };
    const Case cases[] = {{1.0, 2.0, 3.0}, {10.0, 2.0, 10.0}, {20.0, 2.0, 15.0}, {28.0, 1.0, 22.0}};

    for (const auto& c : cases) {
        // ── ① 构造刀具扫掠体 level set ──
        auto t0 = Clock::now();
        auto tool = openvdb::FloatGrid::create(static_cast<float>(band));
        tool->setTransform(xform);
        tool->setGridClass(openvdb::GRID_LEVEL_SET);
        {
            // AABB 外扩一个窄带（与 CNCSIM 的 sweep_aabb + margin 同口径）
            const int i0 = static_cast<int>(std::floor((c.x0 - R - band) / h)) - 1;
            const int i1 = static_cast<int>(std::ceil((c.x0 + c.len + R + band) / h)) + 1;
            const int j0 = static_cast<int>(std::floor((c.y - R - band) / h)) - 1;
            const int j1 = static_cast<int>(std::ceil((c.y + R + band) / h)) + 1;
            const int k0 = static_cast<int>(std::floor((TIP_Z - band) / h)) - 1;
            const int k1 = static_cast<int>(std::ceil((TIP_Z + LEN + band) / h)) + 1;

            // **先把这个 AABB 铺成 −band 的 inactive tile，再覆盖写窄带。**
            // 少了这一步，刀具体只是个"空壳"：内部落在背景值 +band（空气）上，
            // csgDifference 会把刀具内部当成空腔，切出来的槽底是错的
            //（实测 z=11.9 报 −0.05 而不是空气）。
            tool->fill(openvdb::CoordBBox(openvdb::Coord(i0, j0, k0), openvdb::Coord(i1, j1, k1)),
                       static_cast<float>(-band), /*active=*/false);

            auto acc = tool->getAccessor();
            for (int k = k0; k <= k1; ++k) {
                for (int j = j0; j <= j1; ++j) {
                    for (int i = i0; i <= i1; ++i) {
                        const double x = (i + 0.5) * h, y = (j + 0.5) * h, z = (k + 0.5) * h;
                        const float d = flat_prism_sdf(x, y, z, c.x0, c.y, TIP_Z, c.len, R, LEN);
                        // 边界用 1.5×band：`-0.4` 与 `band` 的浮点比较会在边界体素上
                        // 抖动，漏写就留下背景值，槽底出现一条假材料。
                        if (std::abs(d) <= band * 1.5) {
                            acc.setValue(openvdb::Coord(i, j, k), d);
                        }
                    }
                }
            }
        }
        const double tool_ms = ms_since(t0);

        // ── ② CSG：stock = stock − tool ──
        auto t1 = Clock::now();
        openvdb::tools::csgDifference(*stock, *tool);
        const double csg_ms = ms_since(t1);

        std::printf("%-8s %10.2f %10.2f %10.2f %9.2f s\n",
                    (std::to_string(static_cast<int>(c.len)) + " mm").c_str(),
                    tool_ms, csg_ms, tool_ms + csg_ms, c.len / (300.0 / 60.0));
    }

    std::printf("\n最终毛坯：%lld 个激活体素，%zu 个叶\n",
                static_cast<long long>(stock->activeVoxelCount()), stock->tree().leafCount());

    // ── 几何对拍剖面 ──
    //
    // 与 CNCSIM 的 `crates/voxel/tests/ovdb_parity.rs` 逐点比较。
    // 最后一刀是 28 mm（x 1→29, y=22, 刀尖 z=11.5）。
    // 采样点按体素索引取（两边都用 (i+0.5)·h 的体素中心约定）。
    auto acc = stock->getConstAccessor();
    auto probe = [&](double x, double y, double z) {
        openvdb::Coord c(static_cast<int>(std::floor(x / h)),
                         static_cast<int>(std::floor(y / h)),
                         static_cast<int>(std::floor(z / h)));
        return acc.getValue(c);
    };

    std::printf("\n=== PROFILE_BEGIN ===\n");
    // **按体素索引采样**（而不是 11.0 + i*0.1）：0.1 在二进制下不精确，
    // 用浮点坐标再 floor(x/h) 会偶尔掉到前一个体素。输出体素中心坐标，无歧义。
    auto idx_probe = [&](int i, int j, int k) {
        return acc.getValue(openvdb::Coord(i, j, k));
    };
    // ① 槽底：x=15, y=22（k 从 110 到 120）
    for (int k = 110; k <= 120; ++k) {
        std::printf("Z %.2f %.6f\n", (k + 0.5) * h, idx_probe(150, 220, k));
    }
    // ② 槽壁：x=15, z=11.9（j 从 185 到 255，穿过 y=19 与 y=25 两道壁）
    for (int j = 185; j <= 255; ++j) {
        std::printf("Y %.2f %.6f\n", (j + 0.5) * h, idx_probe(150, j, 119));
    }
    std::printf("=== PROFILE_END ===\n");

    mill_out_scenario();
    return 0;
}
