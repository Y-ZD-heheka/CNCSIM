// 切削面 / 刀具着色：**顶点色 R 是调色板索引，不是红色分量**（ADR-005）。
//
// 调色板本身来自材质 uniform（`CutMaterial.palette`），与 Rust 侧的
// `render::shell::PALETTE` 是**同一份数据**——所以"刀刃什么颜色、它切出来的
// 面就是什么颜色"不靠两边对齐常量，而是构造上只有一份。
//
// 为什么不用 `StandardMaterial`：它只有一个 `base_color`，逐顶点的索引表达不了。
// 也不引 `ExtendedMaterial`——那要理解 PBR 的扩展点，而这里只需要"索引 + 两个
// 方向光"，自己写几十行比接 PBR 管线短得多。
//
// 光照方向与 `render::preview` 的 WebGL 着色器**保持一致**（0.45,0.80,0.55 /
// -0.65,-0.25,0.45），这样窗口和预览页看起来是同一个东西。

// **不 import `bevy_pbr::forward_io`**：那个模块在 0.20 里没注册进 shader registry，
// import 会报 "no shader registered for this module path"。自己声明输入结构，
// location 按 Bevy 默认 vertex shader 的约定编号（0=world_pos / 1=world_normal /
// 5=color，与 `forward_io.wesl` 里 `VertexOutput` 的定义一致）。
struct FsIn {
    @builtin(position) position: vec4<f32>,
    @location(0) world_position: vec4<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(5) @interpolate(flat) color: vec4<f32>,
};

// 名字必须与 Rust 结构体一致：`AsBindGroup` 的 derive 用类型名生成 WGSL 结构体名。
//
// **group 是 3，不是 2**：Bevy 0.20 的 `MATERIAL_BIND_GROUP_INDEX = 3`
// （`bevy_pbr::material`），布局是 `descriptor.layout.insert(3, material_layout)`。
// 写 `@group(2)` 不会报"没有这个绑定"，而是报一句很难懂的
// `Storage class Storage { access: LOAD } doesn't match the shader Uniform`
// ——因为 group 2 恰好是别的存储缓冲。这个坑上一版一直潜伏着：
// 那时的 shader 根本没引用材质，多出来的布局条目是合法的。
struct CutMaterial {
    palette: array<vec4<f32>, 32>,
    use_vertex: u32,
    index: u32,
};

@group(3) @binding(0) var<uniform> material: CutMaterial;

const L1: vec3<f32> = vec3<f32>(0.4696, 0.8348, 0.5739);
const L2: vec3<f32> = vec3<f32>(-0.7759, -0.2984, 0.5371);

@fragment
fn fragment(in: FsIn, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    // 剖切/双面：背面翻转法向，否则内腔一片死黑
    var n = normalize(in.world_normal);
    if (!front) { n = -n; }

    let d = max(dot(n, L1), 0.0) * 0.78 + max(dot(n, L2), 0.0) * 0.30;

    // 颜色：**直接查表**。
    //
    // `use_vertex`：1 = 顶点色（毛坯网格，R 通道是色号），0 = 材质 `index`（刀刃 / 刀柄）。
    // 色号由 `cncsim_voxel::creator_of_tool_segment` 算：**刀具号 × 10 段 + 进度段**，
    // 粗加工占 1..=10、精加工占 11..=20，两族色相/饱和度刻意拉开。
    //
    // 这里**不插值**：用户要的是"一眼看出这块面是哪把刀、哪个阶段切的"，
    // 硬边分档比渐变直观。代价是边界沿三角形走（0.4 mm 网格），但色差大，
    // 看起来是清晰的色块边界，不是渐变色带上的毛刺。
    var base: vec3<f32>;
    if (material.use_vertex == 1u) {
        let idx = u32(clamp(in.color.r * 255.0 + 0.5, 0.0, 31.0));
        base = material.palette[idx].rgb;
    } else {
        base = material.palette[material.index].rgb;
    }
    // G = 余量偏差编码（128 = 零）：偏差越大越压暗，作为"离设计面多远"的辅助提示
    let bias = (in.color.g * 255.0 - 128.0) / 128.0;
    base = base * (1.0 - clamp(abs(bias), 0.0, 1.0) * 0.18);

    // **`base` 是 sRGB 值，fragment 输出的是线性值**——不转的话整块发白。
    // 上一版直接 `base * (0.32 + d)` 输出了 0.98 的"线性"值，GPU 再转 sRGB
    // 就变成 0.99（接近纯白），而 `StandardMaterial` 出来的灰色是 0.72。
    // 差的就是这一步 `pow(2.2)`。
    let base_lin = pow(base, vec3<f32>(2.2));

    // 环境项 0.25 + 两个方向光，最大约 1.06 → 顶面 ≈ base_lin、侧面 ≈ 0.33·base_lin，
    // 与 `StandardMaterial` 的明暗对比接近。
    let c = base_lin * (0.25 + d * 0.75);
    return vec4<f32>(c, 1.0);
}
