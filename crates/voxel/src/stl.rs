//! STL 解析：二进制 + ASCII，零依赖。
//!
//! STL 只有两种合法形态，判别**只看字节**：
//!
//! 1. **二进制**：84 + 50×n 字节（80 头 + u32 数量 + n×50 记录）。
//!    尺寸公式先行——不少二进制文件的 80 字节头恰好以 "solid" 开头，
//!    按内容判别会把它们当 ASCII 撞坏（经典陷阱，有测试锁住）。
//! 2. **ASCII**：`solid …` 开头的令牌流，只关心 `vertex` 后面的三个数。
//!
//! 不读法向量——STL 的法向经常是错的/未归一化的，需要时自己按顶点算。
//! 坐标一律 f32 → f64，保持文件里的逐位值（确定性要求：解析结果与平台无关）。

use cncsim_math::DVec3;

/// 一条三角形（顶点按文件顺序，不保证绕向）。
pub type Tri = [DVec3; 3];

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StlError {
    /// 尺寸公式不符，也不是 `solid` 开头——根本不是 STL
    NotStl,
    /// 二进制形态（≥84 字节、非 solid 头）但尺寸公式不符——多半被截断
    Truncated { expected: usize, got: usize },
    /// ASCII 里 `vertex` 后面跟的不是三个数
    BadAscii { line: usize },
}

impl std::fmt::Display for StlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StlError::NotStl => write!(f, "不是 STL 文件（二进制尺寸公式不符，也没有 solid 头）"),
            StlError::Truncated { expected, got } => {
                write!(f, "二进制 STL 尺寸不符：按头应为 {expected} 字节，实际 {got}（多半被截断）")
            }
            StlError::BadAscii { line } => write!(f, "ASCII STL 第 {line} 行：vertex 后不是三个数"),
        }
    }
}

impl std::error::Error for StlError {}

/// 解析 STL 字节流。二进制优先（尺寸公式），其余按 ASCII。
pub fn parse(bytes: &[u8]) -> Result<Vec<Tri>, StlError> {
    if let Some(tris) = try_binary(bytes) {
        return Ok(tris);
    }
    // solid 前允许有空白（个别导出器的习惯）
    let head: &[u8] = {
        let n = bytes.iter().take_while(|b| b.is_ascii_whitespace()).count();
        &bytes[n..]
    };
    if head.len() >= 5 && head[..5].eq_ignore_ascii_case(b"solid") {
        return parse_ascii(bytes);
    }
    // ≥84 字节、非 solid 头、公式又不符 → 只能是截断/损坏的二进制 STL
    if bytes.len() >= 84 {
        let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        return Err(StlError::Truncated {
            expected: 84 + n.saturating_mul(50),
            got: bytes.len(),
        });
    }
    Err(StlError::NotStl)
}

/// 二进制形态校验 + 解析。尺寸公式不符返回 `None`（可能是 ASCII）。
fn try_binary(bytes: &[u8]) -> Option<Vec<Tri>> {
    if bytes.len() < 84 {
        return None;
    }
    let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    let expected = 84 + n.checked_mul(50)?;
    if expected != bytes.len() {
        return None;
    }
    let mut tris = Vec::with_capacity(n);
    let mut off = 84;
    for _ in 0..n {
        let rec = &bytes[off..off + 50];
        off += 50;
        // 记录布局：12×f32（法向 3 + 顶点 9）+ u16 属性。法向与属性都跳过。
        let f = |i: usize| {
            f32::from_le_bytes([rec[4 * i], rec[4 * i + 1], rec[4 * i + 2], rec[4 * i + 3]]) as f64
        };
        let v = |i: usize| DVec3::new(f(3 + 3 * i), f(3 + 3 * i + 1), f(3 + 3 * i + 2));
        tris.push([v(0), v(1), v(2)]);
    }
    Some(tris)
}

fn parse_ascii(bytes: &[u8]) -> Result<Vec<Tri>, StlError> {
    // ASCII STL 按定义是文本；非 UTF-8 字节只可能出现在名字里，
    // 用 lossy 转换保证几何令牌照常解析。
    let text = String::from_utf8_lossy(bytes);
    let mut tris = Vec::new();
    let mut verts: Vec<DVec3> = Vec::with_capacity(3);
    let mut line = 1usize;

    // 手工扫描而不是 split_whitespace：行号要精确，而换行符会被
    // whitespace 切分器吃掉，令牌上数不到。
    let mut rest = text.as_ref();
    loop {
        // ① 吃掉空白并计数换行
        let ws = rest.len() - rest.trim_start_matches(|c: char| c.is_ascii_whitespace()).len();
        line += rest[..ws].matches('\n').count();
        rest = &rest[ws..];
        if rest.is_empty() {
            break;
        }
        // ② 取一个令牌
        let end = rest.find(|c: char| c.is_ascii_whitespace()).unwrap_or(rest.len());
        let (tok, tail) = rest.split_at(end);
        rest = tail;

        if !tok.eq_ignore_ascii_case("vertex") {
            continue;
        }
        // ③ vertex 后面跟三个数 = 一个顶点；每 3 个顶点凑一个三角形。
        //    facet/outer/loop/endloop/endfacet 不携带几何信息（STL 规范保证
        //    每 facet 恰好 3 个 vertex）。
        let mut c = [0.0f64; 3];
        for slot in &mut c {
            let ws2 =
                rest.len() - rest.trim_start_matches(|ch: char| ch.is_ascii_whitespace()).len();
            line += rest[..ws2].matches('\n').count();
            rest = &rest[ws2..];
            if rest.is_empty() {
                return Err(StlError::BadAscii { line });
            }
            let end2 = rest.find(|c: char| c.is_ascii_whitespace()).unwrap_or(rest.len());
            let (num, tail2) = rest.split_at(end2);
            rest = tail2;
            let Ok(x) = num.parse::<f64>() else {
                return Err(StlError::BadAscii { line });
            };
            *slot = x;
        }
        verts.push(DVec3::new(c[0], c[1], c[2]));
        if verts.len() == 3 {
            tris.push([verts[0], verts[1], verts[2]]);
            verts.clear();
        }
    }
    if !verts.is_empty() {
        return Err(StlError::BadAscii { line });
    }
    Ok(tris)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 1 个三角形的二进制 STL 字节流（80 头 + count + 12×f32 + 属性）。
    fn binary_stl(v: [[f32; 3]; 3]) -> Vec<u8> {
        let mut b = vec![0u8; 80];
        b.extend_from_slice(&1u32.to_le_bytes());
        for c in [0.0f32; 3] {
            b.extend_from_slice(&c.to_le_bytes()); // 法向（占位）
        }
        for vert in v {
            for c in vert {
                b.extend_from_slice(&c.to_le_bytes());
            }
        }
        b.extend_from_slice(&0u16.to_le_bytes());
        b
    }

    #[test]
    fn binary_round_trip_preserves_f32_bits() {
        let src = [[1.5f32, -2.25, 3.0], [0.1, 0.2, 0.3], [-4.0, 5.0, 6.5]];
        let tris = parse(&binary_stl(src)).unwrap();
        assert_eq!(tris.len(), 1);
        for (got, want) in tris[0].iter().zip(src) {
            // f32 → f64 是精确的：期望值就是 f32 的 f64 形态
            assert_eq!(got.x, want[0] as f64);
            assert_eq!(got.y, want[1] as f64);
            assert_eq!(got.z, want[2] as f64);
        }
    }

    #[test]
    fn binary_with_solid_header_still_parses_as_binary() {
        // 经典陷阱：二进制头的 80 字节以 "solid" 开头。
        // 尺寸公式必须优先于内容判别。
        let mut b = binary_stl([[0.0; 3]; 3]);
        b[..5].copy_from_slice(b"solid");
        assert_eq!(parse(&b).unwrap().len(), 1);
    }

    #[test]
    fn ascii_facets_become_triangles() {
        let src = b"solid demo
facet normal 0 0 1
  outer loop
    vertex 0.0 0.0 0.0
    vertex 1.0 0.0 0.0
    vertex 0.0 1.0 0.0
  endloop
endfacet
facet normal 0 0 -1
  outer loop
    vertex 0.0 0.0 1.0
    vertex 1.0 0.0 1.0
    vertex 0.0 1.0 1.0
  endloop
endfacet
endsolid demo
";
        let tris = parse(src).unwrap();
        assert_eq!(tris.len(), 2);
        assert_eq!(tris[0][0], DVec3::new(0.0, 0.0, 0.0));
        assert_eq!(tris[0][1], DVec3::new(1.0, 0.0, 0.0));
        assert_eq!(tris[0][2], DVec3::new(0.0, 1.0, 0.0));
        assert_eq!(tris[1][2], DVec3::new(0.0, 1.0, 1.0));
    }

    #[test]
    fn truncated_binary_reports_expected_size() {
        let mut b = binary_stl([[0.0; 3]; 3]);
        b[80..84].copy_from_slice(&2u32.to_le_bytes()); // 声明 2 个，只有 1 个
        assert_eq!(
            parse(&b),
            Err(StlError::Truncated { expected: 184, got: 134 })
        );
    }

    #[test]
    fn garbage_is_not_stl() {
        assert_eq!(parse(b"\x00\x01\x02 trash"), Err(StlError::NotStl));
        assert_eq!(parse(b""), Err(StlError::NotStl));
        assert_eq!(parse(b"junk"), Err(StlError::NotStl));
    }

    #[test]
    fn bad_ascii_vertex_is_an_error_with_line_number() {
        let src = b"solid s\nfacet\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 abc 0\nendsolid\n";
        match parse(src) {
            Err(StlError::BadAscii { line }) => assert_eq!(line, 5, "坏顶点在第 5 行"),
            other => panic!("应为 BadAscii，实际 {other:?}"),
        }
    }

    #[test]
    fn orphan_ascii_vertices_are_an_error() {
        // 只有 2 个顶点就结束了——凑不成三角形
        let src = b"solid s\nvertex 0 0 0\nvertex 1 0 0\nendsolid\n";
        assert!(matches!(parse(src), Err(StlError::BadAscii { .. })));
    }
}
