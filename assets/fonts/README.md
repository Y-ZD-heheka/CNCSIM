# 字体

## MiSans-Regular.ttf

- **来源**：小米 MiSans 字体（用户提供 `MiSans.zip`，取 `ttf/MiSans-Regular.ttf`）
- **用途**：窗口外壳（`render::shell`）的 egui 面板——egui 自带字体没有 CJK
  字形，不装的话中文全是方框。通过 `include_bytes!` 嵌进二进制（约 8 MB），
  不依赖运行时路径。
- **许可**：MiSans 由小米免费提供，个人与商业用途均可（小米官方发布页的许可条款）。
  本项目为纯自用工具；如需对外分发，随附小米的原始许可文本。
- **为什么只有 Regular**：egui 不做字重选择（没有 bold 变体），一个权重够用；
  其余字重（Thin–Heavy）留在压缩包里备查。符号字形（▶ ⏸ 等）由 egui 自带
  字体回退，MiSans 缺字时自动落到后面的字体。
