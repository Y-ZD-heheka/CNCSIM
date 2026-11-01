// 临时探针：G83 展开的块序列 + 孔底体素值
#[test]
fn probe_g83_blocks() {
    let src = "G0 Z15\nG0 X12 Y15\nG83 X12 Y15 Z3 R12.5 Q2 F200\nM30\n";
    let prog = cncsim_gcode::interpret(src, &cncsim_gcode::InterpConfig::default());
    for (i, b) in prog.blocks.iter().enumerate() {
        println!("{:2} line {:2}  tip=({:.2},{:.2},{:.3})", i, b.line_no, b.target.tip.x, b.target.tip.y, b.target.tip.z);
    }
}
