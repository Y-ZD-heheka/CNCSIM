"""在有显示器的 Windows 上回放方槽，并检查实际 GPU 截图。需要 Pillow。

    python tools/check_window.py target/debug/cncsim-app.exe
"""

import argparse
import os
from pathlib import Path
import subprocess
import tempfile

from PIL import Image, ImageChops


def visible_color_groups(image):
    width, height = image.size
    # 取画面右半部以避开报告面板。深色背景、灰色毛坯和文字不应计入。
    viewport = image.crop((width // 2, height // 5, width, 4 * height // 5))
    hue, saturation, value = viewport.convert("HSV").split()
    mask = ImageChops.multiply(
        saturation.point(lambda s: 255 if s >= 110 else 0),
        value.point(lambda v: 255 if v >= 90 else 0),
    )
    histogram = hue.histogram(mask)
    groups = [sum(histogram[i * 256 // 6 : (i + 1) * 256 // 6]) for i in range(6)]
    # 这个演示有多段彩色切面。用占比可适配尺寸、抗锯齿和驱动差异。
    return sum(n > viewport.width * viewport.height * 0.005 for n in groups)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("exe", type=Path, help="带 window feature 的 cncsim-app 可执行文件")
    exe = parser.parse_args().exe.resolve(strict=True)
    with tempfile.TemporaryDirectory() as work:
        shot = Path(work) / "pocket.png"
        env = {**os.environ, "CNCSIM_T": "117.4", "CNCSIM_CAM_SCALE": "1",
               "CNCSIM_VIEW": "", "CNCSIM_HIDE_TOOL": "0"}
        subprocess.run(
            [exe, "--demo", "pocket", "--screenshot", shot],
            cwd=Path(__file__).resolve().parents[1], env=env, check=True, timeout=120,
        )
        with Image.open(shot) as frame:
            groups = visible_color_groups(frame)
        if groups < 3:
            raise SystemExit(f"方槽视区只检出 {groups} 组显著色相，预期至少 3 组")
        print(f"方槽视区检出 {groups} 组显著色相，GPU 截图检查通过")


if __name__ == "__main__":
    main()
