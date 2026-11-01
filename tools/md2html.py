import re, sys, pathlib, html
import markdown
from pygments.formatters import HtmlFormatter

ROOT = pathlib.Path(r"C:/Users/15796/WorkBuddy AI/CNCSIM")

# 用法: md2html.py [src.md] [out.html] [brand]
SRC = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs" / "技术方案-五轴CNC仿真.md"
OUT = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else SRC.with_suffix(".html")
BRAND = sys.argv[3] if len(sys.argv) > 3 else "五轴联动 CNC<br>切割仿真软件"
TITLE = SRC.stem

text = SRC.read_text(encoding="utf-8")

md = markdown.Markdown(
    extensions=[
        "fenced_code", "codehilite", "tables", "toc", "attr_list", "sane_lists", "md_in_html",
    ],
    extension_configs={
        "codehilite": {"guess_lang": False, "noclasses": True, "pygments_style": "one-dark"},
        "toc": {"toc_depth": "2-3", "permalink": False},
    },
)
body = md.convert(text)
toc = md.toc

pyg_css = HtmlFormatter(style="one-dark").get_style_defs(".codehilite")

# 给表格包一层容器，便于横向滚动
body = body.replace("<table>", '<div class="table-wrap"><table>').replace("</table>", "</table></div>")

# 按源码顺序把 fenced code 的语言标注回填到 data-lang（逐行跟踪围栏状态）
langs, in_fence = [], False
for line in text.splitlines():
    m = re.match(r"^```([A-Za-z0-9_+-]*)\s*$", line)
    if m:
        if not in_fence:
            in_fence = True
            langs.append(m.group(1) or "text")
        else:
            in_fence = False
it = iter(langs)


def _tag(m):
    try:
        lang = next(it)
    except StopIteration:
        return m.group(0)
    return f'<div class="codehilite" data-lang="{lang}">'


# 去掉 pygments 内联的浅色背景，统一用页面自己的深色底
body = re.sub(r'<div class="codehilite"[^>]*>', _tag, body)

# 站内 .md 链接改指同目录的 .html
body = re.sub(r'href="([^"]+)\.md"', r'href="\1.html"', body)

# 版本信息胶囊（可选，第 4 个参数用 | 分隔），插到主标题之后
CHIPS = sys.argv[4].split("|") if len(sys.argv) > 4 and sys.argv[4].strip() else []
if CHIPS:
    META = '<div class="meta">' + "".join(
        f'<span class="chip{" hi" if i < 2 else ""}">{html.escape(c.strip())}</span>'
        for i, c in enumerate(CHIPS)
    ) + "</div>"
    i = body.find("</h1>")
    if i != -1:
        body = body[: i + 5] + META + body[i + 5 :]

TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<style>
:root {{
  --bg: #0d1117;
  --bg-soft: #12171f;
  --panel: #161b22;
  --border: #232a35;
  --border-soft: #1c222b;
  --fg: #d6dde6;
  --fg-dim: #8b96a5;
  --fg-bright: #eef3f9;
  --accent: #58a6ff;
  --accent-dim: #1f6feb;
  --warn: #d29922;
  --ok: #3fb950;
  --code-bg: #0b0f14;
}}
* {{ box-sizing: border-box; }}
html {{ scroll-behavior: smooth; }}
body {{
  margin: 0;
  background: var(--bg);
  color: var(--fg);
  font-family: -apple-system, "Segoe UI", "PingFang SC", "Microsoft YaHei", "Hiragino Sans GB", sans-serif;
  font-size: 15px;
  line-height: 1.75;
  -webkit-font-smoothing: antialiased;
}}
.layout {{ display: flex; align-items: flex-start; }}

/* ── 侧栏 ── */
aside {{
  position: sticky; top: 0; height: 100vh; width: 300px; flex: 0 0 300px;
  overflow-y: auto; padding: 28px 18px 60px;
  border-right: 1px solid var(--border-soft);
  background: var(--bg-soft);
}}
aside .brand {{
  font-size: 13px; letter-spacing: .12em; text-transform: uppercase;
  color: var(--fg-dim); margin-bottom: 4px;
}}
aside .brand-title {{
  font-size: 15px; font-weight: 650; color: var(--fg-bright);
  line-height: 1.45; margin-bottom: 18px;
}}
aside .toc > ul {{ list-style: none; margin: 0; padding: 0; }}
aside .toc > ul > li > a {{ font-weight: 600; color: var(--fg); }}
aside .toc ul ul {{ list-style: none; padding-left: 12px; margin: 2px 0 10px; border-left: 1px solid var(--border-soft); }}
aside .toc ul ul a {{ font-size: 13.5px; color: var(--fg-dim); }}
aside a {{
  display: block; padding: 3px 8px; border-radius: 5px;
  text-decoration: none; font-size: 14px; transition: background .12s, color .12s;
}}
aside a:hover {{ background: #1b2330; color: var(--accent); }}
aside ul ul a:hover {{ color: var(--accent); }}

/* ── 正文 ── */
main {{ flex: 1 1 auto; min-width: 0; margin: 0 auto; padding: 44px 56px 120px; max-width: 1020px; }}

h1 {{
  font-size: 30px; line-height: 1.3; color: var(--fg-bright); font-weight: 700;
  margin: 0 0 8px; letter-spacing: -.01em;
}}
h2 {{
  font-size: 21px; color: var(--fg-bright); font-weight: 650;
  margin: 56px 0 16px; padding-bottom: 10px; border-bottom: 1px solid var(--border);
}}
h3 {{ font-size: 17px; color: var(--fg-bright); font-weight: 650; margin: 34px 0 12px; }}
h4 {{ font-size: 15px; color: var(--accent); font-weight: 600; margin: 24px 0 10px; }}

p {{ margin: 12px 0; }}
strong {{ color: var(--fg-bright); font-weight: 650; }}
em {{ color: #c9d3df; }}
a {{ color: var(--accent); text-decoration: none; }}
a:hover {{ text-decoration: underline; }}

hr {{ border: 0; border-top: 1px solid var(--border-soft); margin: 40px 0; }}

blockquote {{
  margin: 18px 0; padding: 12px 18px;
  background: #131a24; border-left: 3px solid var(--accent-dim);
  border-radius: 0 6px 6px 0; color: #b8c4d2;
}}
blockquote p {{ margin: 4px 0; }}

ul, ol {{ padding-left: 24px; margin: 12px 0; }}
li {{ margin: 5px 0; }}
li > ul, li > ol {{ margin: 4px 0; }}

/* ── 表格 ── */
.table-wrap {{ overflow-x: auto; margin: 18px 0; border: 1px solid var(--border); border-radius: 8px; }}
table {{ border-collapse: collapse; width: 100%; font-size: 13.8px; }}
thead {{ background: #1a2230; }}
th {{
  text-align: left; padding: 10px 13px; font-weight: 650; color: var(--fg-bright);
  border-bottom: 1px solid var(--border); white-space: nowrap;
}}
td {{ padding: 9px 13px; border-bottom: 1px solid var(--border-soft); vertical-align: top; }}
tbody tr:last-child td {{ border-bottom: 0; }}
tbody tr:hover {{ background: #141b25; }}

/* ── 代码 ── */
code {{
  font-family: "JetBrains Mono", "Cascadia Code", Consolas, "SF Mono", monospace;
  font-size: .875em; background: #1b222c; color: #e3b341;
  padding: .14em .4em; border-radius: 4px;
}}
.codehilite {{
  position: relative; margin: 18px 0; border: 1px solid var(--border);
  border-radius: 8px; background: var(--code-bg); overflow: hidden;
}}
.codehilite pre {{
  margin: 0; padding: 15px 17px; overflow-x: auto;
  font-family: "JetBrains Mono", "Cascadia Code", Consolas, "SF Mono", monospace;
  font-size: 13px; line-height: 1.62;
}}
.codehilite code {{ background: none; padding: 0; color: inherit; font-size: inherit; }}
.codehilite::before {{
  content: attr(data-lang); position: absolute; top: 0; right: 0;
  font-size: 10.5px; letter-spacing: .1em; text-transform: uppercase;
  color: #5a6675; padding: 4px 10px;
}}

/* ── Pygments ── */
{pyg_css}

/* 首屏 meta */
.meta {{
  display: flex; flex-wrap: wrap; gap: 8px; margin: 16px 0 30px;
}}
.chip {{
  font-size: 12px; padding: 4px 11px; border-radius: 999px;
  background: #16202c; border: 1px solid #223040; color: #9fb0c4;
}}
.chip.hi {{ background: #10243a; border-color: #1f4f7a; color: #79b8ff; }}

@media (max-width: 1080px) {{
  aside {{ display: none; }}
  main {{ padding: 28px 22px 80px; }}
}}
@media print {{
  aside {{ display: none; }}
  body {{ background: #fff; color: #111; }}
  main {{ padding: 0; max-width: none; }}
}}
</style>
</head>
<body>
<div class="layout">
<aside>
  <div class="brand">Technical Design</div>
  <div class="brand-title">{brand}</div>
  {toc}
</aside>
<main>
{body}
</main>
</div>
<script>
// 给代码块标注语言
document.querySelectorAll('.codehilite').forEach(function (el) {{
  var code = el.querySelector('code');
  if (!code) return;
  var cls = Array.from(code.classList).find(function (c) {{ return c.indexOf('language-') === 0; }});
  if (cls) el.setAttribute('data-lang', cls.replace('language-', ''));
}});
// 侧栏当前章节高亮
var links = Array.from(document.querySelectorAll('aside a[href^="#"]'));
var targets = links.map(function (a) {{ return document.getElementById(a.getAttribute('href').slice(1)); }});
function spy() {{
  var y = window.scrollY + 120, best = -1;
  targets.forEach(function (t, i) {{ if (t && t.offsetTop <= y) best = i; }});
  links.forEach(function (a, i) {{ a.style.color = i === best ? 'var(--accent)' : ''; }});
}}
window.addEventListener('scroll', spy, {{ passive: true }});
spy();
</script>
</body>
</html>
"""

html_out = TEMPLATE.format(pyg_css=pyg_css, toc=toc, body=body, title=html.escape(TITLE), brand=BRAND)
OUT.write_text(html_out, encoding="utf-8")
print("written:", OUT, len(html_out), "bytes")
