"""
마크다운 보고서 → PDF 변환 (수식 LaTeX + 이미지 포함).
Chrome 헤드리스 + MathJax 사용. pandoc/LaTeX 설치 불필요.

사용: python pc/md_to_pdf.py docs/자율주행_AI아키텍처_보고서_초안.md out.pdf

동작: MD → (수식 보호) → HTML(MathJax+이미지 base64 임베드) → Chrome 헤드리스 print-to-pdf
"""
import sys
import os
import re
import base64
import subprocess
import tempfile

import markdown

CHROME_CANDIDATES = [
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
]


def find_chrome():
    for c in CHROME_CANDIDATES:
        if os.path.exists(c):
            return c
    raise RuntimeError("Chrome/Edge 못 찾음")


def embed_images(md_text, md_dir):
    """![alt](path) 를 base64 data URI로 임베드 (로컬 이미지)."""
    def repl(m):
        alt, src = m.group(1), m.group(2)
        if src.startswith("http"):
            return m.group(0)
        path = os.path.normpath(os.path.join(md_dir, src))
        if not os.path.exists(path):
            return f"*(이미지 없음: {src})*"
        with open(path, "rb") as f:
            b64 = base64.b64encode(f.read()).decode()
        ext = os.path.splitext(path)[1].lstrip(".") or "png"
        return f'<img alt="{alt}" src="data:image/{ext};base64,{b64}">'
    return re.sub(r"!\[([^\]]*)\]\(([^)]+)\)", repl, md_text)


def protect_math(md_text):
    """마크다운이 $...$ 안의 _ * 를 망가뜨리지 않게 placeholder로 보호."""
    store = []

    def keep(m):
        store.append(m.group(0))
        return f"@@MATH{len(store)-1}@@"

    # 블록 $$...$$ 먼저, 그다음 인라인 $...$
    md_text = re.sub(r"\$\$.*?\$\$", keep, md_text, flags=re.DOTALL)
    md_text = re.sub(r"(?<!\\)\$(?!\s).*?(?<!\\)\$", keep, md_text)
    return md_text, store


def restore_math(html, store):
    for i, s in enumerate(store):
        html = html.replace(f"@@MATH{i}@@", s)
    return html


HTML_TMPL = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<script>
window.MathJax = {{ tex: {{ inlineMath: [['$','$']], displayMath: [['$$','$$']] }},
  svg: {{ fontCache: 'global' }} }};
</script>
<script src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
<style>
  body {{ font-family: 'Malgun Gothic','맑은 고딕',sans-serif; font-size: 11pt;
         line-height: 1.6; color:#111; max-width: 800px; margin: 0 auto; padding: 20px; }}
  h1 {{ font-size: 20pt; border-bottom: 3px solid #333; padding-bottom: 6px; }}
  h2 {{ font-size: 16pt; border-bottom: 1px solid #999; padding-bottom: 4px; margin-top: 24px; }}
  h3 {{ font-size: 13pt; margin-top: 18px; }}
  h4 {{ font-size: 12pt; }}
  table {{ border-collapse: collapse; width: 100%; margin: 10px 0; font-size: 10pt; }}
  th, td {{ border: 1px solid #bbb; padding: 5px 8px; text-align: left; }}
  th {{ background: #f0f0f0; }}
  code {{ background: #f4f4f4; padding: 1px 4px; border-radius: 3px; font-size: 9.5pt; }}
  pre {{ background: #f6f8fa; padding: 10px; border-radius: 5px; overflow-x: auto; font-size: 9pt; }}
  pre code {{ background: none; }}
  img {{ max-width: 100%; height: auto; display: block; margin: 12px auto;
         border: 1px solid #ddd; }}
  blockquote {{ border-left: 4px solid #7b4fbf; margin: 10px 0; padding: 4px 14px;
                background: #f8f6fc; color: #333; }}
  @page {{ margin: 15mm; }}
</style></head><body>
{body}
</body></html>"""


def main():
    if len(sys.argv) < 3:
        print("사용: python pc/md_to_pdf.py <입력.md> <출력.pdf>")
        return
    md_path, pdf_path = sys.argv[1], os.path.abspath(sys.argv[2])
    md_dir = os.path.dirname(os.path.abspath(md_path))

    with open(md_path, encoding="utf-8") as f:
        md_text = f.read()

    md_text = embed_images(md_text, md_dir)
    md_text, math_store = protect_math(md_text)
    body = markdown.markdown(md_text, extensions=["tables", "fenced_code", "sane_lists"])
    body = restore_math(body, math_store)
    html = HTML_TMPL.format(body=body)

    html_path = os.path.join(tempfile.gettempdir(), "report_for_pdf.html")
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(html)

    chrome = find_chrome()
    cmd = [chrome, "--headless=new", "--disable-gpu", "--no-sandbox",
           f"--print-to-pdf={pdf_path}", "--no-pdf-header-footer",
           "--virtual-time-budget=20000",
           "file:///" + html_path.replace("\\", "/")]
    print("Chrome로 PDF 생성 중...")
    subprocess.run(cmd, timeout=120)
    if os.path.exists(pdf_path):
        print(f"[완료] {pdf_path}  ({os.path.getsize(pdf_path)//1024} KB)")
    else:
        print("[실패] PDF 생성 안 됨")


if __name__ == "__main__":
    main()
