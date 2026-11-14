#!/usr/bin/env python3
"""Render docs/REFERENCE.md to a print-quality PDF.

docs/REFERENCE.md stays the editable source of truth. This script converts
it to a styled, self-contained HTML page and drives a headless Chromium
browser (Chrome or Edge, whichever is installed) to print it.

    python tools/analysis/build_reference_pdf.py

Options:
    --input PATH    markdown source        (default docs/REFERENCE.md)
    --output PATH   pdf destination        (default docs/bvr_sim_reference.pdf)
    --no-sources    leave out the docs/SOURCES.md appendix
    --keep-html     leave the intermediate HTML next to the PDF
    --browser PATH  explicit browser executable

A document that cites docs/SOURCES.md gets it as an appendix, with its
SOURCES.md links pointed at the appendix so they resolve inside the PDF.

Requires: mistune (pip install mistune) and Chrome or Edge.
"""

import argparse
import html
import os
import re
import shutil
import subprocess
import sys
import tempfile
from datetime import date
from pathlib import Path

try:
    import mistune
except ImportError:
    sys.exit("mistune is required:  pip install mistune")

REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCES = REPO_ROOT / "docs" / "SOURCES.md"
APPENDIX_HEADING = "Appendix: Sources"

BROWSER_CANDIDATES = [
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    "/usr/bin/google-chrome",
    "/usr/bin/chromium",
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
]


def find_browser(explicit=None):
    if explicit:
        if Path(explicit).exists():
            return explicit
        sys.exit(f"browser not found: {explicit}")
    for name in ("chrome", "chromium", "msedge", "google-chrome"):
        found = shutil.which(name)
        if found:
            return found
    for path in BROWSER_CANDIDATES:
        if Path(path).exists():
            return path
    sys.exit("no Chrome or Edge installation found; pass --browser PATH")


def slugify(text):
    """GitHub-compatible heading anchors, so in-document links resolve.
    mistune hands headings over HTML-escaped ("&amp;"), so unescape first
    or "A & B" would slug to "a-amp-b"."""
    text = html.unescape(re.sub(r"<[^>]+>", "", text)).lower()
    text = re.sub(r"[^a-z0-9 _-]", "", text)
    return text.replace(" ", "-")


class ReferenceRenderer(mistune.HTMLRenderer):
    """Adds heading anchors and marks top-level sections as page breaks."""

    def __init__(self):
        super().__init__(escape=False)
        self.first_h1_seen = False

    def heading(self, text, level, **attrs):
        anchor = slugify(text)
        cls = ""
        if level == 1:
            if self.first_h1_seen:
                cls = ' class="page-break"'
            self.first_h1_seen = True
        return f'<h{level}{cls} id="{anchor}">{text}</h{level}>\n'


CSS = r"""
@page {
    size: Letter;
    margin: 20mm 18mm 20mm 18mm;
}

:root {
    --ink:      #16181d;
    --ink-soft: #4a5160;
    --rule:     #d5d9e0;
    --rule-soft:#e8ebef;
    --accent:   #1f4e79;
    --code-bg:  #f5f6f8;
    --mono: "Cascadia Mono", "Consolas", "SF Mono", "DejaVu Sans Mono", monospace;
    --sans: "Segoe UI", "Helvetica Neue", Arial, sans-serif;
    --serif: "Charter", "Georgia", "Cambria", "Times New Roman", serif;
}

* { box-sizing: border-box; }

body {
    font-family: var(--serif);
    font-size: 10.5pt;
    line-height: 1.55;
    color: var(--ink);
    margin: 0;
    -webkit-print-color-adjust: exact;
    print-color-adjust: exact;
}

/* ---------- cover ---------- */

.cover {
    height: 235mm;
    display: flex;
    flex-direction: column;
    justify-content: center;
    page-break-after: always;
    border-top: 3pt solid var(--accent);
    border-bottom: 1pt solid var(--rule);
}
.cover .eyebrow {
    font-family: var(--sans);
    font-size: 9pt;
    letter-spacing: 0.22em;
    text-transform: uppercase;
    color: var(--ink-soft);
    margin-bottom: 10mm;
}
.cover h1 {
    font-family: var(--sans);
    font-size: 34pt;
    font-weight: 650;
    letter-spacing: -0.015em;
    line-height: 1.1;
    margin: 0 0 4mm 0;
    padding: 0;
    border: none;
    color: var(--ink);
}
.cover .subtitle {
    font-family: var(--sans);
    font-size: 14pt;
    font-weight: 400;
    color: var(--accent);
    margin: 0 0 12mm 0;
}
.cover .blurb {
    font-size: 11pt;
    color: var(--ink-soft);
    max-width: 125mm;
    margin: 0 0 16mm 0;
}
.cover .meta {
    font-family: var(--sans);
    font-size: 9pt;
    color: var(--ink-soft);
    border-top: 1pt solid var(--rule);
    padding-top: 4mm;
}
.cover .meta span { margin-right: 8mm; }

/* ---------- headings ---------- */

h1, h2, h3, h4 {
    font-family: var(--sans);
    color: var(--ink);
    page-break-after: avoid;
    break-after: avoid;
}

h1 {
    font-size: 20pt;
    font-weight: 650;
    letter-spacing: -0.01em;
    margin: 0 0 6mm 0;
    padding-bottom: 2.5mm;
    border-bottom: 2pt solid var(--accent);
}
h1.page-break { page-break-before: always; break-before: page; }

h2 {
    font-size: 13.5pt;
    font-weight: 600;
    color: var(--accent);
    margin: 9mm 0 3mm 0;
}

h3 {
    font-size: 11pt;
    font-weight: 600;
    margin: 6mm 0 2mm 0;
}

h4 {
    font-size: 10pt;
    font-weight: 600;
    color: var(--ink-soft);
    margin: 5mm 0 1.5mm 0;
}

p { margin: 0 0 3.2mm 0; orphans: 2; widows: 2; }

/* ---------- table of contents ---------- */

.toc-page { page-break-after: always; }
.toc-page h1 { border-bottom-width: 1pt; }
.toc-page p {
    font-family: var(--sans);
    font-size: 9.5pt;
    line-height: 1.5;
    margin: 0 0 2.4mm 0;
    padding-left: 6mm;
    text-indent: -6mm;
    color: var(--ink-soft);
}
.toc-page strong { color: var(--ink); }
.toc-page a { color: inherit; text-decoration: none; }

/* ---------- inline ---------- */

a { color: var(--accent); text-decoration: none; }
strong { font-weight: 650; }

code {
    font-family: var(--mono);
    font-size: 0.86em;
    background: var(--code-bg);
    padding: 0.5pt 2pt;
    border-radius: 2pt;
}

pre {
    font-family: var(--mono);
    font-size: 8.6pt;
    line-height: 1.45;
    background: var(--code-bg);
    border: 0.75pt solid var(--rule-soft);
    border-left: 2.5pt solid var(--accent);
    border-radius: 2pt;
    padding: 3mm 4mm;
    margin: 3.5mm 0;
    overflow-x: auto;
    page-break-inside: avoid;
    break-inside: avoid;
}
pre code { background: none; padding: 0; font-size: inherit; }

/* ---------- tables ---------- */

table {
    border-collapse: collapse;
    width: 100%;
    font-family: var(--sans);
    font-size: 8.8pt;
    margin: 4mm 0;
    page-break-inside: avoid;
    break-inside: avoid;
}
thead th {
    background: var(--code-bg);
    border-bottom: 1pt solid var(--ink-soft);
    text-align: left;
    font-weight: 600;
    padding: 1.8mm 2.5mm;
}
tbody td {
    border-bottom: 0.5pt solid var(--rule-soft);
    padding: 1.6mm 2.5mm;
    vertical-align: top;
}
tbody tr:last-child td { border-bottom: 0.75pt solid var(--rule); }
td code, th code { font-size: 0.9em; }

/* ---------- lists, rules, quotes ---------- */

ul, ol { margin: 0 0 3.2mm 0; padding-left: 6mm; }
li { margin-bottom: 1.2mm; }

hr { border: none; border-top: 0.75pt solid var(--rule); margin: 7mm 0; }

blockquote {
    margin: 4mm 0;
    padding: 2.5mm 5mm;
    border-left: 2.5pt solid var(--rule);
    color: var(--ink-soft);
    font-style: italic;
}

@media screen {
    body { max-width: 190mm; margin: 0 auto; padding: 12mm; }
    .cover { height: auto; padding: 30mm 0; }
}
"""

COVER = """
<div class="cover">
  <div class="eyebrow">Modeling &amp; Simulation Reference</div>
  <h1>bvr_sim</h1>
  <div class="subtitle">Physics, sensor models, and guidance mathematics</div>
  <p class="blurb">A single-process air-engagement simulator: six-degree-of-freedom
  aircraft dynamics, a proportional-navigation missile, pulse-Doppler radar and
  radar-warning sensors, and navigation error modeling &mdash; with the equations
  each subsystem evaluates, the derivation of every parameter, and the measured
  validation results behind them.</p>
  <div class="meta">
    <span><strong>Generated</strong> {generated}</span>
    <span><strong>Source</strong> docs/REFERENCE.md</span>
    {revision}
  </div>
</div>
"""


def git_revision():
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=10,
        )
        if out.returncode == 0 and out.stdout.strip():
            return f"<span><strong>Revision</strong> {out.stdout.strip()}</span>"
    except (OSError, subprocess.SubprocessError):
        pass
    return ""


def split_front_matter(markdown_text):
    """Peel the title block and table of contents off the top.

    The cover page replaces the title block (everything before the first
    horizontal rule), and the TOC gets its own styled page, so both are
    rendered separately. Sections between the title block and the TOC
    stay, as the start of the body.
    """
    marker = "## Table of Contents"
    if marker not in markdown_text:
        return "", markdown_text
    before, rest = markdown_text.split(marker, 1)
    # The TOC ends at the first horizontal rule following it.
    parts = rest.split("\n---\n", 1)
    if len(parts) == 1:
        return "", markdown_text
    title_and_intro = before.split("\n---\n", 1)
    intro = title_and_intro[1] if len(title_and_intro) == 2 else ""
    return parts[0], intro + "\n" + parts[1]


def with_sources_appendix(markdown_text, document_name):
    """The document plus docs/SOURCES.md, its SOURCES.md links made local,
    and the appendix's links back into the document (document_name, e.g.
    "REFERENCE.md") made local too. The appendix heading is renamed so its
    anchor cannot collide with a "Sources" section in the document itself."""
    if "SOURCES.md" not in markdown_text:
        return markdown_text
    appendix = SOURCES.read_text(encoding="utf-8")
    appendix = re.sub(r"^# Sources$", f"# {APPENDIX_HEADING}", appendix, count=1, flags=re.MULTILINE)
    combined = markdown_text.rstrip() + "\n\n---\n\n" + appendix
    combined = re.sub(r"\(SOURCES\.md#", "(#", combined)
    combined = combined.replace(f"({document_name}#", "(#")
    return re.sub(r"\(SOURCES\.md\)", f"(#{slugify(APPENDIX_HEADING)})", combined)


def build_html(markdown_text):
    render = mistune.create_markdown(
        renderer=ReferenceRenderer(),
        plugins=["table", "url", "strikethrough"],
    )
    toc_md, body_md = split_front_matter(markdown_text)

    toc_html = ""
    if toc_md.strip():
        toc_renderer = mistune.create_markdown(
            renderer=mistune.HTMLRenderer(escape=False), plugins=["url"]
        )
        toc_html = (
            '<div class="toc-page"><h1>Contents</h1>'
            + toc_renderer(toc_md)
            + "</div>"
        )

    cover = COVER.format(
        generated=date.today().isoformat(),
        revision=git_revision(),
    )

    body_html = render(body_md)
    # A section-separating rule immediately before a forced page break just
    # leaves a stray line at the foot of the previous page.
    body_html = re.sub(r"<hr\s*/?>\s*(?=<h1 class=\"page-break\")", "", body_html)

    return (
        "<!doctype html>\n<html><head><meta charset='utf-8'>"
        "<title>bvr_sim &mdash; Modeling &amp; Simulation Reference</title>"
        f"<style>{CSS}</style></head><body>"
        f"{cover}{toc_html}{body_html}"
        "</body></html>"
    )


def print_pdf(browser, html_path, pdf_path):
    args = [
        browser,
        "--headless=new",
        "--disable-gpu",
        "--no-sandbox",
        "--no-pdf-header-footer",
        "--virtual-time-budget=10000",
        f"--print-to-pdf={pdf_path}",
        html_path.as_uri(),
    ]
    # Remove any previous output first, or a failed run would find the old
    # PDF and report success.
    pdf_path.unlink(missing_ok=True)
    result = subprocess.run(args, capture_output=True, text=True, timeout=180)
    if not pdf_path.exists():
        sys.stderr.write(result.stdout + result.stderr)
        sys.exit(f"browser did not produce {pdf_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default=REPO_ROOT / "docs" / "REFERENCE.md", type=Path)
    parser.add_argument(
        "--output", default=REPO_ROOT / "docs" / "bvr_sim_reference.pdf", type=Path
    )
    parser.add_argument("--no-sources", action="store_true")
    parser.add_argument("--keep-html", action="store_true")
    parser.add_argument("--browser")
    args = parser.parse_args()

    if not args.input.exists():
        sys.exit(f"input not found: {args.input}")

    markdown_text = args.input.read_text(encoding="utf-8")
    if not args.no_sources:
        markdown_text = with_sources_appendix(markdown_text, args.input.name)
    page = build_html(markdown_text)

    if args.keep_html:
        html_path = args.output.with_suffix(".html")
        html_path.write_text(page, encoding="utf-8")
    else:
        handle, tmp = tempfile.mkstemp(suffix=".html")
        os.close(handle)
        html_path = Path(tmp)
        html_path.write_text(page, encoding="utf-8")

    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        print_pdf(find_browser(args.browser), html_path, args.output)
    finally:
        if not args.keep_html:
            html_path.unlink(missing_ok=True)

    size_kb = args.output.stat().st_size / 1024
    print(f"wrote {args.output}  ({size_kb:.0f} KB)")


if __name__ == "__main__":
    main()
