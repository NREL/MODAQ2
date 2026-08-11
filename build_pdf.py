"""
build_pdf.py - Generate a PDF from the mkdocs source files using pandoc.

Usage:
    python build_pdf.py [--output OUTPUT.pdf] [--config pdf_config.yaml]

Reads nav order from mkdocs.yml, concatenates markdown sources from the
docs_dir, and passes the combined document to pandoc with tectonic as the
PDF engine. Cover page title, authors, and build date are configured via
pdf_config.yaml.
"""

import argparse
import subprocess
import sys
import tempfile
from datetime import date
from pathlib import Path

import yaml


DEFAULT_CONFIG = Path("pdf_config.yaml")


def load_pdf_config(config_path: Path) -> dict:
    """Load cover page settings from pdf_config.yaml."""
    with open(config_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    return {
        "title": cfg.get("title", "MODAQ2"),
        "authors": cfg.get("authors", []),
    }


def load_nav_files(mkdocs_yml: Path) -> list[Path]:
    """Return an ordered list of markdown file paths from the mkdocs nav."""
    with open(mkdocs_yml, encoding="utf-8") as f:
        config = yaml.safe_load(f)

    docs_dir = mkdocs_yml.parent / config.get("docs_dir", "docs")
    nav = config.get("nav", [])

    files: list[Path] = []

    def _collect(entries):
        for entry in entries:
            if isinstance(entry, dict):
                for value in entry.values():
                    if isinstance(value, str) and value.endswith(".md"):
                        files.append(docs_dir / value)
                    elif isinstance(value, list):
                        _collect(value)

    _collect(nav)
    return files


def combine_markdown(files: list[Path], resource_path: Path) -> str:
    """Concatenate markdown files, inserting page breaks between sections."""
    chunks: list[str] = []
    for path in files:
        if not path.exists():
            print(f"Warning: {path} not found, skipping.", file=sys.stderr)
            continue
        text = path.read_text(encoding="utf-8")
        # Rewrite relative image paths so pandoc can resolve them
        text = text.replace("img/", str(resource_path / "img") + "/")
        chunks.append(text)

    # Separate sections with a pandoc raw LaTeX page break
    return "\n\n\\newpage\n\n".join(chunks)


def build_pdf(output: Path, mkdocs_yml: Path, config_path: Path) -> None:
    cfg = load_pdf_config(config_path)
    title = cfg["title"]
    authors = cfg["authors"]

    files = load_nav_files(mkdocs_yml)
    if not files:
        sys.exit("No nav entries found in mkdocs.yml")

    print(f"Building PDF from {len(files)} source files...")
    for f in files:
        print(f"  {f.name}")

    resource_path = mkdocs_yml.parent / "src"
    combined = combine_markdown(files, resource_path)

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".md", delete=False, encoding="utf-8"
    ) as tmp:
        tmp.write(combined)
        tmp_path = Path(tmp.name)

    build_date = date.today().strftime("%B %d, %Y")  # e.g. "March 30, 2026"

    cmd = [
        "pandoc",
        str(tmp_path),
        "--pdf-engine=tectonic",
        "--resource-path", str(resource_path),
        # Cover page metadata
        "--metadata", f"title={title}",
        "--metadata", f"date={build_date}",
        # Layout
        "-V", "geometry:margin=1in",
        "-V", "linkcolor=blue",
        "-V", "titlepage=true",
        "--toc",
        "--toc-depth=3",
        "-o", str(output),
    ]

    # Each author gets its own --metadata author= entry
    for author in authors:
        cmd += ["--metadata", f"author={author}"]

    print(f"\nRunning: {' '.join(cmd)}\n")
    result = subprocess.run(cmd)
    tmp_path.unlink(missing_ok=True)

    if result.returncode == 0:
        print(f"\nPDF written to: {output.resolve()}")
    else:
        sys.exit(f"pandoc failed with exit code {result.returncode}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build a PDF from mkdocs sources.")
    parser.add_argument(
        "--output", "-o",
        type=Path,
        default=Path("MODAQ-docs.pdf"),
        help="Output PDF path (default: MODAQ-docs.pdf)",
    )
    parser.add_argument(
        "--mkdocs-yml",
        type=Path,
        default=Path("docs/mkdocs.yml"),
        help="Path to mkdocs.yml (default: docs/mkdocs.yml)",
    )
    parser.add_argument(
        "--config", "-c",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"Cover page config file (default: {DEFAULT_CONFIG})",
    )
    args = parser.parse_args()
    build_pdf(args.output, args.mkdocs_yml, args.config)
