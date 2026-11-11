# SPDX-License-Identifier: MIT
"""Check maintained documentation links and Python syntax without scanning captures."""
import ast
from pathlib import Path
import re


def main():
    root = Path(__file__).resolve().parents[1]
    documents = list(root.glob("*.md")) + list((root / "docs").rglob("*.md"))
    documents += list((root / ".agents/skills").rglob("*.md"))
    errors = []
    pairs = ["README", "AGENTS", "CONTRIBUTING", "docs/FORMAT",
             "docs/HARDWARE", "docs/ROADMAP", "docs/LOCAL_TESTS"]
    for name in pairs:
        for suffix in [".md", ".zh-TW.md"]:
            if not (root / (name+suffix)).is_file():
                errors.append(f"Missing language counterpart: {name+suffix}")
    for document in documents:
        content = document.read_text(encoding="utf-8")
        if "[TODO:" in content:
            errors.append(f"Unfinished skill scaffold: {document.relative_to(root)}")
        links = re.findall(r"!?\[[^\]]*\]\(([^)]+)\)", content)
        links += re.findall(r'src="([^"]+)"', content)
        for target in links:
            if target.startswith(("http:", "https:", "mailto:", "#")):
                continue
            target = target.split("#", 1)[0]
            if target and not (document.parent / target).resolve().exists():
                errors.append(f"{document.relative_to(root)}: missing target {target}")
    sources = list((root / "src").rglob("*.py")) + list((root / "tests").glob("*.py"))
    sources += list((root / "tools").glob("*.py"))
    for source in sources:
        text = source.read_text(encoding="utf-8")
        ast.parse(text, filename=str(source))
        if not text.startswith("# SPDX-License-Identifier: MIT"):
            errors.append(f"Missing MIT header: {source.relative_to(root)}")
    if errors:
        raise SystemExit("\n".join(errors))
    print(f"Checked {len(documents)} documents and {len(sources)} Python files.")


if __name__ == "__main__":
    main()
