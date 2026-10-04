"""Writes the parameter table of docs/helicoptero-uti.md from src/physics/helicopter/params.py (single source).

python tools/heli_docs.py   (replaces the block between the PARAMS markers)
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.physics.helicopter.params import TABLE  # noqa: E402

DOC = ROOT / "docs" / "helicoptero-uti.md"
BEGIN, END = "<!-- PARAMS:BEGIN (gerado por tools/heli_docs.py) -->", "<!-- PARAMS:END -->"


def fmt(v: float) -> str:
    if abs(v) >= 1000:
        return f"{v:,.0f}".replace(",", ".")
    if v == int(v):
        return str(int(v))
    return f"{v:.4g}".replace(".", ",")


def table() -> str:
    rows = ["| Parâmetro | Valor | Unid. | Status | Variante / documento | Fonte / justificativa |",
            "|---|---|---|---|---|---|"]
    for p in TABLE.values():
        src = p.source if not p.source.startswith("http") else f"[link]({p.source})"
        if "http" in p.source and not p.source.startswith("http"):
            head, _, url = p.source.rpartition("(")
            src = f"{head}([link]({url.rstrip(')')}))" if url.startswith("http") else p.source
        rows.append(f"| {p.label} | {fmt(p.value)} | {p.unit} | **{p.status}** | {p.variant} | {src} |")
    return "\n".join(rows)


def main():
    text = DOC.read_text()
    a, b = text.index(BEGIN) + len(BEGIN), text.index(END)
    DOC.write_text(text[:a] + "\n" + table() + "\n" + text[b:])


if __name__ == "__main__":
    main()
