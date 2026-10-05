"""
Passo 4: screenshots of the Helicóptero UTI HUD scenes (web/heli/) for the TCC and the presentation.

The page replays the telemetry written by tools/heli_scenes_export.py; the instants below are taken from that
telemetry (failure, SADPF detection, branch prediction, touchdown), not chosen by hand.

    python -m tools.heli_hud_capture            (headless Chromium, software WebGL; ~1 min)
Writes docs/screenshots/heli_*.png (1920 x 1080).
"""

from __future__ import annotations

import asyncio
import functools
import http.server
import os
import sys
import threading

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB = os.path.join(ROOT, "web", "heli")
OUT = os.path.join(ROOT, "docs", "screenshots")
PORT = 8823


def _serve():
    class Quiet(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *a):
            pass
    h = functools.partial(Quiet, directory=WEB)
    http.server.ThreadingHTTPServer(("127.0.0.1", PORT), h).serve_forever()


def shots(info):
    """(file, scene, run, t) from the telemetry timeline."""
    a, b = info["cat_a"], info["advisory"]
    tr, tc = a["reject"]["timeline"], a["continue"]["timeline"]
    out = [
        ("heli_a1_categoria_a_abortar.png", "cat_a", "reject", tr["landed"] + 1.2),
        ("heli_a2_categoria_a_prosseguir_abaixo_do_deck.png", "cat_a", "continue", tc["min"]),
        ("heli_a3_categoria_a_prosseguir_vtoss.png", "cat_a", "continue", tc["done"] + 0.5),
        ("heli_b1_previsao_alerta.png", "advisory", "procedimento", b["procedimento"]["advisory_t"] + 0.8),
        ("heli_b2_previsao_alerta_seguido.png", "advisory", "consultivo_seguido",
         next((p["t"] for p in b["consultivo_seguido"]["phases"] if p["phase"] == "landed"), b["consultivo_seguido"]["end"]) + 1.0),
        ("heli_c1_autorrotacao_descida.png", "autorotation", "both", info["autorotation"]["vertical"]["touchdown_t"] - 2.5),
        ("heli_c2_autorrotacao_toque.png", "autorotation", "both", info["autorotation"]["forward"]["touchdown_t"] + 1.0),
        ("heli_d_painel_missao.png", "missao", "resgate", 0.0),
    ]
    return out


async def main(only=None):
    from playwright.async_api import async_playwright
    os.makedirs(OUT, exist_ok=True)
    threading.Thread(target=_serve, daemon=True).start()
    async with async_playwright() as p:
        b = await p.chromium.launch(args=["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
                                          "--ignore-gpu-blocklist"])
        pg = await b.new_page(viewport={"width": 1920, "height": 1080})
        pg.on("console", lambda m: print("console:", m.text) if m.type in ("error", "warning") else None)
        pg.on("pageerror", lambda e: print("pageerror:", e))
        await pg.goto(f"http://127.0.0.1:{PORT}/index.html?capture=1")
        await pg.wait_for_function("window.READY === true", timeout=90000)
        info = await pg.evaluate("window.sceneInfo()")
        stage = await pg.query_selector("#stage")
        for name, scene, run, t in shots(info):
            if only and not any(o in name for o in only):
                continue
            await pg.evaluate(f"window.renderAt('{scene}', '{run}', {t})")
            await pg.wait_for_timeout(150)
            await stage.screenshot(path=os.path.join(OUT, name))
            print(name, scene, run, round(t, 2), flush=True)
        await b.close()


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1:] or None))
