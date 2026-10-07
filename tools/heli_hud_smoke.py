"""
Smoke test of the Helicóptero UTI HUD page (web/heli/) in a headless browser.

Loads the page, waits for the scenes to be ready, renders every scene and branch at the start, middle and end of
its telemetry, and fails on any JavaScript error. Checks that the FSTD disclaimer and the footer are visible.

    python -m tools.heli_hud_smoke                         # serves web/heli/ with a static server
    python -m tools.heli_hud_smoke --url http://127.0.0.1:5000/heli/    # against the full Flask server
Exit code 0 = ok.
"""

from __future__ import annotations

import argparse
import asyncio
import functools
import http.server
import os
import sys
import threading

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB = os.path.join(ROOT, "web", "heli")
DISCLAIMER = "Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui dados do fabricante."
FOOTER = "voo, falha e respostas calculados pelo Twin (não é animação)"
CHROMIUM_ARGS = ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"]


def _static_server(port: int) -> str:
    class Quiet(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *a):
            pass
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), functools.partial(Quiet, directory=WEB))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return f"http://127.0.0.1:{srv.server_address[1]}/"


async def run(url: str) -> list:
    from playwright.async_api import async_playwright
    errors = []
    async with async_playwright() as p:
        b = await p.chromium.launch(args=CHROMIUM_ARGS)
        pg = await b.new_page(viewport={"width": 1280, "height": 720})
        pg.on("pageerror", lambda e: errors.append(f"pageerror: {e}"))
        pg.on("console", lambda m: errors.append(f"console.{m.type}: {m.text}") if m.type == "error" else None)
        await pg.goto(url + ("&" if "?" in url else "?") + "capture=1")
        await pg.wait_for_function("window.READY === true", timeout=120000)
        cases = await pg.evaluate("""() => {
            const out = [];
            for (const [s, info] of Object.entries(window.sceneInfo())) {
                for (const [r, v] of Object.entries(info)) {
                    const end = v.end ?? v.touchdown_t ?? 40;
                    for (const t of [0, end / 2, end]) out.push([s, r === 'vertical' || r === 'forward' ? 'both' : r, t]);
                }
            }
            for (const r of ['resgate', 'transferencia', 'conservador']) out.push(['missao', r, 0]);
            return out;
        }""")
        for scene, run_name, t in cases:
            ok = await pg.evaluate(f"window.renderAt('{scene}', '{run_name}', {t})")
            if ok is not True:
                errors.append(f"renderAt({scene}, {run_name}, {t}) returned {ok}")
            for sel, text in (("#disclaimer", DISCLAIMER), ("#foot", FOOTER)):
                el = await pg.query_selector(sel)
                if el is None or not await el.is_visible() or text not in (await el.inner_text()):
                    errors.append(f"{sel} not visible in {scene}/{run_name} at {t}")
        await b.close()
    return errors, len(cases)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=None)
    a = ap.parse_args()
    url = a.url or _static_server(0)
    errors, n = asyncio.run(run(url))
    for e in errors:
        print(e)
    print(f"{n} renders, {len(errors)} errors ({url})")
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
