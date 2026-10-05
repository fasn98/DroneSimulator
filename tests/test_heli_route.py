"""
Passo 4 (aval): the /heli/ route of the full Flask server serves the HUD page, its script, styles, three.js and the
scene data. Skipped when the server's web dependencies (flask_socketio, flask_sqlalchemy, cv2) are not installed;
the CI workflow installs them. The browser smoke test is tools/heli_hud_smoke.py.
"""

import json
import os
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

try:
    import flask_socketio  # noqa: F401
    import flask_sqlalchemy  # noqa: F401
    import cv2  # noqa: F401
    HAVE_SERVER_DEPS = True
except ImportError:
    HAVE_SERVER_DEPS = False


@unittest.skipUnless(HAVE_SERVER_DEPS, "dependências web do servidor não instaladas (ver README_TWIN.md)")
class TestHeliRoute(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("DATABASE_URL", "sqlite://")
        cwd = os.getcwd()
        os.chdir(ROOT)  # the server serves files relative to the repository root
        cls.addClassCleanup(os.chdir, cwd)
        from simulation_server import DroneSimulationServer
        cls.client = DroneSimulationServer(port=0).app.test_client()

    def test_page_and_assets(self):
        r = self.client.get("/heli/")
        self.assertEqual(r.status_code, 200)
        html = r.get_data(as_text=True)
        self.assertIn("Simulador conceitual e educacional", html)
        for path in ("heli.js", "heli.css", "vendor/three.module.min.js", "vendor/three.core.min.js"):
            r = self.client.get(f"/heli/{path}")
            self.assertEqual(r.status_code, 200, path)
            self.assertGreater(len(r.data), 1000, path)

    def test_scene_data(self):
        for name in ("cat_a", "advisory", "autorotation", "mission"):
            r = self.client.get(f"/heli/data/{name}.json")
            self.assertEqual(r.status_code, 200, name)
            d = json.loads(r.data)
            self.assertTrue(d, name)

    def test_unknown_file_404(self):
        self.assertEqual(self.client.get("/heli/nao_existe.js").status_code, 404)


class TestHeliBrowserSmoke(unittest.TestCase):
    """Runs the headless-browser smoke test when Playwright and its Chromium are available."""

    def test_smoke(self):
        try:
            import asyncio
            from playwright.async_api import async_playwright  # noqa: F401

            from tools.heli_hud_smoke import _static_server, run
        except ImportError:
            self.skipTest("playwright não instalado")
        try:
            errors, n = asyncio.run(run(_static_server(0)))
        except Exception as e:  # noqa: BLE001 - no browser binary in this environment
            if "Executable doesn't exist" in str(e) or "BrowserType.launch" in str(e):
                self.skipTest(f"Chromium do Playwright indisponível: {e}")
            raise
        self.assertGreater(n, 10)
        self.assertEqual(errors, [])


if __name__ == "__main__":
    unittest.main()
