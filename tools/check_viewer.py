# SPDX-License-Identifier: MIT
"""Optional real-browser checks for a generated offline preview."""
import argparse
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("preview", type=Path)
    parser.add_argument("--output", type=Path, default=Path("runs/browser-check"))
    parser.add_argument("--browser", default=shutil.which("google-chrome"))
    args = parser.parse_args()
    from playwright.sync_api import sync_playwright

    args.output.mkdir(parents=True, exist_ok=True)
    errors, external = [], []
    with sync_playwright() as runtime:
        browser = runtime.chromium.launch(
            executable_path=args.browser, headless=True, args=["--no-sandbox"])
        page = browser.new_page(viewport={"width": 1280, "height": 900})
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.on("request", lambda request: external.append(request.url)
                if request.url.startswith(("http://", "https://")) else None)
        page.goto(args.preview.resolve().as_uri(), wait_until="load")
        page.wait_for_function("document.querySelector('canvas').width > 0")
        canvas = page.locator("canvas")

        def snapshot():
            return canvas.evaluate("(element) => element.toDataURL()")

        original = snapshot()
        canvas.focus()
        page.keyboard.press("ArrowRight")
        assert snapshot() != original, "Keyboard orbit did not change the scene"
        page.get_by_role("button", name="Reset view").click()
        assert snapshot() == original, "Reset did not restore the scene"
        checkbox = page.get_by_label("Show cameras and route")
        checkbox.uncheck()
        assert snapshot() != original, "Camera visibility did not change the scene"
        checkbox.check()
        assert snapshot() == original, "Camera visibility did not restore the scene"
        canvas.hover()
        page.mouse.wheel(0, -200)
        page.wait_for_timeout(100)
        assert snapshot() != original, "Mouse zoom did not change the scene"
        page.get_by_role("button", name="Reset view").click()
        bounds = canvas.bounding_box()
        x, y = bounds["x"]+150, bounds["y"]+150
        page.mouse.move(x, y)
        page.mouse.down()
        page.mouse.move(x+80, y+30, steps=5)
        page.mouse.up()
        assert snapshot() != original, "Pointer orbit did not change the scene"
        page.get_by_role("button", name="Reset view").click()
        page.screenshot(path=str(args.output / "desktop.png"), full_page=True)
        page.set_viewport_size({"width": 390, "height": 844})
        page.wait_for_timeout(100)
        assert page.evaluate("document.documentElement.scrollWidth <= innerWidth"), (
            "Narrow viewport overflows horizontally")
        page.screenshot(path=str(args.output / "mobile.png"), full_page=True)
        browser.close()
    if errors or external:
        raise RuntimeError(f"Browser errors: {errors}; external requests: {external}")
    print("Browser checks passed: orbit, zoom, camera toggle, reset, keyboard, narrow viewport.")


if __name__ == "__main__":
    main()
