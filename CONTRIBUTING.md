# Contributing

**English** | [繁體中文](CONTRIBUTING.zh-TW.md)

Start with [AGENTS.md](AGENTS.md) and the [roadmap](docs/ROADMAP.md).
Keep each change focused and provide the smallest reproducible input.
Hardware work benefits most from recorded timestamps, frame IDs, units,
calibration provenance and original diagnostics.

From an activated project environment:

```sh
python -m pip install -e .
python -m unittest discover -s tests -v
python tools/smoke.py
python tools/check_project.py
```

Keep the core standard-library-only. ROS and other device libraries stay optional.
Use English documentation with a complete Traditional Chinese counterpart.
Follow the Feature/, Bugfix/ and Enhance/ task-branch convention when using Git.
Respect a local-only delivery instruction before any remote operation.

For processing changes, show an independent geometric or file-format check,
not only a lower internal residual. For hardware changes, distinguish simulated
message tests from a physical capture. Record software/firmware versions,
capture settings, scene, run duration, drops and resource use.

For viewer changes, test a desktop and narrow viewport, keyboard access and the
changed controls. Captures and generated outputs belong outside tracked source.
Do not upload a recording unless its release is authorized.

Contributions are licensed under the repository's [MIT license](LICENSE).
