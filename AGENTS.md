# Repository working agreement

**English** | [繁體中文](AGENTS.zh-TW.md)

SplatRig prepares pose-aware Gaussian splatting datasets from a camera/LiDAR rig.
The current foundation implements synthetic replay, clock fitting, pose association,
COLMAP seed export and an optional ROS 2 odometry recorder. Live MAX2 acquisition,
full GLIM integration and physical-device validation remain separate milestones.

## Scope and delivery

Follow the user's current delivery scope. A local-testing instruction stops before
any GitHub repository creation, upload, push, PR or merge.

When remote delivery is authorized and an authenticated origin exists, use one
task branch from current main: `Feature/<lowercase-description>`,
`Bugfix/<lowercase-description>`, or `Enhance/<lowercase-description>`.
Preserve unrelated work. Complete the change and relevant checks, commit with an
English `feat:`, `fix:`, `enhance:` or `docs:` subject, push the task branch,
and open a PR targeting main. Write English first with a Traditional Chinese
summary, validation results and physical-device checks pending.

After required checks and reviews pass, merge the verified head, normally by
squash, retaining `(#N)` in the commit title. Never push directly to an established
main, bypass protections/reviews or rewrite published history. Verify the merge
before deleting only this task's branches, then fast-forward main. Report an
external gate accurately and preserve the branch when it prevents delivery.

## Read by task

- Session schema, timing, coordinate changes: [data format](docs/FORMAT.md).
- Devices, ROS, SLAM and calibration: [hardware integration](docs/HARDWARE.md).
- Architecture and next milestones: [roadmap](docs/ROADMAP.md).
- Project-specific skill: [.agents/skills/splatrig-development/SKILL.md](.agents/skills/splatrig-development/SKILL.md).

Do not inventory sibling repositories or captured datasets for an ordinary code
change. Read the relevant modules and bounded documentation sections.

## Contracts

- Integer nanoseconds throughout. Subtract integer clock anchors before floating
  point arithmetic. Preserve source timestamp and clock status.
- `T_A_B` maps B to A. Quaternions are wxyz. World is +Z up, distances are meters,
  camera coordinates are OpenCV (+X right, +Y down, +Z forward).
- The trajectory tracks the configured rig frame, which need not be the LiDAR
  origin. Apply both LiDAR and camera extrinsics.
- One pose revision drives cloud replay and photo poses. A loop correction must
  update both; never freeze old photo poses against a corrected map.
- Refuse extrapolation, large interpolation gaps and final export with unfinished
  calibration. Provisional output must stay visibly marked.
- Captures are immutable inputs. New outputs use distinct directories and publish
  only after success. Do not silently discard data when a budget is reached.
- The offline core stays free of ROS, CUDA and external SLAM imports.
- Synthetic fixtures verify software contracts. They establish no sensor accuracy,
  live throughput, 3DGS image quality or physical-device readiness.

## Validation and writing

Run `python -m unittest discover -s tests -v` for changed processing behavior and
`python tools/smoke.py` for CLI/packaging changes. Run
`python tools/check_project.py` for documentation. Viewer changes also need a
browser check at desktop and narrow widths. Record actual checks and limitations.

English documentation has a complete Traditional Chinese counterpart with links
in both directions. Product name is SplatRig; package/CLI name is splatrig.
Use MIT SPDX headers in project Python files. Keep employer names, machine-specific
absolute paths, credentials, scans and generated results out of tracked files.
Documentation, comments, commit messages and PR text contain no attribution to
automated authorship or model/tool names.

The workflow conventions above follow the owner's AnyReplica project. Its source
and noncommercial license are not imported into SplatRig.
