<!--
Thanks for contributing. Keep the summary short and say what changed and why.
-->

## Summary

<!-- One or two sentences. What does this change, and what problem does it solve? -->

## Related issues

<!-- e.g. Closes #12 -->

## Type of change

- [ ] Bug fix
- [ ] New feature
- [ ] Refactor (no behaviour change)
- [ ] Documentation
- [ ] Build, packaging or CI

## Which packages are touched

- [ ] `slam_interfaces`
- [ ] `landmark_detector`
- [ ] `ekf_slam`
- [ ] `landmark_map`
- [ ] `goal_selector`
- [ ] `performance_monitor`
- [ ] `visualizer`
- [ ] `sim_bringup`
- [ ] `client_node`
- [ ] repository root only (`README`, `docs`, `scripts`, CI)

## How this was tested

<!-- Be concrete: the command you ran, the seed you used, what you observed.
     "It builds" is not a test result. -->

```
```

## Checklist

- [ ] `python3 -m flake8 src` passes.
- [ ] `shellcheck scripts/*.sh` passes, if a shell script changed.
- [ ] `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src -q` passes, and new logic
      has a test. Pure logic (no `rclpy`) belongs in a test; a node's wiring does not need
      one.
- [ ] `colcon build --symlink-install` succeeds, if a `package.xml`, `setup.py` or
      `CMakeLists.txt` changed.
- [ ] New parameters are documented in `README.md` and, where relevant, `docs/INSTALL.md`.
- [ ] New source files are listed in the package's `setup.py` or `CMakeLists.txt`.
- [ ] The change is described in the `Unreleased` section of `CHANGELOG.md`, if it is
      user-visible.
- [ ] Line endings are LF. `.gitattributes` handles this; do not commit CRLF.
- [ ] Shell scripts that are meant to be run directly are staged as `100755`
      (`git update-index --chmod=+x <path>`).

## Notes for the reviewer

<!-- Trade-offs, anything you are unsure about, anything you deliberately left out. -->
