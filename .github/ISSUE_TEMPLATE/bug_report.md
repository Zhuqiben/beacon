---
name: Bug report
about: Something does not work as documented
title: "[Bug] "
labels: bug
assignees: ''
---

## What happened

<!-- What you saw, and what you expected instead. -->

## How to reproduce

1.
2.
3.

## Environment

| | |
|---|---|
| ROS 2 distribution | <!-- humble --> |
| Operating system | <!-- Ubuntu 22.04 / WSL2 / Docker --> |
| How BEACON was installed | <!-- scripts/install_deps.sh, docs/INSTALL.md manual route, ... --> |
| `python3 --version` | |

## Command and launch file

<!-- The exact command you ran, for example:
ros2 launch sim_bringup m8_demo.launch.py seed:=42 gui:=false -->

```
```

## Logs

<details>
<summary>Terminal output</summary>

```
```

</details>

<details>
<summary>Node logs (~/.ros/log)</summary>

```
```

</details>

## Does the deterministic demo also fail?

<!-- `ros2 launch sim_bringup demo.launch.py` uses the fixed template world. If that one
works and only the randomised launch fails, say so — it narrows the problem a long way. -->

## Anything else

<!-- Screenshots, rosbag snippets, the seed value, what you already tried. -->
