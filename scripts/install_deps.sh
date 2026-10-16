#!/usr/bin/env bash
#
# BEACON dependency installer for Ubuntu 22.04 (Jammy) and ROS 2 Humble.
#
#   ./scripts/install_deps.sh           install everything that is missing
#   ./scripts/install_deps.sh --check   report what is missing, change nothing
#   ./scripts/install_deps.sh --help    print this message
#
# The script is idempotent: on an already prepared machine it is a no-op apart from an
# apt index refresh and a rosdep database update.

set -euo pipefail

readonly ROS_DISTRO_NAME="humble"
readonly ROS_PREFIX="/opt/ros/${ROS_DISTRO_NAME}"
readonly EXPECTED_CODENAME="jammy"

# Everything the workspace needs on top of a stock Ubuntu 22.04 installation. The
# ros-humble-* entries are resolved by the ROS 2 apt repository that ensure_ros_repo()
# adds when ROS 2 itself is absent.
readonly APT_PACKAGES=(
  # ROS 2 Humble desktop: rclpy, ros2cli, rviz2, tf2, the standard message packages.
  ros-humble-desktop
  # Gazebo Classic 11 and its ROS integration (gazebo_ros, gazebo_plugins, gazebo_msgs).
  gazebo
  ros-humble-gazebo-ros-pkgs
  # TurtleBot 3 Waffle: description package, meshes and the diff-drive plugin.
  ros-humble-turtlebot3-description
  ros-humble-turtlebot3-gazebo
  # Robot model processing and joint state publication for RViz.
  ros-humble-xacro
  ros-humble-robot-state-publisher
  ros-humble-joint-state-publisher
  ros-humble-joint-state-publisher-gui
  # Navigation stack, including the collision monitor the launch file adds to the chain.
  ros-humble-nav2-bringup
  ros-humble-nav2-collision-monitor
  # Python runtime dependencies used by the detector, the EKF and the map.
  python3-numpy
  python3-scipy
  python3-opencv
  # Build, test and development tooling.
  python3-colcon-common-extensions
  python3-pip
  python3-pytest
  build-essential
  cmake
  git
  curl
  gnupg
  lsb-release
  software-properties-common
)

# rosdep is not in the list above because two conflicting providers exist: the
# Ubuntu-universe python3-rosdep2 and the ROS-repository python3-rosdep, which declare a
# Conflicts on each other. ensure_rosdep_tool() keeps an existing rosdep and otherwise
# installs ROSDEP_TOOL_PACKAGE.
readonly ROSDEP_TOOL_PACKAGE="python3-rosdep2"

SUDO=""
if [ "$(id -u)" -ne 0 ]; then
  SUDO="sudo"
fi

MODE="install"
case "${1:-}" in
  --check) MODE="check" ;;
  --help|-h)
    sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
    ;;
  "") ;;
  *) printf 'unknown option: %s\n' "$1" >&2; exit 2 ;;
esac

log() { printf '[install_deps] %s\n' "$*"; }
die() { printf '[install_deps] error: %s\n' "$*" >&2; exit 1; }

require_platform() {
  [ -r /etc/os-release ] || die "/etc/os-release is unreadable; Ubuntu 22.04 is required."
  # shellcheck disable=SC1091
  . /etc/os-release
  [ "${ID:-}" = "ubuntu" ] || die "expected Ubuntu, found ${ID:-unknown}."
  if [ "${VERSION_CODENAME:-}" != "$EXPECTED_CODENAME" ]; then
    log "warning: expected Ubuntu ${EXPECTED_CODENAME}, found ${VERSION_CODENAME:-unknown}."
    log "warning: package versions may differ from the tested configuration."
  fi
}

require_sudo() {
  [ -n "$SUDO" ] || return 0
  log "apt and rosdep need root; sudo may ask for your password once."
  ${SUDO} -v || die "sudo is unavailable. Re-run this script as root, or grant sudo first."
}

is_installed() {
  dpkg-query -W -f='${db:Status-Abbrev}' "$1" 2>/dev/null | grep -q '^ii'
}

missing_packages() {
  local pkg
  for pkg in "${APT_PACKAGES[@]}"; do
    is_installed "$pkg" || printf '%s\n' "$pkg"
  done
}

ensure_ros_repo() {
  if [ -r "${ROS_PREFIX}/setup.bash" ]; then
    return 0
  fi

  log "ROS 2 ${ROS_DISTRO_NAME} is absent; adding the ROS 2 apt repository."
  ${SUDO} apt-get update
  ${SUDO} apt-get install -y curl gnupg lsb-release software-properties-common
  ${SUDO} add-apt-repository -y universe

  local keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
  if [ ! -s "$keyring" ]; then
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key |
      ${SUDO} tee "$keyring" >/dev/null
  fi

  local arch codename
  arch="$(dpkg --print-architecture)"
  # shellcheck disable=SC1091
  codename="$(. /etc/os-release && printf '%s' "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}")"
  printf 'deb [arch=%s signed-by=%s] http://packages.ros.org/ros2/ubuntu %s main\n' \
    "$arch" "$keyring" "$codename" |
    ${SUDO} tee /etc/apt/sources.list.d/ros2.list >/dev/null

  ${SUDO} apt-get update
}

apt_install() {
  log "refreshing the apt index."
  ${SUDO} apt-get update

  log "installing ${#APT_PACKAGES[@]} apt packages (already present ones are skipped)."
  # shellcheck disable=SC2024
  ${SUDO} env DEBIAN_FRONTEND=noninteractive apt-get install -y "${APT_PACKAGES[@]}"
}

ensure_rosdep_tool() {
  if command -v rosdep >/dev/null 2>&1; then
    log "rosdep is already available: $(command -v rosdep)"
    return 0
  fi

  log "installing ${ROSDEP_TOOL_PACKAGE} to provide the rosdep command."
  # shellcheck disable=SC2024
  ${SUDO} env DEBIAN_FRONTEND=noninteractive apt-get install -y "${ROSDEP_TOOL_PACKAGE}"
}

ensure_rosdep() {
  # This setup.bash only exists once ROS 2 is installed, so shellcheck cannot follow it.
  # shellcheck disable=SC1090,SC1091
  . "${ROS_PREFIX}/setup.bash"

  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    log "initialising rosdep."
    ${SUDO} rosdep init || log "warning: rosdep init failed; continuing."
  fi

  log "updating the rosdep database."
  # The apt list above is authoritative; a stale or unreachable rosdep source must not
  # abort an otherwise complete installation.
  rosdep update || log "warning: rosdep update failed; continuing."
}

rosdep_install() {
  local ws_root="$1"

  log "resolving the package.xml dependencies of ${ws_root}/src with rosdep."
  if ! (
    cd "$ws_root"
    rosdep install \
      --from-paths src \
      --ignore-src \
      --rosdistro "${ROS_DISTRO_NAME}" \
      -y
  ); then
    log "warning: rosdep install did not complete; relying on the apt list above."
  fi
}

verify() {
  local failed=0 pkg

  command -v gazebo >/dev/null 2>&1 || { log "missing executable: gazebo"; failed=1; }
  [ -r "${ROS_PREFIX}/setup.bash" ] || { log "missing: ${ROS_PREFIX}/setup.bash"; failed=1; }
  [ "$failed" -eq 0 ] || die "the base environment is still incomplete."

  # shellcheck disable=SC1090,SC1091
  . "${ROS_PREFIX}/setup.bash"

  for pkg in turtlebot3_description turtlebot3_gazebo gazebo_ros nav2_bringup \
             nav2_collision_monitor xacro robot_state_publisher; do
    ros2 pkg prefix "$pkg" >/dev/null 2>&1 || { log "missing ROS package: $pkg"; failed=1; }
  done
  [ "$failed" -eq 0 ] || die "some ROS packages could not be located."

  log "all dependencies are present."
}

main() {
  require_platform

  if [ "$MODE" = "check" ]; then
    local missing
    missing="$(missing_packages)"
    if [ -z "$missing" ]; then
      log "all ${#APT_PACKAGES[@]} apt packages are installed."
    else
      log "apt packages that are not installed:"
      # shellcheck disable=SC2086
      printf '  %s\n' $missing
      log "run ./scripts/install_deps.sh to install them."
    fi
    if command -v rosdep >/dev/null 2>&1; then
      log "rosdep is available: $(command -v rosdep)"
    else
      log "rosdep is missing; ${ROSDEP_TOOL_PACKAGE} provides it."
    fi
    return 0
  fi

  require_sudo
  ensure_ros_repo
  apt_install
  ensure_rosdep_tool
  ensure_rosdep
  rosdep_install "$(cd "$(dirname "$0")/.." && pwd)"
  verify

  log "environment ready. Next:"
  log "  colcon build --symlink-install"
  log "  source install/setup.bash"
}

main "$@"
