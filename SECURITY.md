# Security Policy

## Scope

BEACON is a simulation-only research stack. It contains no authentication, no network
service, no credential handling and no user data store, and it is not intended to be
exposed to a network or to control real hardware. It is built on ROS 2, Gazebo Classic,
Nav2, OpenCV, NumPy and SciPy, which have their own security processes.

Within that, the following are still treated as security issues:

* a malformed ROS message, action goal, service request, world file or parameter value that
  crashes a node or the whole stack, or that causes an unbounded allocation, an infinite
  loop, or a written file outside the intended directory;
* any path built from external input without validation, in particular in the
  `sim_bringup` tools that read a seed, a scene file or an output directory;
* a dependency declared in a `package.xml` or installed by `scripts/install_deps.sh` that
  is fetched from an untrusted source;
* anything that lets a ROS 2 peer on the same `ROS_DOMAIN_ID` execute code or write files
  on the host.

Because ROS 2 discovery is unauthenticated by default, two machines on the same domain can
influence each other's topics and services. That is a property of the middleware rather
than a bug in this project; set a distinct `ROS_DOMAIN_ID` on a shared network.

## Supported versions

The `main` branch is the supported version. BEACON is at `0.0.1` and there is no
maintenance branch for earlier releases.

## Reporting a vulnerability

Do not open a public issue for a security problem. Use the repository's **Security** tab and
choose **Report a vulnerability**, which opens a private advisory that only the maintainer
can see.

Please include a description, the affected file or node, reproduction steps, and the impact
you believe it has. A minimal reproduction is more useful than a long report.

## What to expect

* Acknowledgement of the report as soon as is reasonable.
* An assessment of whether it is in scope and how severe it is.
* Credit in the release notes for the fix, unless you ask not to be named.
* This is a volunteer-maintained project, so there is no guaranteed response time.

## Out of scope

* Anything requiring an attacker who can already edit the source, the parameter files, the
  world files, or the host's environment.
* Denial of service through Gazebo or Nav2 itself, or through a deliberately unreasonable
  simulation configuration.
* Vulnerabilities in ROS 2, Gazebo, Nav2, OpenCV or the distribution packages; report those
  upstream.
