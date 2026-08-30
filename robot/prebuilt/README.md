# Prebuilt ROS overlays

This directory stores compressed, platform-specific `install` prefixes for
third-party ROS libraries and the Stage simulator. It intentionally does not
store `build` or `log` directories.

Restore the bundle matching the current operating system, architecture and ROS
distribution:

```bash
./robot/tools/restore_prebuilt.sh
./robot/tools/build_robot_driver.sh
source robot/setup.bash
```

`--force` replaces an existing install prefix after moving it to a timestamped
backup. A bundle is ABI-specific: the committed Ubuntu 24.04 x86-64 ROS Jazzy
artifacts must not be used on ARM64 or another Ubuntu/ROS release.

After intentionally rebuilding the stable overlays, refresh their artifacts:

```bash
./robot/tools/package_prebuilt.sh
git add robot/prebuilt
```

Generated colcon metadata contains build-time absolute paths. The restore
script rewrites text metadata for the checkout's actual location. Absolute
source paths embedded in ELF debug strings are harmless and are left intact.
