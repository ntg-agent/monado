# How to play Steam games with Monado {#howto-steam-games}

<!--
Copyright 2026, The Monado contributors
SPDX-License-Identifier: BSL-1.0
-->

[TOC]

This page describes how to run Steam VR games on Linux with Monado as the
OpenXR runtime, without SteamVR. It covers native OpenXR games and OpenVR games
(through an OpenVR to OpenXR translation layer).

## Overview

Steam runs Linux games inside the Steam Linux Runtime container
([pressure-vessel][]). For a game to reach Monado from inside the container it
needs three things:

1. The Monado OpenXR runtime library (`libopenxr_monado.so`) and its
   dependencies must be loadable inside the container.
2. The service socket, `$XDG_RUNTIME_DIR/monado_comp_ipc`, must exist inside
   the container.
3. The service must be running when the game starts, or be started on demand.

OpenVR games additionally need an OpenVR runtime that forwards to OpenXR.

[pressure-vessel]: https://gitlab.steamos.cloud/steamrt/steam-runtime-tools/-/blob/main/pressure-vessel/wrap.1.md

## Start the service on demand

Build with `XRT_INSTALL_SYSTEMD_UNIT_FILES` (on by default when systemd is
found). Installing then puts `monado.socket` and `monado.service` user units
into the systemd user unit directory. Enable the socket once:

```sh
systemctl --user daemon-reload
systemctl --user enable --now monado.socket
```

systemd now owns `$XDG_RUNTIME_DIR/monado_comp_ipc` and starts
`monado-service` when the first application connects. There is no need to start
the service by hand before launching a game. The service needs `DISPLAY` (on
X11) in the systemd user environment for direct mode; most desktop sessions
import it, check with `systemctl --user show-environment`.

If you start `monado-service` by hand instead, a desktop launcher or a script
that redirects stdin from `/dev/null` works: the service then simply does not
watch stdin for a key press to quit.

## Realtime priority

The service raises its compositor and driver threads to realtime priority, which
needs `CAP_SYS_NICE`. Configure with `-DXRT_INSTALL_SERVICE_CAP_SYS_NICE=ON` to
have the install step run `setcap cap_sys_nice=eip` on the installed service.
Installing as a normal user cannot set capabilities; the install then prints
the `sudo setcap ...` command to run. Every install replaces the binary and
drops the capability, so this has to be repeated after each install.

## Make Monado visible inside the Steam container

pressure-vessel can import the host's active OpenXR runtime, together with its
library dependencies, and share the Monado socket into the container. Enable it
by setting this variable in the environment Steam is started from:

```sh
PRESSURE_VESSEL_IMPORT_OPENXR_1_RUNTIMES=1
```

With it set, pressure-vessel reads the active runtime from
`~/.config/openxr/1/active_runtime.json` (or the system locations), copies the
runtime and its dependencies into the container, and bind-mounts
`$XDG_RUNTIME_DIR/monado_comp_ipc` if it exists when the game starts. With
socket activation enabled the socket always exists.

Steam must be restarted (in a session that has the variable) for it to take
effect. Where to set it depends on the desktop; for example:

* Display managers such as LightDM source `~/.profile` (and `~/.xprofile`)
  when starting an X11 session, so
  `export PRESSURE_VESSEL_IMPORT_OPENXR_1_RUNTIMES=1` there applies to the
  whole session after logging in again.
* Sessions started by systemd read `~/.config/environment.d/*.conf`.
* Per game, it can be put in the Steam launch options:
  `PRESSURE_VESSEL_IMPORT_OPENXR_1_RUNTIMES=1 %command%`.

On older Steam Linux Runtime versions without this feature, share the socket
explicitly per game with
`PRESSURE_VESSEL_FILESYSTEMS_RW=$XDG_RUNTIME_DIR/monado_comp_ipc %command%`
and make sure every library `libopenxr_monado.so` links against is available
inside the container.

## OpenVR games

Many Steam VR games use OpenVR rather than OpenXR. They load the OpenVR runtime
listed first in `~/.config/openvr/openvrpaths.vrpath`. Point that at an OpenVR
to OpenXR translation layer such as [xrizer][] or [OpenComposite][]:

```json
{
	"config": ["/home/user/.local/share/Steam/config"],
	"external_drivers": null,
	"jsonid": "vrpathreg",
	"log": ["/home/user/.local/share/Steam/logs"],
	"runtime": ["/home/user/.local/share/xrizer"],
	"version": 1
}
```

The path is the directory that contains `bin/linux64/vrclient.so`. Paths in the
home directory are visible inside the container. SteamVR rewrites this file
whenever it starts, so make it read-only (`chmod 444`) once it is set up, and
restore it from a backup to go back to SteamVR.

[xrizer]: https://github.com/Supreeeme/xrizer
[OpenComposite]: https://gitlab.com/znixian/OpenOVR

## Why not the SteamVR plugin?

Monado also builds a SteamVR driver plugin (`XRT_FEATURE_STEAMVR_PLUGIN`).
Current SteamVR releases on Linux run `vrserver` inside the Steam Linux Runtime
container too, so the plugin can only load if every library it links against
exists inside that container. A plugin built against the host's libraries (for
example OpenCV, GStreamer or libuvc for camera-based tracking) fails to load,
and SteamVR reports `VRInitError_Init_HmdNotFound` (error 108). Its log,
`~/.local/share/Steam/logs/vrserver.txt`, shows the missing library:

```
Unable to load driver monado from .../driver_monado.so (libuvc.so.0: cannot open shared object file: No such file or directory)
```

## Troubleshooting

* `$XDG_RUNTIME_DIR/monado_comp_ipc` missing inside the container: check that
  Steam's environment has `PRESSURE_VESSEL_IMPORT_OPENXR_1_RUNTIMES=1`
  (`tr '\0' '\n' < /proc/$(pgrep -x steam)/environ`) and that the socket exists
  on the host before the game starts.
* To see the runtime's client-side errors, which go to the game's stderr, use
  the launch options `XRT_LOG=debug %command% > /tmp/game.log 2>&1`.
* `PRESSURE_VESSEL_VERBOSE=1` makes pressure-vessel log which OpenXR runtime it
  imported and whether it found the socket.
