// Copyright 2020-2021, N Madsen.
// Copyright 2020-2021, Collabora, Ltd.
// Copyright 2021-2023, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
//
/*!
 * @file
 * @brief Implementation of Original & HP WMR controllers
 * @author Jan Schmidt <jan@centricular.com>
 * @author Nis Madsen <nima_zero_one@protonmail.com>
 * @ingroup drv_wmr
 */
#pragma once

#include "wmr_controller_base.h"
#include "xrt/xrt_frameserver.h"

/*!
 * Create a controller device. With @p start, the controller hardware is also initialised (see
 * wmr_controller_base_start()), which needs the controller to be online. Without it, the device is
 * created without talking to the controller, and wmr_controller_base_start() must be called once
 * the controller comes online.
 */
struct wmr_controller_base *
wmr_controller_create(struct wmr_controller_connection *conn,
                      enum xrt_device_type controller_type,
                      uint16_t vid,
                      uint16_t pid,
                      bool start,
                      enum u_logging_level log_level,
                      struct xrt_fs *src);
