// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Common direct mode window code.
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup comp_main
 */


#include "comp_window_direct.h"

#include "util/u_misc.h"
#include "os/os_time.h"
#include "util/u_time.h"

#ifdef VK_USE_PLATFORM_XLIB_XRANDR_EXT
#include <xcb/randr.h>
#include <X11/Xlib-xcb.h>
#endif


static inline struct vk_bundle *
get_vk(struct comp_target_swapchain *cts)
{
	return &cts->base.c->base.vk;
}

static int
choose_best_vk_mode_auto(struct comp_target *ct, VkDisplayModePropertiesKHR *mode_properties, int mode_count)
{
	if (mode_count == 1)
		return 0;

	int best_index = 0;

	VkDisplayModeParametersKHR *current = &mode_properties[0].parameters;

	COMP_DEBUG(ct->c, "Available Vk direct mode %d: %dx%d@%.2f", 0, current->visibleRegion.width,
	           current->visibleRegion.height, (float)current->refreshRate / 1000.);

	// First priority: choose mode that maximizes rendered pixels.
	// Second priority: choose mode with highest refresh rate.
	for (int i = 1; i < mode_count; i++) {
		current = &mode_properties[i].parameters;

		COMP_DEBUG(ct->c, "Available Vk direct mode %d: %dx%d@%.2f", i, current->visibleRegion.width,
		           current->visibleRegion.height, (float)current->refreshRate / 1000.);


		VkDisplayModeParametersKHR best = mode_properties[best_index].parameters;

		int best_pixels = best.visibleRegion.width * best.visibleRegion.height;
		int pixels = current->visibleRegion.width * current->visibleRegion.height;
		if (pixels > best_pixels) {
			best_index = i;
		} else if (pixels == best_pixels && current->refreshRate > best.refreshRate) {
			best_index = i;
		}
	}
	VkDisplayModeParametersKHR best = mode_properties[best_index].parameters;
	COMP_DEBUG(ct->c, "Auto choosing Vk direct mode %d: %dx%d@%.2f", best_index, best.visibleRegion.width,
	           best.visibleRegion.height, (float)best.refreshRate / 1000.f);
	return best_index;
}

static void
print_modes(struct comp_target *ct, VkDisplayModePropertiesKHR *mode_properties, int mode_count)
{
	COMP_PRINT_MODE(ct->c, "Available Vk modes for direct mode");
	for (int i = 0; i < mode_count; i++) {
		VkDisplayModePropertiesKHR *props = &mode_properties[i];
		uint32_t width = props->parameters.visibleRegion.width;
		uint32_t height = props->parameters.visibleRegion.height;
		float refresh = (float)props->parameters.refreshRate / 1000.f;

		COMP_PRINT_MODE(ct->c, "| %2d | %dx%d@%.2f", i, width, height, refresh);
	}
	COMP_PRINT_MODE(ct->c, "Listed %d modes", mode_count);
}

static VkDisplayModeKHR
get_primary_display_mode(struct comp_target_swapchain *cts,
                         VkDisplayKHR display,
                         uint32_t *out_width,
                         uint32_t *out_height)
{
	struct vk_bundle *vk = get_vk(cts);
	struct comp_target *ct = &cts->base;
	VkDisplayModePropertiesKHR *mode_properties = NULL;
	uint32_t mode_count = 0;
	VkResult ret;

	// Get plane properties
	ret = vk_enumerate_display_mode_properties( //
	    vk,                                     //
	    vk->physical_device,                    //
	    display,                                //
	    &mode_count,                            //
	    &mode_properties);                      //
	if (ret != VK_SUCCESS) {
		COMP_ERROR(ct->c, "vk_enumerate_display_mode_properties: %s", vk_result_string(ret));
		return VK_NULL_HANDLE;
	}
	if (mode_count == 0) {
		COMP_ERROR(ct->c, "No Vulkan display modes available");
		free(mode_properties);
		return VK_NULL_HANDLE;
	}


	/*
	 * Debug information.
	 */

	COMP_DEBUG(ct->c, "Found %d modes", mode_count);
	print_modes(ct, mode_properties, mode_count);


	/*
	 * Select the mode.
	 */

	int chosen_mode = 0;

	int desired_mode = ct->c->settings.desired_mode;
	if (desired_mode + 1 > (int)mode_count) {
		COMP_ERROR(ct->c,
		           "Requested mode index %d, but max is %d. Falling "
		           "back to automatic mode selection",
		           desired_mode, mode_count);
		chosen_mode = choose_best_vk_mode_auto(ct, mode_properties, mode_count);
	} else if (desired_mode < 0) {
		chosen_mode = choose_best_vk_mode_auto(ct, mode_properties, mode_count);
	} else {
		COMP_DEBUG(ct->c, "Using manually chosen mode %d", desired_mode);
		chosen_mode = desired_mode;
	}

	VkDisplayModePropertiesKHR props = mode_properties[chosen_mode];

	COMP_DEBUG(ct->c, "found display mode %dx%d@%.2f", props.parameters.visibleRegion.width,
	           props.parameters.visibleRegion.height, (float)props.parameters.refreshRate / 1000.);

	if (props.parameters.refreshRate == 0) {
		COMP_WARN(ct->c, "Display mode reports a refresh rate of zero, keeping the current frame interval");
	} else {
		int64_t new_frame_interval = (int64_t)(1000. * 1000. * 1000. * 1000. / props.parameters.refreshRate);

		COMP_DEBUG(ct->c, "Updating compositor frame interval from %" PRIu64 " (%f Hz) to %" PRIu64 " (%f Hz)",
		           ct->c->frame_interval_ns, 1000. * 1000. * 1000. / (float)ct->c->frame_interval_ns,
		           new_frame_interval, (float)props.parameters.refreshRate / 1000.);

		ct->c->frame_interval_ns = new_frame_interval;
	}

	free(mode_properties);

	*out_width = props.parameters.visibleRegion.width;
	*out_height = props.parameters.visibleRegion.height;

	return props.displayMode;
}

static VkDisplayPlaneAlphaFlagBitsKHR
choose_alpha_mode(VkDisplayPlaneAlphaFlagsKHR flags)
{
	if (flags & VK_DISPLAY_PLANE_ALPHA_PER_PIXEL_PREMULTIPLIED_BIT_KHR) {
		return VK_DISPLAY_PLANE_ALPHA_PER_PIXEL_PREMULTIPLIED_BIT_KHR;
	}
	if (flags & VK_DISPLAY_PLANE_ALPHA_PER_PIXEL_BIT_KHR) {
		return VK_DISPLAY_PLANE_ALPHA_PER_PIXEL_BIT_KHR;
	}
	return VK_DISPLAY_PLANE_ALPHA_GLOBAL_BIT_KHR;
}


/*
 *
 * 'Exported' functions.
 *
 */

VkResult
comp_window_direct_create_surface(struct comp_target_swapchain *cts,
                                  VkDisplayKHR display,
                                  uint32_t width,
                                  uint32_t height)
{
	struct vk_bundle *vk = get_vk(cts);
	VkDisplayPlanePropertiesKHR *plane_properties = NULL;
	uint32_t plane_property_count = 0;
	VkResult ret;

	// Get plane properties
	ret = vk_enumerate_physical_display_plane_properties( //
	    vk,                                               //
	    vk->physical_device,                              //
	    &plane_property_count,                            //
	    &plane_properties);                               //
	if (ret != VK_SUCCESS) {
		COMP_ERROR(cts->base.c, "vk_enumerate_physical_display_plane_properties: %s", vk_result_string(ret));
		return VK_ERROR_INITIALIZATION_FAILED;
	}

	// Select the plane.
	//! @todo actually select the plane.
	uint32_t plane_index = 0;
	uint32_t plane_stack_index = plane_properties[plane_index].currentStackIndex;

	free(plane_properties);
	plane_properties = NULL;

	// Select the mode.
	uint32_t mode_width = 0, mode_height = 0;
	VkDisplayModeKHR display_mode = get_primary_display_mode( //
	    cts,                                                  //
	    display,                                              //
	    &mode_width,                                          //
	    &mode_height);                                        //

	if (display_mode == VK_NULL_HANDLE) {
		COMP_ERROR(cts->base.c, "Failed to find display mode!");
		return VK_ERROR_INITIALIZATION_FAILED;
	}

	/*
	 * This fixes a bug on NVIDIA Jetson. Note this isn't so much the NVIDIA
	 * Jetson fault, while the code was working on desktop, Monado did
	 * something wrong. What happned was that Monado would select a mode
	 * with one size, while then creating a VkSurface/VkSwapchain of a
	 * different size. This would work on hardware with scalers/panning
	 * modes. The NVIDIA Jetson apparently doesn't have support for that so
	 * failed when presenting. This patch makes sure that the VkSurface &
	 * VkSwapchain extents match the mode for all direct mode targets.
	 */
	if (mode_width != width || mode_height != height) {
		COMP_INFO(cts->base.c,
		          "Ignoring given extent %dx%d and using %dx%d from mode, bugs could happen otherwise.",
		          width,        //
		          height,       //
		          mode_width,   //
		          mode_height); //
	}

	// We need the capabilities of the selected plane.
	VkDisplayPlaneCapabilitiesKHR plane_caps;
	vk->vkGetDisplayPlaneCapabilitiesKHR(vk->physical_device, display_mode, plane_index, &plane_caps);

	VkDisplaySurfaceCreateInfoKHR surface_info = {
	    .sType = VK_STRUCTURE_TYPE_DISPLAY_SURFACE_CREATE_INFO_KHR,
	    .pNext = NULL,
	    .flags = 0,
	    .displayMode = display_mode,
	    .planeIndex = plane_index,
	    .planeStackIndex = plane_stack_index,
	    .transform = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR,
	    .globalAlpha = 1.0,
	    .alphaMode = choose_alpha_mode(plane_caps.supportedAlpha),
	    .imageExtent =
	        {
	            .width = mode_width,
	            .height = mode_height,
	        },
	};

	// This function is called seldom so ok to always print.
	vk_print_display_surface_create_info(vk, &surface_info, U_LOGGING_INFO);

	// Everything decided and logged, do the creation.
	VkSurfaceKHR surface = VK_NULL_HANDLE;
	ret = vk->vkCreateDisplayPlaneSurfaceKHR( //
	    vk->instance,                         //
	    &surface_info,                        //
	    NULL,                                 //
	    &surface);                            //
	if (ret != VK_SUCCESS) {
		COMP_ERROR(cts->base.c, "vkCreateDisplayPlaneSurfaceKHR: %s", vk_result_string(ret));
		return ret;
	}

	VK_NAME_SURFACE(vk, surface, "comp_target_swapchain direct surface");
	cts->surface.handle = surface;

	return VK_SUCCESS;
}

#ifdef VK_USE_PLATFORM_XLIB_XRANDR_EXT

int
comp_window_direct_connect(struct comp_target_swapchain *cts, Display **dpy)
{
	*dpy = XOpenDisplay(NULL);
	if (*dpy == NULL) {
		COMP_ERROR(cts->base.c, "Could not open X display.");
		return false;
	}
	return true;
}

/*!
 * Some NVIDIA driver versions don't recognise the HMD EDID as non-desktop and
 * X enables the output as a normal desktop output, which makes
 * vkAcquireXlibDisplayEXT fail. Disable (only) the RandR output that belongs to
 * the given Vulkan display so that it can be acquired.
 *
 * The CRTC is only disabled if the HMD output is its sole output, so that a
 * desktop monitor sharing the CRTC (clone/mirror mode) is never blanked.
 */
static bool
disable_desktop_output(struct comp_target_swapchain *cts, Display *dpy, VkDisplayKHR display)
{
	struct vk_bundle *vk = get_vk(cts);
	VkDisplayPropertiesKHR *props = NULL;
	uint32_t count = 0;
	char name[64] = {0};
	bool disabled = false;

	if (vk_enumerate_physical_device_display_properties(vk, vk->physical_device, &count, &props) != VK_SUCCESS) {
		return false;
	}

	// The display name ends with "(<output name>)".
	for (uint32_t i = 0; i < count; i++) {
		const char *dn = props[i].displayName;
		if (props[i].display != display || dn == NULL) {
			continue;
		}
		const char *open = strrchr(dn, '(');
		const char *close = strrchr(dn, ')');
		if (open != NULL && close != NULL && close > open + 1 && (size_t)(close - open) < sizeof(name)) {
			memcpy(name, open + 1, close - open - 1);
		}
		break;
	}
	free(props);

	if (name[0] == '\0') {
		return false;
	}

	xcb_connection_t *conn = XGetXCBConnection(dpy);
	xcb_window_t root = DefaultRootWindow(dpy);

	xcb_randr_get_screen_resources_cookie_t res_cookie = xcb_randr_get_screen_resources(conn, root);
	xcb_randr_get_screen_resources_reply_t *res = xcb_randr_get_screen_resources_reply(conn, res_cookie, NULL);
	if (res == NULL) {
		return false;
	}

	xcb_randr_output_t *outputs = xcb_randr_get_screen_resources_outputs(res);
	int output_count = xcb_randr_get_screen_resources_outputs_length(res);
	for (int i = 0; i < output_count; i++) {
		xcb_randr_get_output_info_cookie_t oc =
		    xcb_randr_get_output_info(conn, outputs[i], res->config_timestamp);
		xcb_randr_get_output_info_reply_t *oi = xcb_randr_get_output_info_reply(conn, oc, NULL);
		if (oi == NULL) {
			continue;
		}

		int len = xcb_randr_get_output_info_name_length(oi);
		const char *oname = (const char *)xcb_randr_get_output_info_name(oi);
		if ((size_t)len != strlen(name) || strncmp(oname, name, len) != 0) {
			free(oi);
			continue;
		}

		// Found the HMD output, if it has no CRTC X isn't using it.
		if (oi->crtc != XCB_NONE) {
			xcb_randr_get_crtc_info_cookie_t ic =
			    xcb_randr_get_crtc_info(conn, oi->crtc, res->config_timestamp);
			xcb_randr_get_crtc_info_reply_t *ci = xcb_randr_get_crtc_info_reply(conn, ic, NULL);
			if (ci == NULL) {
				COMP_WARN(cts->base.c, "Failed to query CRTC of X output '%s', not disabling it.",
				          name);
			} else if (xcb_randr_get_crtc_info_outputs_length(ci) != 1 ||
			           xcb_randr_get_crtc_info_outputs(ci)[0] != outputs[i]) {
				COMP_WARN(cts->base.c,
				          "CRTC of X output '%s' is shared with other outputs, not disabling it.",
				          name);
			} else {
				xcb_randr_set_crtc_config_cookie_t cc =
				    xcb_randr_set_crtc_config(conn, oi->crtc, XCB_CURRENT_TIME, res->config_timestamp,
				                              0, 0, XCB_NONE, XCB_RANDR_ROTATION_ROTATE_0, 0, NULL);
				xcb_randr_set_crtc_config_reply_t *cr = xcb_randr_set_crtc_config_reply(conn, cc, NULL);
				if (cr != NULL && cr->status == XCB_RANDR_SET_CONFIG_SUCCESS) {
					COMP_INFO(cts->base.c,
					          "Disabled X desktop output '%s' (crtc %u) to acquire it.", name,
					          (unsigned)oi->crtc);
					disabled = true;
				} else {
					COMP_WARN(cts->base.c, "Failed to disable X desktop output '%s'.", name);
				}
				free(cr);
			}
			free(ci);
		}
		free(oi);
		break;
	}
	free(res);

	if (disabled) {
		XSync(dpy, False);
	}
	return disabled;
}

VkResult
comp_window_direct_acquire_xlib_display(struct comp_target_swapchain *cts, Display *dpy, VkDisplayKHR display)
{
	struct vk_bundle *vk = get_vk(cts);
	VkResult ret;

	ret = vk->vkAcquireXlibDisplayEXT(vk->physical_device, dpy, display);
	if (ret != VK_SUCCESS && disable_desktop_output(cts, dpy, display)) {
		for (int i = 0; i < 10 && ret != VK_SUCCESS; i++) {
			os_nanosleep(U_TIME_1MS_IN_NS * 100);
			ret = vk->vkAcquireXlibDisplayEXT(vk->physical_device, dpy, display);
		}
	}
	if (ret != VK_SUCCESS) {
		COMP_ERROR(cts->base.c, "vkAcquireXlibDisplayEXT: %s (0x%016" PRIx64 ")", vk_result_string(ret),
		           (uint64_t)display);
	}
	if (ret == VK_ERROR_INITIALIZATION_FAILED) {
		COMP_ERROR(
		    cts->base.c,
		    "If you are using the NVIDIA proprietary driver the above error can be caused by the AllowHMD "
		    "xorg.conf option. Please make sure that AllowHMD is not set (like in '99-HMD.conf' from OpenHMD) "
		    "and that the desktop is not currently extended to this display.");
	}
	return ret;
}

bool
comp_window_direct_init_swapchain(
    struct comp_target_swapchain *cts, Display *dpy, VkDisplayKHR display, uint32_t width, uint32_t height)
{
	VkResult ret;
	ret = comp_window_direct_acquire_xlib_display(cts, dpy, display);

	if (ret != VK_SUCCESS) {
		return false;
	}

	ret = comp_window_direct_create_surface(cts, display, width, height);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(cts->base.c, "Failed to create surface! '%s'", vk_result_string(ret));
		return false;
	}

	return true;
}

#endif
