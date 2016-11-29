/*
 * v4l2-dv-timings - dv-timings helper functions
 *
 * Copyright 2013 Cisco Systems, Inc. and/or its affiliates. All rights reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/videodev2.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-dv-timings.h>

MODULE_AUTHOR("Hans Verkuil");
MODULE_DESCRIPTION("V4L2 DV Timings Helper Functions");
MODULE_LICENSE("GPL");

const struct v4l2_dv_timings v4l2_dv_timings_presets[] = {
	V4L2_DV_BT_CEA_640X480P59_94,
	V4L2_DV_BT_CEA_720X480I59_94,
	V4L2_DV_BT_CEA_720X480P59_94,
	V4L2_DV_BT_CEA_720X576I50,
	V4L2_DV_BT_CEA_720X576P50,
	V4L2_DV_BT_CEA_1280X720P24,
	V4L2_DV_BT_CEA_1280X720P25,
	V4L2_DV_BT_CEA_1280X720P30,
	V4L2_DV_BT_CEA_1280X720P50,
	V4L2_DV_BT_CEA_1280X720P60,
	V4L2_DV_BT_CEA_1920X1080P24,
	V4L2_DV_BT_CEA_1920X1080P25,
	V4L2_DV_BT_CEA_1920X1080P30,
	V4L2_DV_BT_CEA_1920X1080I50,
	V4L2_DV_BT_CEA_1920X1080P50,
	V4L2_DV_BT_CEA_1920X1080I60,
	V4L2_DV_BT_CEA_1920X1080P60,
	V4L2_DV_BT_DMT_640X350P85,
	V4L2_DV_BT_DMT_640X400P85,
	V4L2_DV_BT_DMT_720X400P85,
	V4L2_DV_BT_DMT_640X480P72,
	V4L2_DV_BT_DMT_640X480P75,
	V4L2_DV_BT_DMT_640X480P85,
	V4L2_DV_BT_DMT_800X600P56,
	V4L2_DV_BT_DMT_800X600P60,
	V4L2_DV_BT_DMT_800X600P72,
	V4L2_DV_BT_DMT_800X600P75,
	V4L2_DV_BT_DMT_800X600P85,
	V4L2_DV_BT_DMT_800X600P120_RB,
	V4L2_DV_BT_DMT_848X480P60,
	V4L2_DV_BT_DMT_1024X768I43,
	V4L2_DV_BT_DMT_1024X768P60,
	V4L2_DV_BT_DMT_1024X768P70,
	V4L2_DV_BT_DMT_1024X768P75,
	V4L2_DV_BT_DMT_1024X768P85,
	V4L2_DV_BT_DMT_1024X768P120_RB,
	V4L2_DV_BT_DMT_1152X864P75,
	V4L2_DV_BT_DMT_1280X768P60_RB,
	V4L2_DV_BT_DMT_1280X768P60,
	V4L2_DV_BT_DMT_1280X768P75,
	V4L2_DV_BT_DMT_1280X768P85,
	V4L2_DV_BT_DMT_1280X768P120_RB,
	V4L2_DV_BT_DMT_1280X800P60_RB,
	V4L2_DV_BT_DMT_1280X800P60,
	V4L2_DV_BT_DMT_1280X800P75,
	V4L2_DV_BT_DMT_1280X800P85,
	V4L2_DV_BT_DMT_1280X800P120_RB,
	V4L2_DV_BT_DMT_1280X960P60,
	V4L2_DV_BT_DMT_1280X960P85,
	V4L2_DV_BT_DMT_1280X960P120_RB,
	V4L2_DV_BT_DMT_1280X1024P60,
	V4L2_DV_BT_DMT_1280X1024P75,
	V4L2_DV_BT_DMT_1280X1024P85,
	V4L2_DV_BT_DMT_1280X1024P120_RB,
	V4L2_DV_BT_DMT_1360X768P60,
	V4L2_DV_BT_DMT_1360X768P120_RB,
	V4L2_DV_BT_DMT_1366X768P60,
	V4L2_DV_BT_DMT_1366X768P60_RB,
	V4L2_DV_BT_DMT_1400X1050P60_RB,
	V4L2_DV_BT_DMT_1400X1050P60,
	V4L2_DV_BT_DMT_1400X1050P75,
	V4L2_DV_BT_DMT_1400X1050P85,
	V4L2_DV_BT_DMT_1400X1050P120_RB,
	V4L2_DV_BT_DMT_1440X900P60_RB,
	V4L2_DV_BT_DMT_1440X900P60,
	V4L2_DV_BT_DMT_1440X900P75,
	V4L2_DV_BT_DMT_1440X900P85,
	V4L2_DV_BT_DMT_1440X900P120_RB,
	V4L2_DV_BT_DMT_1600X900P60_RB,
	V4L2_DV_BT_DMT_1600X1200P60,
	V4L2_DV_BT_DMT_1600X1200P65,
	V4L2_DV_BT_DMT_1600X1200P70,
	V4L2_DV_BT_DMT_1600X1200P75,
	V4L2_DV_BT_DMT_1600X1200P85,
	V4L2_DV_BT_DMT_1600X1200P120_RB,
	V4L2_DV_BT_DMT_1680X1050P60_RB,
	V4L2_DV_BT_DMT_1680X1050P60,
	V4L2_DV_BT_DMT_1680X1050P75,
	V4L2_DV_BT_DMT_1680X1050P85,
	V4L2_DV_BT_DMT_1680X1050P120_RB,
	V4L2_DV_BT_DMT_1792X1344P60,
	V4L2_DV_BT_DMT_1792X1344P75,
	V4L2_DV_BT_DMT_1792X1344P120_RB,
	V4L2_DV_BT_DMT_1856X1392P60,
	V4L2_DV_BT_DMT_1856X1392P75,
	V4L2_DV_BT_DMT_1856X1392P120_RB,
	V4L2_DV_BT_DMT_1920X1200P60_RB,
	V4L2_DV_BT_DMT_1920X1200P60,
	V4L2_DV_BT_DMT_1920X1200P75,
	V4L2_DV_BT_DMT_1920X1200P85,
	V4L2_DV_BT_DMT_1920X1200P120_RB,
	V4L2_DV_BT_DMT_1920X1440P60,
	V4L2_DV_BT_DMT_1920X1440P75,
	V4L2_DV_BT_DMT_1920X1440P120_RB,
	V4L2_DV_BT_DMT_2048X1152P60_RB,
	V4L2_DV_BT_DMT_2560X1600P60_RB,
	V4L2_DV_BT_DMT_2560X1600P60,
	V4L2_DV_BT_DMT_2560X1600P75,
	V4L2_DV_BT_DMT_2560X1600P85,
	V4L2_DV_BT_DMT_2560X1600P120_RB,
	{ }
};
EXPORT_SYMBOL_GPL(v4l2_dv_timings_presets);

bool v4l2_valid_dv_timings(const struct v4l2_dv_timings *t,
			   const struct v4l2_dv_timings_cap *dvcap,
			   v4l2_check_dv_timings_fnc fnc,
			   void *fnc_handle)
{
	const struct v4l2_bt_timings *bt = &t->bt;
	const struct v4l2_bt_timings_cap *cap = &dvcap->bt;
	u32 caps = cap->capabilities;

	if (t->type != V4L2_DV_BT_656_1120)
		return false;
	if (t->type != dvcap->type ||
	    bt->height < cap->min_height ||
	    bt->height > cap->max_height ||
	    bt->width < cap->min_width ||
	    bt->width > cap->max_width ||
	    bt->pixelclock < cap->min_pixelclock ||
	    bt->pixelclock > cap->max_pixelclock ||
	    (cap->standards && !(bt->standards & cap->standards)) ||
	    (bt->interlaced && !(caps & V4L2_DV_BT_CAP_INTERLACED)) ||
	    (!bt->interlaced && !(caps & V4L2_DV_BT_CAP_PROGRESSIVE)))
		return false;
	return fnc == NULL || fnc(t, fnc_handle);
}
EXPORT_SYMBOL_GPL(v4l2_valid_dv_timings);

int v4l2_enum_dv_timings_cap(struct v4l2_enum_dv_timings *t,
			     const struct v4l2_dv_timings_cap *cap,
			     v4l2_check_dv_timings_fnc fnc,
			     void *fnc_handle)
{
	u32 i, idx;

	memset(t->reserved, 0, sizeof(t->reserved));
	for (i = idx = 0; v4l2_dv_timings_presets[i].bt.width; i++) {
		if (v4l2_valid_dv_timings(v4l2_dv_timings_presets + i, cap,
					  fnc, fnc_handle) &&
		    idx++ == t->index) {
			t->timings = v4l2_dv_timings_presets[i];
			return 0;
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(v4l2_enum_dv_timings_cap);

bool v4l2_find_dv_timings_cap(struct v4l2_dv_timings *t,
			      const struct v4l2_dv_timings_cap *cap,
			      unsigned pclock_delta,
			      v4l2_check_dv_timings_fnc fnc,
			      void *fnc_handle)
{
	int i;

	if (!v4l2_valid_dv_timings(t, cap, fnc, fnc_handle))
		return false;

	for (i = 0; i < v4l2_dv_timings_presets[i].bt.width; i++) {
		if (v4l2_valid_dv_timings(v4l2_dv_timings_presets + i, cap,
					  fnc, fnc_handle) &&
		    v4l2_match_dv_timings(t, v4l2_dv_timings_presets + i,
					  pclock_delta)) {
			*t = v4l2_dv_timings_presets[i];
			return true;
		}
	}
	return false;
}
EXPORT_SYMBOL_GPL(v4l2_find_dv_timings_cap);

/**
 * v4l2_match_dv_timings - check if two timings match
 * @t1 - compare this v4l2_dv_timings struct...
 * @t2 - with this struct.
 * @pclock_delta - the allowed pixelclock deviation.
 *
 * Compare t1 with t2 with a given margin of error for the pixelclock.
 */
bool v4l2_match_dv_timings(const struct v4l2_dv_timings *t1,
			   const struct v4l2_dv_timings *t2,
			   unsigned pclock_delta)
{
	if (t1->type != t2->type || t1->type != V4L2_DV_BT_656_1120)
		return false;
	if (t1->bt.width == t2->bt.width &&
	    t1->bt.height == t2->bt.height &&
	    t1->bt.interlaced == t2->bt.interlaced &&
	    t1->bt.polarities == t2->bt.polarities &&
	    t1->bt.pixelclock >= t2->bt.pixelclock - pclock_delta &&
	    t1->bt.pixelclock <= t2->bt.pixelclock + pclock_delta &&
	    t1->bt.hfrontporch == t2->bt.hfrontporch &&
	    t1->bt.vfrontporch == t2->bt.vfrontporch &&
	    t1->bt.vsync == t2->bt.vsync &&
	    t1->bt.vbackporch == t2->bt.vbackporch &&
	    (!t1->bt.interlaced ||
		(t1->bt.il_vfrontporch == t2->bt.il_vfrontporch &&
		 t1->bt.il_vsync == t2->bt.il_vsync &&
		 t1->bt.il_vbackporch == t2->bt.il_vbackporch)))
		return true;
	return false;
}
EXPORT_SYMBOL_GPL(v4l2_match_dv_timings);

void v4l2_print_dv_timings(const char *dev_prefix, const char *prefix,
			   const struct v4l2_dv_timings *t, bool detailed)
{
	const struct v4l2_bt_timings *bt = &t->bt;
	u32 htot, vtot;

	if (t->type != V4L2_DV_BT_656_1120)
		return;

	htot = V4L2_DV_BT_FRAME_WIDTH(bt);
	vtot = V4L2_DV_BT_FRAME_HEIGHT(bt);

	if (prefix == NULL)
		prefix = "";

	pr_info("%s: %s%ux%u%s%u (%ux%u)\n", dev_prefix, prefix,
		bt->width, bt->height, bt->interlaced ? "i" : "p",
		(htot * vtot) > 0 ? ((u32)bt->pixelclock / (htot * vtot)) : 0,
		htot, vtot);

	if (!detailed)
		return;

	pr_info("%s: horizontal: fp = %u, %ssync = %u, bp = %u\n",
			dev_prefix, bt->hfrontporch,
			(bt->polarities & V4L2_DV_HSYNC_POS_POL) ? "+" : "-",
			bt->hsync, bt->hbackporch);
	pr_info("%s: vertical: fp = %u, %ssync = %u, bp = %u\n",
			dev_prefix, bt->vfrontporch,
			(bt->polarities & V4L2_DV_VSYNC_POS_POL) ? "+" : "-",
			bt->vsync, bt->vbackporch);
	pr_info("%s: pixelclock: %llu\n", dev_prefix, bt->pixelclock);
	pr_info("%s: flags (0x%x):%s%s%s%s\n", dev_prefix, bt->flags,
			(bt->flags & V4L2_DV_FL_REDUCED_BLANKING) ?
			" REDUCED_BLANKING" : "",
			(bt->flags & V4L2_DV_FL_CAN_REDUCE_FPS) ?
			" CAN_REDUCE_FPS" : "",
			(bt->flags & V4L2_DV_FL_REDUCED_FPS) ?
			" REDUCED_FPS" : "",
			(bt->flags & V4L2_DV_FL_HALF_LINE) ?
			" HALF_LINE" : "");
	pr_info("%s: standards (0x%x):%s%s%s%s\n", dev_prefix, bt->standards,
			(bt->standards & V4L2_DV_BT_STD_CEA861) ?  " CEA" : "",
			(bt->standards & V4L2_DV_BT_STD_DMT) ?  " DMT" : "",
			(bt->standards & V4L2_DV_BT_STD_CVT) ?  " CVT" : "",
			(bt->standards & V4L2_DV_BT_STD_GTF) ?  " GTF" : "");
}
EXPORT_SYMBOL_GPL(v4l2_print_dv_timings);
