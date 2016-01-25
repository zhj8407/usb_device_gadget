/*
 * v4l2-dv-timings - Internal header with dv-timings helper functions
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

#ifndef __V4L2_DV_TIMINGS_H
#define __V4L2_DV_TIMINGS_H

#include <linux/videodev2.h>

/** v4l2_dv_timings_presets: list of all dv_timings presets.
 */
extern const struct v4l2_dv_timings v4l2_dv_timings_presets[];

/** v4l2_check_dv_timings_fnc - timings check callback
 * @t: the v4l2_dv_timings struct.
 * @handle: a handle from the driver.
 *
 * Returns true if the given timings are valid.
 */
typedef bool v4l2_check_dv_timings_fnc(const struct v4l2_dv_timings *t, void *handle);

/** v4l2_valid_dv_timings() - are these timings valid?
  * @t:	  the v4l2_dv_timings struct.
  * @cap: the v4l2_dv_timings_cap capabilities.
  * @fnc: callback to check if this timing is OK. May be NULL.
  * @fnc_handle: a handle that is passed on to @fnc.
  *
  * Returns true if the given dv_timings struct is supported by the
  * hardware capabilities and the callback function (if non-NULL), returns
  * false otherwise.
  */
bool v4l2_valid_dv_timings(const struct v4l2_dv_timings *t,
			   const struct v4l2_dv_timings_cap *cap,
			   v4l2_check_dv_timings_fnc fnc,
			   void *fnc_handle);

/** v4l2_enum_dv_timings_cap() - Helper function to enumerate possible DV timings based on capabilities
  * @t:	  the v4l2_enum_dv_timings struct.
  * @cap: the v4l2_dv_timings_cap capabilities.
  * @fnc: callback to check if this timing is OK. May be NULL.
  * @fnc_handle: a handle that is passed on to @fnc.
  *
  * This enumerates dv_timings using the full list of possible CEA-861 and DMT
  * timings, filtering out any timings that are not supported based on the
  * hardware capabilities and the callback function (if non-NULL).
  *
  * If a valid timing for the given index is found, it will fill in @t and
  * return 0, otherwise it returns -EINVAL.
  */
int v4l2_enum_dv_timings_cap(struct v4l2_enum_dv_timings *t,
			     const struct v4l2_dv_timings_cap *cap,
			     v4l2_check_dv_timings_fnc fnc,
			     void *fnc_handle);

/** v4l2_find_dv_timings_cap() - Find the closest timings struct
  * @t:	  the v4l2_enum_dv_timings struct.
  * @cap: the v4l2_dv_timings_cap capabilities.
  * @pclock_delta: maximum delta between t->pixelclock and the timing struct
  *		under consideration.
  * @fnc: callback to check if a given timings struct is OK. May be NULL.
  * @fnc_handle: a handle that is passed on to @fnc.
  *
  * This function tries to map the given timings to an entry in the
  * full list of possible CEA-861 and DMT timings, filtering out any timings
  * that are not supported based on the hardware capabilities and the callback
  * function (if non-NULL).
  *
  * On success it will fill in @t with the found timings and it returns true.
  * On failure it will return false.
  */
bool v4l2_find_dv_timings_cap(struct v4l2_dv_timings *t,
			      const struct v4l2_dv_timings_cap *cap,
			      unsigned pclock_delta,
			      v4l2_check_dv_timings_fnc fnc,
			      void *fnc_handle);

/** v4l2_match_dv_timings() - do two timings match?
  * @measured:	  the measured timings data.
  * @standard:	  the timings according to the standard.
  * @pclock_delta: maximum delta in Hz between standard->pixelclock and
  * 		the measured timings.
  *
  * Returns true if the two timings match, returns false otherwise.
  */
bool v4l2_match_dv_timings(const struct v4l2_dv_timings *measured,
			   const struct v4l2_dv_timings *standard,
			   unsigned pclock_delta);

/** v4l2_print_dv_timings() - log the contents of a dv_timings struct
  * @dev_prefix:device prefix for each log line.
  * @prefix:	additional prefix for each log line, may be NULL.
  * @t:		the timings data.
  * @detailed:	if true, give a detailed log.
  */
void v4l2_print_dv_timings(const char *dev_prefix, const char *prefix,
			   const struct v4l2_dv_timings *t, bool detailed);

#endif
