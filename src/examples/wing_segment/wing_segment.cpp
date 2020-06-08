/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 *	@file wing_segment.cpp
 *
 *	Example of winglet uORB topic subscription and printing.
 *
 * 	@author Joe DeFrance
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>

#include <poll.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_wing_segment.h>

extern "C" __EXPORT int wing_segment_main(int argc, char *argv[]);

int wing_segment_main(int argc, char *argv[])
{
	PX4_INFO("Entering Wing Segment Main...");

	// PX4_INFO("Argv:");
	// for (int i = 0; i < argc; ++i)
	// {
	// 	PX4_INFO("  %d: %s", i, argv[i]);
	// }

	int sensor_wing_segment_fd[4];

	px4_pollfd_struct_t wing_segment[4];

	for(int n = 0; n < 4; n++)
	{
		sensor_wing_segment_fd[n] = orb_subscribe_multi(ORB_ID(sensor_wing_segment), n);
		wing_segment[n] = {.fd = sensor_wing_segment_fd[n], .events = POLLIN};
	}

	int error_counter = 0;

	float xyzw[4][4];

	// Print 5 instances
	for (int instance = 0; instance < 5; instance++)
	{
		for(int channel = 0; channel < 4; channel++)
		{
			/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
			int poll_ret = px4_poll(&(wing_segment[channel]), 1, 1000);

			/* handle the poll result */
			if (poll_ret == 0)
			{
				/* this means none of our providers is giving us data */
				PX4_ERR("Got no data from winglet %d within a second", channel);
			}
			else if (poll_ret < 0)
			{
				/* this is seriously bad - should be an emergency */
				if (error_counter < 10 || error_counter % 50 == 0)
				{
					/* use a counter to prevent flooding (and slowing us down) */
					PX4_ERR("ERROR return value from poll(): %d", poll_ret);
				}
				error_counter++;

			}
			else
			{

				if(wing_segment[channel].revents & POLLIN)
				{
					/* obtained data for the file descriptor */
					struct sensor_wing_segment_s wing_segment_msg;

					/* copy sensors raw data into local buffer */
					orb_copy(ORB_ID(sensor_wing_segment), sensor_wing_segment_fd[channel], &wing_segment_msg);

					/* print faw data */
					PX4_INFO("\nChannel %d:\n  Timestamp: %llu\n  ID: %d\n  CAL: %d\n  X: %f\n  Y: %f\n  Z: %f\n  W: %f\n",
						channel,
						wing_segment_msg.timestamp,
						wing_segment_msg.id,
						wing_segment_msg.callibration,
						(double)wing_segment_msg.x,
						(double)wing_segment_msg.y,
						(double)wing_segment_msg.z,
						(double)wing_segment_msg.w
					);

					/* store raw data according to id */
					xyzw[wing_segment_msg.id][0] = wing_segment_msg.x;
					xyzw[wing_segment_msg.id][1] = wing_segment_msg.y;
					xyzw[wing_segment_msg.id][2] = wing_segment_msg.z;
					xyzw[wing_segment_msg.id][3] = wing_segment_msg.w;
				}
			}
		}

		// Calculate quaternion dot product
		double dots[] = {0, 0, 0, 0};
		for(int joint = 0; joint < 4; joint++)
		{
			for(int k = 0; k < 4; k++)
			{
				dots[joint] += (double)(xyzw[joint][k]*xyzw[(joint+1)%4][k]);
			}
		}

		// Compute angles between adjacent unit quaternions
		double angles[4];
		for(int joint = 0; joint < 4; joint++)
		{
			angles[joint] = 2 * acos(dots[joint]) / 3.14 * 180;
		}

		// Print angles
		for(int joint = 0; joint < 4; joint++)
		{
			PX4_INFO("Angle %d: %f deg", joint, angles[joint]);
		}
	}

	PX4_INFO("Exiting Wing Segment Main...");

	return 0;
}
