/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
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

/* Auto-generated by genmsg_cpp from file actuator_controls.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>

constexpr char __orb_actuator_controls_fields[] = "uint64_t timestamp;uint64_t timestamp_sample;float[8] control;";

ORB_DEFINE(actuator_controls, struct actuator_controls_s, 48, __orb_actuator_controls_fields, static_cast<uint8_t>(ORB_ID::actuator_controls));
ORB_DEFINE(actuator_controls_0, struct actuator_controls_s, 48, __orb_actuator_controls_fields, static_cast<uint8_t>(ORB_ID::actuator_controls_0));
ORB_DEFINE(actuator_controls_1, struct actuator_controls_s, 48, __orb_actuator_controls_fields, static_cast<uint8_t>(ORB_ID::actuator_controls_1));
ORB_DEFINE(actuator_controls_2, struct actuator_controls_s, 48, __orb_actuator_controls_fields, static_cast<uint8_t>(ORB_ID::actuator_controls_2));
ORB_DEFINE(actuator_controls_3, struct actuator_controls_s, 48, __orb_actuator_controls_fields, static_cast<uint8_t>(ORB_ID::actuator_controls_3));
ORB_DEFINE(actuator_controls_virtual_fw, struct actuator_controls_s, 48, __orb_actuator_controls_fields, static_cast<uint8_t>(ORB_ID::actuator_controls_virtual_fw));
ORB_DEFINE(actuator_controls_virtual_mc, struct actuator_controls_s, 48, __orb_actuator_controls_fields, static_cast<uint8_t>(ORB_ID::actuator_controls_virtual_mc));


void print_message(const actuator_controls_s &message)
{

	PX4_INFO_RAW(" actuator_controls_s\n");

	const hrt_abstime now = hrt_absolute_time();

	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, (now - message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	
	PX4_INFO_RAW("\ttimestamp_sample: %" PRIu64 "  (%" PRIu64 " us before timestamp)\n", message.timestamp_sample, message.timestamp - message.timestamp_sample);
	
	PX4_INFO_RAW("\tcontrol: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n", (double)message.control[0], (double)message.control[1], (double)message.control[2], (double)message.control[3], (double)message.control[4], (double)message.control[5], (double)message.control[6], (double)message.control[7]);

}
