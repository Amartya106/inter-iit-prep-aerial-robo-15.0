/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "failure_scheduler.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <math.h>
#include <uORB/topics/parameter_update.h>

ModuleBase::Descriptor FailureScheduler::desc{task_spawn, custom_command, print_usage};

int FailureScheduler::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int FailureScheduler::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running(desc)) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance<FailureScheduler>(desc)->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int FailureScheduler::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return FailureScheduler::instantiate(ac, av);
	}, argc, argv);
}

int FailureScheduler::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("module",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  1024,
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	return 0;
}

FailureScheduler *FailureScheduler::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	FailureScheduler *instance = new FailureScheduler(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

FailureScheduler::FailureScheduler(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void FailureScheduler::run()
{
	// initialize parameters
	parameters_update(true);

	bool injected = false;
	bool hovering = false;
	hrt_abstime hover_start = 0;

	static constexpr float TARGET_ALT   = 20.0f;
	static constexpr float ALT_TOL      = 1.0f;    // within +/- 1 m of target
	static constexpr float VZ_TOL       = 0.3f;    // m/s, near-zero climb rate
	static constexpr hrt_abstime HOLD   = 5_s;

	while (!should_exit()) {

		if (!injected && _local_pos_sub.updated()) {
			vehicle_local_position_s pos{};
			_local_pos_sub.copy(&pos);

			const float altitude = -pos.z;   // NED: down is positive

			const bool stable = pos.z_valid && pos.v_z_valid
					    && fabsf(altitude - TARGET_ALT) < ALT_TOL
					    && fabsf(pos.vz) < VZ_TOL;

			if (stable) {
				if (!hovering) {
					hovering = true;
					hover_start = hrt_absolute_time();
					PX4_INFO("Stable hover at %.1f m - holding %llu s",
						 (double)altitude, (unsigned long long)(HOLD / 1000000));
				}

				if (hrt_elapsed_time(&hover_start) > HOLD) {
					vehicle_command_s cmd{};
					cmd.command = vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE;
					cmd.param1 = failure_injection_s::FAILURE_UNIT_SYSTEM_MOTOR;
					cmd.param2 = failure_injection_s::FAILURE_TYPE_OFF;
					cmd.param3 = 1;                 // instance, 1-based
					cmd.target_system = 1;
					cmd.target_component = 1;
					cmd.timestamp = hrt_absolute_time();
					_vehicle_command_pub.publish(cmd);

					injected = true;
					PX4_WARN("Motor 1 failure injected after %.1f s hover at %.1f m",
						 (double)(HOLD / 1e6), (double)altitude);
				}

			} else if (hovering) {
				hovering = false;
				PX4_INFO("Hover lost at %.1f m - restarting hold timer", (double)altitude);
			}
		}

		parameters_update();
		px4_usleep(10_ms);
	}
}

void FailureScheduler::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int FailureScheduler::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

Injects a motor failure automatically once the vehicle reaches a target altitude, for Phase 1 reference characterization.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("failure_scheduler", "command");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int failure_scheduler_main(int argc, char *argv[])
{
	return ModuleBase::main(FailureScheduler::desc, argc, argv);
}
