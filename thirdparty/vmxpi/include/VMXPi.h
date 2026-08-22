/* ============================================
VMX-pi HAL source code is placed under the MIT license
Copyright (c) 2017 Kauai Labs
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef VMXPI_H_
#define VMXPI_H_

#include <stdint.h>
#include "AHRS.h"
#include "VMXCAN.h"
#include "VMXIO.h"
#include "VMXPower.h"
#include "VMXTime.h"
#include "VMXVersion.h"
#include "VMXThread.h"

#define MAX_NUM_TERMINATION_FUNCS 5

class VMXPiImpl;
class VMXRemoteServer;

/** Top-level Library Class providing access to all VMX-pi functionality. */
class VMXPi {

	VMXPiImpl*	p_impl;
	bool		initialized;
	void		Cleanup();
	static void	Terminate(void);
	void 		(*termination_func[MAX_NUM_TERMINATION_FUNCS])(void);
	uint8_t		num_termination_funcs;
	void 		(*final_termination_func)(void);
	VMXRemoteServer *p_remote_server;

#ifndef SWIG /* NOTE:  direct structure access is only available in C++ HAL */
public:
#endif

	/** The {@link #AHRS} object providing access to VMX-pi IMU functionality. */
	vmx::AHRS	ahrs;
	/** The {@link #VMXTime} object providing access to VMX-pi Time functionality. */
	VMXTime 	time;
	/** The {@link #VMXIO} object providing access to VMX-pi IO functionality. */
	VMXIO 		io;
	/** The {@link #VMXCAN} object providing access to VMX-pi CAN functionality. */
	VMXCAN		can;
	/** The {@link #VMXPower} object providing access to VMX-pi Power functionality. */
	VMXPower	power;
	/** The {@link #VMXVersion} object providing access to VMX-pi Version functionality. */
	VMXVersion	version;
	/** The {@link #VMXThread} object providing access to VMX-pi Threading functionality. */
	VMXThread	thread;

public:
	VMXPi(bool realtime, uint8_t ahrs_update_rate_hz);
	virtual ~VMXPi();

	static VMXPi *getInstance();

	bool IsOpen();
	/** Returns a reference to the singleton {@link #AHRS} object providing access to VMX-pi IMU functionality. */
	vmx::AHRS& getAHRS() {return ahrs;}
	/** Returns a reference to the singleton {@link #VMXTime} object providing access to VMX-pi Time functionality. */
	VMXTime& getTime() {return time;}
	/** Returns a reference to the singleton {@link #VMXIO} object providing access to VMX-pi IO functionality. */
	VMXIO& getIO() {return io;}
	/** Returns a reference to the singleton {@link #VMXCAN} object providing access to VMX-pi CAN functionality. */
	VMXCAN& getCAN() {return can;}
	/** Returns a reference to the singleton {@link #VMXPower} object providing access to VMX-pi Power functionality. */
	VMXPower& getPower() {return power;}
	/** Returns a reference to the singleton {@link #VMXVersion} object providing access to VMX-pi Version functionality. */
	VMXVersion& getVersion() {return version;}
	/** Returns a reference to the singleton {@link #VMXThread} object providing access to VMX-pi Threading functionality. */
	VMXThread& getThread() {return thread;}
	/** Registers a termination handler, invoked when VMXPi instance is terminated, but before resources are released. */
	/** Termination functions are invoked in the reverse order of registration. */
	bool registerShutdownHandler(void (*termination_func)(void));
	/** Registers a termination handler, invoked when VMXPi instance is terminated, but only after all resources are released. */
	/** Only a single final shutdown handler may be registered. */
	bool registerFinalShutdownHandler(void (*termination_func)(void));
	/** Selective Enabling/Disabling of Summary Performance statistics. */
	bool getPerformanceLogging();
	void setPerformanceLogging(bool enable);
};

#endif /* VMXPI_H_ */
