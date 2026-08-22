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

#ifndef VMXRESOURCE_H_
#define VMXRESOURCE_H_

/** @file VMXResource.h */

#include <stdint.h>

#include "VMXChannel.h"
#include "VMXErrors.h"

/** Enumerates the various types of VMXResources */
typedef enum {
	Undefined,
	/** VMX Resource providing Digital Input and Digital Output Control on a VMX Digital Channel */
	DigitalIO,
	/** VMX Resource providing PWM Generation Control of a VMX Digital Channel in output mode */
	PWMGenerator,
	/** VMX Resource providing Timer Input Capture of a VMX FlexIO Digital Channel in input mode */
	InputCapture,
	/** VMX Resource providing PWM Capture of a VMX FlexIO Digital Channel in input mode */
	/** NOTE:  This resource type is deprecated, and replaced by the InputCapture resource. */
	PWMCapture = InputCapture,
	/** VMX Resource providing Quadrature Encoder of a VMX Channel pair in input mode */
	Encoder,
	/** VMX Resource providing Oversampling/Averaging/Accumulation of a VMX Analog Input Channel */
	Accumulator,
	/** VMX Resource providing Interrupt generation from a VMX Analog Input Channel */
	AnalogTrigger,
	/** VMX Resource providing Interrupt generation from a VMX Digital Channel in input mode */
	Interrupt,
	/** VMX Resource providing UART communication via a VMX Channel pair */
	UART,
	/** VMX Resource providing SPI communication via a four VMX Channel set */
	SPI,
	/** VMX Resource providing I2C communication via a VMX Channel pair */
	I2C,
	/** VMX Resource providing One-wire LEDArray management via a VMX Channel */
	LEDArrayDriver_OneWire,
	MaxVMXResourceType = LEDArrayDriver_OneWire,
} VMXResourceType;

/** Zero-based index of a particular VMX Resource, relative to a specific VMXResourceType */
typedef uint8_t  VMXResourceIndex;
/** Handle to a particular VMX Resource */
typedef uint16_t VMXResourceHandle;
/** Zero-based index of a particular VMX Resource Port of a VMX Resource */
typedef uint8_t  VMXResourcePortIndex;

/** Value indicating a VMXResourceIndex is invalid */
const VMXResourceIndex INVALID_VMX_RESOURCE_INDEX = 255;

/** Macro that returns true if the provide VMX Resource Handle is invalid */
#define INVALID_VMX_RESOURCE_HANDLE(vmx_res_handle)     (((uint8_t)vmx_res_handle)==INVALID_VMX_RESOURCE_INDEX)
/** Macro that creates a VMX Resource Handle from a VMXResourceType and VMXResourceIndex */
#define CREATE_VMX_RESOURCE_HANDLE(res_type,res_index) 	((((uint16_t)res_type)<<8) | (uint8_t)res_index)
/** Macro that extracts a VMXResourceType from VMXResourceHandle */
#define EXTRACT_VMX_RESOURCE_TYPE(res_handle)			(VMXResourceType)(res_handle >> 8)
/** Macro that extracts a VMXResourcIndex from a VMXResourceHandle */
#define EXTRACT_VMX_RESOURCE_INDEX(res_handle)			(uint8_t)(res_handle & 0x00FF)

#endif /* VMXRESOURCE_H_ */
