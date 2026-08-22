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

#ifndef VMXCHANNEL_H_
#define VMXCHANNEL_H_

/** @file VMXChannel.h */

#include <stdint.h>

/* Each VMXChannel has zero or more of the following Capabilities */
/* Some of these Capabilities are dynamic, and may be changed via
 * jumper (e.g., VMX PWM/DigInput jumper).  However, their state at the
 * beginning of the application will not change during the lifetime of the
 * application.
 *
 * When VMXChannel Capabilities refer to Shared Resources, there is no
 * guarantee that at any instant a VMXChannel can be configured with this,
 * as VMXChannels must be routed to resources w/sufficient availability
 * in order for the VMXChannel capability to be active.
 */

/** Enumeration of VMX Channel types */
typedef enum {
	INVALID = 0,
	/** FlexDIO Channels can be configured as input or output, and can also be
	 * used with VMX-pi Encoder, PWMGenerator, PWMCapture and Interrupt resources.
	 */
	FlexDIO  = 1,
	/** AnalogIn Channels are dedicated as inputs to AnalogAccumulator and AnalogTrigger resources. */
	AnalogIn = 2,
	/** HiCurrDIO Channels can all be configured as either input or output.  In input mode,
	 * they can be used with DigitalInput and Interrupt resources; in output mode, they can be used
	 * with DigitalOutput and PWMGenerator resources.
	 */
	HiCurrDIO = 3,
	/** CommDIO Channels are fixed as either inputs or outputs; they can be used with the UART, SPI or
	 * I2C resources; Output CommDIO Channels can be used with PWMGenerator Resources, and input
	 * CommDIO Chnnels can be used with DigitalInput or Interrupt resources.
	 */
	CommDIO = 4
} VMXChannelType;

/** Enumeration of VMX Channel Capabilities */
typedef enum {
	NoCapabilities		= 0x00000000,
	/** The VMX Channel can be routed to a DigitalIO resource in input mode */
	DigitalInput		= 0x00000001,
	/** The VMX Channel can be routed to a DigitalIO resource in output mode */
	DigitalOutput		= 0x00000002,
	/** The VMX Channel can be routed to a PWM Generator resource's first port */
	PWMGeneratorOutput	= 0x00000004,
	/** The VMX Channel can be routed to a PWM Generator resource's second port */
	PWMGeneratorOutput2 	= 0x00000008,
	/** The VMX Channel can be routed to a Input Capture resource's first port */
	/** NOTE:  This channel capability is deprecated, and replaced by the InputCaptureInput resource. */
	PWMCaptureInput		= 0x00000010,
	InputCaptureInput   = PWMCaptureInput,
	/** The VMX Channel can be routed to a Input Capture's second port */
	/** NOTE:  This channel capability is deprecated, and replaced by the InputCaptureInput2 resource. */
	PWMCaptureInput2	= 0x00000020,
	InputCaptureInput2 = PWMCaptureInput2,
	/** The VMX Channel can be routed to an Encoder resource's A (first) port */
	EncoderAInput		= 0x00000040,
	/** The VMX Channel can be routed to an Encoder resource's B (second) port */
	EncoderBInput		= 0x00000080,
	/** The VMX Channel can be routed to an Accumulator resource */
	AccumulatorInput	= 0x00000100,
	/** The VMX Channel can be routed to an Analog Trigger resource */
	AnalogTriggerInput 	= 0x00000200,
	/** The VMX Channel can be routed to an Interrupt resource */
	InterruptInput		= 0x00000400,
	/** The VMX Channel can be routed to a UART resource's TX port */
	UART_TX			= 0x00000800,
	/** The VMX Channel can be routed to a UART resource's RX port */
	UART_RX			= 0x00001000,
	/** The VMX Channel can be routed to a SPI resource's CLK port */
	SPI_CLK			= 0x00002000,
	/** The VMX Channel can be routed to a SPI resource's MISO port */
	SPI_MISO		= 0x00004000,
	/** The VMX Channel can be routed to a SPI resource's MOSI port */
	SPI_MOSI		= 0x00008000,
	/** The VMX Channel can be routed to a SPI resource's CS port */
	SPI_CS			= 0x00010000,
	/** The VMX Channel can be routed to a I2C resource's SDA port */
	I2C_SDA			= 0x00020000,
	/** The VMX Channel can be routed to a I2C resource's SCL port */
	I2C_SCL			= 0x00040000,
	/** The VMX Channel can be routed to a one-wire LED array (e.g., WS2811) that
            is accessed via "one-wire" protocol */
	LEDArray_OneWire	= 0x00080000,
} VMXChannelCapability;

/** Type representing the 0-based index of a VMX Channel */
typedef uint8_t  VMXChannelIndex;

/** Constant representing an invalid VMXChannelIndex */
const VMXChannelIndex INVALID_VMX_CHANNEL_INDEX = 255;

/** VMXChannel information structure */
struct VMXChannelInfo {
	VMXChannelIndex index;
	VMXChannelCapability capabilities;

	VMXChannelInfo() {
		index = INVALID_VMX_CHANNEL_INDEX;
		capabilities = VMXChannelCapability::NoCapabilities;
	}	
	VMXChannelInfo(VMXChannelIndex i, VMXChannelCapability c) {
		index = i;
		capabilities = c;
	}
	bool IsValid() {
		return (index != INVALID_VMX_CHANNEL_INDEX);
	}
};

#endif /* VMXCHANNEL_H_ */
