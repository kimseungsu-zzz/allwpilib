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

#ifndef VMXIO_H_
#define VMXIO_H_

#include "VMXResourceConfig.h"
#include "VMXErrors.h"
#include "VMXTime.h"
#include <unordered_set>
#include <list>

class PIGPIOClient;
class IOCXClient;
class MISCClient;
class VMXChannelManager;
class VMXResourceManager;

class VMXIO_PIGPIOInterruptSink;
class VMXIO_PulseManager;
class VMXIO_CommAutoTransactionManager;

typedef uint16_t AutoTransmitEngineHandle;
const AutoTransmitEngineHandle INVALID_AUTO_TRANSMIT_ENGINE_HANDLE = 65535;
typedef void *LEDArrayBufferHandle;

/** The VMXIO class provides access to VMX Analog/Digital IO functions, including
 * VMX Channel and VMX Resource Management and functions. */
class VMXIO {

	friend class VMXPi;

	PIGPIOClient& 		pigpio;
	IOCXClient&   		iocx;
	MISCClient&   		misc;
	VMXChannelManager& 	chan_mgr;
	VMXResourceManager& res_mgr;
	VMXTime&			time;
	VMXIO_PIGPIOInterruptSink *p_int_sink;
	VMXIO_PulseManager *p_pulse_manager;
	VMXIO_CommAutoTransactionManager *p_auto_transaction_mgr;

	void Init();

	bool DisconnectAnalogTriggerInterrupt(uint8_t analog_trigger_num); /* ????? */
	bool GetResourcesCompatibleWithChannelAndCapability(VMXChannelIndex channel_index, VMXChannelCapability capability, std::list<VMXResourceHandle>& compatible_res_handles);
	bool GetUnallocatedResourcesCompatibleWithChannelAndCapability(VMXChannelIndex channel_index, VMXChannelCapability capability, std::list<VMXResourceHandle>& unallocated_compatible_res_handles);
	bool GetResourceDefaultConfig(VMXResourceHandle resource, VMXResourceConfig*& p_config, VMXErrorCode *errcode);
	bool DeactivatePIGPIOChannels(VMXResourceHandle resource);
	bool InternalZeroAllPWMGenerators(uint32_t& longest_pwm_period_ms);

	VMXIO(PIGPIOClient& pigpio, IOCXClient& iocx, MISCClient& misc, VMXChannelManager& chan_mgr, VMXResourceManager& res_mgr, VMXTime& time_ref);
	void ReleaseResources();
	virtual ~VMXIO();

public:

	/*** RESOURCE AND CHANNEL ENUMERATION */
	uint8_t GetNumResourcesByType(VMXResourceType resource_type);
	uint8_t GetNumChannelsByCapability(VMXChannelCapability channel_capability);
	uint8_t GetNumChannelsByType(VMXChannelType channel_type, VMXChannelIndex& first_channel_index);
	bool GetChannelCapabilities(VMXChannelIndex channel_index, VMXChannelType& channel_type, VMXChannelCapability& capability_bits);
	bool ChannelSupportsCapability(VMXChannelIndex channel_index, VMXChannelCapability capability);
	VMXChannelIndex GetSoleChannelIndex(VMXChannelCapability capability);

	/*** RESOURCE HANDLE ACQUISITION */
	bool GetResourceHandle(VMXResourceType resource_type, VMXResourceIndex res_index, VMXResourceHandle& resource_handle, VMXErrorCode *errcode);
	bool GetChannelsCompatibleWithResource(VMXResourceHandle resource_handle, VMXChannelIndex& first_channel_index, uint8_t& num_channels);
	bool GetResourceHandleWithAvailablePortForChannel(VMXResourceType resource_type, VMXChannelIndex channel_index, VMXChannelCapability capability, VMXResourceHandle& resource_handle, bool& allocated, VMXErrorCode *errcode);
	bool GetResourceFromRoutedChannel(VMXChannelIndex channel_index, VMXResourceHandle& handle_out, VMXErrorCode *errcode);

	/*** RESOURCE ALLOCATION ***/

	bool IsResourceAllocated(VMXResourceHandle resource, bool& allocated, bool& is_shared, VMXErrorCode *errcode);
	bool AllocateResource(VMXResourceHandle resource, VMXErrorCode* errcode);
	bool DeallocateResource(VMXResourceHandle resource, VMXErrorCode *errcode);
	bool DeallocateAllResources(VMXErrorCode *last_errorcode);

	/*** RESOURCE-CHANNEL ROUTING ***/

	bool RouteChannelToResource(VMXChannelIndex channel, VMXResourceHandle resource, VMXErrorCode* errcode);
	bool UnrouteChannelFromResource(VMXChannelIndex channel, VMXResourceHandle resource, VMXErrorCode *errcode);
	bool UnrouteAllChannelsFromResource(VMXResourceHandle resource, VMXErrorCode *errcode);
	bool GetNumChannelsRoutedToResource(VMXResourceHandle resource, uint8_t& num_routed_channels, VMXErrorCode *errcode);

	/*** RESOURCE CONFIGURATION (see VMXResourceConfig.h for various configuration classes) ***/

	bool SetResourceConfig(VMXResourceHandle resource, const VMXResourceConfig* p_config, VMXErrorCode *errcode);
	bool GetResourceConfig(VMXResourceHandle resource, VMXResourceConfig*& p_config, VMXErrorCode *errcode);

	/*** RESOURCE ACTIVATION ***/

	bool IsResourceActive(VMXResourceHandle, bool &active, VMXErrorCode* errcode);
	bool ActivateResource(VMXResourceHandle resource, VMXErrorCode* errcode);
	bool DeactivateResource(VMXResourceHandle resource, VMXErrorCode* errcode);

	/*** ACTIVATION HELPERS ***/
	bool ActivateSinglechannelResource(const VMXChannelInfo& channel_info, const VMXResourceConfig *res_cfg, 
			VMXResourceHandle& res_handle, VMXErrorCode *errcode);
	bool ActivateDualchannelResource(const VMXChannelInfo& ch1, const VMXChannelInfo& ch2,
			const VMXResourceConfig *res_cfg, VMXResourceHandle& res_handle, VMXErrorCode *errcode);
	bool ActivateQuadchannelResource(const VMXChannelInfo& ch1, const VMXChannelInfo& ch2,
			const VMXChannelInfo& ch3, const VMXChannelInfo& ch4,
			const VMXResourceConfig *res_cfg, VMXResourceHandle& res_handle, VMXErrorCode *errcode);
	bool ActivateMultichannelResource(uint8_t num_channels, const VMXChannelInfo *p_channel_infos, const VMXResourceConfig *res_cfg,
			VMXResourceHandle& res_handle, VMXErrorCode *errcode);

	/*** RESOURCE ACTIONS ***/

	/* DIO Resources */
	bool DIO_Get(VMXResourceHandle dio_res_handle, bool& high, VMXErrorCode *errcode);
	bool DIO_Set(VMXResourceHandle dio_res_handle, bool high, VMXErrorCode *errcode);
	bool DIO_Pulse(VMXResourceHandle dio_res_handle, bool high, uint32_t num_microseconds, VMXErrorCode *errcode);
	bool DIO_IsPulsing(VMXResourceHandle dio_res_handle, bool& is_pulsing, VMXErrorCode *errcode);
	bool DIO_GetNumPulsing(uint8_t& num_pulsing);

	/* PWMGenerator */
	bool PWMGenerator_SetDutyCycle(VMXResourceHandle pwmgen_res_handle, VMXResourcePortIndex port_index, uint16_t duty_cycle, VMXErrorCode *errcode);
	bool PWMGenerator_GetDutyCycle(VMXResourceHandle pwmgen_res_handle, VMXResourcePortIndex port_index, uint16_t *duty_cycle, VMXErrorCode *errcode);

	/* InputCapture */
	/* Returns current Input Capture Channel raw counts for the specified specified timer resource. */
	/* Note:  The channel count units depend upon the InputCapture Clock Source selection. */
	bool InputCapture_GetChannelCounts(VMXResourceHandle pwmcap_res_handle, uint32_t& chan1_counts, uint32_t& chan2_counts, VMXErrorCode *errcode);
	bool InputCapture_InputStatus(VMXResourceHandle inputcap_res_handle, bool& forward_direction, bool& active, VMXErrorCode *errcode);
	bool InputCapture_Reset(VMXResourceHandle inputcap_res_handle, VMXErrorCode *errcode);
	bool InputCapture_GetCount(VMXResourceHandle inputcap_res_handle, int32_t& count, VMXErrorCode *errcode);

	/* PWMCapture */
	/* Returns current PWM (or InputCapture) timing of ticks at current frequency/duty cycle for the specified timer resource. */
	bool PWMCapture_GetCount(VMXResourceHandle inputcap_res_handle, uint32_t& frequency_us, uint32_t& duty_cycle_us, VMXErrorCode *errcode);

	/* Encoder */
	/* Returns current integrated count of encoder ticks (at the current resolution) */
	bool Encoder_GetCount(VMXResourceHandle encoder_res_handle, int32_t& count, VMXErrorCode *errcode);
	/** Enumeration of Encoder Directions */
	typedef enum { EncoderForward, EncoderReverse } EncoderDirection;
	bool Encoder_GetDirection(VMXResourceHandle encoder_res_handle, EncoderDirection& direction, VMXErrorCode *errcode);
	bool Encoder_Reset(VMXResourceHandle encoder_res_handle, VMXErrorCode *errcode);
	bool Encoder_GetLastPulsePeriodMicroseconds(VMXResourceHandle encoder_res_handle, uint16_t& encoder_curr_avg_pulse_period_microseconds, VMXErrorCode *errcode);
    bool Encoder_SetResetSource(VMXResourceHandle encoder_res_handle, VMXResourceHandle interrupt_res_handle, bool clear_on_level, bool clear_level_high, VMXErrorCode *errcode);
    bool Encoder_ClearResetSource(VMXResourceHandle encoder_res_handle, VMXErrorCode *errcode);

	/* Accumulator */
	/* NOTE:  The resolution of Accumulator values is dependent upon the current number of bits */
	/* 0 bits:  12-bit resolution, 1 bit:  13-bit resolution, etc. */
	/* See the AccumulatorConfig for more information on modifying these bits */

	bool Accumulator_GetOversampleValue(VMXResourceHandle accum_res_handle, uint32_t& oversample_value, VMXErrorCode *errcode);
	bool Accumulator_GetAverageValue(VMXResourceHandle accum_res_handle, uint32_t& average_value, VMXErrorCode *errcode);
	bool Accumulator_GetInstantaneousValue(VMXResourceHandle accum_res_handle, uint32_t& average_value, VMXErrorCode *errcode);
	bool Accumulator_GetFullScaleVoltage(float& full_scale_voltage, VMXErrorCode *errcode);
	bool Accumulator_GetAverageVoltage(VMXResourceHandle accum_res_handle, float& average_value, VMXErrorCode *errcode);

	/* Accumulator Counter */
	/* NOTE:  The number of accumulator counters may be less than the number of accumulators.  Therefore, it is required
	 * to assign an accumulator counter to an accumulator in order to use it.  See AccumulatorConfig().
	 */
	bool Accumulator_Counter_Reset(VMXResourceHandle accum_res_handle, VMXErrorCode *errcode);
	bool Accumulator_Counter_GetValueAndCount(VMXResourceHandle accum_res_handle, int64_t& value, uint32_t& count, VMXErrorCode *errcode);

	/* Analog Trigger */
	/** Enumeration of Analog Trigger States */
	typedef enum { BelowThreshold, AboveThreshold, InWindow } AnalogTriggerState;
	bool AnalogTrigger_GetState(VMXResourceHandle antrig_res_handle, AnalogTriggerState& state, VMXErrorCode *errcode);

	/* Interrupt */
	bool Interrupt_GetLastRisingEdgeTimestampMicroseconds(VMXResourceHandle int_res_handle, uint64_t& last_timestamp, VMXErrorCode *errcode);
	bool Interrupt_GetLastFallingEdgeTimestampMicroseconds(VMXResourceHandle int_res_handle, uint64_t& last_timestamp, VMXErrorCode *errcode);
	bool Interrupt_SetEnabled(VMXResourceHandle int_res_handle, bool enabled, VMXErrorCode *errcode);
	bool Interrupt_GetEnabled(VMXResourceHandle int_res_handle, bool& enabled, VMXErrorCode *errcode);

	/* UART */
	bool UART_Write(VMXResourceHandle uart_res_handle, uint8_t *p_send_data, uint16_t size, VMXErrorCode *errcode);
	bool UART_Read(VMXResourceHandle uart_res_handle, uint8_t *p_rcv_data, uint16_t max_size, uint16_t& actual_size_read, VMXErrorCode *errcode);
	bool UART_GetBytesAvailable(VMXResourceHandle uart_res_handle, uint16_t& size, VMXErrorCode *errcode);

	/* SPI */
	bool SPI_Write(VMXResourceHandle spi_res_handle, uint8_t *p_send_data, uint16_t size, VMXErrorCode *errcode);
	bool SPI_Read(VMXResourceHandle spi_res_handle, uint8_t *p_rcv_data, uint16_t size, VMXErrorCode *errcode);
	bool SPI_Transaction(VMXResourceHandle spi_res_handle, uint8_t *p_send_data, uint8_t *p_rcv_data, uint16_t size, VMXErrorCode *errcode);

	/* I2C */
	bool I2C_Write(VMXResourceHandle i2c_res_handle, uint8_t deviceAddress, uint8_t register_address, uint8_t* p_send_data, int32_t sendSize, VMXErrorCode *errcode);
	bool I2C_Read(VMXResourceHandle i2c_res_handle, uint8_t deviceAddress, uint8_t register_address, uint8_t* p_rcv_data, int32_t count, VMXErrorCode *errcode);
	bool I2C_Transaction(VMXResourceHandle i2c_res_handle, uint8_t deviceAddress,
	                    uint8_t* p_send_data, uint16_t sendSize,
	                    uint8_t* p_rcv_data, uint16_t receiveSize, VMXErrorCode *errcode);

	/* SPI Auto-retransmit engine */

	bool AutoTransmit_Allocate(AutoTransmitEngineHandle& engine_handle_out, VMXErrorCode *errcode);
	bool AutoTransmit_Deallocate(AutoTransmitEngineHandle engine_handle, VMXErrorCode *errcode);
	bool AutoTransmit_StartPeriodic(AutoTransmitEngineHandle engine_handle, VMXResourceHandle spi_res_handle, uint32_t repeat_every_ms, VMXErrorCode *errcode);
	bool AutoTransmit_StartTrigger(AutoTransmitEngineHandle engine_handle, VMXResourceHandle spi_res_handle, VMXChannelIndex input_trigger_channel, InterruptConfig::InterruptEdge edge_type, VMXErrorCode *errcode);
	// Stop the transfer engine previously started
	bool AutoTransmit_Stop(AutoTransmitEngineHandle engine_handle, VMXErrorCode *errcode);
	bool AutoTransmit_SetData(AutoTransmitEngineHandle engine_handle, uint8_t* dataToSend, int32_t request_size, int32_t reply_size, VMXErrorCode *errcode);
	// Forces the engine to make a transfer.
	bool AutoTransmit_Immediate(AutoTransmitEngineHandle engine_handle, VMXErrorCode *errcode);
	// Blocks until numToRead bytes have been read or timeout expires.
	// May be called with numToRead=0 to retrieve how many bytes are available.
	bool AutoTransmit_GetData(AutoTransmitEngineHandle engine_handle, uint8_t *p_rcv_data, int32_t numToRead, uint32_t timeout_ms, int32_t& num_bytes_remaining, VMXErrorCode *errcode);
	// Get the number of bytes dropped by the automatic SPI transfer engine due to the receive buffer being full.
	bool AutoTransmit_GetNumDropped(AutoTransmitEngineHandle engine_handle, int& num_dropped, VMXErrorCode *errcode);

	/* IO Watchdog */
	bool GetWatchdogEnabled(bool& enabled, VMXErrorCode *errcode);
	bool SetWatchdogEnabled(bool enabled, VMXErrorCode *errcode);
	bool GetWatchdogExpired(bool& expired, VMXErrorCode *errcode);
	bool GetWatchdogManagedOutputs(bool& flexdio, bool& hicurrdio, bool& commdio, VMXErrorCode *errcode);
	bool SetWatchdogManagedOutputs(bool flexdio, bool hicurrdio, bool commdio, VMXErrorCode *errcode);
	bool GetWatchdogTimeoutPeriodMS(uint16_t& timeout_period_ms, VMXErrorCode *errcode);
	bool SetWatchdogTimeoutPeriodMS(uint16_t timeout_period_ms, VMXErrorCode *errcode);
	bool FeedWatchdog(VMXErrorCode *errcode);
	bool ExpireWatchdogNow(VMXErrorCode *errcode);

	/* LED Array Buffer */
	bool LEDArrayBuffer_Create(int n_pixels, LEDArrayBufferHandle& buffer_handle_out, VMXErrorCode *errcode);
	bool LEDArrayBuffer_Delete(LEDArrayBufferHandle buffer_handle, VMXErrorCode *errorcode);
	bool LEDArrayBuffer_SetRGBValue(LEDArrayBufferHandle buffer_handle, int index, int r, int g, int b, VMXErrorCode *errocode);
	bool LEDArrayBuffer_GetLength(LEDArrayBufferHandle buffer_handle, int& length_out, VMXErrorCode *errcode);
	bool LEDArrayBuffer_GetRBGValue(LEDArrayBufferHandle buffer_handle, int index, int& r, int& g, int& b, VMXErrorCode *errcode);	
	
	/* LED Array */
	bool LEDArray_SetBuffer(VMXResourceHandle led_array_resource_handle, LEDArrayBufferHandle buffer_handle, VMXErrorCode *errcode);
	bool LEDArray_Render(VMXResourceHandle led_array_resource_handle, VMXErrorCode *errcode);
	bool LEDArray_Configure(VMXResourceHandle led_array_resource_handle, LEDArray_OneWireConfig& config, VMXErrorCode *errcode);
};

#endif /* VMXIO_H_ */
