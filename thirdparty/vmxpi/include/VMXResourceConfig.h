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

#ifndef VMXRESOURCECONFIG_H_
#define VMXRESOURCECONFIG_H_

#include "VMXHandlers.h"
#include "VMXPiConstants.h"
#include "VMXResource.h"

#include <cstddef> // size_t
#include <cstring> // memset

/** Base structure representing VMXResourceType-specific configuration data that must be
 * set to a valid default before activating a resource of that type.  Note that once
 * the VMXResource is activated, the VMXResource must first be successfully deactivated
 * before that VMXResource's configuration data can be modified.
 */
struct VMXResourceConfig {
    VMXResourceType res_type;
    VMXResourceConfig(VMXResourceType res_type)
    {
        this->res_type = res_type;
    }
    /** VMXResourceType which this configuration applies to */
    VMXResourceType GetResourceType() const { return res_type; }
    virtual size_t GetSize() const = 0;
    /** Instantiates a copy of the configuration data.  NOTE:  The caller is responsible
	 * to delete the object returned from this method.
	 */
    virtual VMXResourceConfig* GetCopy() const = 0;
    /** Copies the contents of the source VMXResourceConfig object into this object.
	 * NOTE:  The source object's VMXResourceType must match this object's VMXResourceType.
	 * @param[in] p_config The source VMXResourceConfig object from which to copy
	 * configuration data into this object.
	 */
    virtual bool Copy(const VMXResourceConfig* p_config) = 0;
    virtual ~VMXResourceConfig() {}
};

/** Contains the configuration data for a VMXResource whose
 * VMXResourceType is Interrupt
 */
struct InterruptConfig : public VMXResourceConfig {

    /** Specifies which signal edge will generate an interrupt */
    typedef enum { RISING,
        FALLING,
        BOTH } InterruptEdge;

    InterruptEdge edge;
    VMXIO_InterruptHandler p_handler;
    void* p_param;
    bool initially_enabled;

    /** InterruptConfig default constructor; sets all values to defaults. */
    InterruptConfig(bool initially_enabled = true)
        : VMXResourceConfig(VMXResourceType::Interrupt)
    {
        edge = InterruptEdge::RISING;
        p_handler = 0;
        p_param = 0;
        this->initially_enabled = initially_enabled;
    }
    /** InterruptConfig constructor; initializes values with the provided input parameters *
	 * @param edge The signal edge which will generate an interrupt
	 * @param p_handler The interrupt handler which will be invoked when an interrupt occurs
	 * @param p_param The parameter to pass to the interrupt handler when an interrupt occurs; may be null
	 */
    InterruptConfig(InterruptEdge edge, VMXIO_InterruptHandler p_handler, void* p_param, bool initially_enabled = true)
        : VMXResourceConfig(VMXResourceType::Interrupt)
    {
        this->edge = edge;
        this->p_handler = p_handler;
        this->p_param = p_param;
        this->initially_enabled = initially_enabled;
    }

    /** Retrieves the interrupt handler */
    VMXIO_InterruptHandler GetHandler() { return p_handler; }
    /** Retrieves the interrupt handler parameter */
    void* GetParam() { return p_param; }
    /** Retrieves the signal edge which generates an interrupt */
    InterruptEdge GetEdge() { return edge; }
    /** Retrieves whether the interrupt is enabled automatically when activated. **/
    bool GetInitiallyEnabled() { return initially_enabled; }

    /** Sets the interrupt handler */
    void SetHandler(VMXIO_InterruptHandler p_handler) { this->p_handler = p_handler; }
    /** Sets the interrupt handler parameter */
    void SetParam(void* p_param) { this->p_param = p_param; }
    /** Sets the signal edge which generates the interrupt */
    void SetEdge(InterruptEdge edge) { this->edge = edge; }
    /** Sets whether the interrupt is enabled automatically when activated. **/
    void SetInitiallyEnabled(bool initially_enabled) { this->initially_enabled = initially_enabled; }

    virtual size_t GetSize() const
    {
        return sizeof(*this);
    }
    virtual VMXResourceConfig* GetCopy() const
    {
        InterruptConfig* p_new = new InterruptConfig();
        *p_new = *this;
        return p_new;
    }
    virtual bool Copy(const VMXResourceConfig* p_config)
    {
        if (p_config->GetResourceType() != this->GetResourceType())
            return false;
        *this = *((InterruptConfig*)p_config);
        return true;
    }

    virtual ~InterruptConfig() {}
};

/** Contains the configuration data for a VMXResource whose
 * VMXResourceType is DigitalIO
 */
struct DIOConfig : public VMXResourceConfig {
    /** Specifies the electrical behavior of a DigitalIO in output mode */
    typedef enum { PUSHPULL,
        OPENDRAIN } OutputMode;
    /** Specifies the default signal state of a floating DigitalIO in input mode */
    typedef enum { PULLUP,
        PULLDOWN,
        NONE } InputMode;

    /* NOTE:  Certain DIOs may be input-only or output-only at the hardware level. */
    /* Therefore it's possible this configuration may fail if an incompatible state is requested. */
    bool input;
    OutputMode outputmode; /* Not all DIO (e.g., rpi) channels can support !pushpull (opendrain) */
    InputMode inputmode;

    /** DIOConfig default constructor; sets all values to defaults, which are INPUT mode, PULLUP */
    DIOConfig()
        : VMXResourceConfig(VMXResourceType::DigitalIO)
    {
        input = true;
        outputmode = OutputMode::PUSHPULL;
        inputmode = InputMode::PULLUP;
    }
    /** DIOConfig constructor; initializes values with the provided parameters *
	 * @param outputmode Specifies the default electrical behavior of a DigitalIO in output mode
	 */
    DIOConfig(OutputMode outputmode)
        : VMXResourceConfig(VMXResourceType::DigitalIO)
    {
        this->input = false;
        this->inputmode = InputMode::NONE;
        this->outputmode = outputmode;
    }

    /** returns true if the DigitalIO should be configured in inputmode;
	 * returns false if the DigitalIO should be configured as an output */
    bool GetInput() { return input; }
    /** Returns output mode; only valid if this DIOConfig represents a DigitalIO in output mode */
    OutputMode GetOutputMode() { return outputmode; }
    /** Returns input mode; only valid if this DIOConfig represents a DigitalIO in input mode */
    InputMode GetInputMode() { return inputmode; }

    /** Specifies whether this DIOConfig represents an input or output mode configuration
	 * @param input true if this DIOConfig represents a DigitalIO in input mode;
	 * false if this DIOConfig represents a DigitalIO in ouput mode
	 */
    void SetInput(bool input) { this->input = input; }
    /** Specifies the input mode */
    void SetInputMode(InputMode inputmode) { this->inputmode = inputmode; }
    /** Specifies the output mode */
    void SetOutputMode(OutputMode outputmode) { this->outputmode = outputmode; }

    virtual size_t GetSize() const
    {
        return sizeof(*this);
    }
    virtual VMXResourceConfig* GetCopy() const
    {
        DIOConfig* p_new = new DIOConfig();
        *p_new = *this;
        return p_new;
    }
    virtual bool Copy(const VMXResourceConfig* p_config)
    {
        if (p_config->GetResourceType() != this->GetResourceType())
            return false;
        *this = *((DIOConfig*)p_config);
        return true;
    }
    virtual ~DIOConfig() {}
};

/** Contains the configuration data for a VMXResource whose
 * VMXResourceType is PWMGenerator
 */
struct PWMGeneratorConfig : public VMXResourceConfig {

    /** Specifies the possible output frame filters */
    typedef enum { NONE,
        x2,
        x4 } FrameOutputFilter;

    uint32_t frequency_hz;
    FrameOutputFilter frameOutputFilter;
    uint16_t maxDutyCycleValue;

    /** PWMGeneratorConfig default constructor; sets all values to defaults. */
    PWMGeneratorConfig()
        : VMXResourceConfig(VMXResourceType::PWMGenerator)
    {
        frequency_hz = 200;
        frameOutputFilter = NONE;
        maxDutyCycleValue = 255;
    }
    /** PWMGeneratorConfig constructor; initializes values with the provided input parameters *
	 * @param frequency_hz The PWM Generator's frequency
	 */
    PWMGeneratorConfig(uint32_t frequency_hz)
        : VMXResourceConfig(VMXResourceType::PWMGenerator)
    {
        this->frequency_hz = frequency_hz;
        frameOutputFilter = NONE;
        maxDutyCycleValue = 255;
    }

    /** Returns the PWM Generators's frequency in Hz.  This frequency represents the number of full
	 * cycles per second.
	 */
    uint32_t GetFrequencyHz() { return frequency_hz; }

    /** Sets the PWM Generator's frequency in Hz. */
    void SetFrequencyHz(uint32_t frequency_hz) { this->frequency_hz = frequency_hz; }

    FrameOutputFilter GetFrameOutputFilter() { return frameOutputFilter; }
    void SetFrameOutputFilter(FrameOutputFilter filter)
    {
        frameOutputFilter = filter;
    }

    uint16_t GetMaxDutyCycleValue() { return maxDutyCycleValue; }
    void SetMaxDutyCycleValue(uint16_t value)
    {
        maxDutyCycleValue = value;
    }

    virtual size_t GetSize() const
    {
        return sizeof(*this);
    }
    virtual VMXResourceConfig* GetCopy() const
    {
        PWMGeneratorConfig* p_new = new PWMGeneratorConfig();
        *p_new = *this;
        return p_new;
    }
    virtual bool Copy(const VMXResourceConfig* p_config)
    {
        if (p_config->GetResourceType() != this->GetResourceType())
            return false;
        *this = *((PWMGeneratorConfig*)p_config);
        return true;
    }
    virtual ~PWMGeneratorConfig() {}
};

#define INPUT_CAPTURE_STALL_TIMEOUT_MAX 127
#define VMXPI_DEFAULT_INPUTCAP_US_PER_TICK 1
#define VMXPI_MIN_INPUTCAP_FILTER_VALUE 0
#define VMXPI_MAX_INPUTCAP_FILTER_VALUE 15
#define INPUT_CAPTURE_DEFAULT_CAP_CH_FILTER 5
#define INPUT_CAPTURE_DEFAULT_STALL_PERIOD 5
#define NANOSECONDS_PER_SECOND 1000000000

struct InputCaptureConfigBase : public VMXResourceConfig {
    typedef enum { CH1,
        CH2 } CaptureChannel;
    uint8_t capture_channel_filter[2];
    uint8_t stall_timeout_20ms_periods;

    InputCaptureConfigBase(VMXResourceType resource_type)
        : VMXResourceConfig(resource_type)
    {
        capture_channel_filter[0] = INPUT_CAPTURE_DEFAULT_CAP_CH_FILTER;
        capture_channel_filter[1] = INPUT_CAPTURE_DEFAULT_CAP_CH_FILTER;
        stall_timeout_20ms_periods = INPUT_CAPTURE_DEFAULT_STALL_PERIOD;
    }

    void SetStallTimeout20MsPeriods(uint8_t stall_timeout_20ms_periods)
    {
        if (stall_timeout_20ms_periods > INPUT_CAPTURE_STALL_TIMEOUT_MAX) {
            stall_timeout_20ms_periods = INPUT_CAPTURE_STALL_TIMEOUT_MAX;
        }
        this->stall_timeout_20ms_periods = stall_timeout_20ms_periods;
    }

    uint8_t GetStallTimeout20MsPeriods()
    {
        return stall_timeout_20ms_periods;
    }

    void SetCaptureChannelFilter(CaptureChannel capture_channel, uint8_t filter)
    {
        if (filter > VMXPI_MAX_INPUTCAP_FILTER_VALUE) {
            filter = VMXPI_MAX_INPUTCAP_FILTER_VALUE;
        }
        if (capture_channel == CH2) {
            this->capture_channel_filter[1] = filter;
        } else {
            this->capture_channel_filter[0] = filter;
        }
    }

    uint8_t GetCaptureChannelFilter(CaptureChannel capture_channel)
    {
        if (capture_channel == CH2) {
            return capture_channel_filter[1];
        } else {
            return capture_channel_filter[0];
        }
    }

    uint8_t GetClosestCaptureChannelFilter(uint32_t period_nanoseconds)
    {
        uint8_t filter_index;
        for (filter_index = VMXPI_MAX_INPUTCAP_FILTER_VALUE;
             filter_index > VMXPI_MIN_INPUTCAP_FILTER_VALUE;
             filter_index--) {
            uint32_t filter_nanoseconds = GetCaptureChannelFilterPeriodNanoseconds(filter_index);
            if (period_nanoseconds >= filter_nanoseconds) {
                return filter_index;
            }
        }
        return VMXPI_MIN_INPUTCAP_FILTER_VALUE;
    }

    uint8_t GetClosestCaptureCaptureFilterNumSamples(uint32_t num_samples)
    {
        uint8_t filter_index;
        for (filter_index = VMXPI_MAX_INPUTCAP_FILTER_VALUE;
             filter_index > VMXPI_MIN_INPUTCAP_FILTER_VALUE;
             filter_index--) {
            uint32_t filter_num_samples = GetCaptureChannelFilterNumSamples(filter_index);
            if (num_samples >= filter_num_samples) {
                return filter_index;
            }
        }
        return VMXPI_MIN_INPUTCAP_FILTER_VALUE;
    }

    uint32_t GetCaptureChannelFilterPeriodNanoseconds(uint8_t filter)
    {
        double nanoseconds_per_cycle = double(NANOSECONDS_PER_SECOND) / double(VMXPI_TIMER_CLOCK_FREQUENCY_HZ);
        return uint32_t(GetCaptureChannelFilterNumSamples(filter) * nanoseconds_per_cycle);
    }

    uint32_t GetCaptureChannelFilterNumSamples(uint8_t filter)
    {
        if (filter > VMXPI_MAX_INPUTCAP_FILTER_VALUE) {
            filter = VMXPI_MIN_INPUTCAP_FILTER_VALUE;
        }
        switch (filter) {
        default:
        case 0:
            return 1;
        case 1:
            return 2;
        case 2:
            return 4;
        case 3:
            return 8;
        case 4:
            return 12;
        case 5:
            return 16;
        case 6:
            return 24;
        case 7:
            return 32;
        case 8:
            return 48;
        case 9:
            return 64;
        case 10:
            return 80;
        case 11:
            return 96;
        case 12:
            return 128;
        case 13:
            return 160;
        case 14:
            return 192;
        case 15:
            return 256;
        }
    }
};

/** Contains the configuration data for a VMXResource whose
 * VMXResourceType is InputCapture
 */
struct InputCaptureConfig : public InputCaptureConfigBase {
    typedef enum { INTERNAL,
        EDGEDETECT_CH1,
        FILTERED_CH1,
        FILTERED_CH2 } CounterClockSource;
    typedef enum { DIRECTION_UP,
        DIRECTION_DN } CounterDirection;
    typedef enum { SLAVEMODE_DISABLED,
        SLAVEMODE_RESET,
        SLAVEMODE_GATED,
        SLAVEMODE_TRIGGER,
        SLAVEMODE_FILTERED_INPUT_TRIGGER } SlaveMode;
    /*** NOTE:  TRIGGER_DYNAMIC causes first routed-to resource port to be used as slave mode trigger source. ***/
    typedef enum { TRIGGER_EDGEDETECT_CH1,
        TRIGGER_FILTERED_CH1,
        TRIGGER_FILTERED_CH2,
        TRIGGER_DYNAMIC } SlaveModeTriggerSource;
    /*** NOTE:  CAPTURE_SIGNAL_DYNAMIC causes first routed-to resource port to be used as capture channel source. ***/
    typedef enum { CAPTURE_SIGNAL_A,
        CAPTURE_SIGNAL_B,
        CAPTURE_SIGNAL_DYNAMIC } CaptureChannelSource;
    typedef enum { ACTIVE_RISING,
        ACTIVE_FALLING,
        ACTIVE_BOTH } CaptureChannelActiveEdge;
    typedef enum { x1,
        x2,
        x4,
        x8 } CaptureChannelPrescaler;
    typedef enum { ACTION_NONE,
        ACTION_CLEAR_COUNTER } StallAction;
    typedef enum { VC_MODE_DISABLED,
        VC_MODE_DUTYCYCLE_ENCODER,
        VC_MODE_DUAL_INPUT_UPDOWN } VirtualCounterMode;
    typedef enum { VC_SOURCE_CH1,
        VC_SOURCE_CH2 } VirtualCounterSource;

    /** Specifies the Input Capture internal clock source resolution (number of microseconds per tick) */
    uint32_t microseconds_per_tick;
    CounterClockSource counter_clock_source;
    CounterDirection counter_direction;
    SlaveMode slave_mode;
    SlaveModeTriggerSource slave_mode_trigger_source;
    CaptureChannelSource capture_channel_source[2];
    CaptureChannelActiveEdge capture_channel_active_edge[2];
    CaptureChannelPrescaler capture_channel_prescaler[2];
    StallAction stall_action;
    VirtualCounterMode virtual_counter_mode;
    VirtualCounterSource virtual_counter_source;
    uint16_t virtual_counter_parameter1;
    uint16_t virtual_counter_parameter2;

    InputCaptureConfig()
        : InputCaptureConfigBase(VMXResourceType::InputCapture)
    {
        microseconds_per_tick = VMXPI_DEFAULT_INPUTCAP_US_PER_TICK;
        counter_clock_source = INTERNAL;
        counter_direction = DIRECTION_UP;
        slave_mode = SLAVEMODE_DISABLED;
        slave_mode_trigger_source = TRIGGER_FILTERED_CH1;
        capture_channel_source[0] = CAPTURE_SIGNAL_A;
        capture_channel_source[1] = CAPTURE_SIGNAL_B;
        capture_channel_active_edge[0] = ACTIVE_RISING;
        capture_channel_active_edge[1] = ACTIVE_FALLING;
        capture_channel_prescaler[0] = x1;
        capture_channel_prescaler[1] = x1;
        stall_action = ACTION_NONE;
        virtual_counter_mode = VC_MODE_DISABLED;
        virtual_counter_source = VC_SOURCE_CH2;
        virtual_counter_parameter1 = 0;
        virtual_counter_parameter2 = 0;
    }

    /** Returns the configured Input Capture (internal clock source) Microseconds per Tick Rate */
    /** NOTE:  This value is ignored if CounterClockSource is INTERNAL.                         */
    uint32_t GetMicrosecondsPerTick() { return microseconds_per_tick; }

    /** Sets the configured Input Capture (internal clock source) Microseconds per Tick Rate */
    /** NOTE:  This value is ignored if CounterClockSource is INTERNAL.                         */
    void SetMicrosecondsPerTick(uint32_t microseconds_per_tick) { this->microseconds_per_tick = microseconds_per_tick; }

    void SetCounterClockSource(CounterClockSource counter_clock_source) { this->counter_clock_source = counter_clock_source; }
    CounterClockSource GetCounterClockSource() { return counter_clock_source; }
    void SetCounterDirection(CounterDirection counter_direction) { this->counter_direction = counter_direction; }
    CounterDirection GetCounterDirection() { return counter_direction; }

    void SetSlaveMode(SlaveMode slave_mode) { this->slave_mode = slave_mode; }
    SlaveMode GetSlaveMode() { return slave_mode; }
    void SetSlaveModeTriggerSource(SlaveModeTriggerSource slave_mode_trigger_source) { this->slave_mode_trigger_source = slave_mode_trigger_source; }
    SlaveModeTriggerSource GetSlaveModeTriggerSource() { return slave_mode_trigger_source; }

    void SetCaptureChannelSource(CaptureChannel capture_channel, CaptureChannelSource source)
    {
        if (capture_channel == CH2) {
            this->capture_channel_source[1] = source;
        } else {
            this->capture_channel_source[0] = source;
        }
    }
    CaptureChannelSource GetCaptureChannelSource(CaptureChannel capture_channel)
    {
        if (capture_channel == CH2) {
            return capture_channel_source[1];
        } else {
            return capture_channel_source[0];
        }
    }
    void SetCaptureChannelActiveEdge(CaptureChannel capture_channel, CaptureChannelActiveEdge active_edge)
    {
        if (capture_channel == CH2) {
            this->capture_channel_active_edge[1] = active_edge;
        } else {
            this->capture_channel_active_edge[0] = active_edge;
        }
    }
    CaptureChannelActiveEdge GetCaptureChannelActiveEdge(CaptureChannel capture_channel)
    {
        if (capture_channel == CH2) {
            return capture_channel_active_edge[1];
        } else {
            return capture_channel_active_edge[0];
        }
    }
    void SetCaptureChannelPrescaler(CaptureChannel capture_channel, CaptureChannelPrescaler prescaler)
    {
        if (capture_channel == CH2) {
            this->capture_channel_prescaler[1] = prescaler;
        } else {
            this->capture_channel_prescaler[0] = prescaler;
        }
    }
    CaptureChannelPrescaler GetCaptureChannelPrescaler(CaptureChannel capture_channel)
    {
        if (capture_channel == CH2) {
            return capture_channel_prescaler[1];
        } else {
            return capture_channel_prescaler[0];
        }
    }
    void SetStallAction(StallAction stall_action) { this->stall_action = stall_action; }
    StallAction GetStallAction() { return stall_action; }

    // Virtual Counter Configuration

    void SetVirtualCounterMode(VirtualCounterMode vc_mode) { this->virtual_counter_mode = vc_mode; }
    VirtualCounterMode GetVirtualCounterMode() { return virtual_counter_mode; }

    void SetVirtualCounterSource(VirtualCounterSource vc_source) { this->virtual_counter_source = vc_source; }
    VirtualCounterSource GetVirtualCounterSource() { return virtual_counter_source; }

    void SetVirtualCounterParameter1(uint16_t value) { this->virtual_counter_parameter1 = value; }
    uint16_t GetVirtualCounterParameter1() { return virtual_counter_parameter1; }

    void SetVirtualCounterParameter2(uint16_t value) { this->virtual_counter_parameter2 = value; }
    uint16_t GetVirtualCounterParameter2() { return virtual_counter_parameter2; }

    void SetVirtualCounterDutyCycleEncoderLowTicks(uint16_t value) { SetVirtualCounterParameter1(value); }
    uint16_t GetVirtualCounterDutyCycleEncoderLowTicks() { return GetVirtualCounterParameter1(); }

    void SetVirtualCounterDutyCycleEncoderHighTicks(uint16_t value) { SetVirtualCounterParameter2(value); }
    uint16_t GetVirtualCounterDutyCycleEncoderHighTicks() { return GetVirtualCounterParameter2(); }

    virtual size_t GetSize() const
    {
        return sizeof(*this);
    }
    virtual VMXResourceConfig* GetCopy() const
    {
        InputCaptureConfig* p_new = new InputCaptureConfig();
        *p_new = *this;
        return p_new;
    }
    virtual bool Copy(const VMXResourceConfig* p_config)
    {
        if (p_config->GetResourceType() != this->GetResourceType())
            return false;
        *this = *((InputCaptureConfig*)p_config);
        return true;
    }
    virtual ~InputCaptureConfig() {}
};

/** Contains the configuration data for a VMXResource whose
 * VMXResourceType is PWMCaptureConfig
 */
struct PWMCaptureConfig : public InputCaptureConfig {
    /** PWM Capture Timeout **/
    /*  (If a timeout occurs, 0 values are returned for frequency and duration) */
    /*  (If no timeout occurs, the last valid frequency/duration values are returned) */
    /*  NONE:  No Timeout */
    /*  x1:    Times out if no PWM Capture occurs within (1 x microseconds_per_tick * VMXPI_MAX_PWM_PERIOD_TICKS microseconds) */
    /*  x2:    Times out if no PWM Capture occurs within (2 x microseconds_per_tick * VMXPI_MAX_PWM_PERIOD_TICKS microseconds) */
    /*  x3:    Times out if no PWM Cpature occurs within (3 x microseconds_per_tick * VMXPI_MAX_PWM_PERIOD_TICKS microseconds) */
    typedef enum { NONE,
        x1,
        x2,
        x3 } PWMCaptureTimeout;
    PWMCaptureTimeout capture_timeout;

    /** PWMCaptureConfig default constructor; sets all values to defaults */
    PWMCaptureConfig()
        : InputCaptureConfig()
    {
        InternalInit(VMXPI_DEFAULT_INPUTCAP_US_PER_TICK, PWMCaptureTimeout::x2);
    }
    /** PWMCaptureConfig constructor; initializes values with the provided input parameters *
	 * @param edge_type The PWM Capture CaptureEdge
	 */
    PWMCaptureConfig(uint32_t microseconds_per_tick, PWMCaptureTimeout capture_timeout)
        : InputCaptureConfig()
    {
        InternalInit(microseconds_per_tick, capture_timeout);
    }

    void InternalInit(uint32_t microseconds_per_tick, PWMCaptureTimeout capture_timeout)
    {
        this->res_type = VMXResourceType::PWMCapture;
        this->microseconds_per_tick = microseconds_per_tick;
        SetTimeout(capture_timeout);
        SetCounterClockSource(InputCaptureConfig::INTERNAL);
        SetCounterDirection(InputCaptureConfig::DIRECTION_UP);

        /* For PWM Capture, Slave Mode is set to reset; this clears the counter */
        SetSlaveMode(InputCaptureConfig::SLAVEMODE_RESET);

        /* NOTE:  CaptureChannelSource for PWMCapture is Dynamic,        */
        /* since it is dynamically selected during Resource Activation.  */
        SetCaptureChannelSource(InputCaptureConfig::CH1, InputCaptureConfig::CAPTURE_SIGNAL_DYNAMIC);
        SetCaptureChannelSource(InputCaptureConfig::CH2, InputCaptureConfig::CAPTURE_SIGNAL_DYNAMIC);

        /* For PWM Capture, Timer CH1 measures PWM Period */
        SetCaptureChannelActiveEdge(InputCaptureConfig::CH1, InputCaptureConfig::ACTIVE_RISING);
        /* For PWM Capture, Timer CH2 measures (low portion of the) PWM Duty Cycle */
        SetCaptureChannelActiveEdge(InputCaptureConfig::CH2, InputCaptureConfig::ACTIVE_FALLING);

        /* Each time a rising edge occurs on the configured Trigger Source.     */
        /* NOTE:  SlaveModeTrigger is set to dynamic here, */
        /* since it is dynamically selected during Resource Activation.         */
        SetSlaveModeTriggerSource(InputCaptureConfig::TRIGGER_DYNAMIC);

        /* PWM Capture counts all transitions on the input signal. */
        SetCaptureChannelPrescaler(InputCaptureConfig::CH1, InputCaptureConfig::x1);
        SetCaptureChannelPrescaler(InputCaptureConfig::CH2, InputCaptureConfig::x1);

        SetStallAction(InputCaptureConfig::ACTION_CLEAR_COUNTER);
    }

    /** Returns the configured PWMCaptureTimeout.  NOTE:  Deprecated; use InputCapture::GetStallTimeout20MsPeriods() instead. */
    PWMCaptureTimeout GetTimeout() { return capture_timeout; }

    /** Sets the configured PWMCaptureTimeout.   NOTE:  Deprecated; use InputCapture::SetStallTimeout20MsPeriods() instead. */
    void SetTimeout(PWMCaptureTimeout capture_timeout)
    {
        this->capture_timeout = capture_timeout;
        uint32_t pwm_period_ms = (microseconds_per_tick * VMXPI_MAX_PWM_PERIOD_TICKS) / 1000;
        /* Configure InputCapture base-class Stall timeout */
        switch (capture_timeout) {
        case PWMCaptureTimeout::NONE:
            SetStallTimeout20MsPeriods(0);
            break;
        case PWMCaptureTimeout::x1:
            SetStallTimeout20MsPeriods((1 * pwm_period_ms) / 20);
            break;
        case PWMCaptureTimeout::x2:
            SetStallTimeout20MsPeriods((2 * pwm_period_ms) / 20);
            break;
        case PWMCaptureTimeout::x3:
            SetStallTimeout20MsPeriods((3 * pwm_period_ms) / 20);
            break;
        }
    }

    virtual size_t GetSize() const
    {
        return sizeof(*this);
    }
    virtual VMXResourceConfig* GetCopy() const
    {
        PWMCaptureConfig* p_new = new PWMCaptureConfig();
        *p_new = *this;
        return p_new;
    }
    virtual bool Copy(const VMXResourceConfig* p_config)
    {
        if (p_config->GetResourceType() != this->GetResourceType())
            return false;
        *this = *((PWMCaptureConfig*)p_config);
        return true;
    }
    virtual ~PWMCaptureConfig() {}
};

/** Contains the configuration data for a VMXResource whose
 * VMXResourceType is Encoder
 */
struct EncoderConfig : public InputCaptureConfigBase {
    /** Specifies whether the encoder counter is invoked on every edge, every other edge, or every fourth edge */
    typedef enum { x1,
        x2,
        x4 } EncoderEdge;
    EncoderEdge edge_count;

    /** EncoderConfig default constructor; sets all values to defaults */
    EncoderConfig()
        : InputCaptureConfigBase(VMXResourceType::Encoder)
    {
        InternalInit(EncoderEdge::x4);
    }
    /** EncoderConfig constructor; initializes values with the provided input parameters *
	 * @param edge The EncoderEdge configuration to be used
	 */
    EncoderConfig(EncoderEdge edge)
        : InputCaptureConfigBase(VMXResourceType::Encoder)
    {
        InternalInit(edge);
    }

    void InternalInit(EncoderEdge edge)
    {
        edge_count = edge;
    }

    /** Returns the configured EncoderEdge */
    EncoderEdge GetEncoderEdge() { return edge_count; }

    /** Sets the configured EncoderEdge */
    void SetEncoderEdge(EncoderEdge edge) { this->edge_count = edge; }

    virtual size_t GetSize() const
    {
        return sizeof(*this);
    }
    virtual VMXResourceConfig* GetCopy() const
    {
        EncoderConfig* p_new = new EncoderConfig();
        *p_new = *this;
        return p_new;
    }
    virtual bool Copy(const VMXResourceConfig* p_config)
    {
        if (p_config->GetResourceType() != this->GetResourceType())
            return false;
        *this = *((EncoderConfig*)p_config);
        return true;
    }
    virtual ~EncoderConfig() {}
};

/** Contains the configuration data for a VMXResource whose
 * VMXResourceType is Accumulator
 */
struct AccumulatorConfig : public VMXResourceConfig {
    uint8_t num_average_bits;
    uint8_t num_oversample_bits;
    bool assign_to_accumulator;
    int16_t center; // 0 to disable
    int16_t deadband; // only active if center != 0

    /** AccumulatorConfig default constructor; sets all values to defaults */
    AccumulatorConfig()
        : VMXResourceConfig(VMXResourceType::Accumulator)
    {
        num_oversample_bits = 0;
        num_average_bits = 3;
        assign_to_accumulator = false;
        center = 0;
        deadband = 0;
    }
    /** AccumulatorConfig constructor; initializes values with the provided input parameters *
	 * @param num_oversample_bits The configured number of oversample bits
	 * @param num_average_bits The configured number of average bits
	 */
    AccumulatorConfig(uint8_t num_oversample_bits, uint8_t num_average_bits)
        : VMXResourceConfig(VMXResourceType::Accumulator)
    {
        this->num_oversample_bits = num_oversample_bits;
        this->num_average_bits = num_average_bits;
        assign_to_accumulator = false;
        center = 0;
        deadband = 0;
    }

    /** Returns the configured number of average bits.  Each averaged value is comprised
	 * of 2^num_average_bits samples.
	 */
    uint8_t GetNumAverageBits() { return num_average_bits; }
    /** Returns the configured number of oversample bits. */
    /* For more information on oversampling, see <a href="https://en.wikipedia.org/wiki/Oversampling">this Wikipedia article</a> */
    uint8_t GetNumOversampleBits() { return num_oversample_bits; }

    /** Sets the configured number of average bits. */
    void SetNumAverageBits(uint8_t num_average_bits) { this->num_average_bits = num_average_bits; }
    /** Sets the configured number of oversample bits. */
    void SetNumOversampleBits(uint8_t num_oversample_bits) { this->num_oversample_bits = num_oversample_bits; }

    /** Enables an accumulation counter on the accumulator resource. */
    void SetEnableAccumulationCounter(bool assign)
    {
        this->assign_to_accumulator = assign;
    }
    /** Indicates whether an accumulation counter is enabled on the accumulator resource. */
    bool GetEnableAccumulationCounter()
    {
        return assign_to_accumulator;
    }
    /** Sets the deadband on the enabled accmulation counter; if a non-zero center value is configured, and after that value is subtracted,
	 * the remaining value is within the deadband range, the raw value is set to 0 before being added to accumulator counter. */
    void SetAccumulationCounterDeadband(int16_t deadband)
    {
        this->deadband = deadband;
    }
    /** Gets the deadband on the enabled accmulation counter; if a non-zero center value is configured, and after that value is subtracted,
	 * the remaining value is within the deadband range, the raw value is set to 0 before being added to accumulator counter.
	 */
    int16_t GetAccumulationCounterDeadband()
    {
        return deadband;
    }
    /** Sets the center value on the enabled accmulation counter; if non-zero, this value is subracted from each sample added to the counter. */
    void SetAccumulationCounterCenter(int16_t center)
    {
        this->center = center;
    }
    /** Gets the center value on the enabled accmulation counter; if non-zero, this value is subracted from each sample added to the counter. */
    int16_t GetAccumulationCounterCenter()
    {
        return center;
    }

    virtual size_t GetSize() const
    {
        return sizeof(*this);
    }
    virtual VMXResourceConfig* GetCopy() const
    {
        AccumulatorConfig* p_new = new AccumulatorConfig();
        *p_new = *this;
        return p_new;
    }
    virtual bool Copy(const VMXResourceConfig* p_config)
    {
        if (p_config->GetResourceType() != this->GetResourceType())
            return false;
        *this = *((AccumulatorConfig*)p_config);
        return true;
    }
    virtual ~AccumulatorConfig() {}
};

/** Contains the configuration data for a VMXResource whose
 * VMXResourceType is AnalogTrigger
 */
struct AnalogTriggerConfig : public VMXResourceConfig {
    /** Specifies what conditions cause an AnalogTrigger event */
    typedef enum {
        /** Analog Trigger Events are level-triggered (occurring as long as the signal is high) */
        STATE,
        /** Analog Trigger Events occur when a low-to-high transition is detected */
        RISING_EDGE_PULSE,
        /** Analog Trigger Events occur when a high-to-low transition is detected */
        FALLING_EDGE_PULSE
    } AnalogTriggerMode;

    uint16_t threshold_high; /* 12-bit value (0-4095) */
    uint16_t threshold_low; /* 12-bit value (0-4095) */
    AnalogTriggerMode mode;

    /** Default constructor; sets all values to defaults */
    AnalogTriggerConfig()
        : VMXResourceConfig(VMXResourceType::AnalogTrigger)
    {
        threshold_high = 992; /* .8V on a 3.3V scale */
        threshold_low = 2482; /* 2V on a 3.3V scale */
        mode = AnalogTriggerMode::STATE;
    }
    /** AccumulatorConfig constructor; initializes values with the provided input parameters
	 * @param threshold_high The high threshold
	 * @param threshold_low The low threshold
	 * @param mode
	 */
    AnalogTriggerConfig(uint16_t threshold_high, uint16_t threshold_low, AnalogTriggerMode mode)
        : VMXResourceConfig(VMXResourceType::AnalogTrigger)
    {
        this->threshold_high = threshold_high;
        this->threshold_low = threshold_low;
        this->mode = mode;
    }

    /** Returns the high threshold, which is a 12-bit value (0-4095) representing the lowest-possible voltage of a high signal */
    uint16_t GetThresholdHigh() { return threshold_high; }
    /** Returns the low threshold, which is a 12-bit value (0-4095) representing the highest-possible voltage of a low signal */
    uint16_t GetThresholdLow() { return threshold_low; }
    /** Returns the configured AnalogTriggerMode */
    AnalogTriggerMode GetMode() { return mode; }

    /** Sets the configured high threshold */
    void SetThresholdHigh(uint16_t threshold_high) { this->threshold_high = threshold_high; }
    /** Sets the configured low threshold */
    void SetThresholdLow(uint16_t threshold_low) { this->threshold_low = threshold_low; }
    /** Sets the configured AnalogTriggerMode */
    void SetMode(AnalogTriggerMode mode) { this->mode = mode; }

    virtual size_t GetSize() const
    {
        return sizeof(*this);
    }
    virtual VMXResourceConfig* GetCopy() const
    {
        AnalogTriggerConfig* p_new = new AnalogTriggerConfig();
        *p_new = *this;
        return p_new;
    }
    virtual bool Copy(const VMXResourceConfig* p_config)
    {
        if (p_config->GetResourceType() != this->GetResourceType())
            return false;
        *this = *((AnalogTriggerConfig*)p_config);
        return true;
    }
    virtual ~AnalogTriggerConfig() {}
};

/** Contains the configuration data for a VMXResource whose
 * VMXResourceType is UART
 */
struct UARTConfig : public VMXResourceConfig {
    uint32_t baudrate_bps;

    /** UARTConfig default constructor; sets all values to defaults */
    UARTConfig()
        : VMXResourceConfig(VMXResourceType::UART)
    {
        baudrate_bps = 57600;
    }
    /** UARTConfig constructor; initializes values with the provided input parameters
	 * @param baudrate_bps The UART baudrate in bits/second
	 */
    UARTConfig(uint32_t baudrate_bps)
        : VMXResourceConfig(VMXResourceType::UART)
    {
        this->baudrate_bps = baudrate_bps;
    }

    /** Returns the configured UART baudrate */
    uint32_t GetBaudrate() { return baudrate_bps; }

    /** Sets the configured UART baudrate */
    void SetBaudrate(uint32_t baudrate_bps) { this->baudrate_bps = baudrate_bps; }

    virtual size_t GetSize() const
    {
        return sizeof(*this);
    }
    virtual VMXResourceConfig* GetCopy() const
    {
        UARTConfig* p_new = new UARTConfig();
        *p_new = *this;
        return p_new;
    }
    virtual bool Copy(const VMXResourceConfig* p_config)
    {
        if (p_config->GetResourceType() != this->GetResourceType())
            return false;
        *this = *((UARTConfig*)p_config);
        return true;
    }
    virtual ~UARTConfig() {}
};

/** Contains the configuration data for a VMXResource whose
 * VMXResourceType is SPI
 */
struct SPIConfig : public VMXResourceConfig {
    uint32_t bitrate_bps;
    uint8_t mode; /* Range:  0-3 */
    bool cs_active_low;
    bool msbfirst;

    /** SPIConfig default constructor; sets all values to defaults (1Mhz SPI Click, Mode 3, CS Active Low, MSBFirst */
    SPIConfig()
        : VMXResourceConfig(VMXResourceType::SPI)
    {
        bitrate_bps = 1000000;
        mode = 3;
        cs_active_low = true;
        msbfirst = true;
    }
    /** SPIConfig constructor; initializes bitrate, sets remaining value to defaults
	 * @param bitrate The SPI Clock Bitrate
	 */
    SPIConfig(uint32_t bitrate)
        : VMXResourceConfig(VMXResourceType::SPI)
    {
        bitrate_bps = bitrate;
        mode = 3;
        cs_active_low = true;
        msbfirst = true;
    }
    /** SPIConfig constructor; initializes values with the provided input parameters *
	 * @param bitrate The SPI Clock bitrate
	 * @param mode The SPI Mode (0-3)
	 * @param cs_active_low true if the SPI CS signal is active low; false if SPI CS signal is active high
	 * @param msbvirst true if the most significant Bit is transmitted first,
	 * false if the least-significant bit is transmitted first
	 */
    SPIConfig(uint32_t bitrate, uint8_t mode, bool cs_active_low, bool msbfirst)
        : VMXResourceConfig(VMXResourceType::SPI)
    {
        this->bitrate_bps = bitrate;
        this->mode = mode;
        this->cs_active_low = cs_active_low;
        this->msbfirst = msbfirst;
    }

    /** Returns the configured SPI bitrate */
    uint32_t GetBitrate() { return bitrate_bps; }
    /** Returns the configure SPI mode */
    uint8_t GetMode() { return mode; }
    /** Returns true if the SPI CS is active low; false if the SPI CS is active high */
    bool GetCSActiveLow() { return cs_active_low; }
    /** Returns true if the most-significant bit is transmitted first; false if the least-significant
	 * bit is transmitted first
	 */
    bool GetMSBFirst() { return msbfirst; }

    /** Sets the configured SPI bitrate */
    void SetBitrate(uint32_t bitrate) { this->bitrate_bps = bitrate; }
    /** Sets the configured SPI mode, which must be a value from 0-3.  Invalid mode input values
	 * are coerced to a value from 0-3.
	 */
    void SetMode(uint8_t mode) { this->mode = mode % 4; }
    /** Sets the SPI CS Active low if true, Active high if false */
    void SetCSActiveLow(bool cs_active_low) { this->cs_active_low = cs_active_low; }
    /** Sets the SPI bit transmit order to most-significant bit first if true; least-significant bit order is used if false */
    void SetMSBFirst(bool msbfirst) { this->msbfirst = msbfirst; }

    virtual size_t GetSize() const
    {
        return sizeof(*this);
    }
    virtual VMXResourceConfig* GetCopy() const
    {
        SPIConfig* p_new = new SPIConfig();
        *p_new = *this;
        return p_new;
    }
    virtual bool Copy(const VMXResourceConfig* p_config)
    {
        if (p_config->GetResourceType() != this->GetResourceType())
            return false;
        *this = *((SPIConfig*)p_config);
        return true;
    }
    virtual ~SPIConfig() {}
};

/** Contains the configuration data for a VMXResource whose
 * VMXResourceType is I2C
 */
struct I2CConfig : public VMXResourceConfig {
    /** Default constructor */
    I2CConfig()
        : VMXResourceConfig(VMXResourceType::I2C)
    {
    }

    virtual size_t GetSize() const
    {
        return sizeof(*this);
    }
    virtual VMXResourceConfig* GetCopy() const
    {
        I2CConfig* p_new = new I2CConfig();
        *p_new = *this;
        return p_new;
    }
    virtual bool Copy(const VMXResourceConfig* p_config)
    {
        if (p_config->GetResourceType() != this->GetResourceType())
            return false;
        *this = *((I2CConfig*)p_config);
        return true;
    }
    virtual ~I2CConfig() {}
};

#define DEFAULT_LED_ARRAY_RESET_WAIT_TIME_US 80
#define DEFAULT_LED_ARRAY_ONE_SYMBOL_HIGH_TIME_NS 600
#define DEFAULT_LED_ARRAY_ZERO_SYMBOL_HIGH_TIME_NS 300

/** Contains the configuration data for a VMXResource whose
 * VMXResourceType is LEDArray_OneWire
 */
struct LEDArray_OneWireConfig : public VMXResourceConfig {
    int n_pixels;
    int target_freq_hz;
    uint64_t reset_wait_time_us;
    uint64_t one_symbol_high_time_ns;
    uint64_t zero_symbol_high_time_ns;
    /** Specifies the LED Array "pixel" value format */
    typedef enum {
        RGB, // 3-color (order: Red, Green, Blue) - used in WS2811
        RBG, // 3-color (order: Red, Blue, Green)
        GRB, // 3-color (order: Green, Red, Blue) - used in SK6812, WS2812, WS2812B
        GBR, // 3-color (order: Green, Blue, Red)
        BRG, // 3-color (order: Blue, Red, Green)
        BGR, // 3-color (order: Blue, Green, Red)
        RGBW, // 4-color (order: Red, Green, Blue, White)
        RBGW, // 4-color (order: Red, Blue, Green, White)
        GRBW, // 4-color (order: Green, Red, Blue, White) - used in SK6812W
        GBRW, // 4-color (order: Green, Blue, Red, White)
        BRGW, // 4-color (order: Blue, Red, Green, White)
        BGRW // 4-color (order: Blue, Green, Red, White)
    } PixelFormat;
    PixelFormat pixel_format;

    LEDArray_OneWireConfig(int n_pixels, int target_freq_hz)
        : VMXResourceConfig(VMXResourceType::LEDArrayDriver_OneWire)
    {
        this->n_pixels = n_pixels;
        this->target_freq_hz = target_freq_hz;
        reset_wait_time_us = DEFAULT_LED_ARRAY_RESET_WAIT_TIME_US;
        one_symbol_high_time_ns = DEFAULT_LED_ARRAY_ONE_SYMBOL_HIGH_TIME_NS;
        zero_symbol_high_time_ns = DEFAULT_LED_ARRAY_ZERO_SYMBOL_HIGH_TIME_NS;
        pixel_format = PixelFormat::GRB;
    }
    LEDArray_OneWireConfig(int n_pixels)
        : LEDArray_OneWireConfig(n_pixels, 800000)
    {
    }
    /** Default constructor */
    LEDArray_OneWireConfig()
        : LEDArray_OneWireConfig(0)
    {
        n_pixels = 0;
        target_freq_hz = 800000;
    }
    virtual size_t GetSize() const
    {
        return sizeof(*this);
    }
    virtual VMXResourceConfig* GetCopy() const
    {
        LEDArray_OneWireConfig* p_new = new LEDArray_OneWireConfig();
        *p_new = *this;
        return p_new;
    }
    virtual bool Copy(const VMXResourceConfig* p_config)
    {
        if (p_config->GetResourceType() != this->GetResourceType())
            return false;
        *this = *((LEDArray_OneWireConfig*)p_config);
        return true;
    }
    virtual ~LEDArray_OneWireConfig() {}
    /** Gets the number of "pixels" in the physical LED Array. */
    int GetNumPixels() { return n_pixels; }
    /** Sets the number of "pixels" in the physical LED Array. */
    void SetNumPixels(int n_pixels) { this->n_pixels = n_pixels; }
    /** Gets the "symbol" frequency supported by the LED Array.  Defaults to 800000; this value works for all known LED Arrays */
    int GetTargetFrequencyHz() { return target_freq_hz; }
    /** Sets the "symbol" frequency supported by the LED Array.  Defaults to 800000; this value works for all known LED Arrays */
    void SetTargetFrequencyHz(int target_freq_hz) { this->target_freq_hz = target_freq_hz; }
    /** Gets the "reset symbol" duration supported by the LED Array.  Defaults to 80 microseconds. */
    uint64_t GetResetWaitTimeMicroseconds() { return reset_wait_time_us; }
    /** Sets the "reset symbol" duration supported by the LED Array.  Defaults to 80 microseconds. */
    void SetResetWaitTimeMicroseconds(uint64_t reset_wait_time_us) { this->reset_wait_time_us = reset_wait_time_us; }
    /** Gets the "one" symbol duration supported by the LED Array.  Defaults to 600 microseconds. */
    uint64_t GetOneSymbolHighTimeNanoseconds() { return one_symbol_high_time_ns; }
    /** Sets the "one" symbol duration supported by the LED Array.  Defaults to 600 microseconds. */
    void SetOneSymbolHighTimeNanoseconds(uint64_t one_symbol_high_time_ns) { this->one_symbol_high_time_ns = one_symbol_high_time_ns; }
    /** Gets the "zero" symbol duration supported by the LED Array.  Defaults to 300 microseconds. */
    uint64_t GetZeroSymbolHighTimeNanoseconds() { return zero_symbol_high_time_ns; }
    /** Sets the "zero" symbol duration supported by the LED Array.  Defaults to 300 microseconds. */
    void SetZeroSymbolHighTimeNanoseconds(uint64_t zero_symbol_high_time_ns) { this->zero_symbol_high_time_ns = zero_symbol_high_time_ns; }
    /** Gets the Pixel format, which may be one of several configurations.  Formats ending in W send extra 'white' data per pixel. */
    PixelFormat GetPixelFormat() { return pixel_format; }
    /** Sets the Pixel format, which may be one of several configurations.  Formats ending in W send extra 'white' data per pixel. */
    void SetPixelFormat(PixelFormat pixel_format) { this->pixel_format = pixel_format; }
};

/****** SERIALIZATION SUPPORT ******/

typedef struct PWMCaptureConfig LargestVMXResourceConfigClass; // Must be typedef'd to largest VMXResourceConfig-derived class.

// VMXResourceConfigStream represents a serialized VMXResourceConfig class, enabling peristence and IPC.
struct VMXResourceConfigStream {
    uint8_t data[sizeof(LargestVMXResourceConfigClass)];
    VMXResourceConfigStream()
    {
        memset(data, 0, sizeof(data));
    }
    template <typename VMXResourceConfigClass>
    void Serialize(const VMXResourceConfig* p_src)
    {
        memcpy(data, p_src, sizeof(VMXResourceConfigClass));
    }
    template <typename VMXResourceConfigClass>
    VMXResourceConfig* Deserialize(VMXResourceConfig* p_serialized)
    {
        VMXResourceConfigClass* p_dest = new VMXResourceConfigClass();
        p_dest->Copy(p_serialized);
        return p_dest;
    }
    VMXResourceConfig* DeserializeResourceConfig()
    {
        VMXResourceConfig* p_serialized = (VMXResourceConfig*)data;
        VMXResourceConfig* p_new = 0;
        // Warning.  The vfptr in the serialized data class is uninitialized,
        // there virtual functions of this class cannot be invoked.
        switch (p_serialized->res_type) {
        case VMXResourceType::Interrupt:
            p_new = Deserialize<InterruptConfig>(p_serialized);
            break;
        case VMXResourceType::DigitalIO:
            p_new = Deserialize<DIOConfig>(p_serialized);
            break;
        case VMXResourceType::PWMGenerator:
            p_new = Deserialize<PWMGeneratorConfig>(p_serialized);
            break;
        case VMXResourceType::PWMCapture:
            p_new = Deserialize<PWMCaptureConfig>(p_serialized);
            break;
        case VMXResourceType::Encoder:
            p_new = Deserialize<EncoderConfig>(p_serialized);
            break;
        case VMXResourceType::Accumulator:
            p_new = Deserialize<AccumulatorConfig>(p_serialized);
            break;
        case VMXResourceType::AnalogTrigger:
            p_new = Deserialize<AnalogTriggerConfig>(p_serialized);
            break;
        case VMXResourceType::UART:
            p_new = Deserialize<UARTConfig>(p_serialized);
            break;
        case VMXResourceType::SPI:
            p_new = Deserialize<SPIConfig>(p_serialized);
            break;
        case VMXResourceType::I2C:
            p_new = Deserialize<I2CConfig>(p_serialized);
            break;
        case VMXResourceType::LEDArrayDriver_OneWire:
            p_new = Deserialize<LEDArray_OneWireConfig>(p_serialized);
            break;
        default:
            break;
        }
        return p_new;
    }
    explicit VMXResourceConfigStream(const VMXResourceConfig* res_cfg)
        : VMXResourceConfigStream()
    {
        switch (res_cfg->GetResourceType()) {
        case VMXResourceType::Interrupt:
            Serialize<InterruptConfig>(res_cfg);
            break;
        case VMXResourceType::DigitalIO:
            Serialize<DIOConfig>(res_cfg);
            break;
        case VMXResourceType::PWMGenerator:
            Serialize<PWMGeneratorConfig>(res_cfg);
            break;
        case VMXResourceType::PWMCapture:
            Serialize<PWMCaptureConfig>(res_cfg);
            break;
        case VMXResourceType::Encoder:
            Serialize<EncoderConfig>(res_cfg);
            break;
        case VMXResourceType::Accumulator:
            Serialize<AccumulatorConfig>(res_cfg);
            break;
        case VMXResourceType::AnalogTrigger:
            Serialize<AnalogTriggerConfig>(res_cfg);
            break;
        case VMXResourceType::UART:
            Serialize<UARTConfig>(res_cfg);
            break;
        case VMXResourceType::SPI:
            Serialize<SPIConfig>(res_cfg);
            break;
        case VMXResourceType::I2C:
            Serialize<I2CConfig>(res_cfg);
            break;
        case VMXResourceType::LEDArrayDriver_OneWire:
            Serialize<LEDArray_OneWireConfig>(res_cfg);
            break;
        default:
            break;
        }
    }
    ~VMXResourceConfigStream() {}
};

#endif /* VMXRESOURCECONFIG_H_ */
