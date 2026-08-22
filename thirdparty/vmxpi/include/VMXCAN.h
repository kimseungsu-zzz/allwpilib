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

#ifndef VMXCAN_H_
#define VMXCAN_H_

#include <stdint.h>
#include <string.h>
#include <thread>
#include <map>
#include <iterator>
#include "VMXErrors.h"
#include "VMXTime.h"

/* NOTE:  Usage Model similar to that found in the WPILIB Suite HAL */

#define VMXCAN_IS_FRAME_REMOTE			0x80000000
#define VMXCAN_IS_FRAME_11BIT			0x40000000
#define VMXCAN_29BIT_MESSAGE_ID_MASK	0x1FFFFFFF
#define VMXCAN_11BIT_MESSAGE_ID_MASK	0x000007FF

#define VMXCAN_SEND_PERIOD_NO_REPEAT		 0
#define VMXCAN_SEND_PERIOD_STOP_REPEATING	-1

/** The xVMXCANMessage structure encapsulates a CAN Bus message */
typedef struct sVMXCANMessage {
	uint32_t messageID;
	uint8_t  data[8];
	uint8_t  dataSize;
	
	void setData(const uint8_t *p_data_in, uint8_t len) {
		if (len > 8) {
			len = 8;
		}
		for(int i = 0; i < len; i++ ) {
			data[i] = p_data_in[i];
		}
	}
	void getData(uint8_t *p_data_out, uint8_t len) {
		if (len > 8) {
			len = 8;
		}
		for(int i = 0; i < len; i++ ) {
			p_data_out[i] = data[i];
		}
	}
} VMXCANMessage;

/** The sVMXTimestampedCANMessage encapsulates a timestamped CAN Bus message */
typedef struct sVMXTimestampedCANMessage : public sVMXCANMessage {
	uint32_t timeStampMS;
	uint64_t sysTimeStampUS;
} VMXCANTimestampedMessage;

typedef struct sVMXBlackboardTimestampedCANMessage : public sVMXTimestampedCANMessage {
	bool retrieved;
} VMXBlackboardTimestampedCANMessage;

/** The VMXCANBusStatus structure contains CAN Bus utilitization, status and error information. */

typedef struct {
	/** Percentage of total CAN bus bandwidth currently utilized */
	float percentBusUtilization;
	/** busOffCount:  The number of times that sendMessage failed with a busOff error indicating that messages are not
	 * successfully transmitted on the bus.
	 */
	uint32_t busOffCount;
	/** txFullCount:  The number of times that sendMessage failed with a txFifoFull error indicating that messages are not
	 * successfully received by any CAN device.
	 */
	uint32_t txFullCount;
	/** receiveErrorCount:  The count of receive errors as reported by the CAN driver. */
	uint32_t receiveErrorCount;
	/** transmitErrorCount:  The count of transmit errors as reported by the CAN driver. */
	uint32_t transmitErrorCount;
	/** Currently a bus warning condition exists */
	bool busWarning;
	/** Currently a bus passive condition exists */
	bool busPassiveError;
	/** Currently a bus off condition exists */
	bool busOffError;
	/** Hardware-level buffer overflow */
	bool hwRxOverflow;
	/** Firmware-level buffer overflow */
	bool swRxOverflow;
	/** Transceiver bus error detected */
	bool busError;
	/** Transceiver wakeup event occurred */
	bool wake;
	/** Controller message error detected */
	bool messageError;
} VMXCANBusStatus;

typedef uint32_t VMXCANReceiveStreamHandle;

using namespace std;

class CANClient;
class VMXCANReceiveStreamManager;

/** The VMXCAN class provides a hardware-abstraction of the VMX-pi CAN functionality.
 *
 */
class VMXCAN {

	friend class VMXPi;

	typedef struct sVMXTimestampedCANMessageRepeating : public sVMXTimestampedCANMessage {
		uint64_t lastSendTimestampUS;
		uint32_t repeatEveryMS;
	} VMXCANTimestampedMessageRepeating;


	CANClient& can;
	VMXTime& time;
	VMXCANReceiveStreamManager *p_stream_mgr;
	thread * task;
	bool task_done;
	bool initialized;
	thread * repeating_packet_thread_task;
	bool repeating_packet_thread_task_done;
	std::map<uint32_t, VMXCANTimestampedMessageRepeating> repeatingMessageMap;

	VMXCAN(CANClient& can_ref, VMXTime& time_ref);
	virtual ~VMXCAN();
	void ReleaseResources();

	static void CANNewRxDataNotifyHandler(void *param, uint64_t timestamp_us);
	static int ThreadFunc(VMXCAN *);
	static int RepeatingPacketThreadFunc(VMXCAN *);

	bool RemoveFromRepeatingMessageMap(uint32_t);
	bool AddToRepeatingMessageMap(uint32_t, VMXCANMessage&, uint32_t repeatEveryMS);
	bool SendMessageInternal(VMXCANMessage& msg, VMXErrorCode *errcode);

public:
	void DisplayMasksAndFilters();

public:

	typedef enum { CAN_BUS_BITRATE_1MBPS, CAN_BUS_BITRATE_500KBPS, CAN_BUS_BITRATE_250KBPS } CANBusBitrate;

	/**
	 * Enqueues the CAN message for transmission onto the CAN bus to which
	 * the VMX device is currently connected.
	 *
	 * The message is placed into a Fifo and transmitted as soon as possible.
	 *
	 * This function is re-entrant and thus may be called from multiple contexts.
	 *
	 * @param msg The message to be transmitted at the ID specified within the message.
	 * Note that unless the periodMs parameter is negative, the data bytes and length
	 * in the message must be valid.
	 * @param periodMs If >= 0, the message is set immediately.  Additionally, if greater
	 * than zero, the message (and its data) is periodically retransmitted after this 
	 * number of milliseconds.  If zero, the message is only transmitted once. 
	 * If less than zero, any previously-requested periodic message retranmsittal for 
	 * this CAN ID will be cancelled.
	 * @param[out] errcode Pointer to the VMXErrorCode to be returned in case of
	 * error; may be null
	 * @return Returns true if the message succesfully enqueued for transfer.  If false,
	 * the error code will be returned via the errcode parameter.
	 */
	bool SendMessage(VMXCANMessage& msg, int32_t periodMs, VMXErrorCode *errcode);

	/* Opens a new VMXCANReceiveStream.
	 *
	 * messageID:  The CAN Arb ID for the message of interest.  This is basically the Message FILTER.
	 * Since this operates as a filter, all bits in the messageID that are masked in by the
	 * messageMask must match.  Using a combination of messageMask and messageID, it is possible
	 * to allow ranges of CAN Arb IDs to be received with a single VMXCANReceiveStream.
	 *
	 * messsageMask:  The Message MASK to use for the messages of interest.  NOTE: the underlying hardware supports
	 * a total of two masks.  If a third VMXCANReceiveStream is opened which has a different Mask than the two
	 * already-opened VMXCANReceiveStreams, the open will fail.  To work around this limitation, the masks can be
	 * ANDed together (and the bits cleared during the ANDing should be cleared also in the VMXCANReceiveStream
	 * messageIDs.  In this case, it is likely that additional undesired messages may be received; these should be
	 * discarded by the application.
	 *
	 * maxMessages:  This is maximum number of received messages to internally buffer for this stream.
	 */
	bool OpenReceiveStream(VMXCANReceiveStreamHandle& streamHandle, uint32_t messageID, uint32_t messageMask, uint32_t maxMessages, VMXErrorCode *errcode);
	/* Retrieves recently received data for a VMXCANReceiveStream.
	 *
	 * messages:  Pointer to an array VMXCANTimestampedMessage structures.  The number of elements in this array
	 * must be greater than or equal to the messagesToRead parameter to avoid memory corruption.
	 *
	 * messagesToRead:  The maximum number of messages to be read by this function.
	 *
	 * messagesRead:  Output from this function, this indicates the actual number of messages read by this function;
	 * this value will never exceed the value passed in the messagesToRead parameter.
	 *
	 * errcode:  If non-null in case of error this will return the error code.
	 */
	bool ReadReceiveStream(VMXCANReceiveStreamHandle streamHandle, VMXCANTimestampedMessage *messages, uint32_t messagesToRead, uint32_t& messagesRead, VMXErrorCode *errcode);

	/* Closes a previously-opened VMCANReceiveStream.
	 *
	 * errcode:  If non-null in case of error this will return the error code.
	 */
	bool CloseReceiveStream(VMXCANReceiveStreamHandle streamHandle, VMXErrorCode *errcode);

	/* Retrieves current CAN Bus statistics and warning/error states.
	 *
	 * errcode:  If non-null in case of error this will return the error code.
	 */
	bool GetCANBUSStatus(VMXCANBusStatus& bus_status, VMXErrorCode *errcode);

	/* Resets the CAN Transceiver and Controller.  Note that this operation will clear any previously-
	 * configured Receive Masks/Filters and other configuration settings.  This function sets the
	 * CAN bus bitrate to the default (CAN_BUS_BITRATE_1MBPS).
	 *
	 * errcode:  If non-null in case of error this will return the error code.
	 */
	bool Reset(VMXErrorCode *errcode);

	/* Resets the CAN Transceiver and Controller.  Note that this operation will clear any previously-
	 * configured Receive Masks/Filters and other configuration settings.  This function sets the
	 * CAN bus bitrate to the specified bus bitrate.
	 *
	 * can_bus_bitrate:  The specified CANBusBitrate to be used after CAN controller is reset.
	 * errcode:  If non-null in case of error this will return the error code.
	 */
	bool ResetBusBitrate(VMXCAN::CANBusBitrate can_bus_bitrate, VMXErrorCode *errcode);

	/* Empties the Transmit FIFO of any unsent VMXCANMessages.
	 *
	 * errcode:  If non-null in case of error this will return the error code.
	 */
	bool FlushTxFIFO(VMXErrorCode *errcode);
	/* Empties the Receive FIFO of any unreceived VMXCANMessages.
	 *
	 * errcode:  If non-null in case of error this will return the error code.
	 */
	bool FlushRxFIFO(VMXErrorCode *errcode);

	/** Enumeration of VMX CAN operational modes */
	typedef enum {
		/** Places the CAN Transceiver/Controller into a sleep state, which may be automatically
		 * woken from if CAN Bus activity is later detected.
		 */
		VMXCAN_OFF,
		/** Places the CAN Controller into configuration mode, during which the Receive Masks/
		 * Fiters may be modified.
		 */
		VMXCAN_CONFIG,
		/** Places the CAN Controller into normal mode, where it may transmit messages onto the
		 * CAN Bus, and may receive messages from the CAN Bus.  All Receive Masks/Filters which
		 * have previously been configured are active and will be used to filter the reception
		 * of transmitted messages.
		 */
		VMXCAN_NORMAL,
		/** Places the CAN Controller into a listen-only mode where it passively listens to
		 * the CAN Bus.  Note that all Receive Masks/Filters which have previously been configured
		 * will be ignored during this mode.
		 */
		VMXCAN_LISTEN,
		/** Places the CAN Controller into a mode where transmitted messages are looped back internally
		 * to the Receive circuitry.  During this mode, the Transceiver is completely disconnected
		 * from the CAN Bus.  Note that during this mode, all Receive Masks/Filters which have previously
		 * been configured are active and will be used to filter the reception of transmitted messages.
		 */
		VMXCAN_LOOPBACK
	} VMXCANMode;

	/* Sets the CAN Controller/Transceiver's current VMXCANMode.
	 *
	 * errcode:  If non-null in case of error this will return the error code.
	 */
	bool SetMode(VMXCANMode mode, VMXErrorCode *errcode);

	/* Retrieves the CAN Controller/Transceiver's current VMXCANMode.
	 *
	 * errcode:  If non-null in case of error this will return the error code.
	 */
	bool GetMode(VMXCANMode& current_mode, VMXErrorCode *errcode);

	/* Clears any error conditions in the firmware and the CAN Controller/Transceiver.
	 *
	 * This function must be invoked in order to reset a hardware overflow condition.
	 *
	 * errcode:  If non-null in case of error this will return the error code.
	 */
	bool ClearErrors(VMXErrorCode *errcode);

	bool RetrieveAllCANData(uint64_t sys_timestamp, VMXErrorCode *errcode); /* Todo:  This should be removed from the public interface. */

	bool EnableReceiveStreamBlackboard(VMXCANReceiveStreamHandle streamHandle, bool enable, VMXErrorCode *errcode);

	bool IsReceveStreamBlackboardEnabled(VMXCANReceiveStreamHandle streamHandle, bool& enabled, VMXErrorCode *errcode);

	bool GetBlackboardEntry(VMXCANReceiveStreamHandle streamid, uint32_t messageID, VMXCANTimestampedMessage& msg, uint64_t& sys_timestamp, bool& already_retrieved, VMXErrorCode *errcode);
};

#endif /* VMXCAN_H_ */
