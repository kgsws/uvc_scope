#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3uart.h>
#include <cyu3gpif.h>
#include <cyu3i2c.h>
#include <cyu3gpio.h>
#include <cyu3pib.h>
#include <cyu3utils.h>
#include <cyu3socket.h>
#include <sock_regs.h>
#include "uvc.h"

#include "cyfxgpif2config.h"

CyU3PReturnStatus_t CyU3PDmaMultiChannelAcquireLock(CyU3PDmaMultiChannel *handle, uint32_t waitOption);

static CyU3PThread thread_uvc;
static CyU3PThread thread_uvc_ep0;

static CyU3PEvent event_uvc;

static CyU3PDmaMultiChannel dma_uvc_stream;

static volatile uint8_t bmReqType, bRequest;
static volatile uint16_t wValue, wIndex, wLength;

static CyBool_t usb_config_state;
static CyBool_t uvc_active_state;
static CyBool_t streaming_state;

static uint8_t uvc_error_code = CY_FX_UVC_VC_ERROR_CODE_NO_ERROR;

static uint8_t ep_data_buff[CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED];

static uint8_t volatile uvc_header_buff[CY_FX_UVC_MAX_HEADER] =
{
	0x0C,                               // Header Length
	0x8C,                               // Bit field header field
	0x00, 0x00, 0x00, 0x00,             // Presentation time stamp field
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Source clock reference field
};

// UVC Probe Control Settings for a USB 3.0 connection
static uint8_t uvc_probe_ss[CY_FX_UVC_MAX_PROBE_SETTING] =
{
	0x00, 0x00,                 // bmHint : no hit
	0x01,                       // Use 1st Video format index
	0x01,                       // Use 1st Video frame index
	0xC8, 0x77, 0x03, 0x00,     // Desired frame interval in the unit of 100ns: 44 FPS
	0x00, 0x00,                 // Key frame rate in key frame/video frame units: only applicable to video streaming with adjustable compression parameters
	0x00, 0x00,                 // PFrame rate in PFrame / key frame units: only applicable to video streaming with adjustable compression parameters
	0x00, 0x00,                 // Compression quality control: only applicable to video streaming with adjustable compression parameters
	0x00, 0x00,                 // Window size for average bit rate: only applicable to video streaming with adjustable compression parameters
	0x00, 0x00,                 // Internal video streaming i/f latency in ms
	0x00, 0x00, 0x20, 0x00,     // Max video frame size in bytes
	0xE0, 0x8F, 0x00, 0x00,     // No. of bytes device can rx in single payload = 36832 B

#ifndef FX3_UVC_1_0_SUPPORT
	// UVC 1.1 Probe Control has additional fields from UVC 1.0
	0x00, 0x60, 0xE3, 0x16,     // Device Clock
	0x00,                       // Framing Information - Ignored for uncompressed format
	0x00,                       // Preferred payload format version
	0x00,                       // Minimum payload format version
	0x00                        // Maximum payload format version
#endif
};

// UVC Probe Control Setting for a USB 2.0 connection.
static uint8_t uvc_probe_hs[CY_FX_UVC_MAX_PROBE_SETTING] =
{
	0x00, 0x00,                 // bmHint : no hit
	0x01,                       // Use 1st Video format index
	0x01,                       // Use 1st Video frame index
	0x2A, 0x2C, 0x0A, 0x00,     // Desired frame interval in the unit of 100ns: 15 fps
	0x00, 0x00,                 // Key frame rate in key frame/video frame units: only applicable to video streaming with adjustable compression parameters
	0x00, 0x00,                 // PFrame rate in PFrame / key frame units: only applicable to video streaming with adjustable compression parameters
	0x00, 0x00,                 // Compression quality control: only applicable to video streaming with adjustable compression parameters 
	0x00, 0x00,                 // Window size for average bit rate: only applicable to video streaming with adjustable compression parameters
	0x00, 0x00,                 // Internal video streaming i/f latency in ms
	0x00, 0x10, 0x09, 0x00,     // Max video frame size in bytes
	0xE0, 0x8F, 0x00, 0x00,     // No. of bytes device can rx in single payload = 36832 B

#ifndef FX3_UVC_1_0_SUPPORT
	// UVC 1.1 Probe Control has additional fields from UVC 1.0
	0x00, 0x60, 0xE3, 0x16,     // Device Clock
	0x00,                       // Framing Information - Ignored for uncompressed format
	0x00,                       // Preferred payload format version
	0x00,                       // Minimum payload format version
	0x00                        // Maximum payload format version
#endif
};

//
// UVC stuff

void uvc_add_header(uint8_t *buff, uint8_t flags)
{
	CyU3PMemCopy(buff, (uint8_t *)uvc_header_buff, CY_FX_UVC_MAX_HEADER);
	if(flags & CY_FX_UVC_HEADER_EOF)
	{
		buff[1] |= CY_FX_UVC_HEADER_EOF;
		uvc_header_buff[1] ^= CY_FX_UVC_HEADER_FRAME_ID;
	}
}

static void uvc_req_processing()
{
	// nothing is supported
	uvc_error_code = CY_FX_UVC_VC_ERROR_CODE_INVALID_CONTROL;
	CyU3PUsbStall(0, CyTrue, CyFalse);
}

static void uvc_req_camera_terminal()
{
	// nothing is supported
	uvc_error_code = CY_FX_UVC_VC_ERROR_CODE_INVALID_CONTROL;
	CyU3PUsbStall(0, CyTrue, CyFalse);
}

static void uvc_req_interface_ctrl()
{
	switch(wValue)
	{
		case CY_FX_UVC_VC_REQUEST_ERROR_CODE_CONTROL:
			switch(bRequest)
			{
				case CY_FX_USB_UVC_GET_CUR_REQ:
					CyU3PUsbSendEP0Data(1, &uvc_error_code);
					uvc_error_code = CY_FX_UVC_VC_ERROR_CODE_NO_ERROR;
				break;
			}
		break;
		default:
			CyU3PUsbStall(0, CyTrue, CyFalse);
		break;
	}
}

static void uvc_req_extension()
{
	// nothing is supported
	uvc_error_code = CY_FX_UVC_VC_ERROR_CODE_INVALID_CONTROL;
	CyU3PUsbStall(0, CyTrue, CyFalse);
}

static void uvc_req_streaming()
{
	uint16_t read_count;

	switch(wValue)
	{
		case CY_FX_UVC_PROBE_CTRL:
			switch(bRequest)
			{
				case CY_FX_USB_UVC_GET_INFO_REQ:
					ep_data_buff[0] = 3;
					CyU3PUsbSendEP0Data(1, ep_data_buff);
				break;
				case CY_FX_USB_UVC_GET_LEN_REQ:
					ep_data_buff[0] = CY_FX_UVC_MAX_PROBE_SETTING;
					CyU3PUsbSendEP0Data(1, ep_data_buff);
				break;
				case CY_FX_USB_UVC_GET_CUR_REQ:
				case CY_FX_USB_UVC_GET_MIN_REQ:
				case CY_FX_USB_UVC_GET_MAX_REQ:
				case CY_FX_USB_UVC_GET_DEF_REQ:
					if(CyU3PUsbGetSpeed() == CY_U3P_SUPER_SPEED)
						CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, uvc_probe_ss);
					else
						CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, uvc_probe_hs);
				break;
				case CY_FX_USB_UVC_SET_CUR_REQ:
				{
					uint8_t *dst;

					if(CyU3PUsbGetSpeed() == CY_U3P_SUPER_SPEED)
						dst = uvc_probe_ss;
					else
						dst = uvc_probe_hs;

					if(CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED, ep_data_buff, &read_count) == CY_U3P_SUCCESS)
					{
						dst[2] = ep_data_buff[2];
						dst[3] = ep_data_buff[3];
						dst[4] = ep_data_buff[4];
						dst[5] = ep_data_buff[5];
						dst[6] = ep_data_buff[6];
						dst[7] = ep_data_buff[7];
					}
				}
				break;
				default:
					CyU3PUsbStall(0, CyTrue, CyFalse);
				break;
			}
		break;
		case CY_FX_UVC_COMMIT_CTRL:
			switch(bRequest)
			{
				case CY_FX_USB_UVC_GET_INFO_REQ:
					ep_data_buff[0] = 3;
					CyU3PUsbSendEP0Data(1, ep_data_buff);
				break;
				case CY_FX_USB_UVC_GET_LEN_REQ:
					ep_data_buff[0] = CY_FX_UVC_MAX_PROBE_SETTING;
					CyU3PUsbSendEP0Data(1, ep_data_buff);
				break;
				case CY_FX_USB_UVC_GET_CUR_REQ:
					if(CyU3PUsbGetSpeed() == CY_U3P_SUPER_SPEED)
						CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, uvc_probe_ss);
					else
						CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, uvc_probe_hs);
				break;
				case CY_FX_USB_UVC_SET_CUR_REQ:
					if(CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED, ep_data_buff, &read_count) == CY_U3P_SUCCESS)
						CyU3PEventSet(&event_uvc, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_OR);
				break;
				default:
					CyU3PUsbStall(0, CyTrue, CyFalse);
				break;
			}
		break;
		default:
			CyU3PUsbStall(0, CyTrue, CyFalse);
		break;
	}
}

//
// fixed SDK function 'CyU3PDmaChannelDiscardBuffer'

static CyU3PReturnStatus_t dma_custom_discard(CyU3PDmaMultiChannel *handle)
{
	uint8_t type;
	uint32_t status;
	CyU3PDmaDescriptor_t prodDscr, consDscr;

	status = CyU3PDmaMultiChannelAcquireLock(handle, CYU3P_WAIT_FOREVER);
	if(status != CY_U3P_SUCCESS)
		return status;

	type = handle->type;
	if((handle->state != CY_U3P_DMA_ACTIVE) && (handle->state != CY_U3P_DMA_IN_COMPLETION))
	{
		status = CY_U3P_ERROR_NOT_STARTED;
	}

#ifndef CYU3P_DISABLE_ERROR_CHECK
	if(handle->state == CY_U3P_DMA_ERROR)
	{
		status = CY_U3P_ERROR_DMA_FAILURE;
	}
	if(handle->state == CY_U3P_DMA_ABORTED)
	{
		status = CY_U3P_ERROR_ABORTED;
	}
#endif /* CYU3P_DISABLE_ERROR_CHECK */

	if(type != CY_U3P_DMA_TYPE_MANUAL_MANY_TO_ONE)
	{
		status = CY_U3P_ERROR_NOT_SUPPORTED;
	}

	if(status != CY_U3P_SUCCESS)
	{
		/* Release the lock and return. */
		CyU3PMutexPut(&(handle->lock));
		return status;
	}

	/* Configure the consumer descriptor chain to set the discard marker. */
	CyU3PDmaDscrGetConfig(handle->currentProdIndex, &prodDscr);
	CyU3PDmaDscrGetConfig(handle->currentConsIndex, &consDscr);
	if(!(prodDscr.size & CY_U3P_BUFFER_OCCUPIED))
	{
		status = CY_U3P_ERROR_INVALID_SEQUENCE;
	}
	if(consDscr.size & CY_U3P_BUFFER_OCCUPIED)
	{
		status = CY_U3P_ERROR_INVALID_SEQUENCE;
	}

	if(status == CY_U3P_SUCCESS)
	{
		/* Set the discard marker. */
		consDscr.size &= CY_U3P_BUFFER_SIZE_MASK;
		CyU3PDmaDscrSetConfig(handle->currentConsIndex, &consDscr);
		prodDscr.size &= CY_U3P_BUFFER_SIZE_MASK;
		CyU3PDmaDscrSetConfig(handle->currentProdIndex, &prodDscr);

		handle->currentConsIndex = consDscr.chain & CY_U3P_RD_NEXT_DSCR_MASK;
		/* The next producer index is stored in the next consumer descriptor. */
		CyU3PDmaDscrGetConfig(handle->currentConsIndex, &consDscr);
		handle->currentProdIndex = consDscr.chain >> CY_U3P_WR_NEXT_DSCR_POS;
	}

	/* Release the lock. */
	CyU3PMutexPut(&(handle->lock));

	return status;
}

//
// DMA callback

void cb_dma_uvc_stream(CyU3PDmaMultiChannel *chHandle, CyU3PDmaCbType_t type, CyU3PDmaCBInput_t *input)
{
	CyU3PDmaBuffer_t dmaBuffer;
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	if(type == CY_U3P_DMA_CB_PROD_EVENT)
	{
		status = CyU3PDmaMultiChannelGetBuffer(chHandle, &dmaBuffer, CYU3P_NO_WAIT);
		while(status == CY_U3P_SUCCESS)
		{
			if(dmaBuffer.count == CY_FX_UVC_BUF_FULL_SIZE)
				uvc_add_header(dmaBuffer.buffer - CY_FX_UVC_MAX_HEADER, CY_FX_UVC_HEADER_FRAME);
			else
				uvc_add_header(dmaBuffer.buffer - CY_FX_UVC_MAX_HEADER, CY_FX_UVC_HEADER_EOF);

			status = CyU3PDmaMultiChannelCommitBuffer(chHandle, (dmaBuffer.count + CY_FX_UVC_MAX_HEADER), 0);
			if(status != CY_U3P_SUCCESS)
			{
				dma_custom_discard(chHandle);
				break;
			}

			status = CyU3PDmaMultiChannelGetBuffer(chHandle, &dmaBuffer, CYU3P_NO_WAIT);
		}
	} else
	if(type == CY_U3P_DMA_CB_CONS_EVENT)
		streaming_state = CyTrue;
}

//
// GPIF callback

void cb_gpif(uint8_t state)
{
	switch(state)
	{
		case PARTIAL_BUF_IN_SCK0:
			CyU3PDmaSocketSetWrapUp(CY_U3P_PIB_SOCKET_0);
		break;
		case FULL_BUF_IN_SCK0:
		break;
		case PARTIAL_BUF_IN_SCK1:
			CyU3PDmaSocketSetWrapUp(CY_U3P_PIB_SOCKET_1);
		break;
		case FULL_BUF_IN_SCK1:
		break;
		default:
			return;
	}

	CyU3PGpifControlSWInput(CyTrue);
	CyU3PGpifControlSWInput(CyFalse);
}

//
// USB callbacks

static void cb_usb_event(CyU3PUsbEventType_t type, uint16_t data)
{
	switch(type)
	{
		case CY_U3P_USB_EVENT_SUSPEND:
			CyU3PEventSet(&event_uvc, CY_FX_USB_SUSPEND_EVENT_HANDLER, CYU3P_EVENT_OR);
		break;
//		case CY_U3P_USB_EVENT_EP_UNDERRUN:
//		break;
		case CY_U3P_USB_EVENT_SETCONF:
//			switch(CyU3PUsbGetSpeed()) // CY_U3P_SUPER_SPEED // CY_U3P_HIGH_SPEED
			usb_config_state = CyTrue;
			goto skip;
		case CY_U3P_USB_EVENT_RESET:
		case CY_U3P_USB_EVENT_DISCONNECT:
		case CY_U3P_USB_EVENT_CONNECT:
			usb_config_state = CyFalse;
skip:
			CyU3PUsbLPMEnable();
			if(uvc_active_state)
				CyU3PEventSet(&event_uvc, CY_FX_UVC_STREAM_ABORT_EVENT, CYU3P_EVENT_OR);
		break;
		default:
		break;
	}
}

static CyBool_t cb_uvc_setup(uint32_t data0, uint32_t data1)
{
	bmReqType = (uint8_t)(data0 & CY_FX_USB_SETUP_REQ_TYPE_MASK);
	bRequest = (uint8_t)((data0 & CY_FX_USB_SETUP_REQ_MASK) >> 8);
	wValue = (uint16_t)((data0 & CY_FX_USB_SETUP_VALUE_MASK) >> 16);
	wIndex = (uint16_t)(data1 & CY_FX_USB_SETUP_INDEX_MASK);
	wLength = (uint16_t)((data1 & CY_FX_USB_SETUP_LENGTH_MASK) >> 16);

	switch(bmReqType)
	{
		case CY_FX_USB_UVC_GET_REQ_TYPE:
		case CY_FX_USB_UVC_SET_REQ_TYPE:
			switch(wIndex & 0xFF)
			{
				case CY_FX_UVC_CONTROL_INTERFACE:
					if(CyU3PEventSet(&event_uvc, CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT, CYU3P_EVENT_OR) != CY_U3P_SUCCESS)
						CyU3PUsbStall(0, CyTrue, CyFalse);
				return CyTrue;
				case CY_FX_UVC_STREAM_INTERFACE:
					if(CyU3PEventSet(&event_uvc, CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT, CYU3P_EVENT_OR) != CY_U3P_SUCCESS)
						CyU3PUsbStall(0, CyTrue, CyFalse);
				return CyTrue;
				default:
				break;
			}
		break;
		case CY_FX_USB_SET_INTF_REQ_TYPE:
			if(bRequest == CY_FX_USB_SET_INTERFACE_REQ)
			{
				if((wIndex == CY_FX_UVC_STREAM_INTERFACE) && (wValue == 0))
				{
					CyU3PUsbStall(CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);
					CyU3PEventSet(&event_uvc, CY_FX_UVC_STREAM_ABORT_EVENT, CYU3P_EVENT_OR);
					CyU3PUsbAckSetup();
					return CyTrue;
				}
			} else
			if((bRequest == CY_U3P_USB_SC_SET_FEATURE) || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE))
			{
				if(usb_config_state)
				{
					CyU3PUsbAckSetup();
					return CyTrue;
				}
			}
		break;
		case CY_U3P_USB_TARGET_ENDPT:
			if(bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
			{
				if (wIndex == CY_FX_EP_BULK_VIDEO)
				{
					CyU3PUsbStall(CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);
					CyU3PEventSet(&event_uvc, CY_FX_UVC_STREAM_ABORT_EVENT, CYU3P_EVENT_OR);
					CyU3PUsbAckSetup();
					return CyTrue;
				}
			}
		break;
	}

	return CyFalse;
}

//
// UVC state

static void uvc_enable()
{
	uvc_header_buff[1] &= ~CY_FX_UVC_HEADER_FRAME_ID;

	CyU3PUsbLPMDisable();
	if(CyU3PUsbGetSpeed() == CY_U3P_SUPER_SPEED)
	{
		CyU3PUsbSetLinkPowerState(CyU3PUsbLPM_U0);
		CyU3PBusyWait(200);
	}// else
//		CyU3PUsb2Resume();

	CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyTrue);
	CyU3PBusyWait(125);

	CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);
	CyU3PDmaMultiChannelReset(&dma_uvc_stream);

	if(CyU3PDmaMultiChannelSetXfer(&dma_uvc_stream, 0, 0) != CY_U3P_SUCCESS)
	{
		// error - fail
		CyU3PDeviceReset(CyFalse);
	}

	CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyFalse);
	CyU3PBusyWait(125);

	if(CyU3PGpifSMSwitch(CY_FX_UVC_INVALID_GPIF_STATE, START_SCK0, CY_FX_UVC_INVALID_GPIF_STATE, ALPHA_START_SCK0, CY_FX_UVC_GPIF_SWITCH_TIMEOUT) != CY_U3P_SUCCESS)
	{
		// error - fail
		CyU3PDeviceReset(CyFalse);
	}
}

static void uvc_disable()
{
	CyU3PGpifDisable(CyFalse);
	streaming_state = CyFalse;

	CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyTrue);
	CyU3PBusyWait(125);

	CyU3PDmaMultiChannelReset(&dma_uvc_stream);
	CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);
	CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyFalse);
	CyU3PBusyWait(125);

	CyU3PUsbLPMEnable();
}

//
// UVC init

static CyBool_t cb_usb_lpm(CyU3PUsbLinkPowerMode link_mode)
{
	return CyTrue;
}

static void uvc_init()
{
	CyU3PEpConfig_t epcfg;
	CyU3PPibClock_t pibclk =
	{
		.clkDiv = 2,
		.clkSrc = CY_U3P_SYS_CLK,
		.isDllEnable = CyFalse,
		.isHalfDiv = CyFalse
	};
	CyU3PDmaMultiChannelConfig_t dmac =
	{
		.size = CY_FX_UVC_STREAM_BUF_SIZE,
		.count = CY_FX_UVC_STREAM_BUF_COUNT,
		.validSckCount = 2,
		.prodSckId[0] = (CyU3PDmaSocketId_t)CY_U3P_PIB_SOCKET_0,
		.prodSckId[1] = (CyU3PDmaSocketId_t)CY_U3P_PIB_SOCKET_1,
		.consSckId[0] = (CyU3PDmaSocketId_t)(CY_U3P_UIB_SOCKET_CONS_0 | CY_FX_EP_VIDEO_CONS_SOCKET),
		.prodAvailCount = 0,
		.prodHeader = 12, // 12 byte UVC header to be added
		.prodFooter = 4, // 4 byte footer to compensate for the 12 byte header
		.consHeader = 0,
		.dmaMode = CY_U3P_DMA_MODE_BYTE,
		.notification = CY_U3P_DMA_CB_PROD_EVENT | CY_U3P_DMA_CB_CONS_EVENT,
		.cb = cb_dma_uvc_stream
	};

	// init p-port
	if(CyU3PPibInit(CyTrue, &pibclk) != CY_U3P_SUCCESS)
	{
		// error - fail
		CyU3PDeviceReset(CyFalse);
	}

	// init GPIF
	if(CyU3PGpifLoad((CyU3PGpifConfig_t*)&CyFxGpifConfig) != CY_U3P_SUCCESS)
	{
		// error - fail
		CyU3PDeviceReset(CyFalse);
	}
	CyU3PGpifRegisterSMIntrCallback(cb_gpif);

	// init USB
	if(CyU3PUsbStart() != CY_U3P_SUCCESS)
	{
		// error - fail
		CyU3PDeviceReset(CyFalse);
	}

	// USB callbacks
	CyU3PUsbRegisterSetupCallback(cb_uvc_setup, CyFalse);
	CyU3PUsbRegisterEventCallback(cb_usb_event);
	CyU3PUsbRegisterLPMRequestCallback(cb_usb_lpm);

	// USB descriptors
	CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSBDeviceDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSBDeviceDscrSS);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
	CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);

	// USB video streaming endpoint
	epcfg.enable = 1;
	epcfg.epType = CY_U3P_USB_EP_BULK;
	epcfg.pcktSize = CY_FX_EP_BULK_VIDEO_PKT_SIZE;
	epcfg.isoPkts = 1;
	epcfg.burstLen = 16;
	epcfg.streams = 0;
	if(CyU3PSetEpConfig(CY_FX_EP_BULK_VIDEO, &epcfg) != CY_U3P_SUCCESS)
	{
		// error - fail
		CyU3PDeviceReset(CyFalse);
	}

	// USB status IRQ endpoint
	epcfg.enable = 1;
	epcfg.epType = CY_U3P_USB_EP_INTR;
	epcfg.pcktSize = 64;
	epcfg.isoPkts = 0;
	epcfg.streams = 0;
	epcfg.burstLen = 1;
	if(CyU3PSetEpConfig(CY_FX_EP_CONTROL_STATUS, &epcfg) != CY_U3P_SUCCESS)
	{
		// error - fail
		CyU3PDeviceReset(CyFalse);
	}

	// enable USB
	if(CyU3PConnectState(CyTrue, CyTrue) != CY_U3P_SUCCESS)
	{
		// error - fail
		CyU3PDeviceReset(CyFalse);
	}

	// create a DMA manual channel; GPIF -> USB
	if(CyU3PDmaMultiChannelCreate(&dma_uvc_stream, CY_U3P_DMA_TYPE_MANUAL_MANY_TO_ONE, &dmac) != CY_U3P_SUCCESS)
	{
		// error - fail
		CyU3PDeviceReset(CyFalse);
	}
}

//
// UVC thread

static void uvc_thread(uint32_t unused)
{
	static uint32_t flag;

	// init UVC
	uvc_init();

	// event loop
	while(1)
	{
		if(CyU3PEventGet(&event_uvc, CY_FX_UVC_STREAM_ABORT_EVENT | CY_FX_UVC_STREAM_EVENT | CY_FX_USB_SUSPEND_EVENT_HANDLER, CYU3P_EVENT_OR_CLEAR, &flag, CYU3P_WAIT_FOREVER) == CY_U3P_SUCCESS)
		{
			if(flag & CY_FX_UVC_STREAM_EVENT)
			{
				uvc_active_state = CyTrue;
				uvc_enable();
			}

			if(flag & CY_FX_UVC_STREAM_ABORT_EVENT)
			{
				uvc_active_state = CyFalse;
				uvc_disable();
			}

			if(flag & CY_FX_USB_SUSPEND_EVENT_HANDLER)
			{
				CyU3PThreadSleep(5);
				do
				{
					static CyU3PUsbLinkPowerMode usb3mode;
					static uint16_t wakeReason;
					CyU3PReturnStatus_t ret;

					ret = CyU3PSysEnterSuspendMode(CY_U3P_SYS_USB_BUS_ACTVTY_WAKEUP_SRC, 0, &wakeReason);
					if(ret != CY_U3P_SUCCESS || CyU3PUsbGetSpeed() != CY_U3P_SUPER_SPEED)
						break;

					CyU3PThreadSleep(10);

					ret = CyU3PUsbGetLinkPowerState(&usb3mode);
					if(ret != CY_U3P_SUCCESS || usb3mode != CyU3PUsbLPM_U3)
						break;
				} while(1);
			}
		}
		CyU3PThreadRelinquish();
	}
}

//
// UVC EP0 thread

static void uvc_ep0_thread(uint32_t unused)
{
	static uint32_t flag;

	while(1)
	{
		if(CyU3PEventGet(&event_uvc, CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT | CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT, CYU3P_EVENT_OR_CLEAR, &flag, CYU3P_WAIT_FOREVER) == CY_U3P_SUCCESS)
		{
			if(flag & CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT)
			{
				switch(wIndex >> 8)
				{
					case CY_FX_UVC_PROCESSING_UNIT_ID:
						uvc_req_processing();
					break;
					case CY_FX_UVC_CAMERA_TERMINAL_ID:
						uvc_req_camera_terminal();
					break;
					case CY_FX_UVC_INTERFACE_CTRL:
						uvc_req_interface_ctrl();
					break;
					case CY_FX_UVC_EXTENSION_UNIT_ID:
						uvc_req_extension();
					break;
					default:
						CyU3PUsbStall(0, CyTrue, CyFalse);
					break;
				}
			}

			if(flag & CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT)
			{
				if(wIndex != CY_FX_UVC_STREAM_INTERFACE)
					CyU3PUsbStall(0, CyTrue, CyFalse);
				else
					uvc_req_streaming();
			}
		}
		CyU3PThreadRelinquish();
	}
}

//
// system init

void CyFxApplicationDefine()
{
	void *stack0;
	void *stack1;
	uint32_t ret;

	stack0 = CyU3PMemAlloc(UVC_APP_THREAD_STACK);
	stack1 = CyU3PMemAlloc(UVC_APP_EP0_THREAD_STACK);
	if(!stack0 || !stack1)
		goto error;

	if(CyU3PEventCreate(&event_uvc))
		goto error;

	ret = CyU3PThreadCreate(&thread_uvc,
		"30:UVC App Thread",           // thread ID and name
		uvc_thread,                   // thread entry function
		0,                             // input parameter to thread
		stack0,                        // pointer to the allocated thread stack
		UVC_APP_THREAD_STACK,          // thread stack size
		UVC_APP_THREAD_PRIORITY,       // thread priority
		UVC_APP_THREAD_PRIORITY,       // threshold value for thread pre-emption
		CYU3P_NO_TIME_SLICE,           // time slice for the application thread
		CYU3P_AUTO_START               // start the thread immediately
	);
	if(ret)
		goto error;

	ret = CyU3PThreadCreate(&thread_uvc_ep0,
		"31:UVC App EP0 Thread",       // thread ID and name
		uvc_ep0_thread,                // thread entry function
		0,                             // input parameter to thread
		stack1,                        // pointer to the allocated thread stack
		UVC_APP_EP0_THREAD_STACK,      // thread stack size
		UVC_APP_EP0_THREAD_PRIORITY,   // thread priority
		UVC_APP_EP0_THREAD_PRIORITY,   // threshold value for thread pre-emption
		CYU3P_NO_TIME_SLICE,           // time slice for the application thread
		CYU3P_AUTO_START               // start the thread immediately
	);
	if(ret)
		goto error;

	return;

error:
	CyU3PDeviceReset(CyFalse);
}

static uint32_t system_init()
{
	CyU3PSysClockConfig_t clockConfig =
	{
		.setSysClk400  = CyTrue,
		.cpuClkDiv     = 2,
		.dmaClkDiv     = 2,
		.mmioClkDiv    = 2,
		.useStandbyClk = CyTrue,
		.clkSrc        = CY_U3P_SYS_CLK
	};

	CyU3PIoMatrixConfig_t io_cfg =
	{
		.isDQ32Bit = CyTrue,
		.s0Mode = CyFalse,
		.s1Mode = CyFalse,
		.lppMode = CY_U3P_IO_MATRIX_LPP_DEFAULT,
		.gpioSimpleEn[0] = 0,
		.gpioSimpleEn[1] = 0,
		.gpioComplexEn[0] = 0,
		.gpioComplexEn[1] = 0,
		.useUart = CyTrue,
		.useI2C = CyTrue,
		.useI2S = CyFalse,
		.useSpi = CyFalse
	};

	if(CyU3PDeviceInit(&clockConfig) != CY_U3P_SUCCESS)
		return 1;

	if(CyU3PDeviceCacheControl(CyTrue, CyFalse, CyFalse) != CY_U3P_SUCCESS)
		return 1;

	if(CyU3PDeviceConfigureIOMatrix(&io_cfg) != CY_U3P_SUCCESS)
		return 1;

	return 0;
}

//
// main

int main()
{
	if(system_init())
		goto init_error;

	CyU3PKernelEntry();

	return 0;

init_error:
	CyU3PDeviceReset(CyFalse);
}

