/*
 ## Cypress FX3 Camera Kit Source file (cyfxuvcdscr.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2012,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/*
 * This file contains the USB descriptors for the FX3 HD 720p camera kit
 * application.
 */

#include "uvc.h"

/* Standard Device Descriptor */
const uint8_t CyFxUSBDeviceDscr[] =
{
	0x12,                           /* Descriptor Size */
	CY_U3P_USB_DEVICE_DESCR,        /* Device Descriptor Type */
	0x10,0x02,                      /* USB 2.0 */
	0xEF,                           /* Device Class */
	0x02,                           /* Device Sub-class */
	0x01,                           /* Device protocol */
	0x40,                           /* Maxpacket size for EP0 : 64 bytes */
	0xB4,0x04,                      /* Vendor ID */
	0xF8,0x00,                      /* Product ID */
	0x00,0x00,                      /* Device release number */
	0x01,                           /* Manufacture string index */
	0x02,                           /* Product string index */
	0x00,                           /* Serial number string index */
	0x01                            /* Number of configurations */
};

/* Device Descriptor for SS */
const uint8_t CyFxUSBDeviceDscrSS[] =
{
	0x12,                           /* Descriptor Size */
	CY_U3P_USB_DEVICE_DESCR,        /* Device Descriptor Type */
	0x10,0x03,                      /* USB 3.10 */
	0xEF,                           /* Device Class */
	0x02,                           /* Device Sub-class */
	0x01,                           /* Device protocol */
	0x09,                           /* Maxpacket size for EP0 : 2^9 Bytes */
	0xB4,0x04,                      /* Vendor ID */
	0xF9,0x00,                      /* Product ID */
	0x00,0x00,                      /* Device release number */
	0x01,                           /* Manufacture string index */
	0x02,                           /* Product string index */
	0x00,                           /* Serial number string index */
	0x01                            /* Number of configurations */
};

/* Standard Device Qualifier Descriptor */
const uint8_t CyFxUSBDeviceQualDscr[] =
{
	0x0A,                           /* Descriptor Size */
	CY_U3P_USB_DEVQUAL_DESCR,       /* Device Qualifier Descriptor Type */
	0x00,0x02,                      /* USB 2.0 */
	0xEF,                           /* Device Class */
	0x02,                           /* Device Sub-class */
	0x01,                           /* Device protocol */
	0x40,                           /* Maxpacket size for EP0 : 64 bytes */
	0x01,                           /* Number of configurations */
	0x00                            /* Reserved */
};

/* Standard Full Speed Configuration Descriptor */
const uint8_t CyFxUSBFSConfigDscr[] =
{
	/* Configuration Descriptor Type */
	0x09,                           /* Descriptor Size */
	CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
	0x09,0x00,                      /* Length of this descriptor and all sub descriptors */
	0x00,                           /* Number of interfaces */
	0x01,                           /* Configuration number */
	0x00,                           /* COnfiguration string index */
	0xC0,                           /* Config characteristics */
	0x00,                           /* Max power consumption of device (in 2mA unit) */
};

/* Standard High Speed Configuration Descriptor */
const uint8_t CyFxUSBHSConfigDscr[] =
{
	/* Configuration Descriptor Type */
	0x09,                           /* Descriptor Size */
	CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
	0x09,0x00,                      /* Length of this descriptor and all sub descriptors */
	0x00,                           /* Number of interfaces */
	0x01,                           /* Configuration number */
	0x00,                           /* COnfiguration string index */
	0xC0,                           /* Config characteristics */
	0x00,                           /* Max power consumption of device (in 2mA unit) */
};

/* BOS for SS */

#define CY_FX_BOS_DSCR_TYPE             15
#define CY_FX_DEVICE_CAPB_DSCR_TYPE     16
#define CY_FX_SS_EP_COMPN_DSCR_TYPE     48

/* Device Capability Type Codes */
#define CY_FX_WIRELESS_USB_CAPB_TYPE    1
#define CY_FX_USB2_EXTN_CAPB_TYPE       2
#define CY_FX_SS_USB_CAPB_TYPE          3
#define CY_FX_CONTAINER_ID_CAPBD_TYPE   4

const uint8_t CyFxUSBBOSDscr[] =
{
        0x05,                           /* Descriptor Size */
        CY_FX_BOS_DSCR_TYPE,            /* Device Descriptor Type */
        0x16,0x00,                      /* Length of this descriptor and all sub descriptors */
        0x02,                           /* Number of device capability descriptors */

        /* USB 2.0 Extension */
        0x07,                           /* Descriptor Size */
        CY_FX_DEVICE_CAPB_DSCR_TYPE,    /* Device Capability Type descriptor */
        CY_FX_USB2_EXTN_CAPB_TYPE,      /* USB 2.0 Extension Capability Type */
        0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                           Baseline BESL=400 us, Deep BESL=1000 us. */

        /* SuperSpeed Device Capability */
        0x0A,                           /* Descriptor Size */
        CY_FX_DEVICE_CAPB_DSCR_TYPE,    /* Device Capability Type descriptor */
        CY_FX_SS_USB_CAPB_TYPE,         /* SuperSpeed Device Capability Type */
        0x00,                           /* Supported device level features  */
        0x0E,0x00,                      /* Speeds Supported by the device : SS, HS and FS */
        0x03,                           /* Functionality support */
        0x00,                           /* U1 Device Exit Latency */
        0x00,0x00                       /* U2 Device Exit Latency */
};

/* Super Speed Configuration Descriptor */
const uint8_t CyFxUSBSSConfigDscr[] =
{
	/* Configuration Descriptor Type */
	0x09,                           /* Descriptor Size */
	CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
	0xBE,0x00,                      /* Length of this descriptor and all sub descriptors */
	0x02,                           /* Number of interfaces */
	0x01,                           /* Configuration number */
	0x00,                           /* Configuration string index */
	0xC0,                           /* Config characteristics */
	0x00,                           /* Max power consumption of device (in 8mA unit) */

	/* Interface Association Descriptor */
	0x08,                           /* Descriptor Size */
	CY_FX_INTF_ASSN_DSCR_TYPE,      /* Interface Association Descr Type: 11 */
	0x00,                           /* I/f number of first VideoControl i/f */
	0x02,                           /* Number of Video i/f */
	0x0E,                           /* CC_VIDEO : Video i/f class code */
	0x03,                           /* SC_VIDEO_INTERFACE_COLLECTION : Subclass code */
	0x00,                           /* Protocol : Not used */
	0x02,                           /* String desc index for interface */

	/* Standard Video Control Interface Descriptor */
	0x09,                           /* Descriptor size */
	CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
	0x00,                           /* Interface number */
	0x00,                           /* Alternate setting number */
	0x01,                           /* Number of end points */
	0x0E,                           /* CC_VIDEO : Interface class */
	0x01,                           /* CC_VIDEOCONTROL : Interface sub class */
	0x00,                           /* Interface protocol code */
	0x02,                           /* Interface descriptor string index */

	/* Class specific VC Interface Header Descriptor */
	0x0D,                           /* Descriptor size */
	0x24,                           /* Class Specific I/f Header Descriptor type */
	0x01,                           /* Descriptor Sub type : VC_HEADER */
	0x10, 0x01,                     /* Revision of UVC class spec: 1.1 - Minimum version required
		                           for USB Compliance. Not supported on Windows XP*/
	0x51, 0x00,                     /* Total Size of class specific descriptors (till Output terminal) */
	0x00,0x6C,0xDC,0x02,            /* Clock frequency : 48MHz(Deprecated) */
	0x01,                           /* Number of streaming interfaces */
	0x01,                           /* Video streaming I/f 1 belongs to VC i/f */

	/* Input (Camera) Terminal Descriptor */
	0x12,                           /* Descriptor size */
	0x24,                           /* Class specific interface desc type */
	0x02,                           /* Input Terminal Descriptor type */
	0x01,                           /* ID of this terminal */
	0x01,0x02,                      /* Camera terminal type */
	0x00,                           /* No association terminal */
	0x00,                           /* String desc index : Not used */
	0x00,0x00,                      /* No optical zoom supported */
	0x00,0x00,                      /* No optical zoom supported */
	0x00,0x00,                      /* No optical zoom supported */
	0x03,                           /* Size of controls field for this terminal : 3 bytes */
		                        /* A bit set to 1 in the bmControls field indicates that
		                         * the mentioned Control is supported for the video stream.
		                         * D0: Scanning Mode
		                         * D1: Auto-Exposure Mode
		                         * D2: Auto-Exposure Priority
		                         * D3: Exposure Time (Absolute)
		                         * D4: Exposure Time (Relative)
		                         * D5: Focus (Absolute)
		                         * D6: Focus (Relative)
		                         * D7: Iris (Absolute)
		                         * D8: Iris (Relative)
		                         * D9: Zoom (Absolute)
		                         * D10: Zoom (Relative)
		                         * D11: PanTilt (Absolute)
		                         * D12: PanTilt (Relative)
		                         * D13: Roll (Absolute)
		                         * D14: Roll (Relative)
		                         * D15: Reserved
		                         * D16: Reserved
		                         * D17: Focus, Auto
		                         * D18: Privacy
		                         * D19: Focus, Simple
		                         * D20: Window
		                         * D21: Region of Interest
		                         * D22 - D23: Reserved, set to zero
		                         */
	0x00,0x00,0x00,                 /* bmControls field of camera terminal: No controls supported */

	/* Processing Unit Descriptor */
	0x0D,                           /* Descriptor size */
	0x24,                           /* Class specific interface desc type */
	0x05,                           /* Processing Unit Descriptor type */
	0x02,                           /* ID of this terminal */
	0x01,                           /* Source ID : 1 : Connected to input terminal */
	0x00,0x40,                      /* Digital multiplier */
	0x03,                           /* Size of controls field for this terminal : 3 bytes */
	0x00,0x00,0x00,                 /* bmControls field of processing unit: Brightness control supported */
	0x00,                           /* String desc index : Not used */
	0x00,                           /* Analog Video Standards Supported: None */

	/* Output Terminal Descriptor */
	0x09,                           /* Descriptor size */
	0x24,                           /* Class specific interface desc type */
	0x03,                           /* Output Terminal Descriptor type */
	0x04,                           /* ID of this terminal */
	0x01,0x01,                      /* USB Streaming terminal type */
	0x00,                           /* No association terminal */
	0x03,                           /* Source ID : 3 : Connected to Extn Unit */
	0x00,                           /* String desc index : Not used */

	/* Video Control Status Interrupt Endpoint Descriptor */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
	CY_FX_EP_CONTROL_STATUS,        /* Endpoint address and description */
	CY_U3P_USB_EP_INTR,             /* Interrupt End point Type */
	0x40,0x00,                      /* Max packet size = 64 bytes */
	0x01,                           /* Servicing interval */

	/* Super Speed Endpoint Companion Descriptor */
	0x06,                           /* Descriptor size */
	CY_U3P_SS_EP_COMPN_DESCR,       /* SS Endpoint Companion Descriptor Type */
	0x00,                           /* Max no. of packets in a Burst : 1 */
	0x00,                           /* Attribute: N.A. */
	0x00,                           /* Bytes per interval:1024 */
	0x04,

	/* Class Specific Interrupt Endpoint Descriptor */
	0x05,                           /* Descriptor size */
	0x25,                           /* Class Specific Endpoint Descriptor Type */
	CY_U3P_USB_EP_INTR,             /* End point Sub Type */
	0x40,0x00,                      /* Max packet size = 64 bytes */


	/* Standard Video Streaming Interface Descriptor (Alternate Setting 0) */
	0x09,                           /* Descriptor size */
	CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
	0x01,                           /* Interface number */
	0x00,                           /* Alternate setting number */
	0x01,                           /* Number of end points */
	0x0E,                           /* Interface class : CC_VIDEO */
	0x02,                           /* Interface sub class : CC_VIDEOSTREAMING */
	0x00,                           /* Interface protocol code : Undefined */
	0x00,                           /* Interface descriptor string index */

	/* Class-specific Video Streaming Input Header Descriptor */
	0x0E,                           /* Descriptor size */
	0x24,                           /* Class-specific VS I/f Type */
	0x01,                           /* Descriptotor Subtype : Input Header */
	0x01,                           /* 1 format desciptor follows */
	0x47,0x00,                      /* Total size of Class specific VS descr */
	CY_FX_EP_BULK_VIDEO,            /* EP address for BULK video data */
	0x00,                           /* No dynamic format change supported */
	0x04,                           /* Output terminal ID : 4 */
	0x01,                           /* Still image capture method 1 supported */
	0x00,                           /* Hardware trigger NOT supported */
	0x00,                           /* Hardware to initiate still image capture NOT supported */
	0x01,                           /* Size of controls field : 1 byte */
	0x00,                           /* D2 : Compression quality supported */

	/* Class specific Uncompressed VS format descriptor */
	0x1B,                           /* Descriptor size */
	0x24,                           /* Class-specific VS I/f Type */
	0x04,                           /* Subtype : uncompressed format I/F */
	0x01,                           /* Format desciptor index */
	0x01,                           /* Number of frame descriptor followed */
	0x7D, 0xEB, 0x36, 0xE4,         /* MEDIASUBTYPE_RGB888 GUID: E436EB7D-524F-11CE-9F53-0020AF0BA770 */
	0x4F, 0x52, 0xCE, 0x11,
	0x9F, 0x53, 0x00, 0x20,
	0xAF, 0x0B, 0xA7, 0x70,
	0x18,                           /* Number of bits per pixel: 24 */
	0x01,                           /* Optimum Frame Index for this stream: 1 */
	0x05,                           /* X dimension of the picture aspect ratio; Non-interlaced */
	0x03,                           /* Y dimension of the pictuer aspect ratio: Non-interlaced */
	0x00,                           /* Interlace Flags: Progressive scanning, no interlace */
	0x00,                           /* duplication of the video stream restriction: 0 - no restriction */

	/* Class specific Uncompressed VS frame descriptor */
	0x1E,                           /* Descriptor size */
	0x24,                           /* Descriptor type */
	0x05,                           /* Subtype: uncompressed frame I/F */
	0x01,                           /* Frame Descriptor Index */
	0x01,                           /* Still image capture method 1 supported */
	0x20, 0x03,                     /* Image width: 800 px */
	0xE0, 0x01,                     /* Image height: 480 px */
	0x00,0x00,0x00,0x18,            /* Min bit rate bits/s. */
	0x00,0x00,0x00,0x20,            /* Max bit rate bits/s. */
	0x00,0x94,0x11,0x00,            /* Frame size: 800*480*3 */
	0xC8,0x77,0x03,0x00,            /* Frame rate (interval) */
	0x01,
	0xC8,0x77,0x03,0x00,

	/* Endpoint Descriptor for BULK Streaming Video Data */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
	CY_FX_EP_BULK_VIDEO,            /* Endpoint address and description */
	CY_U3P_USB_EP_BULK,             /* BULK End point */
	CY_FX_EP_BULK_VIDEO_PKT_SIZE_L, /* EP MaxPcktSize: 1024B */
	CY_FX_EP_BULK_VIDEO_PKT_SIZE_H, /* EP MaxPcktSize: 1024B */
	0x00,                           /* Servicing interval for data transfers */

	/* Super Speed Endpoint Companion Descriptor */
	0x06,                           /* Descriptor size */
	CY_U3P_SS_EP_COMPN_DESCR,       /* SS Endpoint Companion Descriptor Type */
	0x0F,                           /* Max number of packets per burst: 16 */
	0x00,                           /* Attribute: Streams not defined */
	0x00,                           /* No meaning for bulk */
	0x00
};

/* Standard Language ID String Descriptor */
const uint8_t CyFxUSBStringLangIDDscr[] =
{
	0x04,                           /* Descriptor Size */
	CY_U3P_USB_STRING_DESCR,        /* Device Descriptor Type */
	0x09,0x04                       /* Language ID supported */
};

/* Standard Manufacturer String Descriptor */
const uint8_t CyFxUSBManufactureDscr[] =
{
	0x10,                           /* Descriptor Size */
	CY_U3P_USB_STRING_DESCR,        /* Device Descriptor Type */
	'C',0x00,
	'y',0x00,
	'p',0x00,
	'r',0x00,
	'e',0x00,
	's',0x00,
	's',0x00
};

/* Standard Product String Descriptor */
const uint8_t CyFxUSBProductDscr[] =
{
	0x10,                           /* Descriptor Size */
	CY_U3P_USB_STRING_DESCR,        /* Device Descriptor Type */
	'F',0x00,
	'X',0x00,
	'3',0x00,
	' ',0x00,
	'U',0x00,
	'V',0x00,
	'C',0x00
};

