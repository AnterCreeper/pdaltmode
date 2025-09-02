#ifndef __USBPD_H
#define __USBPD_H

#include <ch32x035_usbpd.h>
#include "usbbc.h"

#define GETBITS(x, n, m) ((x >> n) & ((1 << (m -n + 1)) - 1))

/* Power Data Object of Source Capability
    BIT[31:30] - 00: Fixed Supply;
    BIT29 - Dual-Role Power
    BIT28 - USB Suspend Supported

    BIT27 - Unconstrained Power
    BIT26 - USB Communications Capable
    BIT25 - Dual-Role Data

    BIT[24:22] - 000: Reserved
    BIT[21:20] - Peak Current
    BIT[19:10] - Voltage in 50mV units
    BIT[9:0] - Maximum Current in 10mA units
*/
const uint32_t SRCCAP_5V1A5 = 0x2c019096;

typedef enum {
    STA_VDM_IDLE = 0,
    STA_DISC_IDENT,
    STA_DISC_SVID,
    STA_DISC_MODE,
    STA_ENTER_MODE,
    STA_DP_S_UPDATE,
    STA_DP_CONFIG,
    STA_MPD_HPD,
    STA_MPD_IRQ
} VDM_STATUS;

typedef enum {
    DISCONNECTED = 0,
    CONNECTED,
} PHY_STATUS;

typedef enum {
    ROLE_UFP = 0,
    ROLE_DFP,
} PD_ROLE;

typedef enum {
    ROLE_SNK = 0,
    ROLE_SRC,
} PR_ROLE;

typedef struct DP_STATUS {
    uint8_t Pos;
    uint8_t Enabled;
    uint8_t DFP_Pin;
    uint8_t UFP_Pin;
    uint32_t Link; //Type-C Link Status
    uint8_t Trained; //If Link Trained
    uint8_t Receptable;
} dp_status_t;

typedef struct PD_FSM {
    PD_ROLE PD_Role;
    PR_ROLE PR_Role;
    uint8_t Msg_ID;
    uint8_t Det_Cnt;
    uint8_t Cap_Cnt;
    CC_STATUS Status;
    PHY_STATUS Connect;
    VDM_STATUS VDM_Status;
    void* Rx_Buf;
    void* Tx_Buf;
    int IRQ_ret;
    void* IRQ_func;
    timer_t Timer;
    bc_state_t BC_Ctl;
    dp_status_t DP_Status;
} pd_state_t;

#define DEF_CMDTYPE_INIT            0x0
#define DEF_CMDTYPE_ACK             0x1
#define DEF_CMDTYPE_NAK             0x2
#define DEF_CMDTYPE_BUSY            0x3

#define DEF_SVID_EOF                0x0000
#define DEF_SVID_DEFAULT            0xff00
#define DEF_SVID_DISPLAYPORT        0xff01

#define PD_PARSE_MSGID(x)           GETBITS(((uint8_t*)x)[1], 1, 3)
#define PD_PARSE_TYPE(x)            GETBITS(((uint8_t*)x)[0], 0, 4)
#define PD_PARSE_SIZE(x)            GETBITS(((uint8_t*)x)[1], 4, 6)

/* Vender Defined Message
    BIT[31:16] - Standard or Vendor ID, 0xFF00 for default and 0xFF01 for DisplayPort
    BIT15 - VDM Type, 1 = Structured VDM
    BIT[14:13] - Structured VDM Version, 00 = Version 1.0
    BIT[12:11] - 00: Reserved

    BIT[10:8] - Object Position
        000 = Reserved
        001 ¨C 110 = Index into the list of Vendor Defined Objects (VDOs) to identify the desired
        111 = Exit all Modes (equivalent of a power-on reset). Shall not be used with the Enter
            Mode command.

    BIT[7:6] - Command Type
        00 = Initiator
        01 = Responder ACK
        10 = Responder NAK
        11 = Responder BUSY

    BIT5 - Reserved
    BIT[4:0] - Command
*/

#define PD_PARSE_CMD(x)             GETBITS(((uint8_t*)x)[2], 0, 4)
#define PD_PARSE_CMDTYPE(x)         GETBITS(((uint8_t*)x)[2], 6, 7)
#define PD_PARSE_SVID(x)            ((uint16_t*)x)[2]

/* DisplayPort Capability
    BIT[31:24] - Reserved
    BIT[23:16] - UFP_D Pin Assignments supported
    BIT[15:8] - DFP_D Pin Assignments supported
    BIT7 - USB 2.0 signaling not required when set
    BIT6 - Receptacle indication, 0: plug; 1: receptacle;
    BIT[5:2] - Physical Signaling
    BIT[1:0] - Port capability
        00 = Reserved
        01 = UFP_D capable (including Branch Device)
        10 = DFP_D capable (including Branch Device)
        11 = Both DFP_D and UFP_D capable
*/

#define PD_PARSE_DP_CAP(x)          GETBITS(x, 0, 1)
#define PD_PARSE_DP_CAP_UFP_D(x)    GETBITS(x, 0, 0)
#define PD_PARSE_DP_CAP_DFP_D(x)    GETBITS(x, 1, 1)
#define PD_PARSE_DP_SIG(x)          GETBITS(x, 2, 2)
#define PD_PARSE_RECEPTACLE(x)      GETBITS(x, 6, 6)
#define PD_PARSE_DFP_PIN(x)         GETBITS(x, 8, 15)
#define PD_PARSE_UFP_PIN(x)         GETBITS(x, 16, 23)

typedef struct {
    uint32_t id_header;
    uint32_t cert_state;
    uint32_t product;
    uint32_t product_type;
} pd_vdo_ident_t;

#define PD_PARSE_USB_VID(x)         GETBITS(x, 0, 15)
#define PD_PARSE_MODAL(x)           GETBITS(x, 26, 26)
#define PD_PARSE_PROD_TYPE(x)       GETBITS(x, 27, 29)
#define PD_PARSE_USB_DEVICE(x)      GETBITS(x, 30, 30)
#define PD_PARSE_USB_HOST(x)        GETBITS(x, 31, 31)
#define PD_PARSE_PROD_ID(x)         GETBITS(x, 16, 31)
#define PD_PARSE_PROD_BCD(x)        GETBITS(x, 0, 15)

#define PD_PARSE_USBSS_SIG(x)       GETBITS(x, 0, 2)
#define PD_PARSE_USB_VBUS(x)        GETBITS(x, 3, 3)
#define PD_PARSE_USB_VCONN(x)       GETBITS(x, 4, 4)
#define PD_PARSE_VCONN_WATT(x)      GETBITS(x, 5, 7)
#define PD_PARSE_USBSS_DIR(x)       GETBITS(x, 8, 11)
#define PD_PARSE_FW_VER(x)          GETBITS(x, 24, 27)
#define PD_PARSE_HW_VER(x)          GETBITS(x, 28, 31)

/* DisplayPort Status
    BIT[31:9] - Reserved

    BIT8 - IRQ_HPD, 0 for DFP_D
    BIT7 - HPD State, 0 for DFP_D

    BIT6 - Exit DisplayPort Mode request, 0 for DFP_U
    BIT5 - USB Configuration request, 0 for DFP_U
    BIT4 - Multi-Function preferred, 0 for DFP_U
    BIT3 - Enabled, 0 for DFP_U
    BIT2 - Power Low, 0 for DFP_U

    BIT[1:0] DFP_D/UFP_D Connected
        00 = neither DFP_D nor UFP_D connected or adapter is disabled
        01 = DFP_D Connected
        10 = UFP_D Connected
        11 = Both DFP_D and UFP_D Connected
*/

#define PD_PARSE_DP_STA(x)          GETBITS(x, 0, 1)
#define PD_PARSE_DP_STA_UFP_D(x)    GETBITS(x, 1, 1)
#define PD_PARSE_DP_STA_DFP_D(x)    GETBITS(x, 0, 0)
#define PD_PARSE_DP_LP(x)           GETBITS(x, 2, 2)
#define PD_PARSE_DP_ENABLED(x)      GETBITS(x, 3, 3)
#define PD_PARSE_DP_MUL(x)          GETBITS(x, 4, 4)
#define PD_PARSE_DP_USB(x)          GETBITS(x, 5, 5)
#define PD_PARSE_DP_EXIT(x)         GETBITS(x, 6, 6)
#define PD_PARSE_DP_HPD(x)          GETBITS(x, 7, 7)
#define PD_PARSE_DP_IRQ(x)          GETBITS(x, 8, 8)

#endif