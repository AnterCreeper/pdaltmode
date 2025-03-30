#ifndef __MPD_H
#define __MPD_H

#define BIT(x)          (1 << (x))

#define MPD_DEV_ADR     0x68    //7b1101000
#define MPD_REFCLK      REF_FREQ_26M

#define MPD_BW          DP_SRCCTRL_BW27
#define MPD_SWG         DP_SWG_0P8
#define MPD_PRE         DP_PRE_0
#define MPD_2LANE

#define MPD_TEST
#define MPD_TEST_COLOR  0xffffff
#define MPD_TEST_MODE   COLOR_CHECKER

#define MPD_PXLPARAM    0x01130444

//Video Format
#define MPD_DP_BPP      OPXLFMT_RGB888
#define MPD_DPI_POL     VS_POL_ACTIVE_LOW | HS_POL_ACTIVE_LOW | DE_POL_ACTIVE_HIGH
#define MPD_DPI_FMT     SUB_CFG_TYPE_CONFIG1
#define MPD_DPI_BPP     DPI_BPP_RGB888

//Timing Variable
#define MPD_VSDELAY         0
#define MPD_DP_VIDMNGEN0    0x45ED
#define MPD_DP_VIDMNGEN1    0x8025

//HTIM01, HTIM02, VTIM01, VTIM02
static const uint32_t rgb_timing[4] = {0x0094002C, 0x00580780, 0x00240005, 0x00040438};
//DP0_VidSyncDly, DP0_TotalVal, DP0_StartVal, DP0_ActiveVal, DP0_SyncVal
static const uint32_t dp_timing[5] = {0x00160840, 0x04650898, 0x002900C0, 0x04380780, 0x8005802C};


/* Internal Macro */

#ifdef MPD_2LANE
#define MPD_LANES       2
#else
#define MPD_LANES       1
#endif

//#define MPD_IIC_DBG

#define DPIPXLFMT		0x0440
#define VS_POL_ACTIVE_LOW		(1 << 10)
#define VS_POL_ACTIVE_HIGH		(0 << 10)
#define HS_POL_ACTIVE_LOW		(1 << 9)
#define HS_POL_ACTIVE_HIGH		(0 << 9)
#define DE_POL_ACTIVE_LOW		(1 << 8)
#define DE_POL_ACTIVE_HIGH		(0 << 8)
#define SUB_CFG_TYPE_CONFIG1	(0 << 2) /* LSB aligned */
#define SUB_CFG_TYPE_CONFIG2	(1 << 2) /* Loosely Packed */
#define SUB_CFG_TYPE_CONFIG3	(2 << 2) /* LSB aligned 8-bit */
#define DPI_BPP_RGB888			(0 << 0)
#define DPI_BPP_RGB666			(1 << 0)
#define DPI_BPP_RGB565			(2 << 0)

#define VPCTRL0			0x0450
#define VSDELAY(x)			(x << 20)
#define OPXLFMT_RGB666		(0 << 8)
#define OPXLFMT_RGB888		(1 << 8)
#define FRMSYNC_DISABLED	(0 << 4) /* Video Timing Gen Disabled */
#define FRMSYNC_ENABLED		(1 << 4) /* Video Timing Gen Enabled */
#define MSF_DISABLED		(0 << 0) /* Magic Square FRC disabled */
#define MSF_ENABLED			(1 << 0) /* Magic Square FRC enabled */

#define VP_TIM			0x0454
#define VFUEN0			0x0464

#define TC_IDREG		0x0500
#define SYSBOOT			0x0504
#define SYSSTAT			0x0508
#define SYSRSTENB		0x050c
#define SYSCTRL			0x0510
#define DP0_OSCLK_AVALIABLE     (1 << 11)
#define DP0_AUDSRC_NO_INPUT		(0 << 3)
#define DP0_AUDSRC_I2S_RX		(1 << 3)
#define DP0_VIDSRC_NO_INPUT		(0 << 0)
#define DP0_VIDSRC_DSI_RX		(1 << 0)
#define DP0_VIDSRC_DPI_RX		(2 << 0)
#define DP0_VIDSRC_COLOR_BAR	(3 << 0)

#define DP0CTL			0x0600
#define VID_MN_GEN		BIT(6)      /* Auto-generate M/N values */
#define EF_EN			BIT(5)      /* Enable Enhanced Framing */
#define AUD_EN			BIT(2)      /* Video transmission enable */
#define VID_EN			BIT(1)      /* Video transmission enable */
#define DP_EN			BIT(0)      /* Enable DPTX function */

#define DP0_VIDMNGEN0	0x0610	    /* DP0 Video Force M Value Register */
#define DP0_VIDMNGEN1	0x0614	    /* DP0 Video Force N Value Register */
#define DP0_VMNGENSTATUS	0x0618	/* DP0 Video Current M Value Register */

#define DP0_SECSAMPLE	0x0640
#define DP0_TIM         0x0644

#define DP0_MISC		0x0658
#define MAX_TU_SYMBOL(x)    ((x) << 23)
#define TU_SIZE(x)		    ((x) << 16)
#define FMT_RGB         (0 << 1)
#define FMT_YUV422      (1 << 1)
#define FMT_YUV444      (2 << 1)
#if(MPD_DP_BPP == OPXLFMT_RGB888)
#define BPC             (1 << 5) //BPC_8
#else
#define BPC				(0 << 5) //BPC_6
#endif

#define DP0_AUXCFG0		0x0660
#define DP0_AUXCFG1		0x0664
#define DP0_AUXADDR		0x0668

#define DP0_AUXWDATA    0x066c
#define DP0_AUXRDATA    0x067c
#define DP0_AUXSTATUS	0x068c

#define DP0_LTSTAT		    0x06d0
#define LT_LOOPDONE			BIT(13)

#define DP0_SNKLTCHGREQ		0x06d4
#define DP0_LTLOOPCTRL		0x06d8

#define DP0_SNKLTCTRL		0x06e4

#define DP0_TPATDAT0		0x06e8	    /* DP0 Test Pattern bits 29 to 0 */
#define DP0_TPATDAT1		0x06ec	    /* DP0 Test Pattern bits 59 to 30 */
#define DP0_TPATDAT2		0x06f0	    /* DP0 Test Pattern bits 89 to 60 */
#define DP0_TPATDAT3		0x06f4	    /* DP0 Test Pattern bits 119 to 90 */

#define DP0_SRCCTRL		0x06a0
#define DP1_SRCCTRL		0x07a0
#define DP_PRE_0        0x0
#define DP_PRE_3P5      0x1
#define DP_PRE_6        0x2
#define DP_SWG_0P4      0x0
#define DP_SWG_0P6      0x1
#define DP_SWG_0P8      0x2
#define DP_SWG_1P2      0x3
#define DP_SRCCTRL_PRE1(x)		((x) << 28)
#define DP_SRCCTRL_SWG1(x)		((x) << 24)
#define DP_SRCCTRL_PRE0(x)		((x) << 20)
#define DP_SRCCTRL_SWG0(x)		((x) << 16)
#define DP_SRCCTRL_SCRMBLDIS	BIT(13)
#define DP_SRCCTRL_EN810B		BIT(12)
#define DP_SRCCTRL_NOTP		    (0 << 8)
#define DP_SRCCTRL_TP1			(1 << 8)
#define DP_SRCCTRL_TP2			(2 << 8)
#define DP_SRCCTRL_LANESKEW	    BIT(7)
#define DP_SRCCTRL_SSCG		    BIT(3)
#define DP_SRCCTRL_LANES_1		(0 << 2)
#define DP_SRCCTRL_LANES_2		(1 << 2)
#define DP_SRCCTRL_BW27		    (1 << 1)
#define DP_SRCCTRL_BW162		(0 << 1)
#define DP_SRCCTRL_AUTOCORRECT	BIT(0)

#define DP_PHY_CTRL		0x0800
#define DP_PHY_RST		BIT(28)     /* DP PHY Global Soft Reset */
#define BGREN			BIT(25)     /* AUX PHY BGR Enable */
#define PWR_SW_EN		BIT(24)     /* PHY Power Switch Enable */
#define PHY_M1_RST		BIT(12)     /* Reset PHY1 Main Channel */
#define PHY_RDY			BIT(16)     /* PHY Main Channels Ready */
#define PHY_M0_RST		BIT(8)      /* Reset PHY0 Main Channel */
#define PHY_2LANE		BIT(2)      /* PHY Enable 2 lanes */
#define PHY_A0_EN		BIT(1)      /* PHY Aux Channel0 Enable */
#define PHY_M0_EN		BIT(0)      /* PHY Main Channel0 Enable */

#define DP0_PLLCTRL     0x0900
#define DP1_PLLCTRL		0x0904
#define PXL_PLLCTRL		0x0908
#define PLLUPDATE		BIT(2)
#define PLLBYP			BIT(1)
#define PLLEN			BIT(0)

#define PXL_PLLPARAM	0x0914
#define IN_SEL_REFCLK	(0 << 14)

#define SYS_PLLPARAM	0x0918
#define REF_FREQ_38M4	(0 << 8)    /* 38.4 MHz */
#define REF_FREQ_19M2	(1 << 8)    /* 19.2 MHz */
#define REF_FREQ_26M	(2 << 8)    /* 26 MHz */
#define REF_FREQ_13M	(3 << 8)    /* 13 MHz */
#define LSCLK_DIV_1		(0 << 0)
#define LSCLK_DIV_2		(1 << 0)

#define TSTCTL			0x0a00
#define ENI2CFILTER		BIT(4)
#define SOLID_COLOR     1
#define COLOR_BAR   	2
#define COLOR_CHECKER   3

#define DPCD_REV                    0x00000
#define DPCD_LINK_BW_SET            0x00100
#define DPCD_TRAINING_PATTERN_SET   0x00102
#define DPCD_TRAINING_SET           0x00103
#define DPCD_ML_CODING_SET          0x00108
#define DPCD_LANE_STATUS		    0x00202
#define DPCD_ADJUST_REQUEST         0x00206
#define DPCD_SYMBOL_ERR_CNT         0x00210
#define DPCD_EDP_CONFIGURATION_SET  0x0010a
#define DPCD_DOWNSPREAD_CTRL        0x00107

#define DPCD_SET_ANSI_8B10B         BIT(0)
#define DPCD_SCRAMBLING_DISABLE     BIT(5)
#define DPCD_TRAINING_PATTERN_1     1
#define DPCD_TRAINING_PATTERN_2     2
#if(MPD_BW == DP_SRCCTRL_BW27)
#define DPCD_BW     10
#else
#define DPCD_BW     6
#endif
#define DPCD_ENHANCED_FRAME_EN      BIT(15)
#define DPCD_LANES(x)   ((x) << 8)

void MPD_Init();
int MPD_CfgLink();
void MPD_CfgVideo();
void MPD_CfgTest();
void MPD_Disable();

#endif