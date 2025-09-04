#ifndef __MPD_H
#define __MPD_H

#define BIT(x)          (1 << (x))

#define MPD_DEV_ADR     0x68    //7b1101000
#define MPD_REFCLK      REF_FREQ_26M

//Phy
#define MPD_2LANE
#define MPD_BW          DP_SRCCTRL_BW27
#define MPD_SWG         DP_SWG_0P4
#define MPD_PRE         DP_PRE_0

//Test Pattern
//#define MPD_TEST        COLOR_BAR
#define MPD_TEST_COLOR  0x114514
#define MPD_PXLPARAM    0x01130111

//Video Format
#define MPD_DP_BPP      OPXLFMT_RGB888
#define MPD_DPI_POL     VS_POL_ACTIVE_LOW | HS_POL_ACTIVE_LOW | DE_POL_ACTIVE_HIGH
#define MPD_DPI_FMT     SUB_CFG_TYPE_CONFIG1
#define MPD_DPI_BPP     DPI_BPP_RGB888

//Video Timing Variable
#define MPD_HPW         44
#define MPD_HBPR        148
#define MPD_Hactive     1920
#define MPD_HFPR        88
#define MPD_VPW         5
#define MPD_VBPR        36
#define MPD_Vactive     1080
#define MPD_VFPR        4

//Audio
#define MPD_AUDIO
#define MPD_AUDIO_FS        MPD_AUD_FS48
#define MPD_AUDIO_WIDTH     MPD_AUD_WIDTH16
#define MPD_AUDIO_FMT       MPD_AUD_FMT_STD

/* Internal Macro */

#ifdef MPD_AUDIO
#define MPD_AUDSRC   DP0_AUDSRC_I2S_RX
#else
#define MPD_AUDSRC   DP0_AUDSRC_NO_INPUT
#endif

#ifdef MPD_2LANE
#define MPD_LANES       2
#else
#define MPD_LANES       1
#endif

#ifdef MPD_TEST
#define MPD_TEST_MODE   MPD_TEST
#else
#define MPD_TEST_MODE   0
#endif

#define INTCTL_G    0x0560
#define INTSTS_G    0x0564
#define INT_SYSERR  BIT(16)

#define DPIPXLFMT   0x0440
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

#define VPCTRL0		0x0450
#define VSDELAY(x)	((x) << 20)
#define OPXLFMT_RGB666		(0 << 8)
#define OPXLFMT_RGB888		(1 << 8)
#define FRMSYNC_DISABLED	(0 << 4) /* Video Timing Gen Disabled */
#define FRMSYNC_ENABLED		(1 << 4) /* Video Timing Gen Enabled */
#define MSF_DISABLED		(0 << 0) /* Magic Square FRC disabled */
#define MSF_ENABLED			(1 << 0) /* Magic Square FRC enabled */

#define VP_TIM			0x0454
#define VFUEN0			0x0464
#define VFUEN           BIT(0)

#define TC_IDREG		0x0500
#define SYSBOOT			0x0504
#define SYSSTAT			0x0508
#define SYSRSTENB		0x050c
#define ENBI2C          BIT(0)
#define ENBLCD0         BIT(2)
#define ENBBM           BIT(3)
#define ENBDSIRX        BIT(4)
#define ENBREG          BIT(5)
#define ENBHDCP         BIT(8)
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
#define AUD_MN_GEN      BIT(7)      /* Auto-generate M/N values */
#define EF_EN			BIT(5)      /* Enable Enhanced Framing */
#define AUD_EN			BIT(2)      /* Video transmission enable */
#define VID_EN			BIT(1)      /* Video transmission enable */
#define DP_EN			BIT(0)      /* Enable DPTX function */
#ifdef MPD_2LANE
#define DP_PHY_CTRL_EN  BGREN | PWR_SW_EN | PHY_2LANE | PHY_A0_EN | PHY_M0_EN
#else
#define DP_PHY_CTRL_EN  BGREN | PWR_SW_EN | PHY_A0_EN | PIEC60958_HY_M0_EN
#endif

#define DP0_VIDMNGEN0	    0x0610	/* DP0 Video Force M Value Register */
#define DP0_VIDMNGEN1	    0x0614	/* DP0 Video Force N Value Register */
#define DP0_VMNGENSTATUS	0x0618	/* DP0 Video Current M Value Register */
#define DP0_AUDMNGEN0		0x0628	/* DP0 Audio Force M Value Register */
#define DP0_AUDMNGEN1		0x062c	/* DP0 Audio Force N Value Register */
#define DP0_AMNGENSTATUS	0x0630	/* DP0 Audio Current M Value Register */

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

#define AUDCFG0			0x0700	/* DP0 Audio Config0 Register */
#define AUDCFG1			0x0704	/* DP0 Audio Config1 Register */
#define AUDIFDATA0		0x0708	/* DP0 Audio Info Frame Bytes 3 to 0 */
#define AUDIFDATA1		0x070c	/* DP0 Audio Info Frame Bytes 7 to 4 */
#define AUDIFDATA2		0x0710	/* DP0 Audio Info Frame Bytes 11 to 8 */
#define AUDIFDATA3		0x0714	/* DP0 Audio Info Frame Bytes 15 to 12 */
#define AUDIFDATA4		0x0718	/* DP0 Audio Info Frame Bytes 19 to 16 */
#define AUDIFDATA5		0x071c	/* DP0 Audio Info Frame Bytes 23 to 20 */
#define AUDIFDATA6		0x0720	/* DP0 Audio Info Frame Bytes 27 to 24 */
#define AUD_MUTE        BIT(24)
#define AUD_STEREO      BIT(8)
#define AUD_MONO        0

#define MPD_AUD_FS32        1
#define MPD_AUD_FS44P1      2
#define MPD_AUD_FS48        3
#define MPD_AUD_FS88P2      4
#define MPD_AUD_FS96        5
#define MPD_AUD_FS176P4     6
#define MPD_AUD_FS192       7

#define MPD_AUD_WIDTH16     1
#define MPD_AUD_WIDTH20     2
#define MPD_AUD_WIDTH24     3
#if (MPD_AUDIO_WIDTH == MPD_AUD_WIDTH16)
#define MPD_I2S_BIT     16
#elif (MPD_AUDIO_WIDTH == MPD_AUD_WIDTH20)
#define MPD_I2S_BIT     20
#elif (MPD_AUDIO_WIDTH == MPD_AUD_WIDTH24)
#define MPD_I2S_BIT     24
#endif

//Secondary-data Packet Type
#define AUD_PKT_ID      170 //refer to excel sheet(?)
#define AUD_IF_TYPE     0x84    //CEA-861-E InfoFrame, 0x80 + 0x04(Audio InfoFrame)

#define AUD_CC          1   //2 Channels
#define AUD_CT          1   //PCM

#define AUD_SS          MPD_AUDIO_WIDTH //1:16 bits 2:20bits 3:24bits
#define AUD_SF          MPD_AUDIO_FS    //1:32kHz 2:44.1kHz 3:48kHz 4:88.2kHz 5:96kHz 6:176.4kHz 7:192kHz

#define AUD_CXT         0   //Audio Codec Extension, no need for PCM

#define AUD_CA          0   //Audio Channel Assignment, except zero is invalid for stereo

#define AUD_LFEPBL      0   //LFE Playback Level
#define AUD_LSV         0   //Audio Attenuation Level(?)
#define AUD_DM_IF       0   //Audio Down-mix Inhibit Flag, 1 to prohibit down-spec.mix audio

#define AUD_MAX_VSAMPLE     15  //refer to device spec
#define AUD_MAX_HSAMPLE     15  //refer to device spec

#if (MPD_BW == DP_SRCCTRL_BW27)
#if     (MPD_AUDIO_FS == MPD_AUD_FS32)
#define AUD_M   1024
#define AUD_N   16875
#elif   (MPD_AUDIO_FS == MPD_AUD_FS44P1)
#define AUD_M   784
#define AUD_N   9375
#elif   (MPD_AUDIO_FS == MPD_AUD_FS48)
#define AUD_M   512
#define AUD_N   5625
#elif   (MPD_AUDIO_FS == MPD_AUD_FS88P2)
#define AUD_M   1568
#define AUD_N   9375
#elif   (MPD_AUDIO_FS == MPD_AUD_FS96)
#define AUD_M   1024
#define AUD_N   5625
#elif   (MPD_AUDIO_FS == MPD_AUD_FS176P4)
#define AUD_M   3136
#define AUD_N   9375
#elif   (MPD_AUDIO_FS == MPD_AUD_FS192)
#define AUD_M   2048
#define AUD_N   5625
#endif
#else
#if     (MPD_AUDIO_FS == MPD_AUD_FS32)
#define AUD_M   1024
#define AUD_N   10125
#elif   (MPD_AUDIO_FS == MPD_AUD_FS44P1)
#define AUD_M   784
#define AUD_N   5625
#elif   (MPD_AUDIO_FS == MPD_AUD_FS48)
#define AUD_M   512
#define AUD_N   3375
#elif   (MPD_AUDIO_FS == MPD_AUD_FS88P2)
#define AUD_M   1568
#define AUD_N   5625
#elif   (MPD_AUDIO_FS == MPD_AUD_FS96)
#define AUD_M   1024
#define AUD_N   3375
#elif   (MPD_AUDIO_FS == MPD_AUD_FS176P4)
#define AUD_M   3136
#define AUD_N   5625
#elif   (MPD_AUDIO_FS == MPD_AUD_FS192)
#define AUD_M   2048
#define AUD_N   3375
#endif
#endif

#define DP0_SRCCTRL		0x06a0
#define DP1_SRCCTRL		0x07a0
#define DP_PRE_0        0x0
#define DP_PRE_3P5      0x1
#define DP_PRE_6        0x2
#define DP_SWG_0P4      0x0
#define DP_SWG_0P6      0x1
#define DP_SWG_0P8      0x2
#define DP_SWG_1P2      0x3
#ifdef MPD_2LANE
#define DP_SRCCTRL_PRE1(x)		((x) << 28)
#define DP_SRCCTRL_SWG1(x)		((x) << 24)
#else
#define DP_SRCCTRL_PRE1(x)		0
#define DP_SRCCTRL_SWG1(x)		0
#endif
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
#ifdef MPD_2LANE
#define DP_SRCCTRL_LANES    DP_SRCCTRL_LANES_2
#else
#define DP_SRCCTRL_LANES    DP_SRCCTRL_LANES_1
#endif

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

/* I2S */
#define I2SCFG			0x0880	/* I2S Audio Config 0 Register */
#define I2SCH0STAT0		0x0888	/* I2S Audio Channel 0 Status Bytes 3 to 0 */
#define I2SCH0STAT1		0x088c	/* I2S Audio Channel 0 Status Bytes 7 to 4 */
#define I2SCH0STAT2		0x0890	/* I2S Audio Channel 0 Status Bytes 11 to 8 */
#define I2SCH0STAT3		0x0894	/* I2S Audio Channel 0 Status Bytes 15 to 12 */
#define I2SCH0STAT4		0x0898	/* I2S Audio Channel 0 Status Bytes 19 to 16 */
#define I2SCH0STAT5		0x089c	/* I2S Audio Channel 0 Status Bytes 23 to 20 */
#define I2SCH1STAT0		0x08a0	/* I2S Audio Channel 1 Status Bytes 3 to 0 */
#define I2SCH1STAT1		0x08a4	/* I2S Audio Channel 1 Status Bytes 7 to 4 */
#define I2SCH1STAT2		0x08a8	/* I2S Audio Channel 1 Status Bytes 11 to 8 */
#define I2SCH1STAT3		0x08ac	/* I2S Audio Channel 1 Status Bytes 15 to 12 */
#define I2SCH1STAT4		0x08b0	/* I2S Audio Channel 1 Status Bytes 19 to 16 */
#define I2SCH1STAT5		0x08b4	/* I2S Audio Channel 1 Status Bytes 23 to 20 */

#define I2S_AUD_EN          BIT(0)
#define I2S_IEC60958_EN     BIT(1)
#define I2S_IEC60958_VALID  BIT(2)
#define I2S_SAMPLE_WITDH    ((MPD_I2S_BIT-16) << 4)    //Width - 16

#define MPD_AUD_FMT_STD     0
#define MPD_AUD_FMT_LEFT    1
#define MPD_AUD_FMT_RIGHT   2
#define I2S_SAMPLE_FMT      (MPD_AUDIO_FMT << 8)

//IEC60958 spec.
//Use Consumer Channel Status Structure
//for more refer to IEC-60958-3.

//Word 0 (0~3 bytes)
#define IEC60958_PRO        BIT(0)
#define IEC60958_NOT_LPCM   BIT(1)
#define IEC60958_NO_CP      BIT(2)      //Content Protection
#define IEC60958_AF_INFO    (1 << 3)    //Additional Format Info, no de-emphasis
#define IEC60958_CHAN_MODE  (0 << 6)    //Use mode 0
#define IEC60958_CAT_CODE   (0 << 8)    //Playback Device category code, keep default 0
#define IEC60958_SRC_NUM    (0 << 16)   //Source Number
#define IEC60958_CHAN_L     (1 << 20)
#define IEC60958_CHAN_R     (2 << 20)
#if     (MPD_AUDIO_FS == MPD_AUD_FS32)
#define IEC60958_FS         (3 << 24)
#elif   (MPD_AUDIO_FS == MPD_AUD_FS44P1)
#define IEC60958_FS         (0 << 24)
#elif   (MPD_AUDIO_FS == MPD_AUD_FS48)
#define IEC60958_FS         (2 << 24)
#elif   (MPD_AUDIO_FS == MPD_AUD_FS88P2)
#define IEC60958_FS         (8 << 24)
#elif   (MPD_AUDIO_FS == MPD_AUD_FS96)
#define IEC60958_FS         (10 << 24)
#elif   (MPD_AUDIO_FS == MPD_AUD_FS176P4)
#define IEC60958_FS         (12 << 24)
#elif   (MPD_AUDIO_FS == MPD_AUD_FS192)
#define IEC60958_FS         (14 << 24)
#endif
#define IEC60958_CLK_ACC    (0 << 28)   //Clock Accuracy, 0: Level II(1000ppm); 1: Level I(50ppm); 2: Level III(variable pitch shifted clock mode); 3: Interface frame rate not matched to sampling frequency

#define IEC60958_DATA0  IEC60958_NO_CP | IEC60958_AF_INFO | IEC60958_CHAN_MODE | IEC60958_CAT_CODE | IEC60958_SRC_NUM | IEC60958_FS | IEC60958_CLK_ACC

//Word 1 (4~7 bytes)
//combined with max word length[0] and word length[3:1]
#if     (MPD_AUDIO_WIDTH == MPD_AUD_WIDTH16)
#define IEC60958_WITDH      2
#elif   (MPD_AUDIO_WIDTH == MPD_AUD_WIDTH20)
#define IEC60958_WITDH      10
#elif   (MPD_AUDIO_WIDTH == MPD_AUD_WIDTH24)
#define IEC60958_WITDH      11
#endif
#if     (MPD_AUDIO_FS == MPD_AUD_FS32)
#define IEC60958_ORIFS      (12 << 4)
#elif   (MPD_AUDIO_FS == MPD_AUD_FS44P1)
#define IEC60958_ORIFS      (15 << 4)
#elif   (MPD_AUDIO_FS == MPD_AUD_FS48)
#define IEC60958_ORIFS      (13 << 4)
#elif   (MPD_AUDIO_FS == MPD_AUD_FS88P2)
#define IEC60958_ORIFS      (7 << 4)
#elif   (MPD_AUDIO_FS == MPD_AUD_FS96)
#define IEC60958_ORIFS      (5 << 4)
#elif   (MPD_AUDIO_FS == MPD_AUD_FS176P4)
#define IEC60958_ORIFS      (3 << 4)
#elif   (MPD_AUDIO_FS == MPD_AUD_FS192)
#define IEC60958_ORIFS      (1 << 4)
#endif

#define IEC60958_DATA1  IEC60958_WITDH | IEC60958_ORIFS

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
#define DPCD_LANE_ALIGN_STATUS      0x00204
#define DPCD_SINK_STATUS            0x00205
#define DPCD_ADJUST_REQUEST         0x00206
#define DPCD_SYMBOL_ERR_CNT         0x00210
#define DPCD_EDP_CONFIGURATION_SET  0x0010a
#define DPCD_DOWNSPREAD_CTRL        0x00107

#define DPCD_SET_ANSI_8B10B         BIT(0)
#define DPCD_SCRAMBLING_DISABLE     BIT(5)
#define DPCD_TRAINING_PATTERN_1     1
#define DPCD_TRAINING_PATTERN_2     2
#if (MPD_BW == DP_SRCCTRL_BW27)
#define DPCD_BW     10
#else
#define DPCD_BW     6
#endif
#define DPCD_ENHANCED_FRAME_EN      BIT(15)
#define DPCD_LANES(x)   ((x) << 8)

#define MPD_VSDELAY                 MPD_HFPR
#define MPD_DP_VIDGEN_N             32768 //refer to DP spec, using async mode
#define MPD_MAX_TU_SYMBOL           42 //DIV_ROUND_UP(in_bw * TU_SIZE_RECOMMENDED, out_bw), in_bw=148.5MHz*3B, out_bw=2*2700Mbps/8bpB
#define MPD_TU_SIZE_RECOMMENDED     63

//HTIM01, HTIM02, VTIM01, VTIM02
static const uint16_t rgb_timing[8] = {
    MPD_HPW,
    MPD_HBPR,
    MPD_Hactive,
    MPD_HFPR,
    MPD_VPW,
    MPD_VBPR,
    MPD_Vactive,
    MPD_VFPR
};

//DP0_VidSyncDly, DP0_TotalVal, DP0_StartVal, DP0_ActiveVal, DP0_SyncVal
static const uint16_t dp_timing[10] = {
    MPD_HPW + MPD_HBPR + MPD_Hactive, //vid_sync_dly
    MPD_MAX_TU_SYMBOL,
    MPD_HPW + MPD_HBPR + MPD_Hactive + MPD_HFPR, //h_total
    MPD_VPW + MPD_VBPR + MPD_Vactive + MPD_VFPR, //v_total
    MPD_HPW + MPD_HBPR, //h_start
    MPD_VPW + MPD_VBPR, //v_start
    MPD_Hactive,
    MPD_Vactive,
    MPD_HPW, //active high
    MPD_VPW
};

void MPD_Init();
int MPD_CfgLink();
void MPD_CfgStream();
void MPD_HPD_IRQ();
void MPD_Disable();

#endif
