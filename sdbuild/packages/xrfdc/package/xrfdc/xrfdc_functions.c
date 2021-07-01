/******************************************************************************
* Copyright (C) 2021 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/

/**************************** Type Definitions *******************************/
#define XRFdc_IsADC4GSPS(InstPtr) XRFdc_IsHighSpeedADC(InstPtr, 0)

typedef __u32 u32;
typedef __u16 u16;
typedef __u8 u8;
typedef __s32 s32;
typedef __s16 s16;
typedef __u64 u64;
typedef __s64 s64;
typedef __s8 s8;


/**
* The handler data type allows the user to define a callback function to
* respond to interrupt events in the system. This function is executed
* in interrupt context, so amount of processing should be minimized.
*
* @param    CallBackRef is the callback reference passed in by the upper
*           layer when setting the callback functions, and passed back to
*           the upper layer when the callback is invoked. Its type is
*           not important to the driver, so it is a void pointer.
* @param    Type indicates ADC/DAC.
* @param    Tile_Id indicates Tile number (0-3).
* @param    Block_Id indicates Block number (0-3).
* @param    StatusEvent indicates one or more interrupt occurred.
*/
typedef void (*XRFdc_StatusHandler)(void *CallBackRef, u32 Type, u32 Tile_Id, u32 Block_Id, u32 StatusEvent);

/**
 * PLL settings.
 */
typedef struct {
	u32 Enabled; /* PLL Enables status (not a setter) */
	double RefClkFreq;
	double SampleRate;
	u32 RefClkDivider;
	u32 FeedbackDivider;
	u32 OutputDivider;
	u32 FractionalMode; /* Fractional mode is currently not supported */
	u64 FractionalData; /* Fractional data is currently not supported */
	u32 FractWidth; /* Fractional width is currently not supported */
} XRFdc_PLL_Settings;
/**
* ClkIntraTile Settings.
*/
typedef struct {
	u8 SourceTile;
	u8 PLLEnable;
	XRFdc_PLL_Settings PLLSettings;
	u8 DivisionFactor;
	u8 Delay;
	u8 DistributedClock;
} XRFdc_Tile_Clock_Settings;
/**
* Clk Distribution.
*/
typedef struct {
	u8 Enabled;
	u8 DistributionSource;
	u8 UpperBound;
	u8 LowerBound;
	u8 MaxDelay;
	u8 MinDelay;
	u8 IsDelayBalanced;
} XRFdc_Distribution;
/**
* Clk Distribution Settings.
*/
typedef struct {
	XRFdc_Tile_Clock_Settings DAC[4];
	XRFdc_Tile_Clock_Settings ADC[4];
	XRFdc_Distribution DistributionStatus[8];
} XRFdc_Distribution_Settings;

/**
 * ADC Signal Detect Settings.
 */
typedef struct {
	u8 Mode;
	u8 TimeConstant;
	u8 Flush;
	u8 EnableIntegrator;
	u16 HighThreshold;
	u16 LowThreshold;
	u16 HighThreshOnTriggerCnt; /* the number of times value must exceed HighThreshold before turning on*/
	u16 HighThreshOffTriggerCnt; /* the number of times value must be less than HighThreshold before turning off*/
	u16 LowThreshOnTriggerCnt; /* the number of times value must exceed LowThreshold before turning on*/
	u16 LowThreshOffTriggerCnt; /* the number of times value must be less than LowThreshold before turning off*/
	u8 HysteresisEnable;
} XRFdc_Signal_Detector_Settings;
/**
 * QMC settings.
 */
typedef struct {
	u32 EnablePhase;
	u32 EnableGain;
	double GainCorrectionFactor;
	double PhaseCorrectionFactor;
	s32 OffsetCorrectionFactor;
	u32 EventSource;
} XRFdc_QMC_Settings;

/**
 * Coarse delay settings.
 */
typedef struct {
	u32 CoarseDelay;
	u32 EventSource;
} XRFdc_CoarseDelay_Settings;

/**
 * Mixer settings.
 */
typedef struct {
	double Freq;
	double PhaseOffset;
	u32 EventSource;
	u32 CoarseMixFreq;
	u32 MixerMode;
	u8 FineMixerScale; /* NCO output scale, valid values 0,1 and 2 */
	u8 MixerType;
} XRFdc_Mixer_Settings;

/**
 * ADC block Threshold settings.
 */
typedef struct {
	u32 UpdateThreshold; /* Selects which threshold to update */
	u32 ThresholdMode[2]; /* Entry 0 for Threshold0 and 1 for Threshold1 */
	u32 ThresholdAvgVal[2]; /* Entry 0 for Threshold0 and 1 for Threshold1 */
	u32 ThresholdUnderVal[2]; /* Entry 0 for Threshold0 and 1 for Threshold1 */
	u32 ThresholdOverVal[2]; /* Entry 0 is for Threshold0 and 1 for Threshold1 */
} XRFdc_Threshold_Settings;

/**
 * RFSoC Calibration coefficients generic struct
 */
typedef struct {
	u32 Coeff0;
	u32 Coeff1;
	u32 Coeff2;
	u32 Coeff3;
	u32 Coeff4;
	u32 Coeff5;
	u32 Coeff6;
	u32 Coeff7;
} XRFdc_Calibration_Coefficients;

/**
 * RFSoC Power Mode settings
 */
typedef struct {
	u32 DisableIPControl; /*Disables IP RTS control of the power mode*/
	u32 PwrMode; /*The power mode*/
} XRFdc_Pwr_Mode_Settings;

/**
 * RFSoC DSA settings
 */
typedef struct {
	u32 DisableRTS; /*Disables RTS control of DSA attenuation*/
	float Attenuation; /*Attenuation*/
} XRFdc_DSA_Settings;

/**
 * RFSoC Calibration freeze settings struct
 */
typedef struct {
	u32 CalFrozen; /*Status indicates calibration freeze state*/
	u32 DisableFreezePin; /*Disable the calibration freeze pin*/
	u32 FreezeCalibration; /*Setter for freezing*/
} XRFdc_Cal_Freeze_Settings;

/**
 * RFSoC Tile status.
 */
typedef struct {
	u32 IsEnabled; /* 1, if tile is enabled, 0 otherwise */
	u32 TileState; /* Indicates Tile Current State */
	u8 BlockStatusMask; /* Bit mask for block status, 1 indicates block enable */
	u32 PowerUpState;
	u32 PLLState;
} XRFdc_TileStatus;

/**
 * RFSoC Data converter IP status.
 */
typedef struct {
	XRFdc_TileStatus DACTileStatus[4];
	XRFdc_TileStatus ADCTileStatus[4];
	u32 State;
} XRFdc_IPStatus;

/**
 * status of DAC or ADC blocks in the RFSoC Data converter.
 */
typedef struct {
	double SamplingFreq;
	u32 AnalogDataPathStatus;
	u32 DigitalDataPathStatus;
	u8 DataPathClocksStatus; /* Indicates all required datapath
				clocks are enabled or not, 1 if all clocks enabled, 0 otherwise */
	u8 IsFIFOFlagsEnabled; /* Indicates FIFO flags enabled or not,
				 1 if all flags enabled, 0 otherwise */
	u8 IsFIFOFlagsAsserted; /* Indicates FIFO flags asserted or not,
				 1 if all flags asserted, 0 otherwise */
} XRFdc_BlockStatus;

/**
 * DAC block Analog DataPath Config settings.
 */
typedef struct {
	u32 BlockAvailable;
	u32 InvSyncEnable;
	u32 MixMode;
	u32 DecoderMode;
} XRFdc_DACBlock_AnalogDataPath_Config;

/**
 * DAC block Digital DataPath Config settings.
 */
typedef struct {
	u32 MixerInputDataType;
	u32 DataWidth;
	u32 InterpolationMode;
	u32 FifoEnable;
	u32 AdderEnable;
	u32 MixerType;
} XRFdc_DACBlock_DigitalDataPath_Config;

/**
 * ADC block Analog DataPath Config settings.
 */
typedef struct {
	u32 BlockAvailable;
	u32 MixMode;
} XRFdc_ADCBlock_AnalogDataPath_Config;

/**
 * ADC block Digital DataPath Config settings.
 */
typedef struct {
	u32 MixerInputDataType;
	u32 DataWidth;
	u32 DecimationMode;
	u32 FifoEnable;
	u32 MixerType;
} XRFdc_ADCBlock_DigitalDataPath_Config;

/**
 * DAC Tile Config structure.
 */
typedef struct {
	u32 Enable;
	u32 PLLEnable;
	double SamplingRate;
	double RefClkFreq;
	double FabClkFreq;
	u32 FeedbackDiv;
	u32 OutputDiv;
	u32 RefClkDiv;
	u32 MultibandConfig;
	double MaxSampleRate;
	u32 NumSlices;
	XRFdc_DACBlock_AnalogDataPath_Config DACBlock_Analog_Config[4];
	XRFdc_DACBlock_DigitalDataPath_Config DACBlock_Digital_Config[4];
} XRFdc_DACTile_Config;

/**
 * ADC Tile Config Structure.
 */
typedef struct {
	u32 Enable; /* Tile Enable status */
	u32 PLLEnable; /* PLL enable Status */
	double SamplingRate;
	double RefClkFreq;
	double FabClkFreq;
	u32 FeedbackDiv;
	u32 OutputDiv;
	u32 RefClkDiv;
	u32 MultibandConfig;
	double MaxSampleRate;
	u32 NumSlices;
	XRFdc_ADCBlock_AnalogDataPath_Config ADCBlock_Analog_Config[4];
	XRFdc_ADCBlock_DigitalDataPath_Config ADCBlock_Digital_Config[4];
} XRFdc_ADCTile_Config;

/**
 * RFdc Config Structure.
 */
typedef struct {
	u32 DeviceId;
	metal_phys_addr_t BaseAddr;
	u32 ADCType; /* ADC Type 4GSPS or 2GSPS*/
	u32 MasterADCTile; /* ADC master Tile */
	u32 MasterDACTile; /* DAC Master Tile */
	u32 ADCSysRefSource;
	u32 DACSysRefSource;
	u32 IPType;
	u32 SiRevision;
	XRFdc_DACTile_Config DACTile_Config[4];
	XRFdc_ADCTile_Config ADCTile_Config[4];
} XRFdc_Config;

/**
 * DAC Block Analog DataPath Structure.
 */
typedef struct {
	u32 Enabled; /* DAC Analog Data Path Enable */
	u32 MixedMode;
	double TerminationVoltage;
	double OutputCurrent;
	u32 InverseSincFilterEnable;
	u32 DecoderMode;
	void *FuncHandler;
	u32 NyquistZone;
	u8 AnalogPathEnabled;
	u8 AnalogPathAvailable;
	XRFdc_QMC_Settings QMC_Settings;
	XRFdc_CoarseDelay_Settings CoarseDelay_Settings;
} XRFdc_DACBlock_AnalogDataPath;

/**
 * DAC Block Digital DataPath Structure.
 */
typedef struct {
	u32 MixerInputDataType;
	u32 DataWidth;
	int ConnectedIData;
	int ConnectedQData;
	u32 InterpolationFactor;
	u8 DigitalPathEnabled;
	u8 DigitalPathAvailable;
	XRFdc_Mixer_Settings Mixer_Settings;
} XRFdc_DACBlock_DigitalDataPath;

/**
 * ADC Block Analog DataPath Structure.
 */
typedef struct {
	u32 Enabled; /* ADC Analog Data Path Enable */
	XRFdc_QMC_Settings QMC_Settings;
	XRFdc_CoarseDelay_Settings CoarseDelay_Settings;
	XRFdc_Threshold_Settings Threshold_Settings;
	u32 NyquistZone;
	u8 CalibrationMode;
	u8 AnalogPathEnabled;
	u8 AnalogPathAvailable;
} XRFdc_ADCBlock_AnalogDataPath;

/**
 * ADC Block Digital DataPath Structure.
 */
typedef struct {
	u32 MixerInputDataType;
	u32 DataWidth;
	u32 DecimationFactor;
	int ConnectedIData;
	int ConnectedQData;
	u8 DigitalPathEnabled;
	u8 DigitalPathAvailable;
	XRFdc_Mixer_Settings Mixer_Settings;
} XRFdc_ADCBlock_DigitalDataPath;

/**
 * DAC Tile Structure.
 */
typedef struct {
	u32 TileBaseAddr; /* Tile  BaseAddress*/
	u32 NumOfDACBlocks; /* Number of DAC block enabled */
	XRFdc_PLL_Settings PLL_Settings;
	u8 MultibandConfig;
	XRFdc_DACBlock_AnalogDataPath DACBlock_Analog_Datapath[4];
	XRFdc_DACBlock_DigitalDataPath DACBlock_Digital_Datapath[4];
} XRFdc_DAC_Tile;

/**
 * ADC Tile Structure.
 */
typedef struct {
	u32 TileBaseAddr;
	u32 NumOfADCBlocks; /* Number of ADC block enabled */
	XRFdc_PLL_Settings PLL_Settings;
	u8 MultibandConfig;
	XRFdc_ADCBlock_AnalogDataPath ADCBlock_Analog_Datapath[4];
	XRFdc_ADCBlock_DigitalDataPath ADCBlock_Digital_Datapath[4];
} XRFdc_ADC_Tile;

/**
 * RFdc Structure.
 */
typedef struct {
	XRFdc_Config RFdc_Config; /* Config Structure */
	u32 IsReady;
	u32 ADC4GSPS;
	metal_phys_addr_t BaseAddr; /* BaseAddress */
	struct metal_io_region *io; /* Libmetal IO structure */
	struct metal_device *device; /* Libmetal device structure */
	XRFdc_DAC_Tile DAC_Tile[4];
	XRFdc_ADC_Tile ADC_Tile[4];
	XRFdc_StatusHandler StatusHandler; /* Event handler function */
	void *CallBackRef; /* Callback reference for event handler */
	u8 UpdateMixerScale; /* Set to 1, if user overwrite mixer scale */
} XRFdc;

/***************** Macros (Inline Functions) Definitions *********************/

#define MAX(x, y) (x > y) ? x : y
#define MIN(x, y) (x < y) ? x : y
#define XRFDC_SUCCESS 0U
#define XRFDC_FAILURE 1U
#define XRFDC_GEN3 2
#define XRFDC_COMPONENT_IS_READY 0x11111111U
#define XRFDC_NUM_SLICES_HSADC 2
#define XRFDC_NUM_SLICES_LSADC 4

#define XRFDC_DEVICE_ID_SIZE 4U
#define XRFDC_NUM_INST_SIZE 4U
#define XRFDC_CONFIG_DATA_SIZE sizeof(XRFdc_Config)

#define XRFDC_REGION_SIZE 0x40000U

#define XRFDC_ADC_TILE 0U
#define XRFDC_DAC_TILE 1U
#define XRFDC_TILE_ID_MAX 0x3U
#define XRFDC_BLOCK_ID_MAX 0x3U
#define XRFDC_EVNT_SRC_IMMEDIATE 0x00000000U
#define XRFDC_EVNT_SRC_SLICE 0x00000001U
#define XRFDC_EVNT_SRC_TILE 0x00000002U
#define XRFDC_EVNT_SRC_SYSREF 0x00000003U
#define XRFDC_EVNT_SRC_MARKER 0x00000004U
#define XRFDC_EVNT_SRC_PL 0x00000005U
#define XRFDC_EVENT_MIXER 0x00000001U
#define XRFDC_EVENT_CRSE_DLY 0x00000002U
#define XRFDC_EVENT_QMC 0x00000004U
#define XRFDC_SELECT_ALL_TILES -1
#define XRFDC_ADC_4GSPS 1U

#define XRFDC_CRSE_DLY_MAX 0x7U
#define XRFDC_CRSE_DLY_MAX_EXT 0x28U
#define XRFDC_NCO_FREQ_MULTIPLIER (0x1LLU << 48U) /* 2^48 */
#define XRFDC_NCO_PHASE_MULTIPLIER (1U << 17U) /* 2^17 */
#define XRFDC_QMC_PHASE_MULT (1U << 11U) /* 2^11 */
#define XRFDC_QMC_GAIN_MULT (1U << 14U) /* 2^14 */

#define XRFDC_DATA_TYPE_IQ 0x00000001U
#define XRFDC_DATA_TYPE_REAL 0x00000000U

#define XRFDC_TRSHD_OFF 0x0U
#define XRFDC_TRSHD_STICKY_OVER 0x00000001U
#define XRFDC_TRSHD_STICKY_UNDER 0x00000002U
#define XRFDC_TRSHD_HYSTERISIS 0x00000003U

/* Mixer modes */
#define XRFDC_MIXER_MODE_OFF 0x0U
#define XRFDC_MIXER_MODE_C2C 0x1U
#define XRFDC_MIXER_MODE_C2R 0x2U
#define XRFDC_MIXER_MODE_R2C 0x3U
#define XRFDC_MIXER_MODE_R2R 0x4U

#define XRFDC_I_IQ_COS_MINSIN 0x00000C00U
#define XRFDC_Q_IQ_SIN_COS 0x00001000U
#define XRFDC_EN_I_IQ 0x00000001U
#define XRFDC_EN_Q_IQ 0x00000004U

#define XRFDC_MIXER_TYPE_COARSE 0x1U
#define XRFDC_MIXER_TYPE_FINE 0x2U

#define XRFDC_MIXER_TYPE_OFF 0x0U
#define XRFDC_MIXER_TYPE_DISABLED 0x3U

#define XRFDC_COARSE_MIX_OFF 0x0U
#define XRFDC_COARSE_MIX_SAMPLE_FREQ_BY_TWO 0x2U
#define XRFDC_COARSE_MIX_SAMPLE_FREQ_BY_FOUR 0x4U
#define XRFDC_COARSE_MIX_MIN_SAMPLE_FREQ_BY_FOUR 0x8U
#define XRFDC_COARSE_MIX_BYPASS 0x10U

#define XRFDC_COARSE_MIX_MODE_C2C_C2R 0x1U
#define XRFDC_COARSE_MIX_MODE_R2C 0x2U

#define XRFDC_CRSE_MIX_OFF 0x924U
#define XRFDC_CRSE_MIX_BYPASS 0x0U
#define XRFDC_CRSE_4GSPS_ODD_FSBYTWO 0x492U
#define XRFDC_CRSE_MIX_I_ODD_FSBYFOUR 0x2CBU
#define XRFDC_CRSE_MIX_Q_ODD_FSBYFOUR 0x659U
#define XRFDC_CRSE_MIX_I_Q_FSBYTWO 0x410U
#define XRFDC_CRSE_MIX_I_FSBYFOUR 0x298U
#define XRFDC_CRSE_MIX_Q_FSBYFOUR 0x688U
#define XRFDC_CRSE_MIX_I_MINFSBYFOUR 0x688U
#define XRFDC_CRSE_MIX_Q_MINFSBYFOUR 0x298U
#define XRFDC_CRSE_MIX_R_I_FSBYFOUR 0x8A0U
#define XRFDC_CRSE_MIX_R_Q_FSBYFOUR 0x70CU
#define XRFDC_CRSE_MIX_R_I_MINFSBYFOUR 0x8A0U
#define XRFDC_CRSE_MIX_R_Q_MINFSBYFOUR 0x31CU

#define XRFDC_MIXER_SCALE_AUTO 0x0U
#define XRFDC_MIXER_SCALE_1P0 0x1U
#define XRFDC_MIXER_SCALE_0P7 0x2U

#define XRFDC_MIXER_PHASE_OFFSET_UP_LIMIT 180.0
#define XRFDC_MIXER_PHASE_OFFSET_LOW_LIMIT (-180.0)
#define XRFDC_UPDATE_THRESHOLD_0 0x1U
#define XRFDC_UPDATE_THRESHOLD_1 0x2U
#define XRFDC_UPDATE_THRESHOLD_BOTH 0x4U
#define XRFDC_THRESHOLD_CLRMD_MANUAL_CLR 0x1U
#define XRFDC_THRESHOLD_CLRMD_AUTO_CLR 0x2U
#define XRFDC_DECODER_MAX_SNR_MODE 0x1U
#define XRFDC_DECODER_MAX_LINEARITY_MODE 0x2U
#define XRFDC_OUTPUT_CURRENT_32MA 32U
#define XRFDC_OUTPUT_CURRENT_20MA 20U

#define XRFDC_MIXER_MODE_IQ 0x1U
#define XRFDC_ADC_MIXER_MODE_IQ 0x1U
#define XRFDC_DAC_MIXER_MODE_REAL 0x2U

#define XRFDC_ODD_NYQUIST_ZONE 0x1U
#define XRFDC_EVEN_NYQUIST_ZONE 0x2U

#define XRFDC_INTERP_DECIM_OFF 0x0U
#define XRFDC_INTERP_DECIM_1X 0x1U
#define XRFDC_INTERP_DECIM_2X 0x2U
#define XRFDC_INTERP_DECIM_3X 0x3U
#define XRFDC_INTERP_DECIM_4X 0x4U
#define XRFDC_INTERP_DECIM_5X 0x5U
#define XRFDC_INTERP_DECIM_6X 0x6U
#define XRFDC_INTERP_DECIM_8X 0x8U
#define XRFDC_INTERP_DECIM_10X 0xAU
#define XRFDC_INTERP_DECIM_12X 0xCU
#define XRFDC_INTERP_DECIM_16X 0x10U
#define XRFDC_INTERP_DECIM_20X 0x14U
#define XRFDC_INTERP_DECIM_24X 0x18U
#define XRFDC_INTERP_DECIM_40X 0x28U

#define XRFDC_FAB_CLK_DIV1 0x1U
#define XRFDC_FAB_CLK_DIV2 0x2U
#define XRFDC_FAB_CLK_DIV4 0x3U
#define XRFDC_FAB_CLK_DIV8 0x4U
#define XRFDC_FAB_CLK_DIV16 0x5U

#define XRFDC_CALIB_MODE1 0x1U
#define XRFDC_CALIB_MODE2 0x2U
#define XRFDC_CALIB_MODE_ABS_DIFF 0x1U
#define XRFDC_CALIB_MODE_NEG_ABS_SUM 0x2U
#define XRFDC_TI_DCB_MODE1_4GSPS 0x00007800U
#define XRFDC_TI_DCB_MODE1_2GSPS 0x00005000U

/* PLL Configuration */
#define XRFDC_PLL_UNLOCKED 0x1U
#define XRFDC_PLL_LOCKED 0x2U

#define XRFDC_EXTERNAL_CLK 0x0U
#define XRFDC_INTERNAL_PLL_CLK 0x1U

#define PLL_FPDIV_MIN 13U
#define PLL_FPDIV_MAX 128U
#define PLL_DIVIDER_MIN 2U
#define PLL_DIVIDER_MIN_GEN3 1U
#define PLL_DIVIDER_MAX 28U
#define VCO_RANGE_MIN 8500U
#define VCO_RANGE_MAX 13200U
#define VCO_RANGE_ADC_MIN 8500U
#define VCO_RANGE_ADC_MAX 12800U
#define VCO_RANGE_DAC_MIN 7800U
#define VCO_RANGE_DAC_MAX 13800U
#define XRFDC_PLL_LPF1_VAL 0x6U
#define XRFDC_PLL_CRS2_VAL 0x7008U
#define XRFDC_VCO_UPPER_BAND 0x0U
#define XRFDC_VCO_LOWER_BAND 0x1U
#define XRFDC_REF_CLK_DIV_1 0x1U
#define XRFDC_REF_CLK_DIV_2 0x2U
#define XRFDC_REF_CLK_DIV_3 0x3U
#define XRFDC_REF_CLK_DIV_4 0x4U

#define XRFDC_SINGLEBAND_MODE 0x1U
#define XRFDC_MULTIBAND_MODE_2X 0x2U
#define XRFDC_MULTIBAND_MODE_4X 0x4U

#define XRFDC_MB_DATATYPE_C2C 0x1U
#define XRFDC_MB_DATATYPE_R2C 0x2U
#define XRFDC_MB_DATATYPE_C2R 0x4U

#define XRFDC_MB_DUAL_BAND 2U
#define XRFDC_MB_QUAD_BAND 4U

#define XRFDC_SB_C2C_BLK0 0x82U
#define XRFDC_SB_C2C_BLK1 0x64U
#define XRFDC_SB_C2R 0x40U
#define XRFDC_MB_C2C_BLK0 0x5EU
#define XRFDC_MB_C2C_BLK1 0x5DU
#define XRFDC_MB_C2R_BLK0 0x5CU
#define XRFDC_MB_C2R_BLK1 0x0U

#define XRFDC_MIXER_MODE_BYPASS 0x2U

#define XRFDC_LINK_COUPLING_DC 0x0U
#define XRFDC_LINK_COUPLING_AC 0x1U

#define XRFDC_MB_MODE_SB 0x0U
#define XRFDC_MB_MODE_2X_BLK01 0x1U
#define XRFDC_MB_MODE_2X_BLK23 0x2U
#define XRFDC_MB_MODE_2X_BLK01_BLK23 0x3U
#define XRFDC_MB_MODE_4X 0x4U
#define XRFDC_MB_MODE_2X_BLK01_BLK23_ALT 0x5U

#define XRFDC_MILLI 1000U
#define XRFDC_DAC_SAMPLING_MIN 500
#define XRFDC_DAC_SAMPLING_MAX 6554
#define XRFDC_ADC_4G_SAMPLING_MIN 1000
#define XRFDC_ADC_4G_SAMPLING_MAX 4116
#define XRFDC_ADC_2G_SAMPLING_MIN 500
#define XRFDC_ADC_2G_SAMPLING_MAX 2058
#define XRFDC_REFFREQ_MIN 102 /*102.40625*/
#define XRFDC_REFFREQ_MAX 614

#define XRFDC_DIGITALPATH_ENABLE 0x1U
#define XRFDC_ANALOGPATH_ENABLE 0x1U

#define XRFDC_BLK_ID0 0x0U
#define XRFDC_BLK_ID1 0x1U
#define XRFDC_BLK_ID2 0x2U
#define XRFDC_BLK_ID3 0x3U
#define XRFDC_BLK_ID4 0x4U

#define XRFDC_BLK_ID_NONE -1
#define XRFDC_BLK_ID_ALL -1
#define XRFDC_BLK_ID_INV 0x4

#define XRFDC_TILE_ID0 0x0U
#define XRFDC_TILE_ID1 0x1U
#define XRFDC_TILE_ID2 0x2U
#define XRFDC_TILE_ID3 0x3U
#define XRFDC_TILE_ID4 0x4U

#define XRFDC_TILE_ID_INV 0x4U

#define XRFDC_NUM_OF_BLKS1 0x1U
#define XRFDC_NUM_OF_BLKS2 0x2U
#define XRFDC_NUM_OF_BLKS3 0x3U
#define XRFDC_NUM_OF_BLKS4 0x4U

#define XRFDC_NUM_OF_TILES1 0x1U
#define XRFDC_NUM_OF_TILES2 0x2U
#define XRFDC_NUM_OF_TILES3 0x3U
#define XRFDC_NUM_OF_TILES4 0x4U

#define XRFDC_SM_STATE0 0x0U
#define XRFDC_SM_STATE1 0x1U
#define XRFDC_SM_STATE3 0x3U
#define XRFDC_SM_STATE15 0xFU

#define XRFDC_DECIM_4G_DATA_TYPE 0x3U
#define XRFDC_DECIM_2G_IQ_DATA_TYPE 0x2U

#define XRFDC_DAC_MAX_WR_FAB_RATE 16U
#define XRFDC_ADC_MAX_RD_FAB_RATE(X) ((X < XRFDC_GEN3) ? 8U : 12U)

#define XRFDC_MIN_PHASE_CORR_FACTOR		-26 /*-26.5*/
#define XRFDC_MAX_PHASE_CORR_FACTOR		26 /*26.5*/
#define XRFDC_MAX_GAIN_CORR_FACTOR		2 /*2.0*/
#define XRFDC_MIN_GAIN_CORR_FACTOR		0 /*0.0*/

#define XRFDC_FAB_RATE_16 16
#define XRFDC_FAB_RATE_8 8
#define XRFDC_FAB_RATE_4 4
#define XRFDC_FAB_RATE_2 2
#define XRFDC_FAB_RATE_1 1

#define XRFDC_HSCOM_PWR_STATS_PLL 0xFFC0U
#define XRFDC_HSCOM_PWR_STATS_EXTERNAL 0xF240U
#define XRFDC_HSCOM_PWR_STATS_RX_EXT 0xF2FCU
#define XRFDC_HSCOM_PWR_STATS_DIST_EXT 0xF0FEU
#define XRFDC_HSCOM_PWR_STATS_RX_PLL 0xFFFCU
#define XRFDC_HSCOM_PWR_STATS_DIST_PLL 0xFDFEU
#define XRFDC_HSCOM_PWR_STATS_RX_EXT_DIV 0xF2FCU
#define XRFDC_HSCOM_PWR_STATS_DIST_EXT_DIV 0xF0FE
#define XRFDC_HSCOM_PWR_STATS_DIST_EXT_SRC 0xF2FCU
#define XRFDC_HSCOM_PWR_STATS_DIST_EXT_DIV_SRC 0xF2FCU

#define XRFDC_CLK_DST_TILE_231 0
#define XRFDC_CLK_DST_TILE_230 1
#define XRFDC_CLK_DST_TILE_229 2
#define XRFDC_CLK_DST_TILE_228 3
#define XRFDC_CLK_DST_TILE_227 4
#define XRFDC_CLK_DST_TILE_226 5
#define XRFDC_CLK_DST_TILE_225 6
#define XRFDC_CLK_DST_TILE_224 7
#define XRFDC_CLK_DST_INVALID 0xFFU

#define XRFDC_GLBL_OFST_DAC 0U
#define XRFDC_GLBL_OFST_ADC 4U
#define XRFDC_TILE_GLBL_ADDR(X, Y) (Y + ((X == XRFDC_ADC_TILE) ? XRFDC_GLBL_OFST_ADC : XRFDC_GLBL_OFST_DAC))

#define XRFDC_CLK_DISTR_MUX4A_SRC_INT 0x0008U
#define XRFDC_CLK_DISTR_MUX4A_SRC_STH 0x0000U
#define XRFDC_CLK_DISTR_MUX6_SRC_OFF 0x0000U
#define XRFDC_CLK_DISTR_MUX6_SRC_INT 0x0100U
#define XRFDC_CLK_DISTR_MUX6_SRC_NTH 0x0080U
#define XRFDC_CLK_DISTR_MUX7_SRC_OFF 0x0000U
#define XRFDC_CLK_DISTR_MUX7_SRC_STH 0x0200U
#define XRFDC_CLK_DISTR_MUX7_SRC_INT 0x0400U
#define XRFDC_CLK_DISTR_MUX8_SRC_NTH 0x0000U
#define XRFDC_CLK_DISTR_MUX8_SRC_INT 0x8000U
#define XRFDC_CLK_DISTR_MUX9_SRC_NTH 0x4000U
#define XRFDC_CLK_DISTR_MUX9_SRC_INT 0x0000U
#define XRFDC_CLK_DISTR_MUX5A_SRC_PLL 0x0800U
#define XRFDC_CLK_DISTR_MUX5A_SRC_RX 0x0040U
#define XRFDC_CLK_DISTR_OFF                                                                                            \
	(XRFDC_CLK_DISTR_MUX4A_SRC_INT | XRFDC_CLK_DISTR_MUX6_SRC_OFF | XRFDC_CLK_DISTR_MUX7_SRC_OFF |                 \
	 XRFDC_CLK_DISTR_MUX8_SRC_NTH | XRFDC_CLK_DISTR_MUX9_SRC_INT)
#define XRFDC_CLK_DISTR_LEFTMOST_TILE 0x0000U
#define XRFDC_CLK_DISTR_CONT_LEFT_EVEN 0x8208U
#define XRFDC_CLK_DISTR_CONT_LEFT_ODD 0x8200U
#define XRFDC_CLK_DISTR_RIGHTMOST_TILE 0x4008
#define XRFDC_CLK_DISTR_CONT_RIGHT_EVEN 0x4080
#define XRFDC_CLK_DISTR_CONT_RIGHT_HWL_ODD 0x4088

#define XRFDC_CLK_DISTR_MUX4A_SRC_CLR 0x0008U
#define XRFDC_CLK_DISTR_MUX6_SRC_CLR 0x0180U
#define XRFDC_CLK_DISTR_MUX7_SRC_CLR 0x0600U
#define XRFDC_CLK_DISTR_MUX8_SRC_CLR 0x8000U
#define XRFDC_CLK_DISTR_MUX9_SRC_CLR 0x4000U

#define XRFDC_DIST_MAX 8

#define XRFDC_NET_CTRL_CLK_REC_PLL 0x1U
#define XRFDC_NET_CTRL_CLK_REC_DIST_T1 0x2U
#define XRFDC_NET_CTRL_CLK_T1_SRC_LOCAL 0x4U
#define XRFDC_NET_CTRL_CLK_T1_SRC_DIST 0x8U
#define XRFDC_NET_CTRL_CLK_INPUT_DIST 0x20U
#define XRFDC_DIST_CTRL_TO_PLL_DIV 0x10U
#define XRFDC_DIST_CTRL_TO_T1 0x20U
#define XRFDC_DIST_CTRL_DIST_SRC_LOCAL 0x40U
#define XRFDC_DIST_CTRL_DIST_SRC_PLL 0x800U
#define XRFDC_DIST_CTRL_CLK_T1_SRC_LOCAL 0x1000U
#define XRFDC_PLLREFDIV_INPUT_OFF 0x20U
#define XRFDC_PLLREFDIV_INPUT_DIST 0x40U
#define XRFDC_PLLREFDIV_INPUT_FABRIC 0x60U
#define XRFDC_PLLOPDIV_INPUT_DIST_LOCAL 0x800U

#define XRFDC_TILE_SOURCE_RX 0U
#define XRFDC_TILE_SOURCE_DIST 1U
#define XRFDC_TILE_SOURCE_FABRIC 2U

#define XRFDC_DIST_OUT_NONE 0U
#define XRFDC_DIST_OUT_RX 1U
#define XRFDC_DIST_OUT_OUTDIV 2U

#define XRFDC_PLL_SOURCE_NONE 0U
#define XRFDC_PLL_SOURCE_RX 1U
#define XRFDC_PLL_SOURCE_OUTDIV 2U

#define XRFDC_PLL_OUTDIV_MODE_1 0x0U
#define XRFDC_PLL_OUTDIV_MODE_2 0x1U
#define XRFDC_PLL_OUTDIV_MODE_3 0x2U
#define XRFDC_PLL_OUTDIV_MODE_N 0x3U

#define XRFDC_PLL_OUTDIV_MODE_3_VAL 0x1U

#define XRFDC_DIVISION_FACTOR_MIN 1

#define XRFDC_DITH_ENABLE 1
#define XRFDC_DITH_DISABLE 0

#define XRFDC_SIGDET_MODE_AVG 0
#define XRFDC_SIGDET_MODE_RNDM 1
#define XRFDC_SIGDET_TC_2_0 0
#define XRFDC_SIGDET_TC_2_2 1
#define XRFDC_SIGDET_TC_2_4 2
#define XRFDC_SIGDET_TC_2_8 3
#define XRFDC_SIGDET_TC_2_12 4
#define XRFDC_SIGDET_TC_2_14 5
#define XRFDC_SIGDET_TC_2_16 6
#define XRFDC_SIGDET_TC_2_18 7

#define XRFDC_DISABLED 0
#define XRFDC_ENABLED 1

#define XRFDC_ES1_SI 0U
#define XRFDC_PROD_SI 1U

#define XRFDC_CAL_BLOCK_OCB1 0
#define XRFDC_CAL_BLOCK_OCB2 1
#define XRFDC_CAL_BLOCK_GCB 2
#define XRFDC_CAL_BLOCK_TSCB 3

#define XRFDC_INV_SYNC_MODE_MAX 2

#define XRFDC_INV_SYNC_EN_MAX 1

#define XRFDC_CTRL_MASK 0x0440
#define XRFDC_EXPORTCTRL_CLKDIST 0x0400
#define XRFDC_PREMIUMCTRL_CLKDIST 0x0040
#define XRFDC_EXPORTCTRL_VOP 0x2000
#define XRFDC_EXPORTCTRL_DSA 0x0400

#define XRFDC_DATAPATH_MODE_DUC_0_FSDIVTWO 1U
#define XRFDC_DATAPATH_MODE_DUC_0_FSDIVFOUR 2U
#define XRFDC_DATAPATH_MODE_FSDIVFOUR_FSDIVTWO 3U
#define XRFDC_DATAPATH_MODE_NODUC_0_FSDIVTWO 4U
#define XRFDC_DAC_INT_MODE_FULL_BW 0U
#define XRFDC_DAC_INT_MODE_HALF_BW_IMR 2U
#define XRFDC_DAC_INT_MODE_FULL_BW_BYPASS 3U
#define XRFDC_DAC_MODE_MAX XRFDC_DATAPATH_MODE_NODUC_0_FSDIVTWO

#define XRFDC_FULL_BW_DIVISOR 1U
#define XRFDC_HALF_BW_DIVISOR 2U

#define XRFDC_DAC_IMR_MODE_LOWPASS 0U
#define XRFDC_DAC_IMR_MODE_HIGHPASS 1U
#define XRFDC_DAC_IMR_MODE_MAX XRFDC_DAC_IMR_MODE_HIGHPASS

#define XRFDC_CLOCK_DETECT_CLK 0x1U
#define XRFDC_CLOCK_DETECT_DIST 0x2U
#define XRFDC_CLOCK_DETECT_BOTH 0x3U

#define XRFDC_CAL_UNFREEZE_CALIB 0U
#define XRFDC_CAL_FREEZE_CALIB 1U
#define XRFDC_CAL_FRZ_PIN_ENABLE 0U
#define XRFDC_CAL_FRZ_PIN_DISABLE 1U

#define XRFDC_CLK_REG_EN_MASK 0x2000U

#define XRFDC_GEN1_LOW_I 20000U
#define XRFDC_GEN1_HIGH_I 32000U
#define XRFDC_MIN_I_UA(X) ((X == XRFDC_ES1_SI) ? 6425U : 2250U)
#define XRFDC_MAX_I_UA(X) ((X == XRFDC_ES1_SI) ? 32000U : 40500U)
#define XRFDC_MIN_I_UA_INT(X) ((X == XRFDC_ES1_SI) ? 6425U : 1400U)
#define XRFDC_MAX_I_UA_INT(X) ((X == XRFDC_ES1_SI) ? 32000U : 46000U)
#define XRFDC_STEP_I_UA(X) ((X == XRFDC_ES1_SI) ? 25.0 : 43.75)
#define XRFDC_BLDR_GAIN 0x0000U
#define XRFDC_CSCAS_BLDR 0xE000U
#define XRFDC_OPCAS_BIAS 0x001BU

#define XRFDC_MAX_ATTEN(X) ((X == 0) ? 11.0 : 27.0)
#define XRFDC_MIN_ATTEN 0
#define XRFDC_STEP_ATTEN(X) ((X == 0) ? 0.5 : 1.0)

#define XRFDC_DAC_VOP_CTRL_REG_UPDT_MASK 0x2U
#define XRFDC_DAC_VOP_CTRL_TST_BLD_MASK 0x1U
#define XRFDC_DAC_VOP_BLDR_LOW_BITS_MASK 0xFU

#define XRFDC_PLL_LOCK_DLY_CNT 1000U

#define XRFDC_CLK_DIV_DP_FIRST_MODE 0x10U
#define XRFDC_CLK_DIV_DP_OTHER_MODES 0x20U

#define XRFDC_TILE_STARTED XRFDC_SM_STATE15

#define XRFDC_SI_REV_ES 0U
#define XRFDC_SI_REV_PROD 1U

#define XRFDC_CG_WAIT_CYCLES 3U
#define XRFDC_ADC_CG_WAIT_CYCLES 1U

#define XRFDC_CG_CYCLES_TOTAL_X1_X2_X4_X8 0U
#define XRFDC_CG_CYCLES_KEPT_X1_X2_X4_X8 1U
#define XRFDC_CG_CYCLES_TOTAL_X3_X6_X12 3U
#define XRFDC_CG_CYCLES_KEPT_X3_X6_X12 2U
#define XRFDC_CG_CYCLES_TOTAL_X5_X10 5U
#define XRFDC_CG_CYCLES_KEPT_X5_X10 4U
#define XRFDC_CG_CYCLES_TOTAL_X16 2U
#define XRFDC_CG_CYCLES_KEPT_X16 1U
#define XRFDC_CG_CYCLES_TOTAL_X20 5U
#define XRFDC_CG_CYCLES_KEPT_X20 2U
#define XRFDC_CG_CYCLES_TOTAL_X24 3U
#define XRFDC_CG_CYCLES_KEPT_X24 1U
#define XRFDC_CG_CYCLES_TOTAL_X40 5U
#define XRFDC_CG_CYCLES_KEPT_X40 1U

#define XRFDC_CG_FIXED_OFS 2U

#define XRFDC_FIFO_CHANNEL_ACT 0U
#define XRFDC_FIFO_CHANNEL_OBS 1U
#define XRFDC_FIFO_CHANNEL_BOTH 2U

#define XRFDC_PWR_MODE_OFF 0U
#define XRFDC_PWR_MODE_ON 1U

#define XRFDC_DUAL_TILE 2U
#define XRFDC_QUAD_TILE 4U

/************************** Function Prototypes ******************************/

XRFdc_Config *XRFdc_LookupConfig(u16 DeviceId);
u32 XRFdc_RegisterMetal(XRFdc *InstancePtr, u16 DeviceId, struct metal_device **DevicePtr);
u32 XRFdc_CfgInitialize(XRFdc *InstancePtr, XRFdc_Config *ConfigPtr);
u32 XRFdc_StartUp(XRFdc *InstancePtr, u32 Type, int Tile_Id);
u32 XRFdc_Shutdown(XRFdc *InstancePtr, u32 Type, int Tile_Id);
u32 XRFdc_Reset(XRFdc *InstancePtr, u32 Type, int Tile_Id);
u32 XRFdc_GetIPStatus(XRFdc *InstancePtr, XRFdc_IPStatus *IPStatusPtr);
u32 XRFdc_GetBlockStatus(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, XRFdc_BlockStatus *BlockStatusPtr);
u32 XRFdc_SetMixerSettings(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id,
			   XRFdc_Mixer_Settings *MixerSettingsPtr);
u32 XRFdc_GetMixerSettings(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id,
			   XRFdc_Mixer_Settings *MixerSettingsPtr);
u32 XRFdc_SetQMCSettings(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, XRFdc_QMC_Settings *QMCSettingsPtr);
u32 XRFdc_GetQMCSettings(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, XRFdc_QMC_Settings *QMCSettingsPtr);
u32 XRFdc_GetCoarseDelaySettings(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id,
				 XRFdc_CoarseDelay_Settings *CoarseDelaySettingsPtr);
u32 XRFdc_SetCoarseDelaySettings(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id,
				 XRFdc_CoarseDelay_Settings *CoarseDelaySettingsPtr);
u32 XRFdc_GetInterpolationFactor(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 *InterpolationFactorPtr);
u32 XRFdc_GetDecimationFactor(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 *DecimationFactorPtr);
u32 XRFdc_GetDecimationFactorObs(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 *DecimationFactorPtr);
u32 XRFdc_GetFabWrVldWords(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, u32 *FabricDataRatePtr);
u32 XRFdc_GetFabWrVldWordsObs(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, u32 *FabricDataRatePtr);
u32 XRFdc_GetFabRdVldWords(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, u32 *FabricDataRatePtr);
u32 XRFdc_GetFabRdVldWordsObs(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, u32 *FabricDataRatePtr);
u32 XRFdc_SetFabRdVldWords(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 FabricRdVldWords);
u32 XRFdc_SetFabRdVldWordsObs(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 FabricRdVldWords);
u32 XRFdc_SetFabWrVldWords(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 FabricWrVldWords);
u32 XRFdc_GetThresholdSettings(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id,
			       XRFdc_Threshold_Settings *ThresholdSettingsPtr);
u32 XRFdc_SetThresholdSettings(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id,
			       XRFdc_Threshold_Settings *ThresholdSettingsPtr);
u32 XRFdc_SetDecoderMode(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 DecoderMode);
u32 XRFdc_UpdateEvent(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, u32 Event);
u32 XRFdc_GetDecoderMode(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 *DecoderModePtr);
u32 XRFdc_ResetNCOPhase(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id);
void XRFdc_DumpRegs(XRFdc *InstancePtr, u32 Type, int Tile_Id);
u32 XRFdc_MultiBand(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u8 DigitalDataPathMask, u32 MixerInOutDataType,
		    u32 DataConverterMask);
int XRFdc_GetConnectedIData(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id);
int XRFdc_GetConnectedQData(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id);
u32 XRFdc_GetConnectedIQData(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, int *ConnectedIData,
			     int *ConnectedQData);
u32 XRFdc_IntrHandler(u32 Vector, void *XRFdcPtr);
u32 XRFdc_IntrClr(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, u32 IntrMask);
u32 XRFdc_GetIntrStatus(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, u32 *IntrStsPtr);
u32 XRFdc_IntrDisable(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, u32 IntrMask);
u32 XRFdc_IntrEnable(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, u32 IntrMask);
u32 XRFdc_GetEnabledInterrupts(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, u32 *IntrMask);
u32 XRFdc_SetThresholdClrMode(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 ThresholdToUpdate, u32 ClrMode);
u32 XRFdc_ThresholdStickyClear(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 ThresholdToUpdate);
void XRFdc_SetStatusHandler(XRFdc *InstancePtr, void *CallBackRef, XRFdc_StatusHandler FunctionPtr);
u32 XRFdc_SetupFIFO(XRFdc *InstancePtr, u32 Type, int Tile_Id, u8 Enable);
u32 XRFdc_SetupFIFOObs(XRFdc *InstancePtr, u32 Type, int Tile_Id, u8 Enable);
u32 XRFdc_SetupFIFOBoth(XRFdc *InstancePtr, u32 Type, int Tile_Id, u8 Enable);
u32 XRFdc_GetFIFOStatus(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u8 *EnablePtr);
u32 XRFdc_GetFIFOStatusObs(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u8 *EnablePtr);
u32 XRFdc_SetNyquistZone(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, u32 NyquistZone);
u32 XRFdc_GetNyquistZone(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, u32 *NyquistZonePtr);
u32 XRFdc_GetOutputCurr(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 *OutputCurrPtr);
u32 XRFdc_SetDecimationFactor(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 DecimationFactor);
u32 XRFdc_SetDecimationFactorObs(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 DecimationFactor);
u32 XRFdc_SetInterpolationFactor(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 InterpolationFactor);
u32 XRFdc_SetFabClkOutDiv(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u16 FabClkDiv);
u32 XRFdc_SetCalibrationMode(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u8 CalibrationMode);
u32 XRFdc_GetCalibrationMode(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u8 *CalibrationModePtr);
u32 XRFdc_GetClockSource(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 *ClockSourcePtr);
u32 XRFdc_GetPLLLockStatus(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 *LockStatusPtr);
u32 XRFdc_GetPLLConfig(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, XRFdc_PLL_Settings *PLLSettings);
u32 XRFdc_DynamicPLLConfig(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u8 Source, double RefClkFreq,
			   double SamplingRate);
u32 XRFdc_SetInvSincFIR(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u16 Mode);
u32 XRFdc_GetInvSincFIR(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u16 *ModePtr);
u32 XRFdc_GetLinkCoupling(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 *ModePtr);
u32 XRFdc_GetFabClkOutDiv(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u16 *FabClkDivPtr);
u32 XRFdc_SetDither(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 Mode);
u32 XRFdc_GetDither(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 *ModePtr);
u32 XRFdc_SetClkDistribution(XRFdc *InstancePtr, XRFdc_Distribution_Settings *DistributionSettingsPtr);
u32 XRFdc_GetClkDistribution(XRFdc *InstancePtr, XRFdc_Distribution_Settings *DistributionSettingsPtr);
u32 XRFdc_SetDataPathMode(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 Mode);
u32 XRFdc_GetDataPathMode(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 *ModePtr);
u32 XRFdc_SetIMRPassMode(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 Mode);
u32 XRFdc_GetIMRPassMode(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 *ModePtr);
u32 XRFdc_SetSignalDetector(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, XRFdc_Signal_Detector_Settings *SettingsPtr);
u32 XRFdc_GetSignalDetector(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, XRFdc_Signal_Detector_Settings *SettingsPtr);
u32 XRFdc_DisableCoefficientsOverride(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 CalibrationBlock);
u32 XRFdc_SetCalCoefficients(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 CalibrationBlock,
			     XRFdc_Calibration_Coefficients *CoeffPtr);
u32 XRFdc_GetCalCoefficients(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 CalibrationBlock,
			     XRFdc_Calibration_Coefficients *CoeffPtr);
u32 XRFdc_SetCalFreeze(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, XRFdc_Cal_Freeze_Settings *CalFreezePtr);
u32 XRFdc_GetCalFreeze(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, XRFdc_Cal_Freeze_Settings *CalFreezePtr);
u32 XRFdc_SetDACVOP(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 uACurrent);
u32 XRFdc_SetDACCompMode(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 Enable);
u32 XRFdc_GetDACCompMode(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, u32 *EnabledPtr);
u32 XRFdc_SetDSA(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, XRFdc_DSA_Settings *SettingsPtr);
u32 XRFdc_GetDSA(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id, XRFdc_DSA_Settings *SettingsPtr);
u32 XRFdc_SetPwrMode(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, XRFdc_Pwr_Mode_Settings *SettingsPtr);
u32 XRFdc_GetPwrMode(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id, XRFdc_Pwr_Mode_Settings *SettingsPtr);
u32 XRFdc_ResetInternalFIFOWidth(XRFdc *InstancePtr, u32 Type, u32 Tile_Id, u32 Block_Id);
u32 XRFdc_ResetInternalFIFOWidthObs(XRFdc *InstancePtr, u32 Tile_Id, u32 Block_Id);