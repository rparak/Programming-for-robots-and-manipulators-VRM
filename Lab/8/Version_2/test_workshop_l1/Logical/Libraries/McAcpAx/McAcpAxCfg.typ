TYPE
	McAMEType : STRUCT (*Parameter of hardware elements situated between motor encoder and load which influence the scaling*)
		Gearbox : McCfgGearBoxType; (*Specifies a gearbox by defining the ratio between a gearbox input and output*)
		RotaryToLinearTransformation : McCfgRotToLinTrfType; (*Specifies a transformation factor between the output of the gear and the actual load movement*)
	END_STRUCT;
	McAELEnum :
		( (*Encoder link selector setting*)
		mcAEL_ONE_ENC := 0, (*One encoder -*)
		mcAEL_TWO_ENC := 1, (*Two encoders -*)
		mcAEL_NO_ENC := 10 (*No encoder -*)
		);
	McAELAllEncEnum :
		( (*Motor and position encoder selector setting*)
		mcAELAE_ENC_X6A := 0, (*Encoder X6A - OnBoard encoder 1*)
		mcAELAE_ENC_X6B := 1, (*Encoder X6B - OnBoard encoder 2*)
		mcAELAE_ENC := 2, (*Encoder*)
		mcAELAE_ENC_SS1X11 := 3, (*Encoder SS1.X11 - Plug-in module in SS1*)
		mcAELAE_ENC_SS2X11 := 4, (*Encoder SS2.X11 - Plug-in module in SS2*)
		mcAELAE_ENC_X11A := 5, (*Encoder X11A*)
		mcAELAE_ENC_SS3X11 := 6, (*Encoder SS3.X11 - Plug-in module in SS3*)
		mcAELAE_ENC_SS4X11 := 7, (*Encoder SS4.X11 - Plug-in module in SS4*)
		mcAELAE_ENC_X41 := 8, (*Encoder X41*)
		mcAELAE_ENC_SS1X41X := 9, (*Encoder SS1.X41x - Plug-in module in SS1*)
		mcAELAE_ENC_X42 := 10, (*Encoder X42*)
		mcAELAE_ENC_SS1X42X := 11, (*Encoder SS1.X42x - Plug-in module in SS1*)
		mcAELAE_ENC_X43 := 12, (*Encoder X43*)
		mcAELAE_ENC_SS1X43X := 13 (*Encoder SS1.X43x - Plug-in module in SS1*)
		);
	McAELOneEncMotAndPosEncType : STRUCT
		Type : McAELAllEncEnum; (*Motor and position encoder selector setting*)
	END_STRUCT;
	McAELOneEncType : STRUCT (*Type mcAEL_ONE_ENC settings*)
		MotorAndPositionEncoder : McAELOneEncMotAndPosEncType;
	END_STRUCT;
	McAELTwoEncMotEncType : STRUCT
		Type : McAELAllEncEnum; (*Motor encoder selector setting*)
	END_STRUCT;
	McAELTwoEncPosEncCmnScCntDirEnum :
		( (*Direction of the encoder in which the position value is increasing*)
		mcAELTEPECSCD_STD := 0, (*Standard - Clockwise*)
		mcAELTEPECSCD_INV := 1 (*Inverse - Counter-clockwise*)
		);
	McAELTwoEncPosEncCmnScType : STRUCT (*Encoder scaling based on a gear ratio and / or a movement transformation factor*)
		Gearbox : McCfgGearBoxType; (*Specifies a gearbox by defining the ratio between a gearbox input and output*)
		RotaryToLinearTransformation : McCfgRotToLinTrfType; (*Specifies a transformation factor between the output of the gear and the actual load movement*)
		CountDirection : McAELTwoEncPosEncCmnScCntDirEnum; (*Direction of the encoder in which the position value is increasing*)
	END_STRUCT;
	McAELTwoEncPosEncCmnType : STRUCT (*Common settings for all Type values*)
		Scaling : McAELTwoEncPosEncCmnScType; (*Encoder scaling based on a gear ratio and / or a movement transformation factor*)
	END_STRUCT;
	McAELTwoEncPosEncType : STRUCT
		Type : McAELAllEncEnum; (*Position encoder selector setting*)
		Common : McAELTwoEncPosEncCmnType; (*Common settings for all Type values*)
	END_STRUCT;
	McAELTwoEncType : STRUCT (*Type mcAEL_TWO_ENC settings*)
		MotorEncoder : McAELTwoEncMotEncType;
		PositionEncoder : McAELTwoEncPosEncType;
		PositionDifferenceLimit : REAL; (*Position difference limit between motor and position encoder for stopping a movement [Measurement units]*)
	END_STRUCT;
	McAELType : STRUCT
		Type : McAELEnum; (*Encoder link selector setting*)
		OneEncoder : McAELOneEncType; (*Type mcAEL_ONE_ENC settings*)
		TwoEncoders : McAELTwoEncType; (*Type mcAEL_TWO_ENC settings*)
	END_STRUCT;
	McACModEnum :
		( (*Mode selector setting*)
		mcACM_POS_CTRL := 0, (*Position controller - Position controller*)
		mcACM_POS_CTRL_TORQ_FF := 1, (*Position controller torque ff - Position controller with torque feed forward*)
		mcACM_V_FREQ_CTRL := 2 (*Voltage frequency control - Voltage/frequency control of induction motor*)
		);
	McACPCType : STRUCT (*Position controller parameters*)
		ProportionalGain : REAL; (*Proportional amplification [1/s]*)
		IntegrationTime : REAL; (*Integral action time [s]*)
		PredictionTime : REAL; (*Prediction time [s]*)
		TotalDelayTime : REAL; (*Total delay time [s]*)
	END_STRUCT;
	McACSCType : STRUCT (*Speed controller parameters*)
		ProportionalGain : REAL; (*Proportional amplification [As]*)
		IntegrationTime : REAL; (*Integral action time [s]*)
		FilterTime : REAL; (*Filter time constant [s]*)
	END_STRUCT;
	McACLFSEnum :
		( (*Loop filter 1-3 selector setting*)
		mcACLFS_NOT_USE := 0, (*Not used - Filter is not active*)
		mcACLFS_LP_2ND_ORD := 1, (*Lowpass 2nd order - Lowpass 2nd order*)
		mcACLFS_NOTCH := 2, (*Notch - Notch*)
		mcACLFS_BIQUAD := 3, (*Biquad - Biquad*)
		mcACLFS_DISC_TIME_TRAN_FUN := 4, (*Discrete time transfer function - Discrete time transfer function*)
		mcACLFS_LIM := 5, (*Limiter - Limiter*)
		mcACLFS_LIN_LIM := 6, (*Linear limitation - Linear limitation*)
		mcACLFS_RISE_TIME_LIM := 7, (*Rise time limitation - Rise time limitation*)
		mcACLFS_COMP := 8 (*Compensation - Compensation*)
		);
	McACLFSLP2ndOrdType : STRUCT (*Type mcACLFS_LP_2ND_ORD settings*)
		CutOffFrequency : REAL; (*Cut off frequency [Hz]*)
	END_STRUCT;
	McACLFSNotchType : STRUCT (*Type mcACLFS_NOTCH settings*)
		CenterFrequency : REAL; (*Center frequency [Hz]*)
		Bandwidth : REAL; (*Bandwidth [Hz]*)
	END_STRUCT;
	McACLFSBiquadType : STRUCT (*Type mcACLFS_BIQUAD settings*)
		FrequencyNumerator : REAL; (*Frequency numerator [Hz]*)
		DampingNumerator : REAL; (*Damping numerator*)
		FrequencyDenominator : REAL; (*Frequency denominator [Hz]*)
		DampingDenominator : REAL; (*Damping Denominator*)
	END_STRUCT;
	McACLFSDiscTimeTranFunType : STRUCT (*Type mcACLFS_DISC_TIME_TRAN_FUN settings*)
		a0DenominatorPolynomial : REAL; (*Coefficient a0 of denominator polynomial*)
		a1DenominatorPolynomial : REAL; (*Coefficient a1 of denominator polynomial*)
		b0NumeratorPolynomial : REAL; (*Coefficient b0 of numerator polynomial*)
		b1NumeratorPolynomial : REAL; (*Coefficient b1 of numerator polynomial*)
		b2NumeratorPolynomial : REAL; (*Coefficient b2 of numerator polynomial*)
	END_STRUCT;
	McACLFSLLimEnum :
		( (*Positive limit selector setting*)
		mcACLFSLLim_ACP_PARID := 0, (*ACOPOS ParID - Use an ACOPOS ParID*)
		mcACLFSLLim_FIX_VAL := 1 (*Fixed value - Use a fixed value*)
		);
	McACLFSLLimAcpParIDType : STRUCT (*Type mcACLFSLLim_ACP_PARID settings*)
		ParID : UINT; (*ParID*)
	END_STRUCT;
	McACLFSLLimFixValType : STRUCT (*Type mcACLFSLLim_FIX_VAL settings*)
		Value : REAL; (*Value [Nm]*)
	END_STRUCT;
	McACLFSLLimType : STRUCT (*Select if a fixed value for the positive limit or if the value of an ACOPOS ParID is used therefore*)
		Type : McACLFSLLimEnum; (*Positive limit selector setting*)
		ACOPOSParID : McACLFSLLimAcpParIDType; (*Type mcACLFSLLim_ACP_PARID settings*)
		FixedValue : McACLFSLLimFixValType; (*Type mcACLFSLLim_FIX_VAL settings*)
	END_STRUCT;
	McACLFSLimType : STRUCT (*Type mcACLFS_LIM settings*)
		PositiveLimit : McACLFSLLimType; (*Select if a fixed value for the positive limit or if the value of an ACOPOS ParID is used therefore*)
		NegativeLimit : McACLFSLLimType; (*Select if a fixed value for the negative limit or if the value of an ACOPOS ParID is used therefore*)
	END_STRUCT;
	McACLFSLinLimType : STRUCT (*Type mcACLFS_LIN_LIM settings*)
		InputParID : UINT; (*ACOPOS ParID for the input*)
		InputLimit : REAL; (*Input limit value for full limitation*)
		Gradient : REAL; (*Gradient*)
	END_STRUCT;
	McACLFSRiseTimeLimType : STRUCT (*Type mcACLFS_RISE_TIME_LIM settings*)
		RiseTime : REAL; (*Rise time [s]*)
		NormalizedLimit : REAL; (*Normalized limit*)
	END_STRUCT;
	McACLFSCompType : STRUCT (*Type mcACLFS_COMP settings*)
		MultiplicationFactorParID : UINT; (*Multiplication Factor ParID*)
		AdditiveValueParID : UINT; (*Additive Value ParID*)
	END_STRUCT;
	McACLFSType : STRUCT (*Type of the loop filter*)
		Type : McACLFSEnum; (*Loop filter 1-3 selector setting*)
		Lowpass2ndOrder : McACLFSLP2ndOrdType; (*Type mcACLFS_LP_2ND_ORD settings*)
		Notch : McACLFSNotchType; (*Type mcACLFS_NOTCH settings*)
		Biquad : McACLFSBiquadType; (*Type mcACLFS_BIQUAD settings*)
		DiscreteTimeTransferFunction : McACLFSDiscTimeTranFunType; (*Type mcACLFS_DISC_TIME_TRAN_FUN settings*)
		Limiter : McACLFSLimType; (*Type mcACLFS_LIM settings*)
		LinearLimitation : McACLFSLinLimType; (*Type mcACLFS_LIN_LIM settings*)
		RiseTimeLimitation : McACLFSRiseTimeLimType; (*Type mcACLFS_RISE_TIME_LIM settings*)
		Compensation : McACLFSCompType; (*Type mcACLFS_COMP settings*)
	END_STRUCT;
	McACLFType : STRUCT (*Parameters of the loop filters*)
		LoopFilter : ARRAY[0..2] OF McACLFSType; (*Type of the loop filter*)
	END_STRUCT;
	McACMPCType : STRUCT (*Type mcACM_POS_CTRL settings*)
		Position : McACPCType; (*Position controller parameters*)
		Speed : McACSCType; (*Speed controller parameters*)
		LoopFilters : McACLFType; (*Parameters of the loop filters*)
	END_STRUCT;
	McACPCFFType : STRUCT (*Position controller parameters*)
		ProportionalGain : REAL; (*Proportional amplification [1/s]*)
		IntegrationTime : REAL; (*Integral action time [s]*)
		TotalDelayTime : REAL; (*Total delay time [s]*)
	END_STRUCT;
	McACMPCFFFFwdEnum :
		( (*Feed forward selector setting*)
		mcACMPCFFFF_STD := 0, (*Standard - Based on several parameters the torque feed forward calculation is done on the axis*)
		mcACMPCFFFF_CYC_VAL_FROM_AX_GRP := 1 (*Cyclic value from axes group - The torque feed forward calculation is done by the axes group on the PLC and the value is forwarded to the axis*)
		);
	McACMPCFFFFwdStdType : STRUCT (*Type mcACMPCFFFF_STD settings*)
		TorqueLoad : REAL; (*Load torque [Nm]*)
		TorquePositive : REAL; (*Torque in positive direction [Nm]*)
		TorqueNegative : REAL; (*Torque in negative direction [Nm]*)
		SpeedTorqueFactor : REAL; (*Speed torque factor [Nms]*)
		Inertia : REAL; (*Mass moment of inertia [kgm²]*)
		AccelerationFilterTime : REAL; (*Acceleration filter time constant [s]*)
	END_STRUCT;
	McACMPCFFFFwdType : STRUCT (*Torque feed forward control parameters*)
		Type : McACMPCFFFFwdEnum; (*Feed forward selector setting*)
		Standard : McACMPCFFFFwdStdType; (*Type mcACMPCFFFF_STD settings*)
	END_STRUCT;
	McACMPCFFType : STRUCT (*Type mcACM_POS_CTRL_TORQ_FF settings*)
		Position : McACPCFFType; (*Position controller parameters*)
		Speed : McACSCType; (*Speed controller parameters*)
		FeedForward : McACMPCFFFFwdType; (*Torque feed forward control parameters*)
		LoopFilters : McACLFType; (*Parameters of the loop filters*)
	END_STRUCT;
	McACMVFCVFTypEnum :
		( (*Type of characteristic curve*)
		mcACMVFCVFT_LIN := 129, (*Linear - Linear characteristic curve*)
		mcACMVFCVFT_CONST_LD_TORQ := 131, (*Constant load torque - Characteristic curve for constant load torque*)
		mcACMVFCVFT_QUAD := 130 (*Quadratic - Characteristic curve for quadratic load curves*)
		);
	McACMVFCVFAutCfgEnum :
		( (*Automatic configuration selector setting*)
		mcACMVFCVFAC_MOT_PAR_BASED := 1, (*Motor parameter based - Automatic configuration based on motor parameters*)
		mcACMVFCVFAC_NOT_USE := 0 (*Not used - No automatic configuration*)
		);
	McACMVFCVFAutCfgNotUseType : STRUCT (*Type mcACMVFCVFAC_NOT_USE settings*)
		BoostVoltage : REAL; (*Boost voltage [V]*)
		RatedVoltage : REAL; (*Rated voltage [V]*)
		RatedFrequency : REAL; (*Rated frequency [cps]*)
	END_STRUCT;
	McACMVFCVFAutCfgType : STRUCT (*Automatic configuration of parameters*)
		Type : McACMVFCVFAutCfgEnum; (*Automatic configuration selector setting*)
		NotUsed : McACMVFCVFAutCfgNotUseType; (*Type mcACMVFCVFAC_NOT_USE settings*)
	END_STRUCT;
	McACMVFCVFType : STRUCT (*V/f control parameters*)
		Type : McACMVFCVFTypEnum; (*Type of characteristic curve*)
		AutomaticConfiguration : McACMVFCVFAutCfgType; (*Automatic configuration of parameters*)
		SlipCompensation : REAL; (*Slip compensation: Multiplication factor of compensated frequency*)
		TotalDelayTime : REAL; (*Total delay time [s]*)
	END_STRUCT;
	McACMVFCType : STRUCT (*Type mcACM_V_FREQ_CTRL settings*)
		VoltageFrequency : McACMVFCVFType; (*V/f control parameters*)
	END_STRUCT;
	McACModType : STRUCT (*Mode of the axis controller*)
		Type : McACModEnum; (*Mode selector setting*)
		PositionController : McACMPCType; (*Type mcACM_POS_CTRL settings*)
		PositionControllerTorqueFf : McACMPCFFType; (*Type mcACM_POS_CTRL_TORQ_FF settings*)
		VoltageFrequencyControl : McACMVFCType; (*Type mcACM_V_FREQ_CTRL settings*)
	END_STRUCT;
	McACType : STRUCT (*Axis controller parameters*)
		Mode : McACModType; (*Mode of the axis controller*)
	END_STRUCT;
	McAHModEnum :
		( (*Mode selector setting*)
		mcAHM_DIR := 0, (*Direct - Direct homing*)
		mcAHM_ABS_SW := 1, (*Absolute switch - Homing with absolute reference switch*)
		mcAHM_SW_GATE := 2, (*Switch gate - Homing with reference switch gate*)
		mcAHM_LIM_SW := 3, (*Limit switch - Homing with hardware end switch*)
		mcAHM_ABS := 4, (*Absolute - Homing by setting the home offset*)
		mcAHM_ABS_CORR := 5, (*Absolute correction - Homing by setting the home offset with counting range correction*)
		mcAHM_DIST_C_MARKS := 6, (*Distance coded marks - Homing with distance coded reference marks*)
		mcAHM_DIST_C_MARKS_CORR := 7, (*Distance coded marks correction - Homing with distance coded reference marks and counting range correction*)
		mcAHM_BLK_TORQ := 8, (*Block torque - Homing on block, criterion for homing event: torque*)
		mcAHM_BLK_LAG_ERR := 9, (*Block lag error - Homing on block, criterion for homing event: lag error*)
		mcAHM_RES_POS := 10, (*Restore position - Homing by restoring the position from permanent variable data*)
		mcAHM_NOT_USE := 100 (*Not used - No preconfigured homing settings used*)
		);
	McAHModDirRefPEnum :
		( (*Reference pulse selector setting*)
		mcAHMDRP_NOT_USE := 0, (*Not used - Reference pulse is not used*)
		mcAHMDRP_USE := 1 (*Used - Reference pulse is used*)
		);
	McAHModHomeDirEnum :
		( (*Movement direction in which the homing event is evaluated*)
		mcAHMHD_POS := 0, (*Positive - Positive movement direction*)
		mcAHMHD_NEG := 1 (*Negative - Negative movement direction*)
		);
	McAHModKeepDirEnum :
		( (*Keep direction (move only in one direction)*)
		mcAHMKD_NO := 0, (*No - mcSWITCH_OFF*)
		mcAHMKD_YES := 1 (*Yes - mcSWITCH_ON*)
		);
	McAHModDirRefPUseType : STRUCT (*Type mcAHMDRP_USE settings*)
		HomingVelocity : REAL; (*Speed which is used while searching for the homing event (e.g. after reference switch has been reached) [Measurement units/s]*)
		Acceleration : REAL; (*Acceleration for homing movement [Measurement units/s²]*)
		HomingDirection : McAHModHomeDirEnum; (*Movement direction in which the homing event is evaluated*)
		KeepDirection : McAHModKeepDirEnum; (*Keep direction (move only in one direction)*)
		ReferencePulseBlockingDistance : LREAL; (*Distance for blocking the activation of triggering reference pulse [Measurement units]*)
	END_STRUCT;
	McAHModDirRefPType : STRUCT (*Use reference pulse of encoder*)
		Type : McAHModDirRefPEnum; (*Reference pulse selector setting*)
		Used : McAHModDirRefPUseType; (*Type mcAHMDRP_USE settings*)
	END_STRUCT;
	McAHModDirType : STRUCT (*Type mcAHM_DIR settings*)
		Position : LREAL; (*Home position [Measurement units]*)
		ReferencePulse : McAHModDirRefPType; (*Use reference pulse of encoder*)
	END_STRUCT;
	McAHModSwEdgEnum :
		( (*Edge of reference switch*)
		mcAHMSE_POS := 0, (*Positive - Positive switch edge*)
		mcAHMSE_NEG := 1 (*Negative - Negative switch edge*)
		);
	McAHModRefPEnum :
		( (*Reference pulse selector setting*)
		mcAHMRP_NOT_USE := 0, (*Not used - Reference pulse is not used*)
		mcAHMRP_USE := 1 (*Used - Reference pulse is used*)
		);
	McAHModRefPUseType : STRUCT (*Type mcAHMRP_USE settings*)
		ReferencePulseBlockingDistance : LREAL; (*Distance for blocking the activation of triggering reference pulse [Measurement units]*)
	END_STRUCT;
	McAHModRefPType : STRUCT (*Use reference pulse of encoder*)
		Type : McAHModRefPEnum; (*Reference pulse selector setting*)
		Used : McAHModRefPUseType; (*Type mcAHMRP_USE settings*)
	END_STRUCT;
	McAHModAbsSwType : STRUCT (*Type mcAHM_ABS_SW settings*)
		Position : LREAL; (*Home position [Measurement units]*)
		StartVelocity : REAL; (*Speed for searching the reference switch [Measurement units/s]*)
		HomingVelocity : REAL; (*Speed which is used while searching for the homing event (e.g. after reference switch has been reached) [Measurement units/s]*)
		Acceleration : REAL; (*Acceleration for homing movement [Measurement units/s²]*)
		SwitchEdge : McAHModSwEdgEnum; (*Edge of reference switch*)
		HomingDirection : McAHModHomeDirEnum; (*Movement direction in which the homing event is evaluated*)
		KeepDirection : McAHModKeepDirEnum; (*Keep direction (move only in one direction)*)
		ReferencePulse : McAHModRefPType; (*Use reference pulse of encoder*)
	END_STRUCT;
	McAHModStartDirEnum :
		( (*Start direction of movement for searching the reference switch*)
		mcAHMSD_POS := 0, (*Positive - Positive movement direction*)
		mcAHMSD_NEG := 1 (*Negative - Negative movement direction*)
		);
	McAHModSwGateType : STRUCT (*Type mcAHM_SW_GATE settings*)
		Position : LREAL; (*Home position [Measurement units]*)
		StartVelocity : REAL; (*Speed for searching the reference switch [Measurement units/s]*)
		HomingVelocity : REAL; (*Speed which is used while searching for the homing event (e.g. after reference switch has been reached) [Measurement units/s]*)
		Acceleration : REAL; (*Acceleration for homing movement [Measurement units/s²]*)
		SwitchEdge : McAHModSwEdgEnum; (*Edge of reference switch*)
		StartDirection : McAHModStartDirEnum; (*Start direction of movement for searching the reference switch*)
		HomingDirection : McAHModHomeDirEnum; (*Movement direction in which the homing event is evaluated*)
		KeepDirection : McAHModKeepDirEnum; (*Keep direction (move only in one direction)*)
		ReferencePulse : McAHModRefPType; (*Use reference pulse of encoder*)
	END_STRUCT;
	McAHModLimSwType : STRUCT (*Type mcAHM_LIM_SW settings*)
		Position : LREAL; (*Home position [Measurement units]*)
		StartVelocity : REAL; (*Speed for searching the reference switch [Measurement units/s]*)
		HomingVelocity : REAL; (*Speed which is used while searching for the homing event (e.g. after reference switch has been reached) [Measurement units/s]*)
		Acceleration : REAL; (*Acceleration for homing movement [Measurement units/s²]*)
		SwitchEdge : McAHModSwEdgEnum; (*Edge of reference switch*)
		HomingDirection : McAHModHomeDirEnum; (*Movement direction in which the homing event is evaluated*)
		KeepDirection : McAHModKeepDirEnum; (*Keep direction (move only in one direction)*)
		ReferencePulse : McAHModRefPType; (*Use reference pulse of encoder*)
	END_STRUCT;
	McAHModAbsType : STRUCT (*Type mcAHM_ABS settings*)
		Position : LREAL; (*Home offset [Measurement units]*)
	END_STRUCT;
	McAHModAbsCorrType : STRUCT (*Type mcAHM_ABS_CORR settings*)
		Position : LREAL; (*Home offset [Measurement units]*)
	END_STRUCT;
	McAHModDistCMarksType : STRUCT (*Type mcAHM_DIST_C_MARKS settings*)
		Position : LREAL; (*Home position [Measurement units]*)
		HomingVelocity : REAL; (*Speed which is used while searching for the homing event (e.g. after reference switch has been reached) [Measurement units/s]*)
		Acceleration : REAL; (*Acceleration for homing movement [Measurement units/s²]*)
		HomingDirection : McAHModHomeDirEnum; (*Movement direction in which the homing event is evaluated*)
		KeepDirection : McAHModKeepDirEnum; (*Keep direction (move only in one direction)*)
	END_STRUCT;
	McAHModDistCMarksCorrType : STRUCT (*Type mcAHM_DIST_C_MARKS_CORR settings*)
		Position : LREAL; (*Home position [Measurement units]*)
		HomingVelocity : REAL; (*Speed which is used while searching for the homing event (e.g. after reference switch has been reached) [Measurement units/s]*)
		Acceleration : REAL; (*Acceleration for homing movement [Measurement units/s²]*)
		HomingDirection : McAHModHomeDirEnum; (*Movement direction in which the homing event is evaluated*)
		KeepDirection : McAHModKeepDirEnum; (*Keep direction (move only in one direction)*)
	END_STRUCT;
	McAHModBlkTorqType : STRUCT (*Type mcAHM_BLK_TORQ settings*)
		Position : LREAL; (*Home position [Measurement units]*)
		StartVelocity : REAL; (*Speed for searching the reference switch [Measurement units/s]*)
		HomingVelocity : REAL; (*Speed which is used while searching for the homing event (e.g. after reference switch has been reached) [Measurement units/s]*)
		Acceleration : REAL; (*Acceleration for homing movement [Measurement units/s²]*)
		StartDirection : McAHModStartDirEnum; (*Start direction of movement for searching the reference switch*)
		HomingDirection : McAHModHomeDirEnum; (*Movement direction in which the homing event is evaluated*)
		ReferencePulse : McAHModRefPType; (*Use reference pulse of encoder*)
		TorqueLimit : REAL; (*Torque limit for homing on block [Nm]*)
		PositionErrorStopLimit : LREAL; (*Lag error for stop of the homing movement [Measurement units]*)
	END_STRUCT;
	McAHModBlkLagErrType : STRUCT (*Type mcAHM_BLK_LAG_ERR settings*)
		Position : LREAL; (*Home position [Measurement units]*)
		StartVelocity : REAL; (*Speed for searching the reference switch [Measurement units/s]*)
		HomingVelocity : REAL; (*Speed which is used while searching for the homing event (e.g. after reference switch has been reached) [Measurement units/s]*)
		Acceleration : REAL; (*Acceleration for homing movement [Measurement units/s²]*)
		StartDirection : McAHModStartDirEnum; (*Start direction of movement for searching the reference switch*)
		HomingDirection : McAHModHomeDirEnum; (*Movement direction in which the homing event is evaluated*)
		ReferencePulse : McAHModRefPType; (*Use reference pulse of encoder*)
		TorqueLimit : REAL; (*Torque limit for homing on block [Nm]*)
		PositionErrorStopLimit : LREAL; (*Lag error for stop of the homing movement [Measurement units]*)
		BlockDetectionPositionError : LREAL; (*Lag error for block detection [Measurement units]*)
	END_STRUCT;
	McAHModType : STRUCT (*Homing mode*)
		Type : McAHModEnum; (*Mode selector setting*)
		Direct : McAHModDirType; (*Type mcAHM_DIR settings*)
		AbsoluteSwitch : McAHModAbsSwType; (*Type mcAHM_ABS_SW settings*)
		SwitchGate : McAHModSwGateType; (*Type mcAHM_SW_GATE settings*)
		LimitSwitch : McAHModLimSwType; (*Type mcAHM_LIM_SW settings*)
		Absolute : McAHModAbsType; (*Type mcAHM_ABS settings*)
		AbsoluteCorrection : McAHModAbsCorrType; (*Type mcAHM_ABS_CORR settings*)
		DistanceCodedMarks : McAHModDistCMarksType; (*Type mcAHM_DIST_C_MARKS settings*)
		DistanceCodedMarksCorrection : McAHModDistCMarksCorrType; (*Type mcAHM_DIST_C_MARKS_CORR settings*)
		BlockTorque : McAHModBlkTorqType; (*Type mcAHM_BLK_TORQ settings*)
		BlockLagError : McAHModBlkLagErrType; (*Type mcAHM_BLK_LAG_ERR settings*)
	END_STRUCT;
	McAHType : STRUCT (*Homing mode and parameters which can be used within the application program as preconfigured setting*)
		Mode : McAHModType; (*Homing mode*)
		RestorePositionVariable : STRING[250]; (*Permanent variable used for homing mode: Restore position*)
	END_STRUCT;
	McASRQstopEnum :
		( (*Quickstop selector setting*)
		mcASRQ_DEC_LIM := 0, (*Deceleration limit - Stop with deceleration limits*)
		mcASRQ_DEC_LIM_W_JERK_FLTR := 1, (*Deceleration limit with jerk filter - Stop with deceleration limits and jerk filter*)
		mcASRQ_TORQ_LIM := 2, (*Torque limit - Stop with torque limits*)
		mcASRQ_INDUCT_HALT := 3 (*Induction halt - Stop with an induction halt*)
		);
	McASRQstopDecLimWJerkFltrType : STRUCT (*Type mcASRQ_DEC_LIM_W_JERK_FLTR settings*)
		JerkTime : REAL; (*Jerk filter time, max. Axis Jerk time / 2 [s]*)
	END_STRUCT;
	McASRQstopType : STRUCT (*Deceleration ramp / reaction in case of a quickstop which is caused by an active quickstop input*)
		Type : McASRQstopEnum; (*Quickstop selector setting*)
		DecelerationLimitWithJerkFilter : McASRQstopDecLimWJerkFltrType; (*Type mcASRQ_DEC_LIM_W_JERK_FLTR settings*)
	END_STRUCT;
	McASRDrvErrEnum :
		( (*Drive error selector setting*)
		mcASRDE_DEC_LIM := 0, (*Deceleration limit - Stop with deceleration limits*)
		mcASRDE_INDUCT_HALT := 1, (*Induction halt - Stop with an induction halt*)
		mcASRDE_COAST_TO_STANDSTILL := 2 (*Coast to standstill - Controller is deactivated*)
		);
	McASRDrvErrType : STRUCT (*Deceleration ramp / reaction in case of an error stop which is caused by a drive error*)
		Type : McASRDrvErrEnum; (*Drive error selector setting*)
	END_STRUCT;
	McASRType : STRUCT (*Reactions of the axis in case of certain stop conditions*)
		Quickstop : McASRQstopType; (*Deceleration ramp / reaction in case of a quickstop which is caused by an active quickstop input*)
		DriveError : McASRDrvErrType; (*Deceleration ramp / reaction in case of an error stop which is caused by a drive error*)
	END_STRUCT;
	McAMELVelErrMonEnum :
		( (*Velocity error monitoring selector setting*)
		mcAMELVEM_AUT_1 := 0, (*Automatic 1 - Initialize the stop limit automatically by using calculation method 1*)
		mcAMELVEM_AUT_2 := 1, (*Automatic 2 - Initialize the stop limit automatically by using calculation method 2*)
		mcAMELVEM_USRDEF := 2, (*User-defined - Initialize the stop limit with the value of the parameter velocity error*)
		mcAMELVEM_NOT_USE := 3 (*Not used - Velocity error monitoring is not active*)
		);
	McAMELVelErrMonUsrDefType : STRUCT (*Type mcAMELVEM_USRDEF settings*)
		VelocityError : REAL; (*Velocity error limit for stopping a movement [Measurement units/s]*)
	END_STRUCT;
	McAMELVelErrMonType : STRUCT (*Velocity error monitoring mode*)
		Type : McAMELVelErrMonEnum; (*Velocity error monitoring selector setting*)
		UserDefined : McAMELVelErrMonUsrDefType; (*Type mcAMELVEM_USRDEF settings*)
	END_STRUCT;
	McAMELType : STRUCT (*Limit values that result in a stop reaction when exceeded*)
		PositionError : LREAL; (*Lag error limit for stopping a movement [Measurement units]*)
		VelocityErrorMonitoring : McAMELVelErrMonType; (*Velocity error monitoring mode*)
	END_STRUCT;
	McAJFEnum :
		( (*Jerk filter selector setting*)
		mcAJF_NOT_USE := 0, (*Not used - No jerk filter is applied*)
		mcAJF_USE := 1 (*Used - Jerk filter is applied*)
		);
	McAJFUseType : STRUCT (*Type mcAJF_USE settings*)
		JerkTime : REAL; (*Jerk filter time [s]*)
	END_STRUCT;
	McAJFType : STRUCT (*Jerk filter*)
		Type : McAJFEnum; (*Jerk filter selector setting*)
		Used : McAJFUseType; (*Type mcAJF_USE settings*)
	END_STRUCT;
	McADIAllSrcEnum :
		( (*Source selector setting*)
		mcADIAS_NOT_USE := 0, (*Not used -*)
		mcADIAS_DIG_IN_X8TRG_1 := 1, (*Digital in X8.Trigger 1 -*)
		mcADIAS_DIG_IN_X8TRG_2 := 2, (*Digital in X8.Trigger 2 -*)
		mcADIAS_DIG_IN_SS1X41X1 := 3, (*Digital in SS1.X41x.1 -*)
		mcADIAS_DIG_IN_SS1X41X2 := 4, (*Digital in SS1.X41x.2 -*)
		mcADIAS_DIG_IN_SS1X41X3 := 5, (*Digital in SS1.X41x.3 -*)
		mcADIAS_DIG_IN_SS1X41X4 := 6, (*Digital in SS1.X41x.4 -*)
		mcADIAS_DIG_IN_SS1X41X5 := 7, (*Digital in SS1.X41x.5 -*)
		mcADIAS_DIG_IN_SS1X41X6 := 8, (*Digital in SS1.X41x.6 -*)
		mcADIAS_DIG_IN_SS1X41X7 := 9, (*Digital in SS1.X41x.7 -*)
		mcADIAS_DIG_IN_SS1X41X8 := 10, (*Digital in SS1.X41x.8 -*)
		mcADIAS_DIG_IN_SS1X41X9 := 11, (*Digital in SS1.X41x.9 -*)
		mcADIAS_DIG_IN_SS1X41X10 := 12 (*Digital in SS1.X41x.10 -*)
		);
	McADIHomeSwSrcType : STRUCT (*Source of the digital input hardware which is used for this functionality*)
		Type : McADIAllSrcEnum; (*Source selector setting*)
	END_STRUCT;
	McADILvlEnum :
		( (*Level of the digital input hardware which leads to an active level of the functionality*)
		mcADIL_HIGH := 0, (*High*)
		mcADIL_LOW := 1 (*Low*)
		);
	McADIHomeSwType : STRUCT (*Homing switch input functionality*)
		Source : McADIHomeSwSrcType; (*Source of the digital input hardware which is used for this functionality*)
		Level : McADILvlEnum; (*Level of the digital input hardware which leads to an active level of the functionality*)
	END_STRUCT;
	McADIPosLimSwSrcType : STRUCT (*Source of the digital input hardware which is used for this functionality*)
		Type : McADIAllSrcEnum; (*Source selector setting*)
	END_STRUCT;
	McADIPosLimSwType : STRUCT (*Positive limit switch input functionality*)
		Source : McADIPosLimSwSrcType; (*Source of the digital input hardware which is used for this functionality*)
		Level : McADILvlEnum; (*Level of the digital input hardware which leads to an active level of the functionality*)
	END_STRUCT;
	McADINegLimSwSrcType : STRUCT (*Source of the digital input hardware which is used for this functionality*)
		Type : McADIAllSrcEnum; (*Source selector setting*)
	END_STRUCT;
	McADINegLimSwType : STRUCT (*Negative limit switch input functionality*)
		Source : McADINegLimSwSrcType; (*Source of the digital input hardware which is used for this functionality*)
		Level : McADILvlEnum; (*Level of the digital input hardware which leads to an active level of the functionality*)
	END_STRUCT;
	McADITrg1SrcType : STRUCT (*Source of the digital input hardware which is used for this functionality*)
		Type : McADIAllSrcEnum; (*Source selector setting*)
	END_STRUCT;
	McADITrg1Type : STRUCT (*Trigger 1 input functionality*)
		Source : McADITrg1SrcType; (*Source of the digital input hardware which is used for this functionality*)
		Level : McADILvlEnum; (*Level of the digital input hardware which leads to an active level of the functionality*)
	END_STRUCT;
	McADITrg2SrcType : STRUCT (*Source of the digital input hardware which is used for this functionality*)
		Type : McADIAllSrcEnum; (*Source selector setting*)
	END_STRUCT;
	McADITrg2Type : STRUCT (*Trigger 2 input functionality*)
		Source : McADITrg2SrcType; (*Source of the digital input hardware which is used for this functionality*)
		Level : McADILvlEnum; (*Level of the digital input hardware which leads to an active level of the functionality*)
	END_STRUCT;
	McADIQstopInEnum :
		( (*Digital input functionality triggering an axis quickstop*)
		mcADIQI_TRG_2 := 0, (*Trigger 2*)
		mcADIQI_TRG_1 := 1, (*Trigger 1*)
		mcADIQI_POS_LIM_SW := 2, (*Positive limit switch*)
		mcADIQI_NEG_LIM_SW := 3, (*Negative limit switch*)
		mcADIQI_HOME_SW := 4, (*Homing switch*)
		mcADIQI_NOT_USE := 5 (*Not used*)
		);
	McADIQstopType : STRUCT (*Quickstop functionality*)
		Input : McADIQstopInEnum; (*Digital input functionality triggering an axis quickstop*)
	END_STRUCT;
	McADIType : STRUCT (*Various digital input functionalities e.g. like homing switch or triggers*)
		HomingSwitch : McADIHomeSwType; (*Homing switch input functionality*)
		PositiveLimitSwitch : McADIPosLimSwType; (*Positive limit switch input functionality*)
		NegativeLimitSwitch : McADINegLimSwType; (*Negative limit switch input functionality*)
		Trigger1 : McADITrg1Type; (*Trigger 1 input functionality*)
		Trigger2 : McADITrg2Type; (*Trigger 2 input functionality*)
		Quickstop : McADIQstopType; (*Quickstop functionality*)
	END_STRUCT;
	McASPMEnum :
		( (*Simulation mode on PLC selector setting*)
		mcASPM_SET_VAL_GEN := 0, (*Set value generation - Only set value generation is active, no controlled system is simulated*)
		mcASPM_COMPL := 1 (*Complete - Complete simulation of a controlled system*)
		);
	McASLMEnum :
		( (*Load model selector setting*)
		mcASLM_ONE_MASS_MOT_BASED := 0, (*One mass motor based - 1-mass load model automatically calculated based on motor parameters*)
		mcASLM_ONE_MASS := 1, (*One mass - 1-mass load model, manual parameter setting necessary*)
		mcASLM_TWO_MASS := 2 (*Two masses - 2-mass load model, manual parameter setting necessary*)
		);
	McASLMOneMassMotBasedType : STRUCT (*Type mcASLM_ONE_MASS_MOT_BASED settings*)
		AdditiveLoadParID : UINT; (*Additive load ParID*)
	END_STRUCT;
	McASLMOneMassType : STRUCT (*Type mcASLM_ONE_MASS settings*)
		Inertia : REAL; (*Mass moment of inertia [kgm²]*)
		StaticFriction : REAL; (*Static friction [Nm]*)
		ViscousFriction : REAL; (*Viscous Friction [Nms]*)
		AdditiveLoadParID : UINT; (*Additive load ParID*)
	END_STRUCT;
	McASLMTwoMassMass1Type : STRUCT (*Mass 1*)
		Inertia : REAL; (*Mass moment of inertia [kgm²]*)
		StaticFriction : REAL; (*Static friction [Nm]*)
		ViscousFriction : REAL; (*Viscous Friction [Nms]*)
	END_STRUCT;
	McASLMTwoMassMass2Type : STRUCT (*Mass 2*)
		Inertia : REAL; (*Mass moment of inertia [kgm²]*)
		StaticFriction : REAL; (*Static friction [Nm]*)
		ViscousFriction : REAL; (*Viscous Friction [Nms]*)
		Stiffness : REAL; (*Stiffness of the coupling to mass 1 [Nm/rad]*)
		Damping : REAL; (*Damping of the coupling to mass 1 [Nms/rad]*)
		AdditiveLoadParID : UINT; (*Additive load ParID*)
	END_STRUCT;
	McASLMTwoMassType : STRUCT (*Type mcASLM_TWO_MASS settings*)
		Mass1 : McASLMTwoMassMass1Type; (*Mass 1*)
		Mass2 : McASLMTwoMassMass2Type; (*Mass 2*)
	END_STRUCT;
	McASLMType : STRUCT (*Parameters of the load simulation model*)
		Type : McASLMEnum; (*Load model selector setting*)
		OneMassMotorBased : McASLMOneMassMotBasedType; (*Type mcASLM_ONE_MASS_MOT_BASED settings*)
		OneMass : McASLMOneMassType; (*Type mcASLM_ONE_MASS settings*)
		TwoMasses : McASLMTwoMassType; (*Type mcASLM_TWO_MASS settings*)
	END_STRUCT;
	McASPMComplType : STRUCT (*Type mcASPM_COMPL settings*)
		LoadModel : McASLMType; (*Parameters of the load simulation model*)
	END_STRUCT;
	McASPMType : STRUCT (*Parameters for the simulation of this real axis on the PLC*)
		Type : McASPMEnum; (*Simulation mode on PLC selector setting*)
		Complete : McASPMComplType; (*Type mcASPM_COMPL settings*)
	END_STRUCT;
	McASAMEnum :
		( (*Simulation mode on ACOPOS selector setting*)
		mcASAM_NOT_USE := 0, (*Not used - Load simulation is not switched on during the axis initialization*)
		mcASAM_SET_VAL_GEN := 1, (*Set value generation - Only set value generation is active, no controlled system is simulated*)
		mcASAM_COMPL := 2 (*Complete - Complete load simulation and automatic activation during the axis initialization*)
		);
	McASAMComplType : STRUCT (*Type mcASAM_COMPL settings*)
		LoadModel : McASLMType; (*Parameters of the load simulation model*)
	END_STRUCT;
	McASAMType : STRUCT (*Parameters for the motor and load simulation on the drive*)
		Type : McASAMEnum; (*Simulation mode on ACOPOS selector setting*)
		Complete : McASAMComplType; (*Type mcASAM_COMPL settings*)
	END_STRUCT;
	McASType : STRUCT (*Parameters which influence the simulation possibilities of this axis*)
		ModeOnPLC : McASPMType; (*Parameters for the simulation of this real axis on the PLC*)
		ModeOnACOPOS : McASAMType; (*Parameters for the motor and load simulation on the drive*)
	END_STRUCT;
	McAAFType : STRUCT (*Features for an axis*)
		FeatureReference : McCfgUnboundedArrayType; (*Name of the axis feature reference*)
	END_STRUCT;
	McCfgAcpAxType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_AX*)
		AxisReference : McCfgReferenceType; (*Name of the referenced axis component*)
		MechanicalElements : McAMEType; (*Parameter of hardware elements situated between motor encoder and load which influence the scaling*)
		EncoderLink : McAELType;
		Controller : McACType; (*Axis controller parameters*)
		Homing : McAHType; (*Homing mode and parameters which can be used within the application program as preconfigured setting*)
		StopReaction : McASRType; (*Reactions of the axis in case of certain stop conditions*)
		MovementErrorLimits : McAMELType; (*Limit values that result in a stop reaction when exceeded*)
		JerkFilter : McAJFType; (*Jerk filter*)
		DigitalInputs : McADIType; (*Various digital input functionalities e.g. like homing switch or triggers*)
		Simulation : McASType; (*Parameters which influence the simulation possibilities of this axis*)
		AxisFeatures : McAAFType; (*Features for an axis*)
	END_STRUCT;
	McCfgAcpAxRefType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_AX_REF*)
		AxisReference : McCfgReferenceType; (*Name of the referenced axis component*)
	END_STRUCT;
	McCfgAcpMechElmType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_MECH_ELM*)
		MechanicalElements : McAMEType; (*Parameter of hardware elements situated between motor encoder and load which influence the scaling*)
	END_STRUCT;
	McCfgAcpEncLinkType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_ENC_LINK*)
		EncoderLink : McAELType;
	END_STRUCT;
	McCfgAcpCtrlType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_CTRL*)
		Controller : McACType; (*Axis controller parameters*)
	END_STRUCT;
	McCfgAcpHomeType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_HOME*)
		Homing : McAHType; (*Homing mode and parameters which can be used within the application program as preconfigured setting*)
	END_STRUCT;
	McCfgAcpStopReacType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_STOP_REAC*)
		StopReaction : McASRType; (*Reactions of the axis in case of certain stop conditions*)
	END_STRUCT;
	McCfgAcpMoveErrLimType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_MOVE_ERR_LIM*)
		MovementErrorLimits : McAMELType; (*Limit values that result in a stop reaction when exceeded*)
	END_STRUCT;
	McCfgAcpJerkFltrType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_JERK_FLTR*)
		JerkFilter : McAJFType; (*Jerk filter*)
	END_STRUCT;
	McCfgAcpDigInType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_DIG_IN*)
		DigitalInputs : McADIType; (*Various digital input functionalities e.g. like homing switch or triggers*)
	END_STRUCT;
	McCfgAcpSimType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_SIM*)
		Simulation : McASType; (*Parameters which influence the simulation possibilities of this axis*)
	END_STRUCT;
	McCfgAcpAxFeatType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_AX_FEAT*)
		AxisFeatures : McAAFType; (*Features for an axis*)
	END_STRUCT;
	McAPSMOutParEnum :
		( (*Output parameters selector setting*)
		mcAPSMOP_ACOPOSTRAK := 1, (*ACOPOStrak - Output parameters for ACOPOStrak segments*)
		mcAPSMOP_USR_DEF := 2 (*User defined - User defined output parameters*)
		);
	McAPSMOutParACOPOStrakType : STRUCT (*Type mcAPSMOP_ACOPOSTRAK settings*)
		Voltage : REAL; (*Output voltage [V]*)
		CurrentLimit : REAL; (*Output current limit [A]*)
		CurrentLimitTime : USINT; (*Output current limit time [s]*)
	END_STRUCT;
	McAPSMOutParUsrDefCurLimEnum :
		( (*Current limitation selector setting*)
		mcAPSMOPUDCL_NOT_USE := 0, (*Not used - Current limited automatically to maximum*)
		mcAPSMOPUDCL_USE := 1 (*Used - Current limited to specified value*)
		);
	McAPSMOutParUsrDefCurLimUseType : STRUCT (*Type mcAPSMOPUDCL_USE settings*)
		CurrentLimit : REAL; (*Output current limit [A]*)
		CurrentLimitTime : USINT; (*Output current limit time [s]*)
	END_STRUCT;
	McAPSMOutParUsrDefCurLimType : STRUCT (*Current limitation*)
		Type : McAPSMOutParUsrDefCurLimEnum; (*Current limitation selector setting*)
		Used : McAPSMOutParUsrDefCurLimUseType; (*Type mcAPSMOPUDCL_USE settings*)
	END_STRUCT;
	McAPSMOutParUsrDefType : STRUCT (*Type mcAPSMOP_USR_DEF settings*)
		Voltage : REAL; (*Output voltage [V]*)
		CurrentLimitation : McAPSMOutParUsrDefCurLimType; (*Current limitation*)
	END_STRUCT;
	McAPSMOutParType : STRUCT (*Output parameters selection*)
		Type : McAPSMOutParEnum; (*Output parameters selector setting*)
		ACOPOStrak : McAPSMOutParACOPOStrakType; (*Type mcAPSMOP_ACOPOSTRAK settings*)
		UserDefined : McAPSMOutParUsrDefType; (*Type mcAPSMOP_USR_DEF settings*)
	END_STRUCT;
	McAPSMOutVOnEnum :
		( (*Output voltage on selector setting*)
		mcAPSMOVO_NO := 0, (*No - Output voltage off, switch on by FB call*)
		mcAPSMOVO_YES := 1 (*Yes - Output voltage on*)
		);
	McAPSMOutVOnType : STRUCT (*Output voltage is on after start-up and error reset*)
		Type : McAPSMOutVOnEnum; (*Output voltage on selector setting*)
	END_STRUCT;
	McCfgAcpAuxPwrSupModType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_AUX_PWR_SUP_MOD*)
		AxisReference : STRING[250]; (*Name of the power supply module component*)
		OutputParameters : McAPSMOutParType; (*Output parameters selection*)
		OutputVoltageOn : McAPSMOutVOnType; (*Output voltage is on after start-up and error reset*)
	END_STRUCT;
	McAPSMPSPwrSecType : STRUCT
		AxisReference : STRING[250]; (*Name of the power supply module component*)
	END_STRUCT;
	McCfgAcpPSMPwrSecType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_PSM_PWR_SEC*)
		PowerSection : McAPSMPSPwrSecType;
	END_STRUCT;
	McAPSPwrSupEnum :
		( (*Power supply selector setting*)
		mcAPSPS_AC := 0, (*AC -*)
		mcAPSPS_DC_PWR_SUP_MOD_REF := 1, (*DC power supply module reference -*)
		mcAPSPS_DC_BUS_V := 2, (*DC bus voltage -*)
		mcAPSPS_ETA_SYS_FOR_TR_ONLY := 3 (*ETA system (for training only) - This mode can be used for training purpose, when using the drive within an ETA system with 24 VDC supply voltage*)
		);
	McAPSPwrSupACSngPhOpEnum :
		( (*Single phase operation selector setting*)
		mcAPSPSASPO_NOT_USE := 0, (*Not used -*)
		mcAPSPSASPO_USE := 1 (*Used -*)
		);
	McAPSPwrSupACSngPhOpUseType : STRUCT (*Type mcAPSPSASPO_USE settings*)
		SupplyVoltage : REAL; (*Supply voltage for the single phase operation [V]*)
	END_STRUCT;
	McAPSPwrSupACSngPhOpType : STRUCT
		Type : McAPSPwrSupACSngPhOpEnum; (*Single phase operation selector setting*)
		Used : McAPSPwrSupACSngPhOpUseType; (*Type mcAPSPSASPO_USE settings*)
	END_STRUCT;
	McAPSPwrSupACType : STRUCT (*Type mcAPSPS_AC settings*)
		SinglePhaseOperation : McAPSPwrSupACSngPhOpType;
	END_STRUCT;
	McAPSPwrSupDCPwrSupModRefType : STRUCT (*Type mcAPSPS_DC_PWR_SUP_MOD_REF settings*)
		PowerSupplyModuleReference : STRING[250]; (*Name of the referenced power supply module component*)
	END_STRUCT;
	McAPSPwrSupDCBusVType : STRUCT (*Type mcAPSPS_DC_BUS_V settings*)
		BusVoltage : UINT; (*Bus voltage [V]*)
	END_STRUCT;
	McAPSPwrSupType : STRUCT
		Type : McAPSPwrSupEnum; (*Power supply selector setting*)
		AC : McAPSPwrSupACType; (*Type mcAPSPS_AC settings*)
		DCPowerSupplyModuleReference : McAPSPwrSupDCPwrSupModRefType; (*Type mcAPSPS_DC_PWR_SUP_MOD_REF settings*)
		DCBusVoltage : McAPSPwrSupDCBusVType; (*Type mcAPSPS_DC_BUS_V settings*)
	END_STRUCT;
	McCfgAcpPwrSupType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_PWR_SUP*)
		PowerSupply : McAPSPwrSupType;
	END_STRUCT;
	McAEEncX6AIfTypEnum :
		( (*Interface type selector setting*)
		mcAEX6AIT_ENDAT := 0, (*EnDat -*)
		mcAEX6AIT_SSI := 1, (*SSI -*)
		mcAEX6AIT_SIN := 2, (*Sine -*)
		mcAEX6AIT_INCR := 3, (*Incremental -*)
		mcAEX6AIT_LINMOT := 4, (*LinMot -*)
		mcAEX6AIT_RES := 5 (*Resolver -*)
		);
	McAEX6ASSSIFrmCfgStatBit1Enum :
		( (*Warning or error bit*)
		mcAEX6ASSFCSB1_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX6ASSFCSB1_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX6ASSFCSB1_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX6ASSFCSB1_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX6ASSFCSB1_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX6ASSSIFrmCfgStatBit2Enum :
		( (*Warning or error bit*)
		mcAEX6ASSFCSB2_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX6ASSFCSB2_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX6ASSFCSB2_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX6ASSFCSB2_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX6ASSFCSB2_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX6ASSSIFrmCfgSSIDatCEnum :
		( (*SSI data code*)
		mcAEX6ASSFCSDC_GRAY := 0, (*Gray - Gray*)
		mcAEX6ASSFCSDC_BIN := 1 (*Binary - Binary*)
		);
	McAEX6ASSSIFrmCfgSSIParCkEnum :
		( (*SSI parity check*)
		mcAEX6ASSFCSPC_SSI_PAR_CK_ODD := 0, (*SSI parity check odd - SSI parity check odd*)
		mcAEX6ASSFCSPC_SSI_PAR_CK_EVEN := 1, (*SSI parity check even - SSI parity check even*)
		mcAEX6ASSFCSPC_OFF := 2 (*Off - Off*)
		);
	McAEX6ASSSIFrmCfgType : STRUCT (*Frame structure in the order of transfer*)
		NumberOfLeadingZeros : USINT; (*Zero bits before position data*)
		NumberOfPositionBits : USINT; (*Position data*)
		NumberOfTrailingZeros : USINT; (*Zero bits after position data*)
		StatusBit1 : McAEX6ASSSIFrmCfgStatBit1Enum; (*Warning or error bit*)
		StatusBit2 : McAEX6ASSSIFrmCfgStatBit2Enum; (*Warning or error bit*)
		SSIDataCode : McAEX6ASSSIFrmCfgSSIDatCEnum; (*SSI data code*)
		SSIParityCheck : McAEX6ASSSIFrmCfgSSIParCkEnum; (*SSI parity check*)
	END_STRUCT;
	McAEX6ASType : STRUCT (*Type mcAEX6AIT_SSI settings*)
		SSIFrameConfiguration : McAEX6ASSSIFrmCfgType; (*Frame structure in the order of transfer*)
		IncrementsPerEncoderRevolution : UDINT; (*Absolute resolution of an encoder revolution*)
		BaudRate : DINT; (*Transfer rate [kBaud]*)
	END_STRUCT;
	McAEX6ASinType : STRUCT (*Type mcAEX6AIT_SIN settings*)
		LinesPerEncoderRevolution : UDINT; (*Absolute number of lines of an encoder revolution*)
	END_STRUCT;
	McAEX6AIMaxExpectedOutFreqEnum :
		( (*Maximal expected output frequency [Hz]*)
		mcAEX6AIMEOF_MEOF_25000 := 0, (*MEOF 25000 - 25000 Hz*)
		mcAEX6AIMEOF_MEOF_50000 := 1, (*MEOF 50000 - 50000 Hz*)
		mcAEX6AIMEOF_MEOF_100000 := 2, (*MEOF 100000 - 100000 Hz*)
		mcAEX6AIMEOF_MEOF_200000 := 3 (*MEOF 200000 - 200000 Hz*)
		);
	McAEX6AIType : STRUCT (*Type mcAEX6AIT_INCR settings*)
		LinesPerEncoderRevolution : UDINT; (*Absolute number of lines of an encoder revolution*)
		MaxExpectedOutputFrequency : McAEX6AIMaxExpectedOutFreqEnum; (*Maximal expected output frequency [Hz]*)
		IgnoreCheck : UINT; (*Ignore check*)
	END_STRUCT;
	McAEX6ALinMotType : STRUCT (*Type mcAEX6AIT_LINMOT settings*)
		IncrementsPerEncoderRevolution : UDINT; (*Absolute resolution of an encoder revolution*)
	END_STRUCT;
	McAEX6AResType : STRUCT (*Type mcAEX6AIT_RES settings*)
		PolepairsPerEncoderRevolution : USINT; (*Resolver polepairs per encoder revolution*)
	END_STRUCT;
	McAEEncX6AIfTypType : STRUCT
		Type : McAEEncX6AIfTypEnum; (*Interface type selector setting*)
		SSI : McAEX6ASType; (*Type mcAEX6AIT_SSI settings*)
		Sine : McAEX6ASinType; (*Type mcAEX6AIT_SIN settings*)
		Incremental : McAEX6AIType; (*Type mcAEX6AIT_INCR settings*)
		LinMot : McAEX6ALinMotType; (*Type mcAEX6AIT_LINMOT settings*)
		Resolver : McAEX6AResType; (*Type mcAEX6AIT_RES settings*)
	END_STRUCT;
	McAEEncX6AType : STRUCT
		InterfaceType : McAEEncX6AIfTypType;
	END_STRUCT;
	McAEEncX6BIfTypEnum :
		( (*Interface type selector setting*)
		mcAEX6BIT_ENDAT := 0, (*EnDat -*)
		mcAEX6BIT_SSI := 1, (*SSI -*)
		mcAEX6BIT_SIN := 2, (*Sine -*)
		mcAEX6BIT_INCR := 3, (*Incremental -*)
		mcAEX6BIT_LINMOT := 4, (*LinMot -*)
		mcAEX6BIT_RES := 5 (*Resolver -*)
		);
	McAEX6BSSSIFrmCfgStatBit1Enum :
		( (*Warning or error bit*)
		mcAEX6BSSFCSB1_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX6BSSFCSB1_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX6BSSFCSB1_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX6BSSFCSB1_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX6BSSFCSB1_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX6BSSSIFrmCfgStatBit2Enum :
		( (*Warning or error bit*)
		mcAEX6BSSFCSB2_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX6BSSFCSB2_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX6BSSFCSB2_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX6BSSFCSB2_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX6BSSFCSB2_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX6BSSSIFrmCfgSSIDatCEnum :
		( (*SSI data code*)
		mcAEX6BSSFCSDC_GRAY := 0, (*Gray - Gray*)
		mcAEX6BSSFCSDC_BIN := 1 (*Binary - Binary*)
		);
	McAEX6BSSSIFrmCfgSSIParCkEnum :
		( (*SSI parity check*)
		mcAEX6BSSFCSPC_SSI_PAR_CK_ODD := 0, (*SSI parity check odd - SSI parity check odd*)
		mcAEX6BSSFCSPC_SSI_PAR_CK_EVEN := 1, (*SSI parity check even - SSI parity check even*)
		mcAEX6BSSFCSPC_OFF := 2 (*Off - Off*)
		);
	McAEX6BSSSIFrmCfgType : STRUCT (*Frame structure in the order of transfer*)
		NumberOfLeadingZeros : USINT; (*Zero bits before position data*)
		NumberOfPositionBits : USINT; (*Position data*)
		NumberOfTrailingZeros : USINT; (*Zero bits after position data*)
		StatusBit1 : McAEX6BSSSIFrmCfgStatBit1Enum; (*Warning or error bit*)
		StatusBit2 : McAEX6BSSSIFrmCfgStatBit2Enum; (*Warning or error bit*)
		SSIDataCode : McAEX6BSSSIFrmCfgSSIDatCEnum; (*SSI data code*)
		SSIParityCheck : McAEX6BSSSIFrmCfgSSIParCkEnum; (*SSI parity check*)
	END_STRUCT;
	McAEX6BSType : STRUCT (*Type mcAEX6BIT_SSI settings*)
		SSIFrameConfiguration : McAEX6BSSSIFrmCfgType; (*Frame structure in the order of transfer*)
		IncrementsPerEncoderRevolution : UDINT; (*Absolute resolution of an encoder revolution*)
		BaudRate : DINT; (*Transfer rate [kBaud]*)
	END_STRUCT;
	McAEX6BSinType : STRUCT (*Type mcAEX6BIT_SIN settings*)
		LinesPerEncoderRevolution : UDINT; (*Absolute number of lines of an encoder revolution*)
	END_STRUCT;
	McAEX6BIMaxExpectedOutFreqEnum :
		( (*Maximal expected output frequency [Hz]*)
		mcAEX6BIMEOF_MEOF_25000 := 0, (*MEOF 25000 - 25000 Hz*)
		mcAEX6BIMEOF_MEOF_50000 := 1, (*MEOF 50000 - 50000 Hz*)
		mcAEX6BIMEOF_MEOF_100000 := 2, (*MEOF 100000 - 100000 Hz*)
		mcAEX6BIMEOF_MEOF_200000 := 3 (*MEOF 200000 - 200000 Hz*)
		);
	McAEX6BIType : STRUCT (*Type mcAEX6BIT_INCR settings*)
		LinesPerEncoderRevolution : UDINT; (*Absolute number of lines of an encoder revolution*)
		MaxExpectedOutputFrequency : McAEX6BIMaxExpectedOutFreqEnum; (*Maximal expected output frequency [Hz]*)
		IgnoreCheck : UINT; (*Ignore check*)
	END_STRUCT;
	McAEX6BLinMotType : STRUCT (*Type mcAEX6BIT_LINMOT settings*)
		IncrementsPerEncoderRevolution : UDINT; (*Absolute resolution of an encoder revolution*)
	END_STRUCT;
	McAEX6BResType : STRUCT (*Type mcAEX6BIT_RES settings*)
		PolepairsPerEncoderRevolution : USINT; (*Resolver polepairs per encoder revolution*)
	END_STRUCT;
	McAEEncX6BIfTypType : STRUCT
		Type : McAEEncX6BIfTypEnum; (*Interface type selector setting*)
		SSI : McAEX6BSType; (*Type mcAEX6BIT_SSI settings*)
		Sine : McAEX6BSinType; (*Type mcAEX6BIT_SIN settings*)
		Incremental : McAEX6BIType; (*Type mcAEX6BIT_INCR settings*)
		LinMot : McAEX6BLinMotType; (*Type mcAEX6BIT_LINMOT settings*)
		Resolver : McAEX6BResType; (*Type mcAEX6BIT_RES settings*)
	END_STRUCT;
	McAEEncX6BType : STRUCT
		InterfaceType : McAEEncX6BIfTypType;
	END_STRUCT;
	McAEEncX41IfTypEnum :
		( (*Interface type selector setting*)
		mcAEX41IT_NOT_USE := 0, (*Not used -*)
		mcAEX41IT_BISS := 1, (*BiSS -*)
		mcAEX41IT_SSI := 2, (*SSI -*)
		mcAEX41IT_ENDAT := 3, (*EnDat -*)
		mcAEX41IT_HIPERFACE_DSL := 4, (*HIPERFACE DSL -*)
		mcAEX41IT_TFMT := 5 (*T-Format - Tamagawa digital interface*)
		);
	McAEX41BPwrSupEnum :
		( (*Power supply of the encoder*)
		mcAEX41BPS_EXT := 0, (*External - External (0V)*)
		mcAEX41BPS_PS_5_V := 1, (*PS 5 V - 5 Volt*)
		mcAEX41BPS_PS_12_V := 2 (*PS 12 V - 12 Volt*)
		);
	McAEX41BBiSSFrmCfgStatBit1Enum :
		( (*Warning or error bit*)
		mcAEX41BBFCSB1_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX41BBFCSB1_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX41BBFCSB1_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX41BBFCSB1_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX41BBFCSB1_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX41BBiSSFrmCfgStatBit2Enum :
		( (*Warning or error bit*)
		mcAEX41BBFCSB2_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX41BBFCSB2_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX41BBFCSB2_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX41BBFCSB2_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX41BBFCSB2_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX41BBiSSFrmCfgType : STRUCT (*Frame structure in the order of transfer*)
		NumberOfLeadingZeros : USINT; (*Zero bits before position data*)
		NumberOfPositionBits : USINT; (*Position data*)
		NumberOfTrailingZeros : USINT; (*Zero bits after position data*)
		StatusBit1 : McAEX41BBiSSFrmCfgStatBit1Enum; (*Warning or error bit*)
		StatusBit2 : McAEX41BBiSSFrmCfgStatBit2Enum; (*Warning or error bit*)
	END_STRUCT;
	McAEX41BType : STRUCT (*Type mcAEX41IT_BISS settings*)
		PowerSupply : McAEX41BPwrSupEnum; (*Power supply of the encoder*)
		BiSSFrameConfiguration : McAEX41BBiSSFrmCfgType; (*Frame structure in the order of transfer*)
		CRCPolynomial : UDINT; (*Data verification via CRC (decimal value, 0 deactivates verification)*)
		IncrementsPerEncoderRevolution : UDINT; (*Absolute resolution of an encoder revolution*)
		BaudRate : DINT; (*Transfer rate [kBaud]*)
	END_STRUCT;
	McAEX41SPwrSupEnum :
		( (*Power supply of the encoder*)
		mcAEX41SPS_EXT := 0, (*External - External (0V)*)
		mcAEX41SPS_PS_5_V := 1, (*PS 5 V - 5 Volt*)
		mcAEX41SPS_PS_12_V := 2 (*PS 12 V - 12 Volt*)
		);
	McAEX41SSSIFrmCfgStatBit1Enum :
		( (*Warning or error bit*)
		mcAEX41SSFCSB1_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX41SSFCSB1_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX41SSFCSB1_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX41SSFCSB1_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX41SSFCSB1_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX41SSSIFrmCfgStatBit2Enum :
		( (*Warning or error bit*)
		mcAEX41SSFCSB2_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX41SSFCSB2_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX41SSFCSB2_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX41SSFCSB2_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX41SSFCSB2_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX41SSSIFrmCfgSSIDatCEnum :
		( (*SSI data code*)
		mcAEX41SSFCSDC_GRAY := 0, (*Gray - Gray*)
		mcAEX41SSFCSDC_BIN := 1 (*Binary - Binary*)
		);
	McAEX41SSSIFrmCfgSSIParCkEnum :
		( (*SSI parity check*)
		mcAEX41SSFCSPC_SSI_PAR_CK_ODD := 0, (*SSI parity check odd - SSI parity check odd*)
		mcAEX41SSFCSPC_SSI_PAR_CK_EVEN := 1, (*SSI parity check even - SSI parity check even*)
		mcAEX41SSFCSPC_OFF := 2 (*Off - Off*)
		);
	McAEX41SSSIFrmCfgType : STRUCT (*Frame structure in the order of transfer*)
		NumberOfLeadingZeros : USINT; (*Zero bits before position data*)
		NumberOfPositionBits : USINT; (*Position data*)
		NumberOfTrailingZeros : USINT; (*Zero bits after position data*)
		StatusBit1 : McAEX41SSSIFrmCfgStatBit1Enum; (*Warning or error bit*)
		StatusBit2 : McAEX41SSSIFrmCfgStatBit2Enum; (*Warning or error bit*)
		SSIDataCode : McAEX41SSSIFrmCfgSSIDatCEnum; (*SSI data code*)
		SSIParityCheck : McAEX41SSSIFrmCfgSSIParCkEnum; (*SSI parity check*)
	END_STRUCT;
	McAEX41SType : STRUCT (*Type mcAEX41IT_SSI settings*)
		PowerSupply : McAEX41SPwrSupEnum; (*Power supply of the encoder*)
		SSIFrameConfiguration : McAEX41SSSIFrmCfgType; (*Frame structure in the order of transfer*)
		IncrementsPerEncoderRevolution : UDINT; (*Absolute resolution of an encoder revolution*)
		BaudRate : DINT; (*Transfer rate [kBaud]*)
	END_STRUCT;
	McAEEncX41IfTypType : STRUCT
		Type : McAEEncX41IfTypEnum; (*Interface type selector setting*)
		BiSS : McAEX41BType; (*Type mcAEX41IT_BISS settings*)
		SSI : McAEX41SType; (*Type mcAEX41IT_SSI settings*)
	END_STRUCT;
	McAEEncX41Type : STRUCT
		InterfaceType : McAEEncX41IfTypType;
	END_STRUCT;
	McAEEncX42IfTypEnum :
		( (*Interface type selector setting*)
		mcAEX42IT_NOT_USE := 0, (*Not used -*)
		mcAEX42IT_BISS := 1, (*BiSS -*)
		mcAEX42IT_SSI := 2, (*SSI -*)
		mcAEX42IT_ENDAT := 3, (*EnDat -*)
		mcAEX42IT_HIPERFACE_DSL := 4, (*HIPERFACE DSL -*)
		mcAEX42IT_TFMT := 5 (*T-Format - Tamagawa digital interface*)
		);
	McAEX42BPwrSupEnum :
		( (*Power supply of the encoder*)
		mcAEX42BPS_EXT := 0, (*External - External (0V)*)
		mcAEX42BPS_PS_5_V := 1, (*PS 5 V - 5 Volt*)
		mcAEX42BPS_PS_12_V := 2 (*PS 12 V - 12 Volt*)
		);
	McAEX42BBiSSFrmCfgStatBit1Enum :
		( (*Warning or error bit*)
		mcAEX42BBFCSB1_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX42BBFCSB1_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX42BBFCSB1_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX42BBFCSB1_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX42BBFCSB1_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX42BBiSSFrmCfgStatBit2Enum :
		( (*Warning or error bit*)
		mcAEX42BBFCSB2_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX42BBFCSB2_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX42BBFCSB2_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX42BBFCSB2_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX42BBFCSB2_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX42BBiSSFrmCfgType : STRUCT (*Frame structure in the order of transfer*)
		NumberOfLeadingZeros : USINT; (*Zero bits before position data*)
		NumberOfPositionBits : USINT; (*Position data*)
		NumberOfTrailingZeros : USINT; (*Zero bits after position data*)
		StatusBit1 : McAEX42BBiSSFrmCfgStatBit1Enum; (*Warning or error bit*)
		StatusBit2 : McAEX42BBiSSFrmCfgStatBit2Enum; (*Warning or error bit*)
	END_STRUCT;
	McAEX42BType : STRUCT (*Type mcAEX42IT_BISS settings*)
		PowerSupply : McAEX42BPwrSupEnum; (*Power supply of the encoder*)
		BiSSFrameConfiguration : McAEX42BBiSSFrmCfgType; (*Frame structure in the order of transfer*)
		CRCPolynomial : UDINT; (*Data verification via CRC (decimal value, 0 deactivates verification)*)
		IncrementsPerEncoderRevolution : UDINT; (*Absolute resolution of an encoder revolution*)
		BaudRate : DINT; (*Transfer rate [kBaud]*)
	END_STRUCT;
	McAEX42SPwrSupEnum :
		( (*Power supply of the encoder*)
		mcAEX42SPS_EXT := 0, (*External - External (0V)*)
		mcAEX42SPS_PS_5_V := 1, (*PS 5 V - 5 Volt*)
		mcAEX42SPS_PS_12_V := 2 (*PS 12 V - 12 Volt*)
		);
	McAEX42SSSIFrmCfgStatBit1Enum :
		( (*Warning or error bit*)
		mcAEX42SSFCSB1_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX42SSFCSB1_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX42SSFCSB1_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX42SSFCSB1_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX42SSFCSB1_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX42SSSIFrmCfgStatBit2Enum :
		( (*Warning or error bit*)
		mcAEX42SSFCSB2_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX42SSFCSB2_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX42SSFCSB2_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX42SSFCSB2_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX42SSFCSB2_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX42SSSIFrmCfgSSIDatCEnum :
		( (*SSI data code*)
		mcAEX42SSFCSDC_GRAY := 0, (*Gray - Gray*)
		mcAEX42SSFCSDC_BIN := 1 (*Binary - Binary*)
		);
	McAEX42SSSIFrmCfgSSIParCkEnum :
		( (*SSI parity check*)
		mcAEX42SSFCSPC_SSI_PAR_CK_ODD := 0, (*SSI parity check odd - SSI parity check odd*)
		mcAEX42SSFCSPC_SSI_PAR_CK_EVEN := 1, (*SSI parity check even - SSI parity check even*)
		mcAEX42SSFCSPC_OFF := 2 (*Off - Off*)
		);
	McAEX42SSSIFrmCfgType : STRUCT (*Frame structure in the order of transfer*)
		NumberOfLeadingZeros : USINT; (*Zero bits before position data*)
		NumberOfPositionBits : USINT; (*Position data*)
		NumberOfTrailingZeros : USINT; (*Zero bits after position data*)
		StatusBit1 : McAEX42SSSIFrmCfgStatBit1Enum; (*Warning or error bit*)
		StatusBit2 : McAEX42SSSIFrmCfgStatBit2Enum; (*Warning or error bit*)
		SSIDataCode : McAEX42SSSIFrmCfgSSIDatCEnum; (*SSI data code*)
		SSIParityCheck : McAEX42SSSIFrmCfgSSIParCkEnum; (*SSI parity check*)
	END_STRUCT;
	McAEX42SType : STRUCT (*Type mcAEX42IT_SSI settings*)
		PowerSupply : McAEX42SPwrSupEnum; (*Power supply of the encoder*)
		SSIFrameConfiguration : McAEX42SSSIFrmCfgType; (*Frame structure in the order of transfer*)
		IncrementsPerEncoderRevolution : UDINT; (*Absolute resolution of an encoder revolution*)
		BaudRate : DINT; (*Transfer rate [kBaud]*)
	END_STRUCT;
	McAEEncX42IfTypType : STRUCT
		Type : McAEEncX42IfTypEnum; (*Interface type selector setting*)
		BiSS : McAEX42BType; (*Type mcAEX42IT_BISS settings*)
		SSI : McAEX42SType; (*Type mcAEX42IT_SSI settings*)
	END_STRUCT;
	McAEEncX42Type : STRUCT
		InterfaceType : McAEEncX42IfTypType;
	END_STRUCT;
	McAEEncX43IfTypEnum :
		( (*Interface type selector setting*)
		mcAEX43IT_NOT_USE := 0, (*Not used -*)
		mcAEX43IT_BISS := 1, (*BiSS -*)
		mcAEX43IT_SSI := 2, (*SSI -*)
		mcAEX43IT_ENDAT := 3, (*EnDat -*)
		mcAEX43IT_HIPERFACE_DSL := 4, (*HIPERFACE DSL -*)
		mcAEX43IT_TFMT := 5 (*T-Format - Tamagawa digital interface*)
		);
	McAEX43BPwrSupEnum :
		( (*Power supply of the encoder*)
		mcAEX43BPS_EXT := 0, (*External - External (0V)*)
		mcAEX43BPS_PS_5_V := 1, (*PS 5 V - 5 Volt*)
		mcAEX43BPS_PS_12_V := 2 (*PS 12 V - 12 Volt*)
		);
	McAEX43BBiSSFrmCfgStatBit1Enum :
		( (*Warning or error bit*)
		mcAEX43BBFCSB1_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX43BBFCSB1_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX43BBFCSB1_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX43BBFCSB1_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX43BBFCSB1_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX43BBiSSFrmCfgStatBit2Enum :
		( (*Warning or error bit*)
		mcAEX43BBFCSB2_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX43BBFCSB2_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX43BBFCSB2_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX43BBFCSB2_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX43BBFCSB2_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX43BBiSSFrmCfgType : STRUCT (*Frame structure in the order of transfer*)
		NumberOfLeadingZeros : USINT; (*Zero bits before position data*)
		NumberOfPositionBits : USINT; (*Position data*)
		NumberOfTrailingZeros : USINT; (*Zero bits after position data*)
		StatusBit1 : McAEX43BBiSSFrmCfgStatBit1Enum; (*Warning or error bit*)
		StatusBit2 : McAEX43BBiSSFrmCfgStatBit2Enum; (*Warning or error bit*)
	END_STRUCT;
	McAEX43BType : STRUCT (*Type mcAEX43IT_BISS settings*)
		PowerSupply : McAEX43BPwrSupEnum; (*Power supply of the encoder*)
		BiSSFrameConfiguration : McAEX43BBiSSFrmCfgType; (*Frame structure in the order of transfer*)
		CRCPolynomial : UDINT; (*Data verification via CRC (decimal value, 0 deactivates verification)*)
		IncrementsPerEncoderRevolution : UDINT; (*Absolute resolution of an encoder revolution*)
		BaudRate : DINT; (*Transfer rate [kBaud]*)
	END_STRUCT;
	McAEX43SPwrSupEnum :
		( (*Power supply of the encoder*)
		mcAEX43SPS_EXT := 0, (*External - External (0V)*)
		mcAEX43SPS_PS_5_V := 1, (*PS 5 V - 5 Volt*)
		mcAEX43SPS_PS_12_V := 2 (*PS 12 V - 12 Volt*)
		);
	McAEX43SSSIFrmCfgStatBit1Enum :
		( (*Warning or error bit*)
		mcAEX43SSFCSB1_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX43SSFCSB1_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX43SSFCSB1_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX43SSFCSB1_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX43SSFCSB1_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX43SSSIFrmCfgStatBit2Enum :
		( (*Warning or error bit*)
		mcAEX43SSFCSB2_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX43SSFCSB2_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX43SSFCSB2_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX43SSFCSB2_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX43SSFCSB2_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX43SSSIFrmCfgSSIDatCEnum :
		( (*SSI data code*)
		mcAEX43SSFCSDC_GRAY := 0, (*Gray - Gray*)
		mcAEX43SSFCSDC_BIN := 1 (*Binary - Binary*)
		);
	McAEX43SSSIFrmCfgSSIParCkEnum :
		( (*SSI parity check*)
		mcAEX43SSFCSPC_SSI_PAR_CK_ODD := 0, (*SSI parity check odd - SSI parity check odd*)
		mcAEX43SSFCSPC_SSI_PAR_CK_EVEN := 1, (*SSI parity check even - SSI parity check even*)
		mcAEX43SSFCSPC_OFF := 2 (*Off - Off*)
		);
	McAEX43SSSIFrmCfgType : STRUCT (*Frame structure in the order of transfer*)
		NumberOfLeadingZeros : USINT; (*Zero bits before position data*)
		NumberOfPositionBits : USINT; (*Position data*)
		NumberOfTrailingZeros : USINT; (*Zero bits after position data*)
		StatusBit1 : McAEX43SSSIFrmCfgStatBit1Enum; (*Warning or error bit*)
		StatusBit2 : McAEX43SSSIFrmCfgStatBit2Enum; (*Warning or error bit*)
		SSIDataCode : McAEX43SSSIFrmCfgSSIDatCEnum; (*SSI data code*)
		SSIParityCheck : McAEX43SSSIFrmCfgSSIParCkEnum; (*SSI parity check*)
	END_STRUCT;
	McAEX43SType : STRUCT (*Type mcAEX43IT_SSI settings*)
		PowerSupply : McAEX43SPwrSupEnum; (*Power supply of the encoder*)
		SSIFrameConfiguration : McAEX43SSSIFrmCfgType; (*Frame structure in the order of transfer*)
		IncrementsPerEncoderRevolution : UDINT; (*Absolute resolution of an encoder revolution*)
		BaudRate : DINT; (*Transfer rate [kBaud]*)
	END_STRUCT;
	McAEEncX43IfTypType : STRUCT
		Type : McAEEncX43IfTypEnum; (*Interface type selector setting*)
		BiSS : McAEX43BType; (*Type mcAEX43IT_BISS settings*)
		SSI : McAEX43SType; (*Type mcAEX43IT_SSI settings*)
	END_STRUCT;
	McAEEncX43Type : STRUCT
		InterfaceType : McAEEncX43IfTypType;
	END_STRUCT;
	McAEEncX11AIfTypEnum :
		( (*Interface type selector setting*)
		mcAEX11AIT_ENDAT := 0, (*EnDat -*)
		mcAEX11AIT_SSI_SIN := 1, (*SSI sine -*)
		mcAEX11AIT_SSI := 2, (*SSI -*)
		mcAEX11AIT_BISS := 3, (*BiSS -*)
		mcAEX11AIT_SIN := 4, (*Sine -*)
		mcAEX11AIT_SIN_W_DCM := 5, (*Sine with DCM -*)
		mcAEX11AIT_HIPERFACE := 6 (*HIPERFACE -*)
		);
	McAEX11ASSSSIFrmCfgStatBit1Enum :
		( (*Warning or error bit*)
		mcAEX11ASSFCSB1_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX11ASSFCSB1_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX11ASSFCSB1_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX11ASSFCSB1_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX11ASSFCSB1_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX11ASSSSIFrmCfgStatBit2Enum :
		( (*Warning or error bit*)
		mcAEX11ASSFCSB2_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX11ASSFCSB2_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX11ASSFCSB2_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX11ASSFCSB2_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX11ASSFCSB2_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX11ASSSSIFrmCfgSSIDatCEnum :
		( (*SSI data code*)
		mcAEX11ASSFCSDC_GRAY := 0, (*Gray - Gray*)
		mcAEX11ASSFCSDC_BIN := 1 (*Binary - Binary*)
		);
	McAEX11ASSSSIFrmCfgSSIParCkEnum :
		( (*SSI parity check*)
		mcAEX11ASSFCSPC_SSI_PAR_CK_ODD := 0, (*SSI parity check odd - SSI parity check odd*)
		mcAEX11ASSFCSPC_SSI_PAR_CK_EVEN := 1, (*SSI parity check even - SSI parity check even*)
		mcAEX11ASSFCSPC_OFF := 2 (*Off - Off*)
		);
	McAEX11ASSSSIFrmCfgType : STRUCT (*Frame structure in the order of transfer*)
		NumberOfLeadingZeros : USINT; (*Zero bits before position data*)
		NumberOfPositionBits : USINT; (*Position data*)
		NumberOfTrailingZeros : USINT; (*Zero bits after position data*)
		StatusBit1 : McAEX11ASSSSIFrmCfgStatBit1Enum; (*Warning or error bit*)
		StatusBit2 : McAEX11ASSSSIFrmCfgStatBit2Enum; (*Warning or error bit*)
		SSIDataCode : McAEX11ASSSSIFrmCfgSSIDatCEnum; (*SSI data code*)
		SSIParityCheck : McAEX11ASSSSIFrmCfgSSIParCkEnum; (*SSI parity check*)
	END_STRUCT;
	McAEX11ASSType : STRUCT (*Type mcAEX11AIT_SSI_SIN settings*)
		SSIFrameConfiguration : McAEX11ASSSSIFrmCfgType; (*Frame structure in the order of transfer*)
		LinesPerEncoderRevolution : UDINT; (*Absolute number of lines of an encoder revolution*)
		PositionValuesEncoderRevolution : DINT; (*Absolute resolution of an encoder revolution*)
		SerialPositionPhaseShift : DINT; (*Serial position phase shift*)
		BaudRate : DINT; (*Transfer rate [kBaud]*)
	END_STRUCT;
	McAEX11ASSSIFrmCfgStatBit1Enum :
		( (*Warning or error bit*)
		mcAEX11ASFCSB1_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX11ASFCSB1_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX11ASFCSB1_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX11ASFCSB1_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX11ASFCSB1_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX11ASSSIFrmCfgStatBit2Enum :
		( (*Warning or error bit*)
		mcAEX11ASFCSB2_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX11ASFCSB2_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX11ASFCSB2_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX11ASFCSB2_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX11ASFCSB2_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX11ASSSIFrmCfgSSIDatCEnum :
		( (*SSI data code*)
		mcAEX11ASFCSDC_GRAY := 0, (*Gray - Gray*)
		mcAEX11ASFCSDC_BIN := 1 (*Binary - Binary*)
		);
	McAEX11ASSSIFrmCfgSSIParCkEnum :
		( (*SSI parity check*)
		mcAEX11ASFCSPC_SSI_PAR_CK_ODD := 0, (*SSI parity check odd - SSI parity check odd*)
		mcAEX11ASFCSPC_SSI_PAR_CK_EVEN := 1, (*SSI parity check even - SSI parity check even*)
		mcAEX11ASFCSPC_OFF := 2 (*Off - Off*)
		);
	McAEX11ASSSIFrmCfgType : STRUCT (*Frame structure in the order of transfer*)
		NumberOfLeadingZeros : USINT; (*Zero bits before position data*)
		NumberOfPositionBits : USINT; (*Position data*)
		NumberOfTrailingZeros : USINT; (*Zero bits after position data*)
		StatusBit1 : McAEX11ASSSIFrmCfgStatBit1Enum; (*Warning or error bit*)
		StatusBit2 : McAEX11ASSSIFrmCfgStatBit2Enum; (*Warning or error bit*)
		SSIDataCode : McAEX11ASSSIFrmCfgSSIDatCEnum; (*SSI data code*)
		SSIParityCheck : McAEX11ASSSIFrmCfgSSIParCkEnum; (*SSI parity check*)
	END_STRUCT;
	McAEX11ASType : STRUCT (*Type mcAEX11AIT_SSI settings*)
		SSIFrameConfiguration : McAEX11ASSSIFrmCfgType; (*Frame structure in the order of transfer*)
		IncrementsPerEncoderRevolution : UDINT; (*Absolute resolution of an encoder revolution*)
		BaudRate : DINT; (*Transfer rate [kBaud]*)
	END_STRUCT;
	McAEX11ABBiSSFrmCfgStatBit1Enum :
		( (*Warning or error bit*)
		mcAEX11ABBFCSB1_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX11ABBFCSB1_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX11ABBFCSB1_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX11ABBFCSB1_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX11ABBFCSB1_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX11ABBiSSFrmCfgStatBit2Enum :
		( (*Warning or error bit*)
		mcAEX11ABBFCSB2_NOT_AVL := 0, (*Not available - Not available*)
		mcAEX11ABBFCSB2_ERR_BIT_ACT_HIGH := 1, (*Error bit active high - Error bit active high*)
		mcAEX11ABBFCSB2_ERR_BIT_ACT_LOW := 2, (*Error bit active low - Error bit active low*)
		mcAEX11ABBFCSB2_WAR_BIT_ACT_HIGH := 3, (*Warning bit active high - Warning bit active high*)
		mcAEX11ABBFCSB2_WAR_BIT_ACT_LOW := 4 (*Warning bit active low - Warning bit active low*)
		);
	McAEX11ABBiSSFrmCfgType : STRUCT (*Frame structure in the order of transfer*)
		NumberOfLeadingZeros : USINT; (*Zero bits before position data*)
		NumberOfPositionBits : USINT; (*Position data*)
		NumberOfTrailingZeros : USINT; (*Zero bits after position data*)
		StatusBit1 : McAEX11ABBiSSFrmCfgStatBit1Enum; (*Warning or error bit*)
		StatusBit2 : McAEX11ABBiSSFrmCfgStatBit2Enum; (*Warning or error bit*)
	END_STRUCT;
	McAEX11ABType : STRUCT (*Type mcAEX11AIT_BISS settings*)
		BiSSFrameConfiguration : McAEX11ABBiSSFrmCfgType; (*Frame structure in the order of transfer*)
		CRCPolynomial : UDINT; (*Data verification via CRC (decimal value, 0 deactivates verification)*)
		IncrementsPerEncoderRevolution : UDINT; (*Absolute resolution of an encoder revolution*)
	END_STRUCT;
	McAEX11ASinType : STRUCT (*Type mcAEX11AIT_SIN settings*)
		LinesPerEncoderRevolution : UDINT; (*Absolute number of lines of an encoder revolution*)
	END_STRUCT;
	McAEX11ASinWDCMType : STRUCT (*Type mcAEX11AIT_SIN_W_DCM settings*)
		LinesPerEncoderRevolution : UDINT; (*Absolute number of lines of an encoder revolution*)
		DCMBasicDistance : UDINT; (*DCM basic distance*)
		DCMBasicDifference : DINT; (*DCM basic difference*)
	END_STRUCT;
	McAEEncX11AIfTypType : STRUCT
		Type : McAEEncX11AIfTypEnum; (*Interface type selector setting*)
		SSISine : McAEX11ASSType; (*Type mcAEX11AIT_SSI_SIN settings*)
		SSI : McAEX11ASType; (*Type mcAEX11AIT_SSI settings*)
		BiSS : McAEX11ABType; (*Type mcAEX11AIT_BISS settings*)
		Sine : McAEX11ASinType; (*Type mcAEX11AIT_SIN settings*)
		SineWithDCM : McAEX11ASinWDCMType; (*Type mcAEX11AIT_SIN_W_DCM settings*)
	END_STRUCT;
	McAEEncX11AType : STRUCT
		InterfaceType : McAEEncX11AIfTypType;
	END_STRUCT;
	McAEEncIfTypEnum :
		( (*Interface type selector setting*)
		mcAEIT_ENDAT := 0 (*EnDat -*)
		);
	McAEEncIfTypType : STRUCT
		Type : McAEEncIfTypEnum; (*Interface type selector setting*)
	END_STRUCT;
	McAEEncType : STRUCT
		InterfaceType : McAEEncIfTypType;
	END_STRUCT;
	McCfgAcpEncType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_ENC*)
		EncoderX6A : McAEEncX6AType;
		EncoderX6B : McAEEncX6BType;
		EncoderX41 : McAEEncX41Type;
		EncoderX42 : McAEEncX42Type;
		EncoderX43 : McAEEncX43Type;
		EncoderX11A : McAEEncX11AType;
		Encoder : McAEEncType;
	END_STRUCT;
	McAVAVirtAxEnum :
		( (*Virtual axis selector setting*)
		mcAVAVA_NOT_USE := 0, (*Not used - Virtual axis is not used*)
		mcAVAVA_USE := 1 (*Used - Virtual axis is used*)
		);
	McAVAVirtAxUseHomeModEnum :
		( (*Mode selector setting*)
		mcAVAVAUHM_DIR := 0, (*Direct - Direct homing*)
		mcAVAVAUHM_RES_POS := 1, (*Restore position - Homing by restoring the position from permanent variable data*)
		mcAVAVAUHM_NOT_USE := 100 (*Not used - No preconfigured homing settings used*)
		);
	McAVAVirtAxUseHomeModDirType : STRUCT (*Type mcAVAVAUHM_DIR settings*)
		Position : LREAL; (*Home position [Measurement units]*)
	END_STRUCT;
	McAVAVirtAxUseHomeModType : STRUCT (*Homing mode*)
		Type : McAVAVirtAxUseHomeModEnum; (*Mode selector setting*)
		Direct : McAVAVirtAxUseHomeModDirType; (*Type mcAVAVAUHM_DIR settings*)
	END_STRUCT;
	McAVAVirtAxUseHomeType : STRUCT (*Homing mode and parameters which can be used within the application program as preconfigured setting*)
		Mode : McAVAVirtAxUseHomeModType; (*Homing mode*)
		RestorePositionVariable : STRING[250]; (*Permanent variable used for homing mode: Restore position*)
	END_STRUCT;
	McAVAVirtAxUseType : STRUCT (*Type mcAVAVA_USE settings*)
		AxisReference : McCfgReferenceType; (*Name of the referenced axis component*)
		Homing : McAVAVirtAxUseHomeType; (*Homing mode and parameters which can be used within the application program as preconfigured setting*)
		JerkFilter : McAJFType; (*Jerk filter*)
		AxisFeatures : McAAFType; (*Features for an axis*)
	END_STRUCT;
	McAVAVirtAxType : STRUCT
		Type : McAVAVirtAxEnum; (*Virtual axis selector setting*)
		Used : McAVAVirtAxUseType; (*Type mcAVAVA_USE settings*)
	END_STRUCT;
	McCfgAcpVirtAxType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_VIRT_AX*)
		VirtualAxis : McAVAVirtAxType;
	END_STRUCT;
	McCfgAcpVirtAxRefType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_VIRT_AX_REF*)
		AxisReference : McCfgReferenceType; (*Name of the referenced axis component*)
	END_STRUCT;
	McAVHHomeModEnum :
		( (*Mode selector setting*)
		mcAVHHM_DIR := 0, (*Direct - Direct homing*)
		mcAVHHM_RES_POS := 1, (*Restore position - Homing by restoring the position from permanent variable data*)
		mcAVHHM_NOT_USE := 100 (*Not used - No preconfigured homing settings used*)
		);
	McAVHHomeModDirType : STRUCT (*Type mcAVHHM_DIR settings*)
		Position : LREAL; (*Home position [Measurement units]*)
	END_STRUCT;
	McAVHHomeModType : STRUCT (*Homing mode*)
		Type : McAVHHomeModEnum; (*Mode selector setting*)
		Direct : McAVHHomeModDirType; (*Type mcAVHHM_DIR settings*)
	END_STRUCT;
	McAVHHomeType : STRUCT (*Homing mode and parameters which can be used within the application program as preconfigured setting*)
		Mode : McAVHHomeModType; (*Homing mode*)
		RestorePositionVariable : STRING[250]; (*Permanent variable used for homing mode: Restore position*)
	END_STRUCT;
	McCfgAcpVirtHomeType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_VIRT_HOME*)
		Homing : McAVHHomeType; (*Homing mode and parameters which can be used within the application program as preconfigured setting*)
	END_STRUCT;
	McCfgAcpVirtJerkFltrType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_VIRT_JERK_FLTR*)
		JerkFilter : McAJFType; (*Jerk filter*)
	END_STRUCT;
	McCfgAcpVirtAxFeatType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_VIRT_AX_FEAT*)
		AxisFeatures : McAAFType; (*Features for an axis*)
	END_STRUCT;
	McACFChFeatType : STRUCT (*Features for the channel of a module*)
		FeatureReference : McCfgUnboundedArrayType; (*Name of the axis feature reference*)
	END_STRUCT;
	McCfgAcpChFeatType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_ACP_CH_FEAT*)
		ChannelFeatures : McACFChFeatType; (*Features for the channel of a module*)
	END_STRUCT;
	McCfgAxFeatAcpParTblType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_AX_FEAT_ACP_PAR_TBL*)
		ACOPOSParameterTableReference : STRING[250]; (*Name of the ACOPOS parameter table*)
	END_STRUCT;
END_TYPE