TYPE
	McAcpAxDataTypeEnum :
	(
		mcACPAX_PARTYPE_BOOL := 1,  (*Data type: Digital information, 1 bit (1 byte)*)
		mcACPAX_PARTYPE_SINT,  (*Data type: Whole number, 1 byte*)
		mcACPAX_PARTYPE_INT,  (*Data type: Whole number, 2 bytes*)
		mcACPAX_PARTYPE_DINT,  (*Data type: Whole number, 4 bytes*)
		mcACPAX_PARTYPE_USINT,  (*Data type: Whole number, 1 byte, positive numbers only*)
		mcACPAX_PARTYPE_UINT,  (*Data type: Whole number, 2 bytes, positive numbers only*)
		mcACPAX_PARTYPE_UDINT,  (*Data type: Whole number, 4 bytes, positive numbers only*)
		mcACPAX_PARTYPE_REAL,  (*Data type: Floating point, 4 bytes*)
		mcACPAX_PARTYPE_VOID := 65535   (*General data type*)
	);

	McAcpAxProcessParIDModeEnum :
	(
		mcACPAX_PARID_GET := 0,  (*Read ParID(s)*)
		mcACPAX_PARID_SET,	 (*Write ParID(s)*)
		mcACPAX_PARID_GET_NO_NCT  (*Read ParID(s) without entry in the NCT*)
	);

	McAcpAxCycParIDModeEnum :
	(
		mcACPAX_CYCLIC_PARID_READ := 0,  (*Read ParID(s) cyclically*)
		mcACPAX_CYCLIC_PARID_WRITE  (*Write ParID(s) cyclically*)
	);

	McAcpAxCycParIDRefreshModeEnum :
	(
		mcACPAX_CYCLIC_MULTIPLEXED := 0,  (*Entry in only one data telegram*)
		mcACPAX_CYCLIC_EVERY_RECORD  (*Entry in the current data telegram and every subsequent automatically created data telegram (ParID value is updated every network cycle)*)
	);

	McAcpAxTriggerEnum :
	(
		mcACPAX_TRIGGER_1,	 (*Selects trigger input 1*)
		mcACPAX_TRIGGER_2		 (*Selects trigger input 2*)
	);

	McAcpAxLimitLoadModeEnum :
	(
		mcACPAX_LL_WITH_FEED_FORWARD := 0,  (*The overall torque is limited, i.e. the sum of feed-forward torque and corrective action*)
		mcACPAX_LL_WITHOUT_FEED_FORWARD    (*Only the share of torque that results from control deviations is limited. Feed-forward torque is ignored*)
	);

	McAcpAxBrakeTestCmdEnum :
	(
		mcACPAX_BRAKE_TEST_INIT := 0,		 (*Transfers parameters for holding brake test*)
		mcACPAX_BRAKE_TEST_START := 1,	 (*Starts the holding brake test*)
		mcACPAX_BRAKE_TEST_INIT_START 	 (*Initializes and starts the holding brake test*)
	);

	McAcpAxBrakeTestModeEnum :
	(
		mcACPAX_BRAKE_TEST_STANDARD := 0,		 (*Standard brake test*)
		mcACPAX_BRAKE_TEST_SAFETY			 (*Safe brake test*)
	);

	McAcpAxAxisTypeEnum :
	(
		mcACPAX_AXIS_REAL,	 (*Real axis*)
		mcACPAX_AXIS_VIRTUAL	 (*Virtual axis*)
	);

	McAcpAxProductFamilyEnum :
	(
		mcACPAX_ACOPOS, (*ACOPOS*)
		mcACPAX_ACOPOS_MULTI,		 (*ACOPOSmulti*)
		mcACPAX_ACOPOS_MICRO,		 (*ACOPOSmicro*)
		mcACPAX_ACOPOS_REMOTE, 	 (*ACOPOSremote*)
		mcACPAX_ACOPOS_MOTOR,		 (*ACOPOSmotor*)
		mcACPAX_ACOPOS_P3			 (*ACOPOS P3*)
	);

	McAcpAxModuleTypeEnum :
	(
		mcACPAX_MODULE_INVERTER,				 (*Inverter module*)
		mcACPAX_MODULE_ACTIVE_SUPPLY, 	 (*Active power supply module*)
		mcACPAX_MODULE_PASSIVE_SUPPLY	 (*Passive power supply module*)
	);

	McAcpAxSimulationOnPlcEnum :
	(
		mcACPAX_SIM_STATE_OFF,				 (*Simulation disabled*)
		mcACPAX_SIM_STATE_ON, 				 (*Simulation enabled*)
		mcACPAX_SIM_SET_VALUE_GENERATION, 	 (*Simulation based on target value generation only*)
		mcACPAX_SIM_COMPLETE_CTRL_SYSTEM	 (*Full simulation*)
	);

	McAcpAxCtrlModeEnum :
	(
		mcACPAX_CTRL_POSITION := 1, 		 (*Position control*)
		mcACPAX_CTRL_POSITION_WITH_FF := 33 	 (*Position controller with feed-forward control*)
	);

	McAcpAxCtrlParSelectEnum :
	(
		mcACPAX_CTRL_SELECT_ALL_PAR, 		 (*Default setting; all parameters are transferred*)
		mcACPAX_CTRL_SELECT_POSITION,		 (*Used for selecting the position controller parameters*)
		mcACPAX_CTRL_SELECT_SPEED,			 (*Used for selecting the velocity controller parameters*)
		mcACPAX_CTRL_SELECT_FEED_FORWARD,	 (*Used for selecting the feed-forward control parameters*)
		mcACPAX_CTRL_SELECT_ADV_PAR_ONLY	 (*Used for selecting advanced parameters (if written)*)
	);

	McAcpAxLoopFilterTypeEnum :
	(
		mcACPAX_LOOP_FILTER_NO_TRANSFER, 	 (*Filter parameter not transferred*)
		mcACPAX_LOOP_FILTER_OFF, 			 (*Filter switched off*)
		mcACPAX_LOOP_FILTER_LOWPASS, 		 (*Low-pass filter*)
		mcACPAX_LOOP_FILTER_NOTCH,			 (*Notch*)
		mcACPAX_LOOP_FILTER_Z_TRANS_FUN,	 (*Z-transfer function*)
		mcACPAX_LOOP_FILTER_COMPENSATION,	 (*Compensation*)
		mcACPAX_LOOP_FILTER_BIQUAD,			 (*Biquad filter*)
		mcACPAX_LOOP_FILTER_LIM,			 (*Limits*)
		mcACPAX_LOOP_FILTER_LIM_LINEAR,		 (*Linear limit*)
		mcACPAX_LOOP_FILTER_LIM_R_TIME 		 (*Rise limit*)
	);

	McAcpAxAutoTuneExSignalEnum :
	(
		mcACPAX_EX_SIGNAL_PRBS := 0, 		 (*Standard*)
		mcACPAX_EX_SIGNAL_CHIRP := 1, 		 (*Chirp (linear frequency modulation)*)
		mcACPAX_EX_SIGNAL_CHIRP_TRAPEZ := 2  (*Chirp (linear frequency modulation) with trapezoidal amplitude response*)
	);

	McAcpAxAutoTuneTestModeEnum:
	(
		mcACPAX_TEST,  (*Controller test*)
		mcACPAX_TEST_POSITION,  (*Controller test (position controller)*)
		mcACPAX_TEST_SPEED  (*Controller test (rotary speed controller)*)
	);

	McAcpAxAutoTuneOrientationEnum:
	(
		mcACPAX_ORIENTATION_HORIZONTAL,  (*Horizontal orientation*)
		mcACPAX_ORIENTATION_VERTICAL     (*Vertical orientation*)
	);

	McAcpAxFilterTimeModeEnum:
	(
		mcACPAX_FILTER_TIME_USE,  (*The determination of filter time constants is disabled; however, filter time constants are taken into account for autotuning*)
		mcACPAX_FILTER_TIME_TUNE_MODE1,  (*The controlled variable is the unfiltered actual speed n*)
		mcACPAX_FILTER_TIME_TUNE_MODE2   (*The controlled variable is the filtered actual speed n*)
	);

	McAcpAxLoopFilterModeEnum:
	(
		mcACPAX_LOOP_FILTER_IGNORE,  (*Loop filters are neither taken into account nor calculated. *)
		mcACPAX_LOOP_FILTER_USE,  (*The parameters for all loop filters are taken into account for autotuning*)
		mcACPAX_LOOP_FILTER_TUNE_NOTCH  (*The parameters for the loop filter are calculated*)
	);

	McAcpAxIntegrationTimeModeEnum:
	(
		mcACPAX_INTEGRATION_TIME_IGNORE,  (*Integral action time is neither taken into account nor calculated*)
		mcACPAX_INTEGRATION_TIME_USE,  (*Integral action time is taken into account for autotuning*)
		mcACPAX_INTEGRATION_TIME_TUNE  (*Integral action time is calculated for autotuning*)
	);

	McAcpAxAutoTuneOperatPointEnum:
	(
		mcACPAX_OP_TUNE_STANDSTILL,  (*Autotuning at standstill*)
		mcACPAX_OP_TUNE_V_CONSTANT  (*Autotuning at constant velocity*)
	);

	McAcpAxSimulationModeEnum :
	(
		mcACPAX_SIMULATION_1MASS_AUTO,	 (*Standard (1-mass model, parameters determined automatically)*)
		mcACPAX_SIMULATION_1MASS,		 (*1-mass load model*)
		mcACPAX_SIMULATION_2MASS,		 (*2-mass load model*)
		mcACPAX_SIMULATION_SET_GEN_ONLY		 (*set value generation only*)
	);

   	McAcpAxHomingParType : STRUCT
        HomingMode : McHomingModeEnum; (*Mode for homing*)
		Position : LREAL; (*Absolute position or homing offset when homing signal [Measurement units] occurs*)
		StartVelocity : REAL; (*Velocity for reference switch search [Measurement units/s]*)
		HomingVelocity : REAL; (*Velocity (after reaching reference switch) [Measurement units/s]*)
		Acceleration : REAL; (*Maximum acceleration [Measurement units/s]*)
		SwitchEdge : McDirectionEnum; (*Edge of reference switch*)
		StartDirection : McDirectionEnum; (*Start direction for searching the reference edge*)
		HomingDirection : McDirectionEnum; (*Direction for homing (after reaching reference switch) *)
		ReferencePulse : McSwitchEnum; (*The encoder's reference pulse is used for homing*)
		KeepDirection : McSwitchEnum; (*The direction of movement is or is not permitted to be changed during the homing procedure*)
		ReferencePulseBlockingDistance : REAL; (*Distance for blocking activation of "triggering reference pulse" [Measurement units]*)
		TorqueLimit : REAL; (*Torque limit value for homing to blocks [Nm]*)
		BlockDetectionPositionError : REAL; (*Lag error for block detection [Measurement units]*)
		PositionErrorStopLimit : REAL; (*Lag error for canceling homing procedure [Measurement units]*)
		RestorePositionVariableAddress : UDINT; (*Address of a permanent variable of type McAcpAxRestorePosType that is needed for "HomingMode" mcHOMING_RESTORE_POSITION*)
	END_STRUCT;

	McAcpAxProcessParIDType : STRUCT
		ParID : UINT; (*Parameter ID number to be read or written*)
		VariableAddress : UDINT; (*Address of the variable that receives the read value or the corresponding value that is to be written*)
		DataType : McAcpAxDataTypeEnum; (*Data type of the variable:*)
	END_STRUCT;

	McAcpAxCycParIDType : STRUCT
		ParID : UINT; (*Parameter ID number to be read or written*)
		VariableAddress : UDINT; (*Address of the variable that receives the read value or the corresponding value that is to be written*)
		DataType : McAcpAxDataTypeEnum; (*Data type of the variable:*)
		RefreshMode : McAcpAxCycParIDRefreshModeEnum; (*Defines how often a ParID should be read/written*)
	END_STRUCT;

	McAcpAxTriggerStopType : STRUCT
		TriggerSource : McAcpAxTriggerEnum; (*Source of the trigger event*)
		TriggerEdge : McEdgeEnum; (*Selection of the edge for the trigger event*)
		TriggerDistance : LREAL; (*Distance after the trigger event occurs [Measurement units]*)
		ForceTriggerDistance : McSwitchEnum; (*Forces the trigger distance to be traveled even if it exceeds the end position*)
	END_STRUCT;

	McAcpAxAdvLimitLoadParType : STRUCT
		LoadPosAccelParID : UINT; (*ParID with limit value for accelerating torque in the positive direction*)
		LoadPosDecelParID : UINT; (*ParID with limit value for decelerating torque in the positive direction*)
		LoadNegAccelParID : UINT; (*ParID with limit value for accelerating torque in the negative direction*)
		LoadNegDecelParID : UINT; (*ParID with limit value for decelerating torque in the negative direction*)
	END_STRUCT;

	McAcpAxBrakeParType : STRUCT
		AutomaticControl : McSwitchEnum := mcSWITCH_ON; (*Automatic control on/off (Default setting: *)
		RestrictedBrakeControl : McSwitchEnum := mcSWITCH_ON; (*Holding brake can only be applied and released (Default setting: *)
		ControlMonitoring : McSwitchEnum := mcSWITCH_ON; (*Enables/disables control monitoring (Default setting: *)
		MovementMonitoring : McSwitchEnum := mcSWITCH_ON; (*Enables/disables movement monitoring (Default setting: *)
		VoltageMonitoring : McSwitchEnum := mcSWITCH_ON; (*Enables/disables monitoring of external voltage over 24 V (Default setting: *)
		TestAtPowerOn : McSwitchEnum := mcSWITCH_OFF; (*Enables/disables automatic torque testing when the controller is switched on (Default setting: *)
		TestAtPowerOff : McSwitchEnum := mcSWITCH_OFF; (*Enables/disables automatic torque testing when the controller is switched off (Default setting: *)
		AutomaticInductionStop : McSwitchEnum := mcSWITCH_ON; (*Enables/disables automatic induction stop (Default setting: *)
		EnableSBTRequestBySMC : McSwitchEnum := mcSWITCH_OFF; (*Enables the automatic safe brake test requested and monitored by module SafeMC (Default setting: *)
		ControlMonitoringFilterTime : REAL := 0.5; (*Time after which an error is reported after control monitoring is enabled. [s] (Default setting: 0.5)*)
	END_STRUCT;

	McAcpAxBrakeTestParType : STRUCT
		Mode : McAcpAxBrakeTestModeEnum; (*Mode for brake test*)
		Torque : REAL; (*Torque for holding brake test [Nm]*)
		Duration : REAL; (*Duration of holding brake test [s]*)
		PositionLimit : LREAL; (*Position error limit for holding brake test [Measurement units]*)
	END_STRUCT;

	McAcpAxSimulationMass1Type : STRUCT
		Inertia : REAL; (*Inertia [kgm²]*)
		StaticFriction : REAL; (*Static friction [Nm]*)
		ViscousFriction : REAL; (*Viscous friction*)
	END_STRUCT;

	McAcpAxSimulationMass2Type : STRUCT
		Inertia : REAL; (*Inertia [kgm²]*)
		StaticFriction : REAL; (*Static friction [Nm]*)
		ViscousFriction : REAL; (*Viscous friction*)
		Stiffness : REAL; (*Stiffness to coupled mass 1 [Nm]*)
		Damping : REAL; (*Damping to coupled mass 1 [Nms]*)
	END_STRUCT;

	McAcpAxSimulationParType : STRUCT
		Mode : McAcpAxSimulationModeEnum; (*Mode for load simulation on the drive*)
		AdditiveLoadParID : UINT; (*Parameter ID for applying an additional load*)
		Mass1 :  McAcpAxSimulationMass1Type; (*Parameter for determining the first mass*)
		Mass2 :  McAcpAxSimulationMass2Type; (*Parameter for determining the second mass*)
	END_STRUCT;

	McAcpAxRestorePosType: STRUCT
		Data : ARRAY[0..17] OF DINT; (*Data for restoring the position*)
	END_STRUCT;

	McAcpAxAxisInfoType : STRUCT
		AxisType : McAcpAxAxisTypeEnum; (*Axis type*)
		ChannelNumber : UDINT; (*Channel number of the axis*)
		AcoposSimulationOnPlcMode : McAcpAxSimulationOnPlcEnum; (*Information about the ACOPOS drive simulation mode that is enabled on the controller for this axis*)
	END_STRUCT;

	McAcpAxModuleInfoType : STRUCT
		ProductFamily : McAcpAxProductFamilyEnum; (*Product family*)
		ModuleType : McAcpAxModuleTypeEnum; (*Module type*)
		NetworkType : McNetworkTypeEnum; (*Network type*)
		NodeNumber : UDINT; (*Node number*)
		AcoposSimulationOnPlc : McAcpAxSimulationOnPlcEnum; (*Information about whether ACOPOS drive simulation is enabled on the controller for this module*)
	END_STRUCT;

	McAcpAxPosCtrlParType : STRUCT
		ProportionalGain : REAL; (*Proportional gain [1/s]*)
		IntegrationTime : REAL; (*Integral action time of integral component [s]*)
		PredictionTime : REAL; (*Prediction time [s]*)
		TotalDelayTime : REAL; (*Total delay [s]*)
	END_STRUCT;

	McAcpAxSpeedCtrlParType : STRUCT
		ProportionalGain : REAL; (*Proportional gain [As/Rev.]*)
		IntegrationTime : REAL; (*Integral action time of integral component [s]*)
		FilterTime : REAL; (*Filter time constant [s]*)
	END_STRUCT;

	McAcpAxFeedForwardParType : STRUCT
		TorqueLoad : REAL; (*Load torque [Nm}*)
		TorquePositive : REAL; (*Torque in positive direction [Nm]*)
		TorqueNegative : REAL; (*Torque in negative direction [Nm]*)
		SpeedTorqueFactor : REAL; (*Velocity torque factor [Nms]*)
		Inertia : REAL; (*Moment of inertia [kgm²]*)
		AccelerationFilterTime : REAL; (*Acceleration filter time constant [s]*)
	END_STRUCT;

	McAcpAxLoopFilterParType : STRUCT
		Type : McAcpAxLoopFilterTypeEnum; (*Loop filter type used*)
		LowPass : McAcpAxLoopFilterLowPassType; (*Parameter for low pass*)
		Notch : McAcpAxLoopFilterNotchType; (*Parameter for band-stop filter*)
		ZTransferFunction : McAcpAxLoopFilterZTransFunType; (*Parameter for z-transfer function*)
		Compensation : McAcpAxLoopFilterCompType; (*Parameter for compensation*)
		Biquad : McAcpAxLoopFilterBiquadType; (*Parameter for biquad filter*)
		Limiter : McAcpAxLoopFilterLimType; (*Parameter for limit*)
		LimiterLinear : McAcpAxLoopFilterLimLinearType; (*Parameter for linear limit*)
		LimiterRiseTime : McAcpAxLoopFilterLimRiseTimeType; (*Parameter for rise limit*)
	END_STRUCT;

	McAcpAxLoopFilterLowPassType : STRUCT
		CutOffFrequency : REAL; (*Low pass: Limit frequency [Hz]*)
	END_STRUCT;

	McAcpAxLoopFilterNotchType : STRUCT
		CenterFrequency : REAL; (*Band-stop: Center frequency [Hz]*)
		Bandwidth : REAL; (*Band-stop: bandwidth [Hz]*)
	END_STRUCT;

	McAcpAxLoopFilterZTransFunType: STRUCT
		A0 : REAL; (*Coefficient a0 of the denominator polynomial*)
		A1 : REAL; (*Coefficient a1 of the denominator polynomial*)
		B0 : REAL; (*Coefficient b0 of the numerator polynomial*)
		B1 : REAL; (*Coefficient b1 of the numerator polynomial*)
		B2 : REAL; (*Coefficient b2 of the numerator polynomial*)
	END_STRUCT;

	McAcpAxLoopFilterCompType : STRUCT
		MultiplicationFactorParID : UINT; (*ACOPOS drive parameter ID for multiplication point*)
		AdditiveValueParID : UINT; (*ACOPOS drive parameter ID for addition point*)
	END_STRUCT;

	McAcpAxLoopFilterBiquadType : STRUCT
		FrequencyNumerator : REAL; (*Biquad filter: Known frequency counter [Hz]*)
		DampingNumerator : REAL; (*Biquad filter: Damping counter*)
		FrequencyDenominator : REAL; (*Biquad filter: Known frequency denominator [Hz]*)
		DampingDenominator : REAL; (*Biquad filter: Damping denominator*)
	END_STRUCT;

	McAcpAxLoopFilterLimType : STRUCT
		PositiveLimit : REAL; (*Positive limit [A]*)
		NegativeLimit : REAL; (*Negative limit [A]*)
		PositiveLimitParID : UINT; (*ACOPOS drive parameter ID for the magnitude of the positive limit*)
		NegativeLimitParID : UINT; (*ACOPOS drive parameter ID for the magnitude of the negative limit*)
	END_STRUCT;

	McAcpAxLoopFilterLimLinearType : STRUCT
		InputParID : UINT; (*ACOPOS drive parameter ID for the function input*)
		InputLimit : REAL; (*Input limit value for complete limit [Hz]*)
		Gradient : REAL; (*Slope [As]*)
	END_STRUCT;

	McAcpAxLoopFilterLimRiseTimeType : STRUCT
		RiseTime : REAL; (*Rise time [s]*)
		NormalizedLimit : REAL; (*Normalized limit*)
	END_STRUCT;

	McAcpAxAutoTuneExSignalType: STRUCT
		SignalType:  McAcpAxAutoTuneExSignalEnum; (*Type of excitation signal*)
		SignalOrder : UDINT := 9; (*Order of the excitation signal (only for signal type PRBS)*)
		DelayTime : REAL; (*Delay time for transient operations [s] (only for signal type PRBS)*)
		SignalStartFrequency : REAL; (*Starting frequency of the excitation signal [Hz] (only for chirp signal types)*)
		SignalStopFrequency : REAL; (*Stopping frequency of the excitation signal [Hz] (only for chirp signal types)*)
		SignalTime : REAL; (*Duration of the excitation signal [s] (only for chirp signal types)*)
	END_STRUCT;

	McAcpAxAutoTuneParType : STRUCT
		Orientation : McAcpAxAutoTuneOrientationEnum; (*Selects the orientation for autotuning*)
		MaxCurrentPercent : REAL; (*Percentage of the rated current that is used during autotuning [%]*)
		MaxDistance : LREAL; (*Maximum distance traveled during autotuning [Measurement units]*)
		MaxPositionError : LREAL; (*Maximum permitted lag error during autotuning[Measurement units]*)
	END_STRUCT;

	McAcpAxAutoTuneSpeedCtrlOutType : STRUCT
		Quality : REAL; (*Quality of parameter identification [%]*)
		EstimatedInertia : REAL; (*Estimated drive inertia [kg/m*)
		ProportionalGain : REAL; (*Estimated proportional gain factor [As]*)
		IntegrationTime : REAL; (*Estimated integral action time [s]*)
		FilterTime : REAL; (*Filter time constant [s]*)
		LoopFilter1 : McAcpAxLoopFilterParType; (*LoopFilter1 settings*)
		PhaseCrossoverFrequency : REAL; (*Phase crossover frequency of the controlled system [Hz]*)
	END_STRUCT;

	McAcpAxAutoTuneLoopFilterOutType : STRUCT
		Quality : REAL; (*Quality of parameter identification [%]*)
		LoopFilter1 : McAcpAxLoopFilterParType; (*Parameter for first control loop filter*)
		LoopFilter2 : McAcpAxLoopFilterParType; (*Parameter for additional control loop filter*)
		LoopFilter3 : McAcpAxLoopFilterParType; (*Parameter for additional control loop filter*)
	END_STRUCT;

	McAcpAxAutoTunePosCtrlOutType : STRUCT
		Quality : REAL; (*Quality of parameter identification [%]*)
		ProportionalGain : REAL; (*Estimated proportional gain factor [As/U]*)
	END_STRUCT;

	McAcpAxAutoTuneTestOutType : STRUCT
		Quality : REAL; (*Quality of parameter identification [%]*)
	END_STRUCT;

	McAcpAxAutoTuneLoopFiltersType : STRUCT
		LoopFilter1Mode : McAcpAxLoopFilterModeEnum := mcACPAX_LOOP_FILTER_IGNORE; (*Mode for loop filter tuning:*)
		LoopFilter2Mode : McAcpAxLoopFilterModeEnum := mcACPAX_LOOP_FILTER_IGNORE; (*Mode for loop filter tuning:*)
		LoopFilter3Mode : McAcpAxLoopFilterModeEnum := mcACPAX_LOOP_FILTER_IGNORE; (*Mode for loop filter tuning:*)
	END_STRUCT;

	McAcpAxAdvAutoTuneSpeedCtrlType : STRUCT
	    LoopFilter1Mode : McAcpAxLoopFilterModeEnum; (*Mode for taking LoopFilter1 into account*)
	    FilterTimeMode : McAcpAxFilterTimeModeEnum; (*Mode for taking the filter time constant into account*)
	    IntegrationTimeMode : McAcpAxIntegrationTimeModeEnum; (*Mode for taking integral action time into account*)
	    OperatingPoint : McAcpAxAutoTuneOperatPointEnum; (*Selects the operating point for autotuning*)
	    Velocity : REAL; (*Maximum velocity applied during autotuning if "OperatingPoint = *)
	    Acceleration : REAL; (*Acceleration applied during autotuning if "OperatingPoint = *)
	    MaxProportionalGain : REAL := 2000; (*Maximum proportional gain [As]*)
	    ProportionalGainPercent : REAL := 100; (*Percentage of the proportional gain determined during autotuning that will be used for the control parameters [%]*)
	    ResonanceFactor : REAL := 2; (*Factor for detecting resonance*)
	    InertiaEstimationLowerFrequency  : REAL := 10; (*Lower frequency for estimating the mass moment of inertia of the drive [Hz]*)
	    InertiaEstimationUpperFrequency  : REAL := 40; (*Upper frequency for estimating the mass moment of inertia of the drive [Hz]*)
	    ExcitationSignal : McAcpAxAutoTuneExSignalType; (*Parameter for excitation signal*)
	END_STRUCT;

	McAcpAxAdvAutoTuneLoopFilterType : STRUCT
	    OperatingPoint : McAcpAxAutoTuneOperatPointEnum; (*Selects the operating point for autotuning*)
	    Velocity : REAL; (*Maximum velocity applied during autotuning if "OperatingPoint = *)
	    Acceleration : REAL; (*Acceleration applied during autotuning if "OperatingPoint = *)
	    ResonanceFactor : REAL := 2; (*Factor for detecting resonance*)
	    ExcitationSignal : McAcpAxAutoTuneExSignalType; (*Parameter for excitation signal*)
	END_STRUCT;

	McAcpAxAdvAutoTunePosCtrlType : STRUCT
	    OperatingPoint : McAcpAxAutoTuneOperatPointEnum; (*Selects the operating point for autotuning*)
	    Velocity : REAL; (*Maximum velocity applied during autotuning if "OperatingPoint = *)
	    Acceleration : REAL; (*Acceleration applied during autotuning if "OperatingPoint = *)
	    MaxProportionalGain : REAL := 2000; (*Maximum proportional gain [As]*)
	    ProportionalGainPercent : REAL := 100; (*Percentage of the proportional gain determined during autotuning that will be used for the control parameters [%]*)
	    ExcitationSignal : McAcpAxAutoTuneExSignalType; (*Parameter for excitation signal*)
	END_STRUCT;

	McAcpAxAdvAutoTuneTestType : STRUCT
	    ExcitationSignal : McAcpAxAutoTuneExSignalType; (*Parameter for excitation signal*)
	END_STRUCT;

	McAcpAxAdvCamAutSetParType : STRUCT
		ParLock : McCamAutParLockCmdEnum; (*Command for the transfer of the parameter*)
	END_STRUCT;

	McAcpAxCamAutEventParType : STRUCT
	    Type : McCamAutEventTypeEnum; (*Event type*)
	    Transition :	McCamAutEventTransitionEnum; (*Event transition*)
	    SynchronousUpdate: McSwitchEnum; (*Synchronous parameter update if event occurs*)
	    NextState : USINT; (*Index of the next state if the event occurs*)
	END_STRUCT;

	McAcpAxCamAutCompParType : STRUCT
	    MasterCompDistance : LREAL; (*Compensation distance for the master axis [measurement units of master]*)
	    SlaveCompDistance : LREAL; (*Compensation distance for the slave axis [measurement units of slave]*)
	    MasterCamLeadIn : LREAL; (*Relative position at which the master axis enters the state [measurement units of master]*)
	    MinMasterCompDistance : LREAL; (*Minimum compensation distance for the master axis [measurement units of master]*)
	    MinSlaveCompDistance : LREAL; (*Minimum compensation distance for the slave axis [slave measurement unit]*)
	    MaxSlaveCompDistance : LREAL; (*Maximum compensation distance for the slave axis [slave measurement unit]*)
	    MinSlaveCompVelocity : REAL; (*Minimum velocity of the slave axis during compensation [measurement units of slave / s]*)
	    MaxSlaveCompVelocity : REAL; (*Maximum velocity of the slave axis during compensation [measurement units of slave / s]*)
	    MaxSlaveCompAccel1 : REAL; (*Maximum acceleration of the slave axis during compensation phase 1 [measurement units of slave / s²]*)
	    MaxSlaveCompAccel2 : REAL; (*Maximum acceleration of the slave axis during compensation phase 2 [measurement units of slave / s²]*)
	    SlaveCompJoltTime : REAL; (*Jerk time of the slave axis during compensation [s]*)
	END_STRUCT;

	McAcpAxCamAutAdvStateParType : STRUCT
	    RepeatCounterInit : UINT; (*Starting value of state repetitions for event *)
	    RepeatCounterSetTransfer : McSwitchEnum; (*"RepeatCounterInit" is transferred*)
	    RepeatCounterSet : UINT; (*State repetitions counter for event *)
	    MasterAxis : REFERENCE TO McAxisType; (*Master axis for this state*)
	    MasterParID : UINT; (*Parameter ID of the master axis for this state*)
	END_STRUCT;

	McAcpAxCamAutStateParType : STRUCT
	    CamID : UINT; (*Index of the cam data for a state*)
	    MasterFactor : DINT; (*Master gauge factor for the cam profile for this state*)
	    SlaveFactor: DINT; (*Slave gauge factor for the cam profile for this state*)
	    CompensationMode : McCamAutCompModeEnum; (*Compensation gear mode*)
	    CompensationParameters : McAcpAxCamAutCompParType; (*Parameter for the compensation gear*)
	    AdvancedParameters : McAcpAxCamAutAdvStateParType; (*Advanced state parameter*)
	    Event : ARRAY[0..4] OF McAcpAxCamAutEventParType; (*Definition of the event for a state*)
	END_STRUCT;

	McAcpAxCamAutCtrlSettingsType : STRUCT
	    CrossLeftBoundary : McCamAutCrossLeftBoundEnum; (*Function on the left cam edge during backward movement of the master without event *)
	    CamChangeImmediately : McCamAutCamChangeImmedEnum; (*Direction of the cam change on event transition *)
	END_STRUCT;

	McAcpAxCamAutMsgSettingsType : STRUCT
	    ErrorsInStandby : McCamAutErrorsInStandbyEnum; (*Used for setting an error message in stand-by mode*)
	    ExceedingLimits : McCamAutExceedingLimitsEnum; (*Used for setting a message if the limit values are exceeded*)
	END_STRUCT;

	McAcpAxCamAutTriggerAndLatchType : STRUCT
	    Trigger1Delay : REAL; (*Trigger1 delay time [s] to compensate for a signal delay*)
	    Trigger2Delay : REAL; (*Trigger2 delay time [s] to compensate for a signal delay*)
	    SlaveLatchParID : UINT; (*Parameter ID for the latch value of the slave axis *)
	END_STRUCT;

	McAcpAxCamAutStartStateParType : STRUCT
	    StartState : USINT; (*State in which the cam automat is started*)
	    MasterStartRelPos : LREAL; (*Master position within the starting state at which the cam automat is started [measurements units of the master]*)
	END_STRUCT;

	McAcpAxCamAutAddAxesType : STRUCT
	    AdditiveMasterAxis : REFERENCE TO McAxisType; (*Axis reference of additive master axis*)
	    AdditiveMasterParID : UINT; (*ParID of the additive master axis*)
	    AdditiveSlaveAxis : REFERENCE TO McAxisType; (*Axis reference of additive slave axis*)
	    AdditiveSlaveParID : UINT; (*ParID of additive slave axis*)
	END_STRUCT;

	McAcpAxCamAutCommonFactorsType : STRUCT
	    SlaveFactorParID : UINT; (*Parameter ID for multiplication factor of the slave axis*)
	END_STRUCT;

	McAcpAxCamAutAdvParType : STRUCT
		StartStateParam : McAcpAxCamAutStartStateParType; (*Parameter used for starting directly from a state*)
		AdditiveAxes : McAcpAxCamAutAddAxesType; (*Parameter for additive axes*)
	    MasterStartPosMode : McCamAutMaStartPosModeEnum; (*Mode for event type *)
	    ControlSettings : McAcpAxCamAutCtrlSettingsType; (*Control settings for cam automat*)
	    MessageSettings : McAcpAxCamAutMsgSettingsType; (*Settings for warnings and error messages*)
	    TriggerAndLatch : McAcpAxCamAutTriggerAndLatchType; (*Settings for triggering delay times and for latch value*)
	    EventParID1 : UINT; (*Parameter ID for event input 1*)
	    EventParID2 : UINT; (*Parameter ID for event input 2*)
	    EventParID3 : UINT; (*Parameter ID for event input 3*)
	    EventParID4 : UINT; (*Parameter ID for event input 4*)
	    StartIntervalPos1 : LREAL; (*Relative starting positions of the master axis in the interval for generating event*)
	    StartIntervalPos2 : LREAL; (*Relative starting positions of the master axis in the interval for generating event*)
	    StartIntervalPos3 : LREAL; (*Relative starting positions of the master axis in the interval for generating event*)
	    StartIntervalPos4 : LREAL; (*Relative starting positions of the master axis in the interval for generating event*)
	    Factors : McAcpAxCamAutCommonFactorsType; (*Multiplication factors for all states of the cam automat*)
	END_STRUCT;

	McAcpAxCamAutMasterParType : STRUCT
	    MasterAxis : REFERENCE TO McAxisType; (*Axis reference of the master axis*)
	    MasterParID : UINT; (*ParID of the master axis*)
	    MasterStartPosition : LREAL; (*Starting position of the master axis [measurement units of master]*)
	    MasterStartInterval : LREAL; (*Starting interval of the master axis [measurement units of master]*)
	    MaxMasterVelocity : REAL; (*Maximum velocity of the master axis [measurement units of master] *)
	END_STRUCT;

	McAcpAxCamAutCommonParType : STRUCT
	    Master : McAcpAxCamAutMasterParType; (*Parameter for the cam automat master*)
	    AdvancedParameters : McAcpAxCamAutAdvParType; (*Optional parameter for the cam automat*)
	END_STRUCT;

	McAcpAxCamAutParType : STRUCT
	    Common : McAcpAxCamAutCommonParType; (*General parameter for all states of the cam automat*)
	    State : ARRAY[0..14] OF McAcpAxCamAutStateParType; (*Parameter for the states of the cam automat*)
	END_STRUCT;

	 McAcpAxCamAutDefineType : STRUCT
	    DataObjectName : STRING[32]; (*Name of the cam automat configuration object*)
	    DataAddress : UDINT; (*Address of a variable of data type McAcpAxCamAutParType*)
	END_STRUCT;

	McAcpAxAdvPhasingParType : STRUCT
	    VelocityParID : UINT; (*ParID from which the velocity for the phase shift is read*)
	    PosVelocityTriggerParID : UINT; (*ParID controls the addition of velocity "CyclicVelocity" or the value of "VelocityParID"*)
	    NegVelocityTriggerParID : UINT; (*ParID controls the subtraction of velocity "CyclicVelocity" or the value of "VelocityParID"*)
	END_STRUCT;

	McAcpAxAdvOffsetParType : STRUCT
	    VelocityParID : UINT; (*ParID from which the velocity for the phase shift is read*)
	    PosVelocityTriggerParID : UINT; (*ParID controls the addition of velocity "CyclicVelocity" or the value of "VelocityParID"*)
	    NegVelocityTriggerParID : UINT; (*ParID controls the subtraction of velocity "CyclicVelocity" or the value of "VelocityParID"*)
	END_STRUCT;

	McAcpAxLoadSimInputDataType : STRUCT
		Position : LREAL; (*Position value [rad]*)
		Velocity : REAL; (*Velocity value [rad/s]*)
 		Acceleration : REAL; (*Acceleration value [rad/s²] Note: This structure element is not supported currently, and always the value "0.0" is output*)
	END_STRUCT;

	McAcpAxCtrlParType : STRUCT
		Mode : McAcpAxCtrlModeEnum; (*Controller mode*)
		PositionController : McAcpAxPosCtrlParType; (*Structure of the parameters for the position controller*)
		SpeedController : McAcpAxSpeedCtrlParType; (*Structure of the parameters for the velocity controller*)
		FeedForward : McAcpAxFeedForwardParType; (*Structure of the parameters for feed-forward control*)
		ParameterSelector : McAcpAxCtrlParSelectEnum; (*Used for selecting the parameter sets that are to be transferred to the axis when there is a rising edge on input "Execute"*)
	END_STRUCT;
	
	McAcpAxAdvAutoTuneFeedFwdType : STRUCT
	    ExcitationSignal : McAcpAxAutoTuneExSignalType; (*Parameter for excitation signal*)
	END_STRUCT;

END_TYPE
