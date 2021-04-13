
TYPE
	MpAxisBasicParType : 	STRUCT 
		Position : LREAL := 0.0; (*Target position for the movement [measurement units]*)
		Distance : LREAL := 0.0; (*Traverse path for the movement [measurement units]*)
		Velocity : REAL := 5; (*Maximum velocity [measurement units / s]*)
		Acceleration : REAL := 50; (*Maximum acceleration [measurement units / s²]*)
		Deceleration : REAL := 50; (*Maximum deceleration [measurement units / s²]*)
		Direction : McDirectionEnum := mcDIR_POSITIVE; (*Direction of movement *)
		Homing : MpAxisHomingType; (*Homing parameter*)
		Jog : MpAxisJogType; (*Jog parameter*)
		Stop : MpAxisStopType; (*Stop parameter*)
		LimitLoad : MpAxisLimitLoadType; (*Limit load parameter*)
	END_STRUCT;
	MpAxisHomingType : 	STRUCT 
		Mode : McHomingModeEnum; (*Selects the mode for homing.*)
		Position : LREAL; (*Absolute position after the homing signal is detected [measurement units] *)
	END_STRUCT;
	MpAxisJogType : 	STRUCT 
		Velocity : REAL := 2; (*Maximum velocity [measurement units / s]*)
		Acceleration : REAL := 25; (*Maximum acceleration [measurement units / s²]*)
		Deceleration : REAL := 50; (*Maximum deceleration [measurement units / s²]*)
		LimitPosition : MpAxisJogLimitPositionType; (*Limit positions for the jog movement *)
	END_STRUCT;
	MpAxisStopType : 	STRUCT 
		Deceleration : REAL := 50; (*Maximum deceleration [measurement units / s²]*)
	END_STRUCT;
	MpAxisJogLimitPositionType : 	STRUCT 
		FirstPosition : LREAL := 0.0; (*"Lower" limit position for the jog movement [measurement units]*)
		LastPosition : LREAL := 0.0; (*"Upper" limit position for the jog movement [measurement units]*)
	END_STRUCT;
	MpAxisLimitLoadType : 	STRUCT 
		Load : REAL := 0.0; (*Value for limiting the accelerating torque [Nm]*)
		Direction : McDirectionEnum := mcDIR_CURRENT; (*Direction of movement *)
	END_STRUCT;
	MpAxisBasicInfoType : 	STRUCT 
		CommunicationReady : BOOL; (*TRUE if MpAxisBasic is ready to communicate*)
		ReadyToPowerOn : BOOL; (*TRUE if MpAxisBasic is ready for operation*)
		Simulation : BOOL; (*TRUE if axis is operated in simulation mode*)
		Jogging : BOOL; (*Performing movement*)
		JogLimitReached : BOOL; (*Indicates that the axis has reached one of the two position limits *)
		LimitLoadActive : BOOL; (*Limit active*)
		PLCopenState : McAxisPLCopenStateEnum; (*PLCopen state*)
		DigitalInputsStatus : McDigitalInputStatusType; (*Status of the digital inputs*)
		Diag : MpAxisDiagExtType; (*Diagnostic structure for the status ID*)
		LibraryInfo : McLibraryInfoType; (*Library info about the specific axis implementation*)
		CommunicationState : McCommunicationStateEnum; (*Communication state*)
	END_STRUCT;
	MpAxisDiagExtType : 	STRUCT 
		StatusID : MpAxisStatusIDType; (*StatusID information*)
		Internal : MpAxisInternalType; (*Internal data*)
		ExecutingCommand : MpAxisExecutingCmdEnum; (*Command information*)
	END_STRUCT;
	MpAxisInternalType : 	STRUCT 
		ID : DINT; (*ID*)
		Severity : MpComSeveritiesEnum; (*Severity*)
		Facility : MpComFacilitiesEnum; (*Facility*)
		Code : UINT; (*UINT*)
	END_STRUCT;
	MpAxisStatusIDType : 	STRUCT 
		ID : MpAxisErrorEnum; (*Error*)
		Severity : MpComSeveritiesEnum; (*Severity*)
		Code : UINT; (*Code*)
	END_STRUCT;
	MpAxisExecutingCmdEnum : 
		(
		mcAXIS_CMD_IDLE := 0,
		mcAXIS_CMD_HOMING,
		mcAXIS_CMD_STOP,
		mcAXIS_CMD_HALT,
		mcAXIS_CMD_MOVE_VELOCITY,
		mcAXIS_CMD_MOVE_ABSOLUTE,
		mcAXIS_CMD_MOVE_ADDITIVE,
		mcAXIS_CMD_JOG_POSITIVE,
		mcAXIS_CMD_JOG_NEGATIVE,
		mcAXIS_CMD_REMOTE_CONTROL,
		mcAXIS_CMD_ERROR_RESET,
		mcAXIS_CMD_GEAR,
		mcAXIS_CMD_CAM,
		mcAXIS_CMD_GEAR_IN_POS,
		mcAXIS_CMD_OFFSET_SHIFT,
		mcAXIS_CMD_PHASE_SHIFT,
		mcAXIS_CMD_GET_CAM_POSITION,
		mcAXIS_CMD_CAM_PREPARE,
		mcAXIS_CMD_CAM_RECOVERY
		);
	MpAxisOffsetParType : 	STRUCT 
		Shift : LREAL; (*Offset shift for slave axis [Measurement Units des Slaves]*)
		Velocity : REAL := 10; (*Maximum velocity [Measurement units/s]*)
		Acceleration : REAL := 50.0; (*Maximum acceleration [Measurement units/s]*)
		Options : McAdvOffsetParType; (*Structure for using optional  functions
Note:
Parameters left at default values disable the associated optional functions.*)
	END_STRUCT;
	MpAxisPhasingParType : 	STRUCT 
		Shift : LREAL; (*Phase shift in the master position of the slave axis [Measurement units].*)
		Velocity : REAL := 10; (*Maximum velocity [Measurement units/s]*)
		Acceleration : REAL := 50.0; (*Maximum acceleration [Measurement units/s]*)
		Options : McAdvPhasingParType; (*Structure for using optional  functions
Note:
Parameters left at default values disable the associated optional functions.*)
	END_STRUCT;
	MpAxisCouplingInfoType : 	STRUCT 
		SlaveReady : BOOL; (*Slave axis ready for operation (Powered + IsHomed( when needed ))*)
		MasterReady : BOOL; (*Master axis ready for operation (CommunicationReady)*)
		Cam : MpAxisCamInfoType; (*Cam status info type*)
		ActualOffsetShift : LREAL; (*Offset shift currently being executed [Measurement units]
*)
		ActualPhaseShift : LREAL; (*Phase shift currently being executed [Measurement units].
*)
		GetCamPosition : MpAxisGetCamPositionInfoType; (*GetCamPosition command result*)
		Recovery : MpAxisRecoveryInfoType; (*Recovery command related information*)
		Diag : MpAxisDiagExtType; (*Diagnostic structure for the status ID*)
	END_STRUCT;
	MpAxisGetCamPositionModeEnum : 
		(
		mcAXIS_GET_CAM_POSITION_SLAVE, (*Get slave postion based on defined cam and dependent on master position*)
		mcAXIS_GET_CAM_POSITION_MASTER, (*Get master postion based on defined cam and dependent on slave position*)
		mcAXIS_MOVE_CAM_POSITION_SLAVE (*Get slave postion based on defined cam and dependent on master position and move absolute to it*)
		);
	MpAxisGetCamPositionParType : 	STRUCT 
		Mode : MpAxisGetCamPositionModeEnum := mcAXIS_GET_CAM_POSITION_SLAVE; (*Mode for GetCamPosition command*)
		Cam : McCamDefineType; (*Cam for getting the position*)
		MasterFactor : DINT := 36000; (*Master streching factor for a cam*)
		SlaveFactor : DINT := 36000; (*Slave streching factor for a cam*)
		MasterPosition : LREAL; (*MasterPosition for which slave position is to be found. Only for mode mpAXIS_GET_CAM_POSITION_SLAVE*)
		SlavePosition : LREAL; (*SlavePosition for which slave position is to be found. Only for mode mpAXIS_GET_CAM_POSITION_MASTER*)
		MasterStartPosition : LREAL; (*Starting position of master from which search of slave matching position will be done. Only for mode mpAXIS_GET_CAM_POSITION_MASTER*)
		Move : MpAxisGetCamPositionMoveParType;
	END_STRUCT;
	MpAxisGetCamPositionMoveParType : 	STRUCT 
		Velocity : REAL := 10.0; (*Velocity for movement to cam slave position[Measurement units/s]*)
		Acceleration : REAL := 50.0; (*Acceleration for movement to cam slave position[Measurement units/s]*)
		Deceleration : REAL := 50.0; (*Deceleration for movement to cam slave position[Measurement units/s]*)
		Jerk : REAL; (*Jerk for movement to cam slave position[Measurement units/s]*)
		Direction : McDirectionEnum := mcDIR_POSITIVE; (*Direction of movement *)
	END_STRUCT;
	MpAxisCouplingParType : 	STRUCT 
		Gear : MpAxisGearParType := (0); (*Gear parameters*)
		Cam : MpAxisCamParType := (0); (*Cam parameters*)
		GearInPos : MpAxisGearInPosParType := (0); (*Gear parameters*)
		Offset : MpAxisOffsetParType; (*Offset parameters*)
		Phasing : MpAxisPhasingParType := (0); (*Phasing parameters*)
		GetCamPosition : MpAxisGetCamPositionParType; (*GetCamPosition parameters*)
		CamList : ARRAY[0..13]OF MpAxisCamListType; (*List of cam to be send to drive. As alternative cam defined in axis cam list feature can be used*)
		Recovery : MpAxisCouplingRecoveryParType;
	END_STRUCT;
	MpAxisGetCamPositionInfoType : 	STRUCT 
		MasterPosition : LREAL; (*GetCamPosition resulting master position*)
		SlavePosition : LREAL; (*GetCamPosition resulting slave position*)
	END_STRUCT;
	MpAxisCamInfoType : 	STRUCT 
		StandBy : BOOL; (*Cam active in background*)
		InLeadIn : BOOL; (*Currently slave axis is in lead in to a cam*)
		InCam : BOOL; (*Slave is following master with defined cam*)
		InLeadOut : BOOL; (*Currently slave axis is in lead out from a cam*)
		EndOfProfile : BOOL; (*Pulse output indicating end of profile*)
		DataInitialized : BOOL; (*Cam data initialized after Update*)
	END_STRUCT;
	MpAxisCamParType : 	STRUCT 
		ID : UINT; (*Id of a cam used for coupling*)
		MasterStartPosition : LREAL; (*Master axis start position*)
		MasterScaling : DINT := 36000; (*Master stretching factor for a cam*)
		SlaveScaling : DINT := 36000; (*Slave stretching factor for a cam*)
		Options : McAdvBrCamInParType; (*Structure for using optional  functions
Note:
Parameters left at default values disable the associated optional functions.*)
		Mode : MpAxisCamStartModeEnum; (*Mode of Cam command execution*)
	END_STRUCT;
	MpAxisGearInPosParType : 	STRUCT 
		RatioNumerator : DINT := 36000; (*Gear ratio numerator*)
		RatioDenominator : DINT := 36000; (*Gear ratio denominator*)
		MasterValueSource : McValueSrcEnum; (*Source of the master-position.*)
		MasterSyncPosition : LREAL; (*Master position at which the axes begin moving in sync 
[measurement units of master]*)
		SlaveSyncPosition : LREAL; (*Slave position at which the axes begin moving in sync 
[measurement units of slave]*)
		SyncMode : McSyncModeEnum; (*Defines the type of synchronization*)
		MasterStartDistance : LREAL; (*The master distance for the slave to start to synchronize to
the master [measurement units of master]*)
		Velocity : REAL := 10.0; (* Maximum velocity during the movement between StartSync and InSync [measurement units of slave/s]*)
		Acceleration : REAL := 50.0; (*Maximum acceleration [Measurement units/s]
Note:
If the value is set to 0.0, the limit value for the slave axis is used.*)
		Deceleration : REAL := 50.0; (*Maximum deceleration [Measurement units/s]
Note:
If the value is set to 0.0, the limit value for the slave axis is used.*)
		Jerk : REAL; (*Maximum jerk [measurement units / s³]*)
		Options : McAdvGearInPosParType; (*Structure for using optional  functions
Note:
Parameters left at default values disable the associated optional functions.*)
	END_STRUCT;
	MpAxisGearParType : 	STRUCT 
		RatioNumerator : DINT := 36000; (*Gear ratio numerator*)
		RatioDenominator : DINT := 36000; (*Gear ratio denominator*)
		MasterValueSource : McValueSrcEnum; (*Source of master position
Note:
Only mcVALUE_SET and mcVALUE_ACTUAL are currently supported.*)
		Acceleration : REAL := 50.0; (*Maximum acceleration [Measurement units/s]
Note:
If the value is set to 0.0, the limit value for the slave axis is used.*)
		Deceleration : REAL := 50.0; (*Maximum deceleration [Measurement units/s]
Note:
If the value is set to 0.0, the limit value for the slave axis is used.*)
		Jerk : REAL; (*Maximum jerk [measurement units / s³]*)
		Options : McAdvGearInParType; (*Structure for using optional  functions
Note:
Parameters left at default values disable the associated optional functions.*)
	END_STRUCT;
	MpAxisCamListType : 	STRUCT 
		Index : UINT;
		Cam : McCamDefineType;
	END_STRUCT;
	MpAxisCamStartModeEnum : 
		(
		mcAXIS_CAM_START_ENTER_CAM, (* Mode for starting Cam from start*)
		mcAXIS_CAM_START_RESTART (* Mode for restarting previously running Cam*)
		);
	MpAxisCouplingRecoveryParType : 	STRUCT 
		Mode : McCamAutPrepRestartModeEnum; (* Mode of repositioning*)
		Velocity : REAL; (* Maximum velocity*)
		Acceleration : REAL; (* Maximum acceleration*)
		Deceleration : REAL; (* Maximum deceleration*)
		Jerk : REAL; (* Maximum jerk*)
		Options : McAdvCamAutPrepRestartParType; (* Recovery optional parameters*)
	END_STRUCT;
	MpAxisRecoveryInfoType : 	STRUCT 
		RestartPosition : LREAL; (* Resulting restart position*)
	END_STRUCT;
END_TYPE
