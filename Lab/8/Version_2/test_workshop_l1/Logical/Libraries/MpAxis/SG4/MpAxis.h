/* Automation Studio generated header file */
/* Do not edit ! */
/* MpAxis 5.12.2 */

#ifndef _MPAXIS_
#define _MPAXIS_
#ifdef __cplusplus
extern "C" 
{
#endif
#ifndef _MpAxis_VERSION
#define _MpAxis_VERSION 5.12.2
#endif

#include <bur/plctypes.h>

#ifndef _BUR_PUBLIC
#define _BUR_PUBLIC
#endif

#ifdef _SG4
#include <McAxis.h> 
#include <MpBase.h>
#endif
 
#ifdef _SG3
#include <McAxis.h> 
#include <MpBase.h>
#endif
 
#ifdef _SGC
#include <McAxis.h> 
#include <MpBase.h>
#endif

/* Datatypes and datatypes of function blocks */
typedef enum MpAxisErrorEnum
{	mcAXIS_NO_ERROR = 0,
	mcAXIS_ERR_PLC_OPEN = -1067278080,
	mcAXIS_WRN_PLC_OPEN = -2141019903,
	mcAXIS_WRN_MULTIPLE_COMMAND = -2141019902,
	mcAXIS_ERR_ACTIVATION = -1064239103,
	mcAXIS_ERR_MPLINK_NULL = -1064239102,
	mcAXIS_ERR_MPLINK_INVALID = -1064239101,
	mcAXIS_ERR_MPLINK_CHANGED = -1064239100,
	mcAXIS_ERR_MPLINK_CORRUPT = -1064239099,
	mcAXIS_ERR_MPLINK_IN_USE = -1064239098,
	mcAXIS_ERR_PAR_NULL = -1064239097
} MpAxisErrorEnum;

typedef enum MpAxisExecutingCmdEnum
{	mcAXIS_CMD_IDLE = 0,
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
} MpAxisExecutingCmdEnum;

typedef enum MpAxisGetCamPositionModeEnum
{	mcAXIS_GET_CAM_POSITION_SLAVE,
	mcAXIS_GET_CAM_POSITION_MASTER,
	mcAXIS_MOVE_CAM_POSITION_SLAVE
} MpAxisGetCamPositionModeEnum;

typedef enum MpAxisCamStartModeEnum
{	mcAXIS_CAM_START_ENTER_CAM,
	mcAXIS_CAM_START_RESTART
} MpAxisCamStartModeEnum;

typedef struct MpAxisHomingType
{	enum McHomingModeEnum Mode;
	double Position;
} MpAxisHomingType;

typedef struct MpAxisJogLimitPositionType
{	double FirstPosition;
	double LastPosition;
} MpAxisJogLimitPositionType;

typedef struct MpAxisJogType
{	float Velocity;
	float Acceleration;
	float Deceleration;
	struct MpAxisJogLimitPositionType LimitPosition;
} MpAxisJogType;

typedef struct MpAxisStopType
{	float Deceleration;
} MpAxisStopType;

typedef struct MpAxisLimitLoadType
{	float Load;
	enum McDirectionEnum Direction;
} MpAxisLimitLoadType;

typedef struct MpAxisBasicParType
{	double Position;
	double Distance;
	float Velocity;
	float Acceleration;
	float Deceleration;
	enum McDirectionEnum Direction;
	struct MpAxisHomingType Homing;
	struct MpAxisJogType Jog;
	struct MpAxisStopType Stop;
	struct MpAxisLimitLoadType LimitLoad;
} MpAxisBasicParType;

typedef struct MpAxisStatusIDType
{	enum MpAxisErrorEnum ID;
	enum MpComSeveritiesEnum Severity;
	unsigned short Code;
} MpAxisStatusIDType;

typedef struct MpAxisInternalType
{	signed long ID;
	enum MpComSeveritiesEnum Severity;
	enum MpComFacilitiesEnum Facility;
	unsigned short Code;
} MpAxisInternalType;

typedef struct MpAxisDiagExtType
{	struct MpAxisStatusIDType StatusID;
	struct MpAxisInternalType Internal;
	enum MpAxisExecutingCmdEnum ExecutingCommand;
} MpAxisDiagExtType;

typedef struct MpAxisBasicInfoType
{	plcbit CommunicationReady;
	plcbit ReadyToPowerOn;
	plcbit Simulation;
	plcbit Jogging;
	plcbit JogLimitReached;
	plcbit LimitLoadActive;
	enum McAxisPLCopenStateEnum PLCopenState;
	struct McDigitalInputStatusType DigitalInputsStatus;
	struct MpAxisDiagExtType Diag;
	struct McLibraryInfoType LibraryInfo;
	enum McCommunicationStateEnum CommunicationState;
} MpAxisBasicInfoType;

typedef struct MpAxisOffsetParType
{	double Shift;
	float Velocity;
	float Acceleration;
	struct McAdvOffsetParType Options;
} MpAxisOffsetParType;

typedef struct MpAxisPhasingParType
{	double Shift;
	float Velocity;
	float Acceleration;
	struct McAdvPhasingParType Options;
} MpAxisPhasingParType;

typedef struct MpAxisCamInfoType
{	plcbit StandBy;
	plcbit InLeadIn;
	plcbit InCam;
	plcbit InLeadOut;
	plcbit EndOfProfile;
	plcbit DataInitialized;
} MpAxisCamInfoType;

typedef struct MpAxisGetCamPositionInfoType
{	double MasterPosition;
	double SlavePosition;
} MpAxisGetCamPositionInfoType;

typedef struct MpAxisRecoveryInfoType
{	double RestartPosition;
} MpAxisRecoveryInfoType;

typedef struct MpAxisCouplingInfoType
{	plcbit SlaveReady;
	plcbit MasterReady;
	struct MpAxisCamInfoType Cam;
	double ActualOffsetShift;
	double ActualPhaseShift;
	struct MpAxisGetCamPositionInfoType GetCamPosition;
	struct MpAxisRecoveryInfoType Recovery;
	struct MpAxisDiagExtType Diag;
} MpAxisCouplingInfoType;

typedef struct MpAxisGetCamPositionMoveParType
{	float Velocity;
	float Acceleration;
	float Deceleration;
	float Jerk;
	enum McDirectionEnum Direction;
} MpAxisGetCamPositionMoveParType;

typedef struct MpAxisGetCamPositionParType
{	enum MpAxisGetCamPositionModeEnum Mode;
	struct McCamDefineType Cam;
	signed long MasterFactor;
	signed long SlaveFactor;
	double MasterPosition;
	double SlavePosition;
	double MasterStartPosition;
	struct MpAxisGetCamPositionMoveParType Move;
} MpAxisGetCamPositionParType;

typedef struct MpAxisGearParType
{	signed long RatioNumerator;
	signed long RatioDenominator;
	enum McValueSrcEnum MasterValueSource;
	float Acceleration;
	float Deceleration;
	float Jerk;
	struct McAdvGearInParType Options;
} MpAxisGearParType;

typedef struct MpAxisCamParType
{	unsigned short ID;
	double MasterStartPosition;
	signed long MasterScaling;
	signed long SlaveScaling;
	struct McAdvBrCamInParType Options;
	enum MpAxisCamStartModeEnum Mode;
} MpAxisCamParType;

typedef struct MpAxisGearInPosParType
{	signed long RatioNumerator;
	signed long RatioDenominator;
	enum McValueSrcEnum MasterValueSource;
	double MasterSyncPosition;
	double SlaveSyncPosition;
	enum McSyncModeEnum SyncMode;
	double MasterStartDistance;
	float Velocity;
	float Acceleration;
	float Deceleration;
	float Jerk;
	struct McAdvGearInPosParType Options;
} MpAxisGearInPosParType;

typedef struct MpAxisCamListType
{	unsigned short Index;
	struct McCamDefineType Cam;
} MpAxisCamListType;

typedef struct MpAxisCouplingRecoveryParType
{	enum McCamAutPrepRestartModeEnum Mode;
	float Velocity;
	float Acceleration;
	float Deceleration;
	float Jerk;
	struct McAdvCamAutPrepRestartParType Options;
} MpAxisCouplingRecoveryParType;

typedef struct MpAxisCouplingParType
{	struct MpAxisGearParType Gear;
	struct MpAxisCamParType Cam;
	struct MpAxisGearInPosParType GearInPos;
	struct MpAxisOffsetParType Offset;
	struct MpAxisPhasingParType Phasing;
	struct MpAxisGetCamPositionParType GetCamPosition;
	struct MpAxisCamListType CamList[14];
	struct MpAxisCouplingRecoveryParType Recovery;
} MpAxisCouplingParType;

typedef struct MpAxisBasic
{
	/* VAR_INPUT (analog) */
	struct McAxisType* MpLink;
	struct MpAxisBasicParType* Parameters;
	/* VAR_OUTPUT (analog) */
	signed long StatusID;
	double Position;
	float Velocity;
	struct MpAxisBasicInfoType Info;
	/* VAR (analog) */
	struct MpComInternalDataType Internal;
	/* VAR_INPUT (digital) */
	plcbit Enable;
	plcbit ErrorReset;
	plcbit Update;
	plcbit Power;
	plcbit Home;
	plcbit MoveVelocity;
	plcbit MoveAbsolute;
	plcbit MoveAdditive;
	plcbit Stop;
	plcbit JogPositive;
	plcbit JogNegative;
	plcbit LimitLoad;
	plcbit ReleaseBrake;
	/* VAR_OUTPUT (digital) */
	plcbit Active;
	plcbit Error;
	plcbit UpdateDone;
	plcbit CommandBusy;
	plcbit CommandAborted;
	plcbit PowerOn;
	plcbit IsHomed;
	plcbit InVelocity;
	plcbit InPosition;
	plcbit MoveActive;
	plcbit MoveDone;
	plcbit Stopped;
	plcbit LimitLoadReady;
	plcbit BrakeReleased;
} MpAxisBasic_typ;

typedef struct MpAxisCoupling
{
	/* VAR_INPUT (analog) */
	struct McAxisType* MpLink;
	struct MpAxisCouplingParType* Parameters;
	struct McAxisType* MpLinkMaster;
	/* VAR_OUTPUT (analog) */
	signed long StatusID;
	struct MpAxisCouplingInfoType Info;
	/* VAR (analog) */
	struct MpComInternalDataType Internal;
	/* VAR_INPUT (digital) */
	plcbit Enable;
	plcbit ErrorReset;
	plcbit Update;
	plcbit Gear;
	plcbit Cam;
	plcbit GearInPos;
	plcbit OffsetShift;
	plcbit PhaseShift;
	plcbit GetCamPosition;
	plcbit CamPrepare;
	plcbit Recovery;
	/* VAR_OUTPUT (digital) */
	plcbit Active;
	plcbit Error;
	plcbit UpdateDone;
	plcbit CommandBusy;
	plcbit CommandAborted;
	plcbit InCompensation;
	plcbit InSync;
	plcbit OffsetDone;
	plcbit PhasingDone;
	plcbit RecoveryDone;
	plcbit GetCamPositionDone;
	plcbit CamPrepareDone;
} MpAxisCoupling_typ;



/* Prototyping of functions and function blocks */
_BUR_PUBLIC void MpAxisBasic(struct MpAxisBasic* inst);
_BUR_PUBLIC void MpAxisCoupling(struct MpAxisCoupling* inst);


#ifdef __cplusplus
};
#endif
#endif /* _MPAXIS_ */

