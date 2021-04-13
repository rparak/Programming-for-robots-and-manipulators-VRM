TYPE
	McABTEnum :
		( (*Base type selector setting*)
		mcABT_LIN_BD := 0, (*Linear bounded - Linear axis with bounded movement scope*)
		mcABT_LIN := 1, (*Linear - Linear axis*)
		mcABT_LIN_PER := 2, (*Linear periodic - Linear axis with periodic movement scope*)
		mcABT_ROT_BD := 10, (*Rotary bounded - Rotary axis with bounded movement scope*)
		mcABT_ROT := 11, (*Rotary - Rotary axis*)
		mcABT_ROT_PER := 12 (*Rotary periodic - Rotary axis with periodic movement scope*)
		);
	McABTLinBdType : STRUCT (*Type mcABT_LIN_BD settings*)
		MeasurementUnit : McCfgLocLenUnitEnum; (*Measurement unit for the axis*)
		MeasurementResolution : LREAL; (*Possible resolution of measurement unit that can be achieved [Measurement units]*)
		CountDirection : McCfgCntDirEnum; (*Direction of the axis in which the position value is increasing*)
	END_STRUCT;
	McABTLinType : STRUCT (*Type mcABT_LIN settings*)
		MeasurementUnit : McCfgLocLenUnitEnum; (*Measurement unit for the axis*)
		MeasurementResolution : LREAL; (*Possible resolution of measurement unit that can be achieved [Measurement units]*)
		CountDirection : McCfgCntDirEnum; (*Direction of the axis in which the position value is increasing*)
	END_STRUCT;
	McABTLinPerPerSetType : STRUCT (*Possible position value range of a periodic axis*)
		Period : LREAL; (*The value range for axis positions is [0 , Period[ [Measurement units]*)
	END_STRUCT;
	McABTLinPerType : STRUCT (*Type mcABT_LIN_PER settings*)
		MeasurementUnit : McCfgLocLenUnitEnum; (*Measurement unit for the axis*)
		MeasurementResolution : LREAL; (*Possible resolution of measurement unit that can be achieved [Measurement units]*)
		CountDirection : McCfgCntDirEnum; (*Direction of the axis in which the position value is increasing*)
		PeriodSettings : McABTLinPerPerSetType; (*Possible position value range of a periodic axis*)
	END_STRUCT;
	McABTRotBdType : STRUCT (*Type mcABT_ROT_BD settings*)
		MeasurementUnit : McCfgLocRotUnitEnum; (*Measurement unit for the axis*)
		MeasurementResolution : LREAL; (*Possible resolution of measurement unit that can be achieved [Measurement units]*)
		CountDirection : McCfgCntDirEnum; (*Direction of the axis in which the position value is increasing*)
	END_STRUCT;
	McABTRotType : STRUCT (*Type mcABT_ROT settings*)
		MeasurementUnit : McCfgLocRotUnitEnum; (*Measurement unit for the axis*)
		MeasurementResolution : LREAL; (*Possible resolution of measurement unit that can be achieved [Measurement units]*)
		CountDirection : McCfgCntDirEnum; (*Direction of the axis in which the position value is increasing*)
	END_STRUCT;
	McABTRotPerPerSetType : STRUCT (*Possible position value range of a periodic axis*)
		Period : LREAL; (*The value range for axis positions is [0 , Period[ [Measurement units]*)
	END_STRUCT;
	McABTRotPerType : STRUCT (*Type mcABT_ROT_PER settings*)
		MeasurementUnit : McCfgLocRotUnitEnum; (*Measurement unit for the axis*)
		MeasurementResolution : LREAL; (*Possible resolution of measurement unit that can be achieved [Measurement units]*)
		CountDirection : McCfgCntDirEnum; (*Direction of the axis in which the position value is increasing*)
		PeriodSettings : McABTRotPerPerSetType; (*Possible position value range of a periodic axis*)
	END_STRUCT;
	McABTType : STRUCT (*Defines the basic movement possibilities of the axis*)
		Type : McABTEnum; (*Base type selector setting*)
		LinearBounded : McABTLinBdType; (*Type mcABT_LIN_BD settings*)
		Linear : McABTLinType; (*Type mcABT_LIN settings*)
		LinearPeriodic : McABTLinPerType; (*Type mcABT_LIN_PER settings*)
		RotaryBounded : McABTRotBdType; (*Type mcABT_ROT_BD settings*)
		Rotary : McABTRotType; (*Type mcABT_ROT settings*)
		RotaryPeriodic : McABTRotPerType; (*Type mcABT_ROT_PER settings*)
	END_STRUCT;
	McAMLEnum :
		( (*Movement limits selector setting*)
		mcAML_INT := 0, (*Internal - Internal definition of limits*)
		mcAML_INT_PATH_AX := 1, (*Internal path axis - Internal definition of limits with additional settings for a path controlled axis*)
		mcAML_EXT := 2 (*External - External definition of limits*)
		);
	McAMoveLimVelEnum :
		( (*Velocity selector setting*)
		mcAMLV_BASIC := 0, (*Basic -*)
		mcAMLV_ADV := 1 (*Advanced -*)
		);
	McAMoveLimVelType : STRUCT (*Limits for the velocity of the axis*)
		Type : McAMoveLimVelEnum; (*Velocity selector setting*)
		Basic : McCfgLimVelBaseType; (*Type mcAMLV_BASIC settings*)
		Advanced : McCfgLimVelAdvType; (*Type mcAMLV_ADV settings*)
	END_STRUCT;
	McAMoveLimAccEnum :
		( (*Acceleration selector setting*)
		mcAMLA_BASIC := 0, (*Basic -*)
		mcAMLA_ADV := 1 (*Advanced -*)
		);
	McAMoveLimAccType : STRUCT (*Limits for the acceleration of the axis*)
		Type : McAMoveLimAccEnum; (*Acceleration selector setting*)
		Basic : McCfgLimAccBaseType; (*Type mcAMLA_BASIC settings*)
		Advanced : McCfgLimAccAdvType; (*Type mcAMLA_ADV settings*)
	END_STRUCT;
	McAMoveLimDecEnum :
		( (*Deceleration selector setting*)
		mcAMLD_BASIC := 0, (*Basic -*)
		mcAMLD_ADV := 1 (*Advanced -*)
		);
	McAMoveLimDecType : STRUCT (*Limits for the deceleration of the axis*)
		Type : McAMoveLimDecEnum; (*Deceleration selector setting*)
		Basic : McCfgLimDecBaseType; (*Type mcAMLD_BASIC settings*)
		Advanced : McCfgLimDecAdvType; (*Type mcAMLD_ADV settings*)
	END_STRUCT;
	McAMLIntType : STRUCT (*Type mcAML_INT settings*)
		Position : McCfgLimPosType; (*Movement range of the axis via two position boundaries*)
		Velocity : McAMoveLimVelType; (*Limits for the velocity of the axis*)
		Acceleration : McAMoveLimAccType; (*Limits for the acceleration of the axis*)
		Deceleration : McAMoveLimDecType; (*Limits for the deceleration of the axis*)
	END_STRUCT;
	McAMLIntPathAxType : STRUCT (*Type mcAML_INT_PATH_AX settings*)
		Position : McCfgLimPosType; (*Movement range of the axis via two position boundaries*)
		Velocity : McAMoveLimVelType; (*Limits for the velocity of the axis*)
		Acceleration : McAMoveLimAccType; (*Limits for the acceleration of the axis*)
		Deceleration : McAMoveLimDecType; (*Limits for the deceleration of the axis*)
		Jerk : McCfgLimJerkType; (*Jerk limits*)
		Torque : McCfgLimTorqType; (*Torque limits*)
		Force : McCfgLimForType; (*Limits for the force of the axis*)
	END_STRUCT;
	McAMLType : STRUCT (*Various limit values that will be considered for axis movements*)
		Type : McAMLEnum; (*Movement limits selector setting*)
		Internal : McAMLIntType; (*Type mcAML_INT settings*)
		InternalPathAxis : McAMLIntPathAxType; (*Type mcAML_INT_PATH_AX settings*)
		External : McCfgExtLimRefType; (*Type mcAML_EXT settings*)
	END_STRUCT;
	McCfgAxType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_AX*)
		BaseType : McABTType; (*Defines the basic movement possibilities of the axis*)
		MovementLimits : McAMLType; (*Various limit values that will be considered for axis movements*)
	END_STRUCT;
	McCfgAxBaseTypType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_AX_BASE_TYP*)
		BaseType : McABTType; (*Defines the basic movement possibilities of the axis*)
	END_STRUCT;
	McCfgAxMoveLimType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_AX_MOVE_LIM*)
		MovementLimits : McAMLType; (*Various limit values that will be considered for axis movements*)
	END_STRUCT;
	McAFDCSTypEnum :
		( (*Digital cam switch type selector setting*)
		mcAFDCST_ACP := 0 (*ACOPOS - ACOPOS specific part is used*)
		);
	McAFDCSTypAcpValSrcRTOAEnum :
		( (*Reference to other axis selector setting*)
		mcAFDCSTAVSRTOA_NOT_USE := 0, (*Not used -*)
		mcAFDCSTAVSRTOA_USE := 1 (*Used -*)
		);
	McAFDCSTypAcpValSrcRTOAUseType : STRUCT (*Type mcAFDCSTAVSRTOA_USE settings*)
		AxisReference : McCfgReferenceType; (*Name of the referenced axis component*)
	END_STRUCT;
	McAFDCSTypAcpValSrcRTOAType : STRUCT (*Another axis then the output axis is used as value source*)
		Type : McAFDCSTypAcpValSrcRTOAEnum; (*Reference to other axis selector setting*)
		Used : McAFDCSTypAcpValSrcRTOAUseType; (*Type mcAFDCSTAVSRTOA_USE settings*)
	END_STRUCT;
	McAFDCSTypAcpValSrcParIDEnum :
		( (*ParID selector setting*)
		mcAFDCSTAVSP_NOT_USE := 1, (*Not used - Not used*)
		mcAFDCSTAVSP_USE := 2 (*Used - ParID used*)
		);
	McAFDCSTypAcpValSrcParIDUseType : STRUCT (*Type mcAFDCSTAVSP_USE settings*)
		ParID : UINT; (*ParID which is used as value source*)
	END_STRUCT;
	McAFDCSTypAcpValSrcParIDType : STRUCT (*ParID used as Value source*)
		Type : McAFDCSTypAcpValSrcParIDEnum; (*ParID selector setting*)
		Used : McAFDCSTypAcpValSrcParIDUseType; (*Type mcAFDCSTAVSP_USE settings*)
	END_STRUCT;
	McAFDCSTypAcpValSrcType : STRUCT (*Value source for generating the cam switches*)
		ReferenceToOtherAxis : McAFDCSTypAcpValSrcRTOAType; (*Another axis then the output axis is used as value source*)
		ParID : McAFDCSTypAcpValSrcParIDType; (*ParID used as Value source*)
	END_STRUCT;
	McAFDCSTypAcpOutEnum :
		( (*Output selector setting*)
		mcAFDCSTAO_ACP_HW := 0, (*ACOPOS hardware - ACOPOS hardware is used*)
		mcAFDCSTAO_ACP_PARID := 1 (*ACOPOS ParID - A destination ParID on an ACOPOS axis is used*)
		);
	McAFDCSTypAcpOutAcpHwProdFamEnum :
		( (*ACOPOS product family selector setting*)
		mcAFDCSTAOAHPF_ACP := 0, (*ACOPOS -*)
		mcAFDCSTAOAHPF_ACPM := 1, (*ACOPOSmulti -*)
		mcAFDCSTAOAHPF_ACP_P3 := 2 (*ACOPOS P3 -*)
		);
	McAFDCSACOPOSDigOutEnum :
		( (*Digital output selector setting*)
		mcAFDCSACOPOSDO_SS2X111 := 0, (*SS2.X11.1 -*)
		mcAFDCSACOPOSDO_SS2X112 := 1, (*SS2.X11.2 -*)
		mcAFDCSACOPOSDO_SS2X113 := 2, (*SS2.X11.3 -*)
		mcAFDCSACOPOSDO_SS2X114 := 3, (*SS2.X11.4 -*)
		mcAFDCSACOPOSDO_SS2X115 := 4, (*SS2.X11.5 -*)
		mcAFDCSACOPOSDO_SS2X116 := 5, (*SS2.X11.6 -*)
		mcAFDCSACOPOSDO_SS2X117 := 6, (*SS2.X11.7 -*)
		mcAFDCSACOPOSDO_SS2X118 := 7, (*SS2.X11.8 -*)
		mcAFDCSACOPOSDO_SS2X119 := 8, (*SS2.X11.9 -*)
		mcAFDCSACOPOSDO_SS2X1110 := 9, (*SS2.X11.10 -*)
		mcAFDCSACOPOSDO_SS3X111 := 10, (*SS3.X11.1 -*)
		mcAFDCSACOPOSDO_SS3X112 := 11, (*SS3.X11.2 -*)
		mcAFDCSACOPOSDO_SS3X113 := 12, (*SS3.X11.3 -*)
		mcAFDCSACOPOSDO_SS3X114 := 13, (*SS3.X11.4 -*)
		mcAFDCSACOPOSDO_SS3X115 := 14, (*SS3.X11.5 -*)
		mcAFDCSACOPOSDO_SS3X116 := 15, (*SS3.X11.6 -*)
		mcAFDCSACOPOSDO_SS3X117 := 16, (*SS3.X11.7 -*)
		mcAFDCSACOPOSDO_SS3X118 := 17, (*SS3.X11.8 -*)
		mcAFDCSACOPOSDO_SS3X119 := 18, (*SS3.X11.9 -*)
		mcAFDCSACOPOSDO_SS3X1110 := 19, (*SS3.X11.10 -*)
		mcAFDCSACOPOSDO_SS4X111 := 20, (*SS4.X11.1 -*)
		mcAFDCSACOPOSDO_SS4X112 := 21, (*SS4.X11.2 -*)
		mcAFDCSACOPOSDO_SS4X113 := 22, (*SS4.X11.3 -*)
		mcAFDCSACOPOSDO_SS4X114 := 23, (*SS4.X11.4 -*)
		mcAFDCSACOPOSDO_SS4X115 := 24, (*SS4.X11.5 -*)
		mcAFDCSACOPOSDO_SS4X116 := 25, (*SS4.X11.6 -*)
		mcAFDCSACOPOSDO_SS4X117 := 26, (*SS4.X11.7 -*)
		mcAFDCSACOPOSDO_SS4X118 := 27, (*SS4.X11.8 -*)
		mcAFDCSACOPOSDO_SS4X119 := 28, (*SS4.X11.9 -*)
		mcAFDCSACOPOSDO_SS4X1110 := 29 (*SS4.X11.10 -*)
		);
	McAFDCSACOPOSDigOutType : STRUCT
		Type : McAFDCSACOPOSDigOutEnum; (*Digital output selector setting*)
	END_STRUCT;
	McAFDCSACOPOSType : STRUCT (*Type mcAFDCSTAOAHPF_ACP settings*)
		DigitalOutput : McAFDCSACOPOSDigOutType;
	END_STRUCT;
	McAFDCSACOPOSmultiDigOutEnum :
		( (*Digital output selector setting*)
		mcAFDCSACOPOSMULTIDO_SS1X111 := 0, (*SS1.X11.1 -*)
		mcAFDCSACOPOSMULTIDO_SS1X113 := 1, (*SS1.X11.3 -*)
		mcAFDCSACOPOSMULTIDO_SS1X115 := 2, (*SS1.X11.5 -*)
		mcAFDCSACOPOSMULTIDO_SS1X116 := 3 (*SS1.X11.6 -*)
		);
	McAFDCSACOPOSmultiDigOutType : STRUCT
		Type : McAFDCSACOPOSmultiDigOutEnum; (*Digital output selector setting*)
	END_STRUCT;
	McAFDCSACOPOSmultiType : STRUCT (*Type mcAFDCSTAOAHPF_ACPM settings*)
		DigitalOutput : McAFDCSACOPOSmultiDigOutType;
	END_STRUCT;
	McAFDCSACOPOSP3DigOutEnum :
		( (*Digital output selector setting*)
		mcAFDCSACOPOSP3DO_SS1X41E1 := 0, (*SS1.X41E.1 -*)
		mcAFDCSACOPOSP3DO_SS1X41E2 := 1, (*SS1.X41E.2 -*)
		mcAFDCSACOPOSP3DO_SS1X41E3 := 2, (*SS1.X41E.3 -*)
		mcAFDCSACOPOSP3DO_SS1X41E4 := 3, (*SS1.X41E.4 -*)
		mcAFDCSACOPOSP3DO_SS1X41E5 := 4, (*SS1.X41E.5 -*)
		mcAFDCSACOPOSP3DO_SS1X41E6 := 5, (*SS1.X41E.6 -*)
		mcAFDCSACOPOSP3DO_SS1X41E7 := 6, (*SS1.X41E.7 -*)
		mcAFDCSACOPOSP3DO_SS1X41E8 := 7, (*SS1.X41E.8 -*)
		mcAFDCSACOPOSP3DO_SS1X41E9 := 8, (*SS1.X41E.9 -*)
		mcAFDCSACOPOSP3DO_SS1X41E10 := 9 (*SS1.X41E.10 -*)
		);
	McAFDCSACOPOSP3DigOutType : STRUCT
		Type : McAFDCSACOPOSP3DigOutEnum; (*Digital output selector setting*)
	END_STRUCT;
	McAFDCSACOPOSP3Type : STRUCT (*Type mcAFDCSTAOAHPF_ACP_P3 settings*)
		DigitalOutput : McAFDCSACOPOSP3DigOutType;
	END_STRUCT;
	McAFDCSTypAcpOutAcpHwProdFamType : STRUCT
		Type : McAFDCSTypAcpOutAcpHwProdFamEnum; (*ACOPOS product family selector setting*)
		ACOPOS : McAFDCSACOPOSType; (*Type mcAFDCSTAOAHPF_ACP settings*)
		ACOPOSmulti : McAFDCSACOPOSmultiType; (*Type mcAFDCSTAOAHPF_ACPM settings*)
		ACOPOSP3 : McAFDCSACOPOSP3Type; (*Type mcAFDCSTAOAHPF_ACP_P3 settings*)
	END_STRUCT;
	McAFDCSTypAcpOutAcpHwType : STRUCT (*Type mcAFDCSTAO_ACP_HW settings*)
		ProductFamily : McAFDCSTypAcpOutAcpHwProdFamType;
	END_STRUCT;
	McAFDCSTypAcpOutAcpParIDType : STRUCT (*Type mcAFDCSTAO_ACP_PARID settings*)
		ParID : UINT;
	END_STRUCT;
	McAFDCSTypAcpOutType : STRUCT (*Target of the output value*)
		Type : McAFDCSTypAcpOutEnum; (*Output selector setting*)
		ACOPOSHardware : McAFDCSTypAcpOutAcpHwType; (*Type mcAFDCSTAO_ACP_HW settings*)
		ACOPOSParID : McAFDCSTypAcpOutAcpParIDType; (*Type mcAFDCSTAO_ACP_PARID settings*)
	END_STRUCT;
	McAFDCSTypAcpType : STRUCT (*Type mcAFDCST_ACP settings*)
		ValueSource : McAFDCSTypAcpValSrcType; (*Value source for generating the cam switches*)
		Output : McAFDCSTypAcpOutType; (*Target of the output value*)
	END_STRUCT;
	McAFDCSTypType : STRUCT (*Type of the digital output configuration*)
		Type : McAFDCSTypEnum; (*Digital cam switch type selector setting*)
		ACOPOS : McAFDCSTypAcpType; (*Type mcAFDCST_ACP settings*)
	END_STRUCT;
	McCfgAxFeatDigCamSwType : STRUCT (*Main data type corresponding to McCfgTypeEnum mcCFG_AX_FEAT_DIG_CAM_SW*)
		DigitalCamSwitchType : McAFDCSTypType; (*Type of the digital output configuration*)
	END_STRUCT;
END_TYPE