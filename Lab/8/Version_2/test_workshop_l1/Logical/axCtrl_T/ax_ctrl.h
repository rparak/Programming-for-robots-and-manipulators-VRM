/**
 * MIT License

 * Copyright(c) 2020 Roman Parak
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Author   : Roman Parak
 * Email    : Roman.Parak@outlook.com
 * Date     : 12.12.2020
 * File Name: axCtrl_T.h
 */

/** < Include B&R Automation libraries (declarations for B&R ANSI C extensions) */
#include <bur/plctypes.h>

#ifdef _DEFAULT_INCLUDES
	#include <AsDefault.h>
#endif

/**
 * Main states for simple control of a single axis.
 */
typedef enum motion_ctrl_state_enum{
	LIN_AX_CTRL_ACTIVE, /**< The Function block is active. */
	LIN_AX_CTRL_POWER,  /**< Axis is power on. */
	LIN_AX_CTRL_HOME,	/**< Axis is homed. */
	LIN_AX_CTRL_WAIT,   /**< Wait for the command. */
	LIN_AX_CTRL_MOVE_1, /**< Move Axis (1) -> Set Parameters. */
	LIN_AX_CTRL_MOVE_2, /**< Move Axis (2) -> Wait until the axis is in specified position. */
	LIN_AX_CTRL_ERROR	/**< The function block is in an error state, wait for reset. */
}motion_ctrl_state_enum;

/**
 * Command execution structure for control of a single axis.
 */
typedef struct motion_ctrl_command{
	BOOL power;                 /**< Power on. */
	BOOL home;                  /**< Home. */
	BOOL move_absolute_postion; /**< Move to the absolute position. */
	BOOL reset_error;           /**< Reset error. */
}motion_ctrl_command;

/**
 * Structure for reading some axis parameters.
 */
typedef struct motion_ctrl_read{
	LREAL actual_position; /**< Actual moving position in units. (Controlled Value) */
	LREAL set_position;    /**< Acutal set position in units. (Desired Value) */
	REAL actual_velocity;  /**< Actual moving velocity in units/s.(Controlled Value) */
	REAL set_velocity;	   /**< Actual set velocity in units/s. (Desired Value) */
	BOOL power_on;         /**< Axis is power on. */
	BOOL is_homed;		   /**< Axis is homed. */
	BOOL is_active;        /**< The Function block is active. */
	BOOL error;	           /**< The function block is in an error state. */
}motion_ctrl_read;

/**
 * Structure for controlling a 3d model of a digital twin.
 */
typedef struct motion_ctrl_digitalTwin{
	LREAL position; /**< Position in millimeters. */
}motion_ctrl_digitalTwin;

/**
 * Structure for writing some axis parameters.
 */
typedef struct motion_ctrl_write{
	LREAL position;		/**< Position in units. */
	REAL velocity;		/**< Velocity in units/s. */
	REAL acceleration;	/**< Acceleration in units/s^2. */
	REAL deceleration;	/**< Deceleration in units/s^2. */
	motion_ctrl_digitalTwin dT;
}motion_ctrl_write;

/**
 * Main structure for axis control and data collection.
 */
typedef struct motion_ctrl_main{
	motion_ctrl_command cmd;
	motion_ctrl_read read;
	motion_ctrl_write write;
	motion_ctrl_state_enum actual_state;
}motion_ctrl_main;