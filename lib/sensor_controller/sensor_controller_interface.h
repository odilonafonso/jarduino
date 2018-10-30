/*
 * sensor_controller_interface.h
 *
 *  Created on: 30 de out de 2018
 *      Author: odilon
 */

#ifndef LIB_SENSOR_CONTROLLER_SENSOR_CONTROLLER_INTERFACE_H_
#define LIB_SENSOR_CONTROLLER_SENSOR_CONTROLLER_INTERFACE_H_

#include <stdint.h>
#include "jarduino_error.h"

typedef enum sensor_ctrl_op
{
	SENSOR_CTRL_OP_INIT,
	SENSOR_CTRL_OP_READ,
	SENSOR_CTRL_OP_CONFIG
}sensor_ctrl_op_t;

typedef enum sensor_ctrl_op_type
{
	SENSOR_CTRL_OP_TYPE_REQUEST,
	SENSOR_CTRL_OP_TYPE_RESPONSE,
}sensor_ctrl_op_type_t;

typedef struct sensor_ctrl_read_rsp
{
	uint16_t temperature;
	uint16_t umidity;
	uint16_t luminosity;
	uint16_t soil;
}sensor_ctrl_read_rsp_t;

typedef struct sensor_ctrl_cmd_req
{
	union
	{

	};
}sensor_ctrl_cmd_req_t;

typedef struct sensor_ctrl_cmd_rsp
{
	union
	{
		sensor_ctrl_read_rsp_t read;
	};
}sensor_ctrl_cmd_rsp_t;

typedef struct sensor_ctrl_cmd
{
	sensor_ctrl_op_t operation;
	sensor_ctrl_op_type_t type;
	union
	{
		sensor_ctrl_cmd_req_t req;
		sensor_ctrl_cmd_rsp_t rsp;
	};
}sensor_ctrl_cmd_t;

#endif /* LIB_SENSOR_CONTROLLER_SENSOR_CONTROLLER_INTERFACE_H_ */
