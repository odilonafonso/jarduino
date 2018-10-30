/*
 * sensor_controller.c
 *
 *  Created on: 30 de out de 2018
 *      Author: odilon
 */

#include <stddef.h>
#include "sensor_controller.h"

static jarduino_error_t sensor_ctrl_init(sensor_ctrl_cmd_t * cmd);
static jarduino_error_t sensor_ctrl_read(sensor_ctrl_cmd_t * cmd);
static jarduino_error_t sensor_ctrl_config(sensor_ctrl_cmd_t * cmd);

jarduino_error_t sensor_ctrl_cmd(sensor_ctrl_cmd_t * cmd)
{
	if(cmd == NULL)
	{
		return JARDUINO_ERROR_NULL_POINTER;
	}

	jarduino_error_t err_code = JARDUINO_SUCCESS;

	switch(cmd-> operation)
	{
	case SENSOR_CTRL_OP_CONFIG:
		err_code = sensor_ctrl_config(cmd);
		break;

	case SENSOR_CTRL_OP_INIT:
		err_code = sensor_ctrl_init(cmd);
		break;

	case SENSOR_CTRL_OP_READ:
		err_code = sensor_ctrl_read(cmd);
		break;

	default:
		err_code = JARDUINO_ERROR_NOT_SUPPORTED;
	}

	return err_code;
}

static jarduino_error_t sensor_ctrl_init(sensor_ctrl_cmd_t * cmd)
{
	(*cmd).type = SENSOR_CTRL_OP_TYPE_RESPONSE;
	return JARDUINO_SUCCESS;
}

static jarduino_error_t sensor_ctrl_read(sensor_ctrl_cmd_t * cmd)
{
	(*cmd).type = SENSOR_CTRL_OP_TYPE_RESPONSE;
	(*cmd).rsp.read.luminosity = 100;
	(*cmd).rsp.read.temperature = 200;
	(*cmd).rsp.read.umidity = 300;
	(*cmd).rsp.read.soil = 400;

	return JARDUINO_SUCCESS;
}

static jarduino_error_t sensor_ctrl_config(sensor_ctrl_cmd_t * cmd)
{
	(*cmd).type = SENSOR_CTRL_OP_TYPE_RESPONSE;
	return JARDUINO_SUCCESS;
}


/*
int interface(sensor_ctrl_cmd_t * cmd);

int test(other_ifc_t *ifc)
{
	sensor_ctrl_cmd_t cmd;
	cmd.operation = SENSOR_CTRL_OP_READ;
	cmd.type = SENSOR_CTRL_OP_TYPE_REQUEST;

	int error = interface(&cmd);

	ifc.data = cmd.rsp;
}

int interface(sensor_ctrl_cmd_t * cmd)
{
	*cmd->type = SENSOR_CTRL_OP_TYPE_RESPONSE;
	*cmd->rsp.read.luminosity = 100;
	*cmd->rsp.read.temperature = 100;
	*cmd->rsp.read.umidity = 100;

	return 0;
}

*/
