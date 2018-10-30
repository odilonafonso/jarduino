/*
 * sensor_interface.h
 *
 *  Created on: 23 de out de 2018
 *      Author: odilon
 */

#ifndef LIB_SENSOR_CONTROLLER_SENSOR_INTERFACE_H_
#define LIB_SENSOR_CONTROLLER_SENSOR_INTERFACE_H_

typedef enum sensor_operation
{
	SENSOR_OP_INIT,
	SENSOR_OP_READ,
	SENSOR_OP_CONFIG
}sensor_operation_t;

typedef enum sensor_type
{
	SENSOR_TYPE_TEMPERATURE,
	SENSOR_TYPE_SOIL_MOISTURE,
	SENSOR_TYPE_LUMINOSITY,
}sensor_type_t;

typedef struct sensor_cmd
{
	sensor_operation_t cmd;
	sensor_type_t sensor_type;
}sensor_cmd_t;

#endif /* LIB_SENSOR_CONTROLLER_SENSOR_INTERFACE_H_ */
