// VESC communication library
// Ian Glen <ian@ianglen.me>

#ifndef VESC_H
#define VESC_H


#include <Arduino.h>

#include "vesc_datatypes.h"


// Telemetry data returned by VESC
typedef struct
{
	uint8_t id;
	float tempFET;
	float tempMotor;
	float avgMotorCurrent;
	float avgInputCurrent;
	float avgId;
	float avgIq;
	float dutyCycle;
	float rpm;
	float inputVoltage;
	float ampHours;
	float ampHoursCharged;
	float wattHours;
	float wattHoursCharged;
	int32_t tachometer;
	int32_t tachometerAbs;
	vesc_fault_code_t fault;
	float position;
	float ntc1;
	float ntc2;
	float ntc3;
	float avgVd;
	float avgVq;
} vesc_values_t;


extern vesc_values_t vesc_values[2];


// register a vesc
size_t vesc_register(uint8_t id);

// vesc initialization
void vesc_init(void);

// request new values from a vesc
void vesc_update_values(uint8_t id);

// set motor duty
void vesc_set_duty(uint8_t id, float duty);

// set motor current
void vesc_set_current(uint8_t id, float current);

// set motor brake current
void vesc_set_current_brake(uint8_t id, float current);

// set motor rpm
void vesc_set_rpm(uint8_t id, float rpm);

// set motor position
void vesc_set_position(uint8_t id, float position);

// reboot vesc
void vesc_reboot(uint8_t id);

// send alive packet
void vesc_keep_alive(uint8_t id);

// get a description of a VESC fault code
//String getFaultMsg(uint8_t id, vesc_fault_code_t code);


#endif // VESC_H
