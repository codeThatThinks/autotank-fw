// VESC communication library

#include <Arduino.h>

#include "can.h"
#include "vesc.h"
#include "vesc_buffer.h"
#include "vesc_crc.h"
#include "vesc_datatypes.h"
#include "vesc_packet.h"


vesc_values_t vesc_values[2];

static uint8_t num_vescs;
static uint8_t receive_buffer[263];


static bool is_vesc(uint8_t id);	// check if a vesc exists
static void send(uint8_t id, uint8_t *payload, size_t len);	// send a comm command to the Vesc
static void receive(uint8_t id, uint16_t cmd, const uint8_t *payload, size_t len);	// receive a comm command from the Vesc
static void process_comm(uint8_t id, size_t len);	// process a comm command received from the vesc


// register a vesc
size_t vesc_register(uint8_t id)
{
	if(num_vescs == 2)
	{
		Serial.println("[ERROR] vesc: Max number of Vescs registered!");
		return 0;
	}

	vesc_values[num_vescs].id = id;
	num_vescs++;

	return num_vescs - 1;
}

// vesc initialization
void vesc_init(void)
{
	can_register_cmd(CAN_PACKET_PROCESS_SHORT_BUFFER, receive);
	can_register_cmd(CAN_PACKET_FILL_RX_BUFFER, receive);
	can_register_cmd(CAN_PACKET_FILL_RX_BUFFER_LONG, receive);
	can_register_cmd(CAN_PACKET_PROCESS_RX_BUFFER, receive);

	//Serial.println("[INFO] vesc: Initialization complete");
}

// request new values from a vesc
void vesc_update_values(uint8_t id)
{
	if(!is_vesc(id)) return;

	size_t len = 0;
	uint8_t payload[1];

	payload[len++] = COMM_GET_VALUES;

	send(id, payload, len);
}

// set motor duty
void vesc_set_duty(uint8_t id, float duty)
{
	if(!is_vesc(id)) return;

	size_t len = 0;
	uint8_t payload[5];

	payload[len++] = COMM_SET_DUTY;
	buffer_append_float32(payload, duty, 1e1, &len);

	send(id, payload, len);
}

// set motor current
void vesc_set_current(uint8_t id, float current)
{
	if(!is_vesc(id)) return;

	size_t len = 0;
	uint8_t payload[5];

	payload[len++] = COMM_SET_CURRENT;
	buffer_append_float32(payload, current, 1e3, &len);

	send(id, payload, len);
}

// set motor brake current
void vesc_set_current_brake(uint8_t id, float current)
{
	if(!is_vesc(id)) return;

	size_t len = 0;
	uint8_t payload[5];

	payload[len++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_float32(payload, current, 1e3, &len);

	send(id, payload, len);
}

// set motor rpm
void vesc_set_rpm(uint8_t id, float rpm)
{
	if(!is_vesc(id)) return;

	size_t len = 0;
	uint8_t payload[5];

	payload[len++] = COMM_SET_RPM;
	buffer_append_float32(payload, rpm, 1, &len);

	send(id, payload, len);
}

// set motor position
void vesc_set_position(uint8_t id, float position)
{
	if(!is_vesc(id)) return;

	size_t len = 0;
	uint8_t payload[5];

	payload[len++] = COMM_SET_POS;
	buffer_append_float32(payload, position, 1e6, &len);

	send(id, payload, len);
}

// reboot vesc
void vesc_reboot(uint8_t id)
{
	if(!is_vesc(id)) return;

	size_t len = 0;
	uint8_t payload[1];

	payload[len++] = COMM_REBOOT;

	send(id, payload, len);
}

// send alive packet
void vesc_keep_alive(uint8_t id)
{
	if(!is_vesc(id)) return;

	size_t len = 0;
	uint8_t payload[1];

	payload[len++] = COMM_ALIVE;

	send(id, payload, len);
}

/*
// get a description of a VESC fault code
String getFaultMsg(uint8_t id, vesc_fault_code_t code)
{
	if(!is_vesc(id)) return;

	if(code >= NUM_VESC_FAULT_CODE) {
		return "Unknown fault";
	}

	return VESC_FAULT_CODE_STR[code];
}
*/

// check if a vesc exists
static bool is_vesc(uint8_t id)
{
	size_t i;
	for(i = 0; i < num_vescs; i++)
	{
		if(vesc_values[i].id == id) break;
	}

	if(i == num_vescs)
	{
		Serial.println("[ERROR] vesc: Vesc ID 0x");
		Serial.print(id, HEX);
		Serial.println(" not found!");
		return false;
	}

	return true;
}

/*
	VESC Comm Protocol:

	Since CAN messages are limited to a payload of 8 bytes, messages are split
	up amoung multiple CAN messages. Buffer messages are sent first with the
	payload data, then a process message is sent to execute the comm command.

	CAN_PACKET_PROCESS_SHORT_BUFFER:
		If payload len <= 6, send data and proccess in one CAN message.
		Byte 0: CAN id that sent the message
		Byte 1: Process mode (set to 0 for process and reply)
		Bytes 2-7: Comm payload

	CAN_PACKET_FILL_RX_BUFFER:
		Payload data bytes 0-255.
		Byte 0: Comm payload byte index
		Bytes 1-7: Comm payload

	CAN_PACKET_FILL_RX_BUFFER_LONG:
		Payload data bytes 256-65535.
		Byte 0: Comm payload byte index (high byte)
		Byte 1: Comm payload byte index (low byte)
		Byte 2-7: Comm payload

	CAN_PACKET_PROCESS_RX_BUFFER:
		Process comm command w/ already received payload data.
		Byte 0: CAN id that sent the message
		Byte 1: Process mode (set to 0 for process and reply)
		Byte 2: Comm payload length (high byte)
		Byte 3: Comm payload length (low byte)
		Byte 4: Comm payload crc (high byte)
		Byte 5: Comm payload crc (low byte)
*/

// send a comm command to the Vesc
static void send(uint8_t id, uint8_t *payload, size_t len)
{
	size_t payload_index = 0;
	uint8_t msg[8];
	size_t msg_len;

	if(len == 0 || len > PACKET_MAX_PL_LEN || len > 65535)
	{
		Serial.print("[ERROR] vesc: Comm payload length must be between 1 and ");
		Serial.println(PACKET_MAX_PL_LEN, DEC);
		return;
	}

	if(len <= 6)
	{
		// fit entire comm payload into one can message

		msg_len = 0;
		msg[msg_len++] = CAN_ID;
		msg[msg_len++] = 0;	// vesc should process comm message and reply
		memcpy(msg + msg_len, payload, len);
		msg_len += len;

		can_send_cmd(id, CAN_PACKET_PROCESS_SHORT_BUFFER, msg, msg_len);

	}
	else
	{
		// split up comm payload across multiple can messages

		// send bytes 0 to 255 in Fill RX Buffer msgs, 7 bytes at a time
		while(payload_index < len && payload_index <= 255) {
			msg_len = 0;
			msg[msg_len++] = payload_index;

			if((len - payload_index) >= 7) {
				memcpy(msg + msg_len, payload + payload_index, 7);
				msg_len += 7;
				payload_index += 7;

			} else {
				memcpy(msg + msg_len, payload + payload_index, len - payload_index);
				msg_len += len - payload_index;
				payload_index += len - payload_index;
			}

			can_send_cmd(id, CAN_PACKET_FILL_RX_BUFFER, msg, msg_len);
		}

		// send bytes 256 and beyond in Fill RX Buffer Long msgs, 6 bytes at a time
		while(payload_index < len) {
			msg_len = 0;
			msg[msg_len++] = payload_index >> 8;
			msg[msg_len++] = payload_index & 0xFF;

			if((len - payload_index) >= 6) {
				memcpy(msg + msg_len, payload + payload_index, 6);
				msg_len += 6;
				payload_index += 6;

			} else {
				memcpy(msg + msg_len, payload + payload_index, len - payload_index);
				msg_len += len - payload_index;
				payload_index += len - payload_index;
			}

			can_send_cmd(id, CAN_PACKET_FILL_RX_BUFFER_LONG, msg, msg_len);
		}

		// all bytes sent, send can message to process comm packet
		msg_len = 0;
		msg[msg_len++] = CAN_ID;
		msg[msg_len++] = 0;	// vesc should process comm message and reply
		msg[msg_len++] = len >> 8;
		msg[msg_len++] = len & 0xFF;
		uint16_t crc = crc16(payload, len);
		msg[msg_len++] = crc >> 8;
		msg[msg_len++] = crc & 0xFF;

		can_send_cmd(id, CAN_PACKET_PROCESS_RX_BUFFER, msg, msg_len);
	}

	Serial.print("[INFO] vesc: Sent comm command to ID 0x");
	Serial.println(id, HEX);
}

// receive a comm command from the Vesc
static void receive(uint8_t id, uint16_t cmd, const uint8_t *payload, size_t len)
{
	if(cmd == CAN_PACKET_PROCESS_SHORT_BUFFER)
	{
		if(len < 3)
		{
			Serial.println("[WARN] vesc: Received invalid PROCESS_SHORT_BUFFER msg");
			return;
		}

		memcpy(receive_buffer, payload + 2, len - 2);
		process_comm(payload[0], len - 2);
	}
	else if(cmd == CAN_PACKET_FILL_RX_BUFFER)
	{
		if(len < 2)
		{
			Serial.println("[WARN] vesc: Received invalid CAN_PACKET_FILL_RX_BUFFER msg");
			return;
		}

		memcpy(receive_buffer + payload[0], payload + 1, len - 1);
	}
	else if(cmd == CAN_PACKET_FILL_RX_BUFFER_LONG)
	{
		if(len < 3)
		{
			Serial.println("[WARN] vesc: Received invalid CAN_PACKET_FILL_RX_BUFFER_LONG msg");
			return;
		}

		Serial.println("[WARN] vesc: CAN_PACKET_FILL_RX_BUFFER_LONG msg not supported");
	}
	else if(cmd == CAN_PACKET_PROCESS_RX_BUFFER)
	{
		if(len != 6)
		{
			Serial.println("[WARN] vesc: Received invalid CAN_PACKET_PROCESS_RX_BUFFER msg");
			return;
		}

		size_t receive_len = (payload[2] << 8) | payload[3];
		if(receive_len > 263)
		{
			Serial.println("[WARN] vesc: Received comm command longer than 263 bytes not supported");
			return;
		}

		uint16_t crc = (payload[4] << 8) | payload[5];
		if(crc != crc16(receive_buffer, receive_len))
		{
			Serial.println("[WARN] vesc: Received comm command with invalid CRC");
			return;
		}

		process_comm(payload[0], receive_len);
	}
}

// process a comm command received from the vesc
static void process_comm(uint8_t id, size_t len)
{
	if(!is_vesc(id))
	{
		Serial.print("[WARN] vesc: Received comm command from unknown ID 0x");
		Serial.println(id, HEX);
		return;
	}

	if(receive_buffer[0] == COMM_GET_VALUES)
	{
		//Serial.print("[INFO] vesc: Received COMM_GET_VALUES command from ID 0x");
		//Serial.println(id, HEX);

		// received reply to get values command
		size_t i;
		for(i = 0; i < num_vescs; i++)
		{
			if(vesc_values[i].id == id) break;
		}

		size_t index = 1;
		vesc_values[i].tempFET = buffer_get_float16(receive_buffer, 1e1, &index, len);
		vesc_values[i].tempMotor = buffer_get_float16(receive_buffer, 1e1, &index, len);
		vesc_values[i].avgMotorCurrent = buffer_get_float32(receive_buffer, 1e2, &index, len);
		vesc_values[i].avgInputCurrent = buffer_get_float32(receive_buffer, 1e2, &index, len);
		vesc_values[i].avgId = buffer_get_float32(receive_buffer, 1e2, &index, len);
		vesc_values[i].avgIq = buffer_get_float32(receive_buffer, 1e2, &index, len);
		vesc_values[i].dutyCycle = buffer_get_float16(receive_buffer, 1e3, &index, len);
		vesc_values[i].rpm = buffer_get_float32(receive_buffer, 1, &index, len);
		vesc_values[i].inputVoltage = buffer_get_float16(receive_buffer, 1e1, &index, len);
		vesc_values[i].ampHours = buffer_get_float32(receive_buffer, 1e4, &index, len);
		vesc_values[i].ampHoursCharged = buffer_get_float32(receive_buffer, 1e4, &index, len);
		vesc_values[i].wattHours = buffer_get_float32(receive_buffer, 1e4, &index, len);
		vesc_values[i].wattHoursCharged = buffer_get_float32(receive_buffer, 1e4, &index, len);
		vesc_values[i].tachometer = buffer_get_int32(receive_buffer, &index, len);
		vesc_values[i].tachometerAbs = buffer_get_int32(receive_buffer, &index, len);
		vesc_values[i].fault = (vesc_fault_code_t)buffer_get_uint8(receive_buffer, &index, len);
		vesc_values[i].position = buffer_get_float32(receive_buffer, 1e6, &index, len);
		index++;
		//vesc_values[i].id = buffer_get_uint8(receive_buffer, &index, len);
		vesc_values[i].ntc1 = buffer_get_float16(receive_buffer, 1e1, &index, len);
		vesc_values[i].ntc2 = buffer_get_float16(receive_buffer, 1e1, &index, len);
		vesc_values[i].ntc3 = buffer_get_float16(receive_buffer, 1e1, &index, len);
		vesc_values[i].avgVd = buffer_get_float32(receive_buffer, 1e3, &index, len);
		vesc_values[i].avgVq = buffer_get_float32(receive_buffer, 1e3, &index, len);
	}
	else
	{
		// unknown command
		Serial.println("[WARN] vesc: Received unknown comm command from ID 0x");
		Serial.println(id, HEX);
	}
}
