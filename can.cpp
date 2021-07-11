// CAN command handler
// Ian Glen <ian@ianglen.me>

#include <FlexCAN_T4.h>

#include "can.h"


typedef struct
{
	uint16_t cmd;
	void (*callback)(uint8_t id, uint16_t cmd, const uint8_t *payload, size_t len);
} can_command_t;


FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can_main;

static uint8_t num_cmds = 0;
static can_command_t cmds[16];


// initialize FlexCAN
void can_init(void)
{
	can_main.begin();
	can_main.setBaudRate(CAN_BAUDRATE);
	can_main.enableFIFO();
	can_main.setFIFOFilter(REJECT_ALL);
	can_main.enableFIFOInterrupt();
	can_main.onReceive(can_receive_msg);

	//Serial.println("[INFO] can: Initialization complete");
}

// callback for FlexCAN library to process received CAN messages
void can_receive_msg(const CAN_message_t &msg)
{
	uint8_t id = msg.id & 0xFF;
	uint16_t cmd = msg.id >> 8;

	for(size_t i = 0; i < num_cmds; i++)
	{
		if(cmds[i].cmd == cmd && id == CAN_ID)
		{
			//Serial.print("[INFO] can: Received command ");
			//Serial.println(cmd, DEC);

			cmds[i].callback(id, cmd, msg.buf, msg.len);
			return;
		}
	}
}

// listen for a command and execute a callback function when received
void can_register_cmd(uint16_t cmd, void (*callback)(uint8_t id, uint16_t cmd, const uint8_t *payload, size_t len))
{
	if(num_cmds == 16 || can_main.setFIFOFilter(num_cmds, (cmd << 8) | CAN_ID, EXT) == 0)
	{
		Serial.println("[ERROR] can: Max number of CAN commands registered!");
		return;
	}

	for(size_t i = 0; i < num_cmds; i++)
	{
		if(cmds[i].cmd == cmd)
		{
			Serial.print("[WARN] can: Command ");
			Serial.print(cmd, DEC);
			Serial.println(" already registered!");
			return;
		}
	}

	cmds[num_cmds].cmd = cmd;
	cmds[num_cmds].callback = callback;
	num_cmds++;
}

// send a command to another CAN id
void can_send_cmd(uint8_t id, uint16_t cmd, const uint8_t *payload, size_t len)
{
	if(len > 8)
	{
		Serial.println("[WARN] can: Command payload capped at 8 bytes!");
		len = 8;
	}

	CAN_message_t msg = {0};
	msg.id = (cmd << 8) | id;
	msg.flags.extended = 1;
	msg.len = len;
	memcpy(msg.buf, payload, len);
	can_main.write(msg);

	Serial.print("[INFO] can: Send command ");
	Serial.print(cmd, DEC);
	Serial.print(" to ID 0x");
	Serial.println(id, HEX);
}

// process CAN things
void can_task(void)
{
	can_main.events();
}
