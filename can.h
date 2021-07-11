// CAN command handler
// Ian Glen <ian@ianglen.me>

#ifndef AT_CAN_H
#define AT_CAN_H


#include <FlexCAN_T4.h>


#define CAN_ID			0x21	// our node id
#define CAN_BAUDRATE	1000000	// bps

extern FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can_main;


// initialize FlexCAN
void can_init(void);

// callback for FlexCAN library to process received CAN messages
void can_receive_msg(const CAN_message_t &msg);

// listen for a command and execute a callback function when received
void can_register_cmd(uint16_t cmd, void (*callback)(uint8_t id, uint16_t cmd, const uint8_t *payload, size_t len));

// send a command to another CAN id
void can_send_cmd(uint8_t id, uint16_t cmd, const uint8_t *payload, size_t len);

// process CAN things
void can_task(void);


#endif // AT_CAN_H
