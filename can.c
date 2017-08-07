#include <avr/io.h>
#include <avr/interrupt.h> 

#include "can.h"

#define TX_BUF_NUMBER 1
volatile int8_t data[8]; 

can_buf_t can_rx_buf[CAN_RX_BUF_SIZE];
volatile uint8_t can_rx_buf_top = 0;
volatile uint8_t can_rx_buf_bottom = 0;

can_std_registries_t can_std_reg;

ISR(CAN_INT_vect)
{
	uint8_t canhpmob = CANHPMOB;
	uint8_t cangit = CANGIT;
	if (canhpmob != 0xf0) {
		uint8_t savecanpage = CANPAGE;
		CANPAGE = canhpmob;
		if (CANSTMOB & (1 << RXOK)) {
			can_rx_buf[can_rx_buf_top].canidt1 = CANIDT1;
			can_rx_buf[can_rx_buf_top].canidt2 = CANIDT2;
			can_rx_buf[can_rx_buf_top].canidt3 = CANIDT3;
			can_rx_buf[can_rx_buf_top].canidt4 = CANIDT4;
			can_rx_buf[can_rx_buf_top].cancdmob = CANCDMOB & 0x1f;
			for (uint8_t i = 0; i < 8; ++i) {
				can_rx_buf[can_rx_buf_top].data[i] = CANMSG;
			}
			can_rx_buf_top = (uint8_t)((can_rx_buf_top + 1) & CAN_RX_BUF_INDEX_MASK);
			/*CANIDM1 = 0;
			CANIDM2 = 0;
			CANIDT3 = can_std_reg.node_id;
			CANIDM3 = 0xff;
			CANIDM4 = 1 << IDEMSK;*/
			CANCDMOB = (1<<CONMOB1) | (1<<IDE); //rx mob
		}
		else if (CANSTMOB & (1 << TXOK)) {
			CANCDMOB = 0; //disable mob
		}
		CANSTMOB = 0x00;  // Reset reason on selected channel
		CANPAGE = savecanpage;
	}
	//other interrupt
	CANGIT |= (cangit & 0x7f);
}

void
set_can_id(uint8_t priority, uint8_t type, uint16_t destination_id, uint16_t source_id)
{
	CANIDT1 = ((priority << 7) & 0x80) | ((type >> 1) & 0x7f);
	CANIDT2 = ((type << 7) & 0x80) | ((H9_RESERVED_VALUE << 5) & 0x60) | ((destination_id >> 4) & 0x1f);
	CANIDT3 = ((destination_id << 4) & 0xf0) | ((source_id >> 5) & 0x0f);
	CANIDT4 = ((source_id << 3) & 0xf8);
}

void
set_can_id_mask(uint8_t priority, uint8_t type, uint16_t destination_id, uint16_t source_id)
{
	CANIDM1 = ((priority << 7) & 0x80) | ((type >> 1) & 0x7f);
	CANIDM2 = ((type << 7) & 0x80) /*| ((H9_RESERVED_VALUE << 5) & 0x60)*/ | ((destination_id >> 4) & 0x1f);
	CANIDM3 = ((destination_id << 4) & 0xf0) | ((source_id >> 5) & 0x0f);
	CANIDM4 = ((source_id << 3) & 0xf8);
}

uint8_t
process_msg(h9msg_t *cm)
{
	// unicast
	if (cm->destination_id == can_std_reg.node_id) {
		if (cm->type == H9_TYPE_SET_REG && cm->dlc > 1 && cm->data[0] < 10) {
			h9msg_t cm_res;
			CAN_init_response_msg(cm, &cm_res);
			cm_res.dlc = 2;
			cm_res.data[0] = cm->data[0];
			switch (cm_res.data[0]) {
			case 1: //node id
				can_std_reg.node_id = (cm->data[1] & 0x01) << 8 | cm->data[2];
				cm_res.data[1] = (can_std_reg.node_id >> 8) & 0x01;
				cm_res.data[2] = (can_std_reg.node_id) & 0xff;
				cm_res.dlc = 3;
				break;
			default:
				return 0;
			}
			CAN_put_msg(&cm_res);
			return 0;
		}
	}
	
	// unicast or broadcast
	if (cm->destination_id == can_std_reg.node_id || cm->destination_id == H9_BROADCAST_ID) {
		if (cm->type == H9_TYPE_GET_REG && cm->dlc == 1 && cm->data[0] < 10) {
			h9msg_t cm_res;
			CAN_init_response_msg(cm, &cm_res);
			cm_res.dlc = 2;
			cm_res.data[0] = cm->data[0];
			switch (cm_res.data[0]) {
			case 0: //flags
				cm_res.data[1] = 0;
				cm_res.dlc = 2;
				break;
			case 1: //node id
				cm_res.data[1] = (can_std_reg.node_id >> 8) & 0x01;
				cm_res.data[2] = (can_std_reg.node_id) & 0xff;
				cm_res.dlc = 3;
				break;
			default:
				return 0;
			}
			CAN_put_msg(&cm_res);
			return 0;
		}
	}
	return 1;
}

void
CAN_reinit(uint16_t node_id)
{
	can_std_reg.node_id = node_id;

	for ( int8_t mob=0; mob<6; mob++ ) { 
		CANPAGE = ( mob << MOBNB0 );        // Selects Message Object 0-5
		CANCDMOB = 0x00;             // Disable mob
		CANSTMOB = 0x00;           // Clear mob status register;
	}

	CANPAGE = 0x01 << MOBNB0; //select mob 1
	set_can_id(0, 0, node_id, 0);
	set_can_id_mask(0, 0, (1<<H9_DESTINATION_ID_BIT_LENGTH)-1, 0);
	CANIDM4 |= 1 << IDEMSK; // set filter
	CANCDMOB = (1<<CONMOB1) | (1<<IDE); //rx mob, 29-bit only

	CANPAGE = 0x02 << MOBNB0; //select mob 2
	set_can_id(0, 0, H9_BROADCAST_ID, 0);
	set_can_id_mask(0, 0, (1<<H9_DESTINATION_ID_BIT_LENGTH)-1, 0);
	CANIDM4 |= 1 << IDEMSK; // set filter
		
	CANIE2 = ( 1 << IEMOB0 ) | ( 1 << IEMOB1 ) | ( 1 << IEMOB2 ); //interupt mob 0 1 and 2
}

void
CAN_init(uint16_t node_id)
{
	CANGCON = ( 1 << SWRES );   // Software reset 
	CANTCON = 0x00;         // CAN timing prescaler set to 0; 

	/*
	http://karmacarpentry.com/can_calc.html?clock=8000000&baud=125000&quanta=16&propseg=2&phaseseg1=7&phaseseg2=6
	*/
	CANBT1 = 0x1e;
	CANBT2 = 0x04;
	CANBT3 = 0x12;

	CAN_reinit(node_id);

	CANGIE = (1<<ENBOFF) | (1<<ENIT) | (1<<ENRX) | (1<<ENTX) | (1<<ENERR) | (1<<ENBX) | (1<<ENERG);
	CANGCON = 1<<ENASTB;
}

void
CAN_send_turned_on_broadcast() 
{
		h9msg_t cm;
		CAN_init_new_msg(&cm);

		cm.type = H9_TYPE_DEV_TURNED_ON;
		cm.destination_id = H9_BROADCAST_ID;
		cm.dlc = 0;
		CAN_put_msg(&cm);
}

void
CAN_set_mob(uint8_t priority, uint8_t priority_mask,
            uint8_t type, uint8_t type_mask,
			uint16_t destination_id, uint16_t destination_id_mask,
			uint16_t source_id, uint16_t source_id_mask)
{
		CANPAGE = 0x03 << MOBNB0; //select mob 3
		set_can_id(priority, type, destination_id, source_id);
		set_can_id_mask(priority_mask, type_mask, destination_id_mask, source_id_mask);
		CANIDM4 |= 1 << IDEMSK; // set filter
		CANCDMOB = (1<<CONMOB1) | (1<<IDE); //rx mob, 29-bit only
		
		CANIE2 |= 1 << IEMOB3;
}

/*
1. Several fields must be initialized before sending:
– Identifier tag (IDT)
– Identifier extension (IDE)
– Remote transmission request (RTRTAG)
– Data length code (DLC)
– Reserved bit(s) tag (RBnTAG)
– Data bytes of message (MSG)
2. The MOb is ready to send a data or a remote frame when the MOb configuration is set (CONMOB).
3. Then, the CAN channel scans all the MObs in Tx configuration, finds the MOb hav- ing the highest priority and tries to send it.
4. When the transmission is completed the TXOK flag is set (interrupt).
5. All the parameters and data are available in the MOb until a new initialization.
*/
void
CAN_put_msg(h9msg_t *cm)
{
	CANPAGE = 0 << MOBNB0;      		// Select MOb0 for transmission
	while ( CANEN2 & ( 1 << ENMOB0 ) ); // Wait for MOb 0 to be free
	CANSTMOB = 0x00;       				// Clear mob status register

	set_can_id(cm->priority, cm->type, cm->destination_id, cm->source_id);

	uint8_t idx = 0;
	for (; idx < 8; ++idx)
		CANMSG = cm->data[idx];

	CANCDMOB = (1 << CONMOB0) | (1 << IDE) | (cm->dlc & 0x0f);
}

/*
1. Several fields must be initialized before receiving:
– Identifier tag (IDT)
– Identifier mask (IDMSK)
– Identifier extension (IDE)
– Identifier extension mask (IDEMSK)
– Remote transmission request (RTRTAG)
– Remote transmission request mask (RTRMSK)
– Data length code (DLC)
– Reserved bit(s) tag (RBnTAG)
2. The MOb is ready to receive a data or a remote frame when the MOb configuration is set (CONMOB).
3. When a frame identifier is received on CAN network, the CAN channel scans all the MObs in receive mode, tries to find the MOb having the highest priority which is matching.
4. On a hit, the IDT, the IDE and the DLC of the matched MOb are updated from the incoming (frame) values.
5. Once the reception is completed, the data bytes of the received message are stored (not for remote frame) in the data buffer of the matched MOb and the RXOK flag is set (interrupt).
6. All the parameters and data are available in the MOb until a new initialization.
*/
uint8_t
CAN_get_msg(h9msg_t *cm)
{
	if (can_rx_buf_top != can_rx_buf_bottom) {
		cm->priority = (can_rx_buf[can_rx_buf_bottom].canidt1 >> 7) & 0x01;
		cm->type = ((can_rx_buf[can_rx_buf_bottom].canidt1 << 1) & 0xfe) | ((can_rx_buf[can_rx_buf_bottom].canidt2 >> 7) & 0x01);
		cm->destination_id = ((can_rx_buf[can_rx_buf_bottom].canidt2 << 4) & 0x1f0) | ((can_rx_buf[can_rx_buf_bottom].canidt3 >> 4) & 0x0f);
		cm->source_id = ((can_rx_buf[can_rx_buf_bottom].canidt3 << 5) & 0x1e0) | ((can_rx_buf[can_rx_buf_bottom].canidt4 >> 3) & 0x1f);

		cm->dlc = can_rx_buf[can_rx_buf_bottom].cancdmob & 0x0f;
		uint8_t idx = 0;
		for (; idx < 8; ++idx)
			cm->data[idx] = can_rx_buf[can_rx_buf_bottom].data[idx];

		can_rx_buf_bottom = (uint8_t)((can_rx_buf_bottom + 1) & CAN_RX_BUF_INDEX_MASK);

		return process_msg(cm);
	}
	return 0;
}

void
CAN_init_new_msg(h9msg_t *mes)
{
	mes->priority = H9_PRIORITY_LOW;
	mes->source_id = can_std_reg.node_id;
	mes->dlc = 0;
}

void
CAN_init_response_msg(const h9msg_t *req, h9msg_t *res)
{
	res->priority = req->priority;
	switch (req->type) {
		case H9_TYPE_GET_REG:
			res->type = H9_TYPE_REG_VALUE;
			break;
		case H9_TYPE_SET_REG:
			res->type = H9_TYPE_REG_EXTERNALLY_CHANGED;
			break;
	}
	res->source_id = can_std_reg.node_id;
}
