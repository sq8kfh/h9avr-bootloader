/*
 * h9avr-can
 *
 * Created by SQ8KFH on 2017-08-07.
 *
 * Copyright (C) 2017-2019 Kamil Palkowski. All rights reserved.
 */

#include <avr/io.h>
#include <avr/interrupt.h> 
#include <avr/wdt.h>

#include "can.h"
#include "common.h"

#define CAN_RX_BUF_SIZE 16
#define CAN_RX_BUF_INDEX_MASK 0x0F

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

typedef struct {
    uint8_t canidt1;
    uint8_t canidt2;
    uint8_t canidt3;
    uint8_t canidt4;
    uint8_t cancdmob;
    uint8_t data[8];
} can_buf_t;

can_buf_t can_rx_buf[CAN_RX_BUF_SIZE];
volatile uint8_t can_rx_buf_top = 0;
volatile uint8_t can_rx_buf_bottom = 0;


void write_node_id(uint16_t id) {
    cli();
    eeprom_write_word(&ee_node_id, id);
    sei();
    //read_node_id();
}


/* for software reset */
__attribute__((naked)) __attribute__((section(".init3"))) void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();
    return;
}


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


uint8_t process_msg(h9msg_t *cm) {
    // 1st msg filter: mob filter/mask
    // 2nd msg filter: CAN_get_msg
    // 3rd msg filter
    if (cm->destination_id == can_node_id || cm->destination_id == H9MSG_BROADCAST_ID) {
        if (cm->type == H9MSG_TYPE_SET_REG && cm->dlc > 1 && cm->data[0] < 10) {
            h9msg_t cm_res;
            CAN_init_response_msg(cm, &cm_res);
            cm_res.dlc = 2;
            cm_res.data[0] = cm->data[0];
            if (cm_res.data[0] == 1 && cm->dlc == 3) { //reg 1
                write_node_id((cm->data[1] & 0x01) << 8 | cm->data[2]);

                cm_res.data[1] = (can_node_id >> 8) & 0x01;
                cm_res.data[2] = (can_node_id) & 0xff;
                cm_res.dlc = 3;
            }
            CAN_put_msg(&cm_res);
            return 0;
        }
        else if (cm->type == H9MSG_TYPE_GET_REG && cm->dlc == 1 && cm->data[0] < 10) {
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
                    cm_res.data[1] = (can_node_id >> 8) & 0x01;
                    cm_res.data[2] = (can_node_id) & 0xff;
                    cm_res.dlc = 3;
                    break;
                default:
                    return 0;
            }
            CAN_put_msg(&cm_res);
            return 0;
        }
        else if (cm->type == H9MSG_TYPE_DISCOVERY && cm->dlc == 0) {
            h9msg_t cm_res;
            CAN_init_response_msg(cm, &cm_res);
            cm_res.dlc = 0;
            //TODO: add node info to res
            CAN_put_msg(&cm_res);
            return 0;
        }
        else if (cm->type == H9MSG_TYPE_NODE_RESET && cm->dlc == 0) {
            do {
                wdt_enable(WDTO_15MS);
                for(;;) {
                }
            } while(0);
        }
#ifdef BOOTSTART
        else if (cm->type == H9MSG_TYPE_NODE_UPGRADE && cm->dlc == 0) {
            cli();
            asm volatile ( "jmp " STR(BOOTSTART) );
        }
#else
        #warning "Node upgrade (bootloader) disable"
#endif //BOOTSTART
    }
    return 1;
}


void CAN_init(void) {
    init_common_CAN();

    //select mob 2 for unicast
    CANPAGE = 0x01 << MOBNB0;
    set_CAN_id(0, 0, 0, can_node_id, 0);
    set_CAN_id_mask(0, 0, 0, (1<<H9MSG_DESTINATION_ID_BIT_LENGTH)-1, 0);
    CANIDM4 |= 1 << IDEMSK; // set filter
    CANCDMOB = (1<<CONMOB1) | (1<<IDE); //rx mob, 29-bit only

    //select mob 2 for broadcast with type form 3rd group
    CANPAGE = 0x02 << MOBNB0;
    set_CAN_id(0, H9MSG_TYPE_GROUP_3, 0, H9MSG_BROADCAST_ID, 0);
    set_CAN_id_mask(0, H9MSG_TYPE_GROUP_MASK, 0, (1<<H9MSG_DESTINATION_ID_BIT_LENGTH)-1, 0);
    CANIDM4 |= 1 << IDEMSK; // set filter
    CANCDMOB = (1<<CONMOB1) | (1<<IDE); //rx mob, 29-bit only


    CANIE2 = ( 1 << IEMOB0 ) | ( 1 << IEMOB1 ) | ( 1 << IEMOB2 ); //interupt mob 0 1 and 2

    CANGIE = (1<<ENBOFF) | (1<<ENIT) | (1<<ENRX) | (1<<ENTX) | (1<<ENERR) | (1<<ENBX) | (1<<ENERG);
    CANGCON = 1<<ENASTB;
}

void CAN_send_turned_on_broadcast(void) {
    h9msg_t cm;
    CAN_init_new_msg(&cm);

    cm.type = H9MSG_TYPE_NODE_TURNED_ON;
    cm.destination_id = H9MSG_BROADCAST_ID;
    cm.dlc = 0;
    CAN_put_msg(&cm);
}


void CAN_set_mob_for_remote_node1(uint16_t remote_node_id) {
    CANPAGE = 0x03 << MOBNB0; //select mob 3
    set_CAN_id(0, H9MSG_TYPE_GROUP_1, 0, 0, remote_node_id);
    set_CAN_id_mask(0, H9MSG_TYPE_GROUP_MASK, 0, 0, (1<<H9MSG_SOURCE_ID_BIT_LENGTH)-1);
    CANIDM4 |= 1 << IDEMSK; // set filter
    CANCDMOB = (1<<CONMOB1) | (1<<IDE); //rx mob, 29-bit only

    CANIE2 |= 1 << IEMOB3;
}


void CAN_set_mob_for_remote_node2(uint16_t remote_node_id) {
    CANPAGE = 0x04 << MOBNB0; //select mob 3
    set_CAN_id(0, H9MSG_TYPE_GROUP_1, 0, 0, remote_node_id);
    set_CAN_id_mask(0, H9MSG_TYPE_GROUP_MASK, 0, 0, (1<<H9MSG_SOURCE_ID_BIT_LENGTH)-1);
    CANIDM4 |= 1 << IDEMSK; // set filter
    CANCDMOB = (1<<CONMOB1) | (1<<IDE); //rx mob, 29-bit only
    
    CANIE2 |= 1 << IEMOB4;
}


void CAN_set_mob_for_remote_node3(uint16_t remote_node_id) {
    CANPAGE = 0x05 << MOBNB0; //select mob 3
    set_CAN_id(0, H9MSG_TYPE_GROUP_1, 0, 0, remote_node_id);
    set_CAN_id_mask(0, H9MSG_TYPE_GROUP_MASK, 0, 0, (1<<H9MSG_SOURCE_ID_BIT_LENGTH)-1);
    CANIDM4 |= 1 << IDEMSK; // set filter
    CANCDMOB = (1<<CONMOB1) | (1<<IDE); //rx mob, 29-bit only
    
    CANIE2 |= 1 << IEMOB5;
}


uint8_t CAN_get_msg(h9msg_t *cm) {
    if (can_rx_buf_top != can_rx_buf_bottom) {
        cm->priority = (can_rx_buf[can_rx_buf_bottom].canidt1 >> 7) & 0x01;
        cm->type = ((can_rx_buf[can_rx_buf_bottom].canidt1 >> 2) & 0x1f);
        cm->seqnum = ((can_rx_buf[can_rx_buf_bottom].canidt1 >> 5) & 0x18) | ((can_rx_buf[can_rx_buf_bottom].canidt2 >> 5) & 0x0f);
        cm->destination_id = ((can_rx_buf[can_rx_buf_bottom].canidt2 << 4) & 0x1f0) | ((can_rx_buf[can_rx_buf_bottom].canidt3 >> 4) & 0x0f);
        cm->source_id = ((can_rx_buf[can_rx_buf_bottom].canidt3 << 5) & 0x1e0) | ((can_rx_buf[can_rx_buf_bottom].canidt4 >> 3) & 0x1f);

        cm->dlc = can_rx_buf[can_rx_buf_bottom].cancdmob & 0x0f;
        uint8_t idx = 0;
        for (; idx < 8; ++idx)
            cm->data[idx] = can_rx_buf[can_rx_buf_bottom].data[idx];

        can_rx_buf_bottom = (uint8_t)((can_rx_buf_bottom + 1) & CAN_RX_BUF_INDEX_MASK);

        // 1st msg filter: mob filter/mask
        // 2nd msg filter
        if (cm->source_id == H9MSG_BROADCAST_ID) { //invalid message, drop
            return 0;
        }

        return process_msg(cm);
    }
    return 0;
}


void CAN_init_response_msg(const h9msg_t *req, h9msg_t *res) {
    CAN_init_new_msg(res);
    res->priority = req->priority;
    switch (req->type) {
        case H9MSG_TYPE_GET_REG:
            res->type = H9MSG_TYPE_REG_VALUE;
            break;
        case H9MSG_TYPE_SET_REG:
            res->type = H9MSG_TYPE_REG_EXTERNALLY_CHANGED;
            break;
        case H9MSG_TYPE_DISCOVERY:
            res->type = H9MSG_TYPE_NODE_INFO;
            break;
    }
    res->seqnum = req->seqnum;
    res->destination_id = req->source_id;
}

