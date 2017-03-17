/**
   Licensed to the Apache Software Foundation (ASF) under one
   or more contributor license agreements.  See the NOTICE file
   distributed with this work for additional information
   regarding copyright ownership.  The ASF licenses this file
   to you under the Apache License, Version 2.0 (the
   "License"); you may not use this file except in compliance
   with the License.  You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
   Unless required by applicable law or agreed to in writing,
   software distributed under the License is distributed on an
   "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.  See the License for the
   specific language governing permissions and limitations
   under the License.
 *******************************************************************************
 * @license This project is demain_leaf.c: In function 'process_thread_init_system_proc':
   livered under APACHE 2.0.
 * @file main_leaf.c
 * @author Ânderson Ignacio da Silva
 * @date 16 Mar 2017
 * @brief Main file for devices - 6LoWPAN
 * @see http://www.aignacio.com
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"
#include "debug_info.h"
#include "net/ipv6/uip-ds6.h"
#include "sys/etimer.h"
#include "simple-udp.h"
#include <stdlib.h>
#include <string.h>

#define UDP_PORT_CENTRAL 7878
#define UDP_PORT_OUT 5555

static struct simple_udp_connection broadcast_connection;
static uip_ipaddr_t server_addr;
static char device_id[17];
static uint16_t central_addr[] = {0xaaaa, 0, 0, 0, 0, 0, 0, 0x1};

void connect_udp_server();
void uip_debug_ipaddr_print(const uip_ipaddr_t *addr);

/*---------------------------------------------------------------------------*/
PROCESS(init_system_proc, "Init system process");
AUTOSTART_PROCESSES(&init_system_proc);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(init_system_proc, ev, data)
{
        char device_address[30];
        static struct etimer periodic_timer;
        uint8_t buf[5] = "TESTE";

        PROCESS_BEGIN();


        sprintf(device_id,"%02X%02X%02X%02X%02X%02X%02X%02X",
                linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],
                linkaddr_node_addr.u8[2],linkaddr_node_addr.u8[3],
                linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],
                linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);

        sprintf(device_address,"[%c%c%c%c]-Device-%s",device_id[12],device_id[13],device_id[14],device_id[15],device_id);
        connect_udp_server();
        etimer_set(&periodic_timer, 5 * CLOCK_SECOND);
        debug_os("No inicializado - %s\n", device_address);

        while (1) {
                // PROCESS_WAIT_EVENT();
                PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
                etimer_reset(&periodic_timer);
                debug_os("Enviando dados a central...");
                simple_udp_sendto(&broadcast_connection, buf, sizeof(buf), &server_addr);
        }
        PROCESS_END();
}
/*---------------------------------------------------------------------------*/

void cb_receive_udp(struct simple_udp_connection *c,
                    const uip_ipaddr_t *sender_addr,
                    uint16_t sender_port,
                    const uip_ipaddr_t *receiver_addr,
                    uint16_t receiver_port,
                    const uint8_t *data,
                    uint16_t datalen) {
        debug_os("########## UDP #########");
        printf("\nRecebido via UDP:%s\n",data);
}

void connect_udp_server(){
        uip_ip6addr(&server_addr,
                    central_addr[0],
                    central_addr[1],
                    central_addr[2],
                    central_addr[3],
                    central_addr[4],
                    central_addr[5],
                    central_addr[6],
                    central_addr[7]);
        debug_os("Endereco do servidor IPv6: ");
        uip_debug_ipaddr_print(&server_addr);

        simple_udp_register(&broadcast_connection,
                            UDP_PORT_OUT,
                            &server_addr,
                            UDP_PORT_CENTRAL,
                            cb_receive_udp);

}
