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
#include "dev/serial-line.h"
#include "debug_info.h"
#include "net/ipv6/uip-ds6.h"
#include "sys/etimer.h"
#include "simple-udp.h"
#include <stdlib.h>
#include <string.h>
#include "dev/cc26xx-uart.h"
//ADC Libs
#include "ti-lib.h"
#include "driverlib/aux_adc.h"
#include "driverlib/aux_wuc.h"

#define UDP_PORT_CENTRAL 7878
#define UDP_PORT_OUT 5555

static struct simple_udp_connection broadcast_connection;
static uip_ipaddr_t server_addr;
static char device_id[17];
static uint16_t central_addr[] = {0xaaaa, 0, 0, 0, 0, 0, 0, 0x1};
static uint16_t adcSample;

void connect_udp_server();
void uip_debug_ipaddr_print(const uip_ipaddr_t *addr);
int serial_cb_rx(unsigned char c);
uint16_t readADC(void);

/*---------------------------------------------------------------------------*/
PROCESS(init_system_proc, "Init system process");
AUTOSTART_PROCESSES(&init_system_proc);
/*---------------------------------------------------------------------------*/
// PROCESS(display_teste, "Teste de processos");
//
// PROCESS_THREAD(display_teste, ev, data){
//         PROCESS_BEGIN();
//         static struct etimer periodic_timer2;
//         etimer_set(&periodic_timer2, 4*CLOCK_SECOND);
//         printf("Inicio do paralelo");
//         while(1) {
//                 PROCESS_WAIT_EVENT();
//                 printf("\n\rTocando em paralelo");
//                 etimer_reset(&periodic_timer2);
//         }
//         PROCESS_END();
// }

PROCESS_THREAD(init_system_proc, ev, data){
        PROCESS_BEGIN();
        // process_start(&display_teste, NULL);

        char device_address[30];
        static struct etimer periodic_timer;
        uint8_t buf[5] = "TESTE";
        cc26xx_uart_set_input(serial_cb_rx);
        sprintf(device_id,"%02X%02X%02X%02X%02X%02X%02X%02X",
                linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],
                linkaddr_node_addr.u8[2],linkaddr_node_addr.u8[3],
                linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],
                linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);
        sprintf(device_address,"[%c%c%c%c]-Device-%s",device_id[12],device_id[13],device_id[14],device_id[15],device_id);
        connect_udp_server();
        etimer_set(&periodic_timer, CLOCK_SECOND);
        debug_os("No inicializado - %s\n", device_address);

        while (1) {
                PROCESS_YIELD();

                if (etimer_expired(&periodic_timer)) {
                        etimer_reset(&periodic_timer);
                        adcSample = readADC();
                        debug_os("Enviando dados UDP - Tensao:[%d mV]",adcSample);
                        simple_udp_sendto(&broadcast_connection, buf, sizeof(buf), &server_addr);
                }
        }
        PROCESS_END();
}
/*---------------------------------------------------------------------------*/

int serial_cb_rx(unsigned char c){
        debug_os("RX Serial: %c", c);
        return 1;
}

uint16_t readADC(void ){
  uint16_t singleSample;

  ti_lib_aon_wuc_aux_wakeup_event(AONWUC_AUX_WAKEUP);
  while(!(ti_lib_aon_wuc_power_status_get() & AONWUC_AUX_POWER_ON))
  { }

  // Enable clock for ADC digital and analog interface (not currently enabled in driver)
  // Enable clocks
  ti_lib_aux_wuc_clock_enable(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK);
  while(ti_lib_aux_wuc_clock_status(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK) != AUX_WUC_CLOCK_READY)
  { }

  // printf("clock selected\r\n");

  // Connect AUX IO7 (DIO23, but also DP2 on XDS110) as analog input.
  AUXADCSelectInput(ADC_COMPB_IN_AUXIO7);
  // printf("input selected\r\n");

  // Set up ADC range
  // AUXADC_REF_FIXED = nominally 4.3 V
  AUXADCEnableSync(AUXADC_REF_FIXED,  AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL);
  // printf("init adc --- OK\r\n");

  //Trigger ADC converting
  AUXADCGenManualTrigger();
  // printf("trigger --- OK\r\n");

  //reading adc value
  singleSample = AUXADCReadFifo();

  // printf("%d mv on ADC\r\n",singleSample);

  //shut the adc down
  AUXADCDisable();
  // printf("disable --- OK\r\n");
  return singleSample;
}

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
