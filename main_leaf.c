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
#include <math.h>
#include "contiki.h"
#include "contiki-net.h"
#include "dev/serial-line.h"
#include "debug_info.h"
#include "net/ipv6/uip-ds6.h"
#include "sys/etimer.h"
#include "simple-udp.h"
#include "dev/cc26xx-uart.h"
#include "core/net/ipv6/sicslowpan.h"
#include "dev/batmon-sensor.h"
#include "dev/adc-sensor.h"
#include "core/lib/sensors.h"
#include "ti-lib.h"
#include "driverlib/aux_adc.h"
#include "driverlib/aux_wuc.h"

#define UDP_PORT_CENTRAL 7878
#define UDP_PORT_OUT 5555

#define CLOCK_MINUTE CLOCK_SECOND*60

static struct simple_udp_connection broadcast_connection;
static uip_ipaddr_t server_addr;
static uint16_t central_addr[] = {0xaaaa, 0, 0, 0, 0, 0, 0, 0x1};

void connect_udp_server();
void uip_debug_ipaddr_print(const uip_ipaddr_t *addr);
int serial_cb_rx(unsigned char c);
uint16_t readBat(void);
uint16_t readADC(void);
void getDecStr(uint8_t* str, uint8_t len, uint32_t val);
void formatDataFeedback(uint8_t *buffer, uint8_t *buff_udp);
uint16_t readTempNTC(void);

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
        static struct etimer periodic_timer;

        //Inicializando rede IPv6
        uint8_t buff_udp[50],
                device_address[30],
                device_id[17];
        sprintf((char *)device_id,"%02X%02X%02X%02X%02X%02X%02X%02X",
                linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],
                linkaddr_node_addr.u8[2],linkaddr_node_addr.u8[3],
                linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],
                linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);
        sprintf((char *)device_address,"[%c%c%c%c]-Device-%s",device_id[12],device_id[13],device_id[14],device_id[15],device_id);
        connect_udp_server();
        etimer_set(&periodic_timer, CLOCK_SECOND);
        debug_os("Dispositivo inicializado - %s\n", device_address);

        //Inicializando RX-UART
        cc26xx_uart_set_input(serial_cb_rx);

        //Inicializando ADC
        SENSORS_ACTIVATE(batmon_sensor);
        adc_sensor.configure(SENSORS_ACTIVE, 1);
        adc_sensor.configure(ADC_SENSOR_SET_CHANNEL, ADC_COMPB_IN_AUXIO7);

        //formatDataFeedback(buff_udp, buff_udp);
        //simple_udp_sendto(&broadcast_connection, buff_udp, strlen((const char *)buff_udp), &server_addr);

        while (1) {
                PROCESS_YIELD();

                if (etimer_expired(&periodic_timer)) {
                        etimer_reset(&periodic_timer);
                        formatDataFeedback(buff_udp, buff_udp);
                        debug_os("Enviando dados UDP a border router...");
                        simple_udp_sendto(&broadcast_connection, buff_udp, strlen((const char *)buff_udp), &server_addr);
                }
        }
        PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void formatDataFeedback(uint8_t *buffer, uint8_t *buff_udp){
        static uint16_t ADC_IO7, ADCBat;
        int measurement;
        int def_rt_rssi = sicslowpan_get_last_rssi();
        ADC_IO7 = readADC();
        ADCBat = readBat();

        // uint32_t teste2 = 8400/0.005412;
        // double teste = 8400/0.005412;
        // printf("\nln(8400/54.12) = %d\n",log(teste));
        // printf("\nln(8400/54.12) = %d\n",log(teste2));
        // value = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);

        measurement = adc_sensor.value(ADC_SENSOR_VALUE);
        printf("\nADC_Sensor_Driver=%d", measurement);
        printf("\nADC Manual - IO7=%d", ADC_IO7);

        // sprintf((char *)buff_udp, "[%02X%02X/%d/%d/%d]",linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7], ADCBat, ADC_IO7, def_rt_rssi);
        // printf("\n%s - len:%d bytes",buff_udp, (unsigned int)strlen((const char *)buff_udp));
        // Formato JSON, não envio para poupar bateria mas caso seja necessário
        //sprintf((char *)buffer, "{'dev':'%02X%02X','bat':'%dmV','ad':'%dmV'}",linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7], ADCBat, ADC_IO7);
        //printf("\n%s - Tamanho:%d bytes",buffer, (unsigned int)strlen((const char *)buffer));
}

void getDecStr(uint8_t* str, uint8_t len, uint32_t val){
        uint8_t i;

        for(i=1; i<=len; i++)
        {
                str[len-i] = (uint8_t) ((val % 10UL) + '0');
                val/=10;
        }

        str[i-1] = '\0';
}

int serial_cb_rx(unsigned char c){
        debug_os("RX Serial: %c", c);
        return 1;
}

uint16_t readBat(void){
        uint16_t singleSample;
        ti_lib_aon_wuc_aux_wakeup_event(AONWUC_AUX_WAKEUP);
        while(!(ti_lib_aon_wuc_power_status_get() & AONWUC_AUX_POWER_ON)) ;
        ti_lib_aux_wuc_clock_enable(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK);
        while(ti_lib_aux_wuc_clock_status(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK) != AUX_WUC_CLOCK_READY) ;
        AUXADCSelectInput(ADC_COMPB_IN_VDDS);
        AUXADCEnableSync(AUXADC_REF_FIXED,  AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL);
        AUXADCGenManualTrigger();
        singleSample = AUXADCReadFifo();
        AUXADCDisable();
        return singleSample;
}

uint16_t readADC(void){
        uint16_t singleSample;
        ti_lib_aon_wuc_aux_wakeup_event(AONWUC_AUX_WAKEUP);
        while(!(ti_lib_aon_wuc_power_status_get() & AONWUC_AUX_POWER_ON)) ;
        ti_lib_aux_wuc_clock_enable(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK);
        while(ti_lib_aux_wuc_clock_status(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK) != AUX_WUC_CLOCK_READY) ;
        AUXADCSelectInput(ADC_COMPB_IN_AUXIO7);
        AUXADCEnableSync(AUXADC_REF_FIXED,  AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL);
        AUXADCGenManualTrigger();
        singleSample = AUXADCReadFifo();
        AUXADCDisable();
        return singleSample;
}

uint16_t readTempNTC(void){
        uint16_t ADCSamples[10];
        for (size_t i = 0; i < 10; i++)
                ADCSamples[i] = readADC();
        //
        // double result;
        // result = log(1.5);
        // printf("\nln(1.5) = %2.2f\n",result);
        // Fundo de escala 4.3V - 4096 BITS

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
