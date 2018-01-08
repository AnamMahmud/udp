/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "net/uip-udp-packet.h"
#include "net/rpl/rpl.h"
#include "dev/serial-line.h"
#include "cc2420.h"
#if CONTIKI_TARGET_Z1
#include "dev/uart0.h"
#else
#include "dev/uart1.h"
#endif
#include "collect-common.h"
#include "collect-view.h"

#include <stdio.h>
#include <string.h>

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
signed char Parents_RSSI;
int current;
/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process, &collect_common_process);
/*---------------------------------------------------------------------------*/
void
collect_common_set_sink(void)
{
  /* A udp client can never become sink */
}
/*---------------------------------------------------------------------------*/

void
collect_common_net_print(void)
{
  rpl_dag_t *dag;
  uip_ds6_route_t *r;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag->preferred_parent != NULL) {
    PRINTF("Preferred parent: ");
    PRINT6ADDR(rpl_get_parent_ipaddr(dag->preferred_parent));
    PRINTF("\n");
  }
  for(r = uip_ds6_route_head();
      r != NULL;
      r = uip_ds6_route_next(r)) {
    PRINT6ADDR(&r->ipaddr);
  }
  PRINTF("---\n");
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  if(uip_newdata()) {
    /* Ignore incoming data */
char *str;

  if(uip_newdata()) {
    str = uip_appdata;
    str[uip_datalen()] = '\0';
    printf("DATA recv '%s'\n", str);
  }
  }
}
struct energy_time {
  unsigned short source;
  long cpu;
  long lpm;
  long transmit;
  long listen;
};
static struct energy_time last;
static struct energy_time diff;
/*---------------------------------------------------------------------------*/
int Tx;

Tx=0;
int 
Tx_power_tuning(signed char Parent_RSSI)
{
//cc2420_set_txpower(31);
//-----------------------------------------------------------//
if ((Parent_RSSI)>=(-61) && (Parent_RSSI)<(-0) ){

Tx=3;

} 
 
else if ((Parent_RSSI)>(-71) && (Parent_RSSI)<=(-61) ){
Tx=7;
}
else if ((Parent_RSSI)>(-76) && (Parent_RSSI)<=(-71) ){
 Tx=11;
}

else if ((Parent_RSSI)>(-79) && (Parent_RSSI)<=(-76) ){
 Tx=15;
}

else if ((Parent_RSSI)>(-81) && (Parent_RSSI)<=(-79) ){
 Tx=19;
}

else if ((Parent_RSSI)>(-83) && (Parent_RSSI)<=(-81) ){
 Tx=23;
}

else if ((Parent_RSSI)>(-85) && (Parent_RSSI)<=(-83) ){
 Tx=27;
}

else {
  Tx=31;
}
//--------------------------------------------------------//

return Tx;
}
/*
 /* struct collect_neighbor *n; */
  uint16_t parent_etx;
  uint16_t rtmetric;
  uint16_t num_neighbors;
  uint16_t beacon_interval;
  rpl_parent_t *preferred_parent;
  rimeaddr_t parent;
  rpl_dag_t *dag;


/**/

void
collect_common_send(void)
{
 

    static count;
  static uint8_t seqno;
  struct {
    uint8_t seqno;
    uint8_t for_alignment;
    struct collect_view_data_msg msg;
  } msg;
  /* struct collect_neighbor *n; */
  uint16_t parent_etx;
  uint16_t rtmetric;
  uint16_t num_neighbors;
  uint16_t beacon_interval;
  rpl_parent_t *preferred_parent;
  rimeaddr_t parent;
  rpl_dag_t *dag;

  if(client_conn == NULL) {
    /* Not setup yet */
    return;
  }
  memset(&msg, 0, sizeof(msg));
  seqno++;
  if(seqno == 0) {
    /* Wrap to 128 to identify restarts */
    seqno = 128;
  }
  msg.seqno = seqno;
 
  rimeaddr_copy(&parent, &rimeaddr_null);
  parent_etx = 0;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag != NULL) {
    preferred_parent = dag->preferred_parent;
    if(preferred_parent != NULL) {
      uip_ds6_nbr_t *nbr;
      nbr = uip_ds6_nbr_lookup(rpl_get_parent_ipaddr(preferred_parent));
      if(nbr != NULL) {
        
        /* Use parts of the IPv6 address as the parent address, in reversed byte order. */
        parent.u8[RIMEADDR_SIZE - 1] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 2];
        parent.u8[RIMEADDR_SIZE - 2] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 1];
        parent_etx = rpl_get_parent_rank((rimeaddr_t *) uip_ds6_nbr_get_ll(nbr)) / 2;
      }
    }
    rtmetric = dag->rank;
    beacon_interval = (uint16_t) ((2L << dag->instance->dio_intcurrent) / 1000);
    num_neighbors = RPL_PARENT_COUNT(dag);
  } else {
    rtmetric = 0;
    beacon_interval = 0;
    num_neighbors = 0;
  }

unsigned char rss=(unsigned)preferred_parent->mc.obj.rssi;

signed char Parent_RSSI=-1*rss;
//printf("\n Parents_ETX =%u\n", (unsigned)preferred_parent->mc.obj.rssi);
//printf("Parent_RSSI =%d\n", Parent_RSSI);


//----------------- Transmission Power Tuning ---------------------------------------//
//if (count==0){
	if (Parent_RSSI != 0 && Parent_RSSI>= -87){

		cc2420_set_txpower(Tx_power_tuning(Parent_RSSI));
//count++;
	} else {
		cc2420_set_txpower(31);
		printf("tx levdel= 31\n");
            //   count=0;
	  }

//}
//printf("tx level =%d\n", tx_power);
//
printf("Sending tx-level =%d\n", cc2420_get_txpower());

  //cc2420_set_txpower(27);
  /* num_neighbors = collect_neighbor_list_num(&tc.neighbor_list); */

 // printf("\n parent_etx=%u\n", (unsigned)preferred_parent->mc.obj.rssi);
  collect_view_construct_message(&msg.msg, &parent,
                                 parent_etx, rtmetric,
                                 num_neighbors, beacon_interval);
  //printf("Sending tx-level =%d\n", cc2420_get_txpower());



  uip_udp_packet_sendto(client_conn, &msg, sizeof(msg),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
/*int tx_power;
     tx_power= cc2420_get_txpower();
diff.cpu = energest_type_time(ENERGEST_TYPE_CPU) - last.cpu;
    diff.lpm = energest_type_time(ENERGEST_TYPE_LPM) - last.lpm;
    diff.transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT) - last.transmit;
    diff.listen = energest_type_time(ENERGEST_TYPE_LISTEN) - last.listen;
    last.cpu = energest_type_time(ENERGEST_TYPE_CPU);
    last.lpm = energest_type_time(ENERGEST_TYPE_LPM);
    last.transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
    last.listen = energest_type_time(ENERGEST_TYPE_LISTEN);*/
//printf("sending rssi= %d\n",(cc2420_last_rssi-45));
    //printf("seq. no=(%u), Tx time x rtimer_tics= %lu\n",msg.seqno,diff.transmit);
//printf("tx_time= %li\n", diff.transmit,current);
//PRINT6ADDR(rpl_get_parent_ipaddr(preferred_parent));
     
//printf("tx level =%d\n", tx_power);
//printf("Sending tx-level =%d\n", tx_power);
	/*	if (tx_power==31){
		current=17.4*10;
		}  else if(tx_power==27){
		current=16.5*10;
		   }else if(tx_power==23){
		   current=15.2*10;
		    }else if(tx_power==19){
		 current=13.9*10;
		}else if(tx_power==15){
		 current=12.5*10;
		}else if(tx_power==11){
		 current=11.2*10;
		}else if(tx_power==7){
		current=9.9*10;
		}
		else {
		current=8.5;
		}
printf("%li\t %d\n ", diff.transmit,current);*/
//printf("current consumption =%d \n", current);
//printf("Ticks per second: %u\n", RTIMER_SECOND);
    


  
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_init(void)
{
#if CONTIKI_TARGET_Z1
  uart0_set_input(serial_line_input_byte);
#else
  uart1_set_input(serial_line_input_byte);
#endif
  serial_line_init();
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  /* set server address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  PROCESS_BEGIN();

  PROCESS_PAUSE();

  set_global_address();
 // cc2420_set_txpower(31);
  PRINTF("UDP client process started\n");

  print_local_addresses();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
        UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));
    
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      //printf("\n rssi= %d\n",cc2420_last_rssi);
      tcpip_handler();
     
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
