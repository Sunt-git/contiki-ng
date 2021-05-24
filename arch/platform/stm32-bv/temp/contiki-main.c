// Platform specific includes
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

// Contiki includes
#include "contiki.h"
#include <stdio.h>
#include <string.h>

#include "dev/cc2520/cc2520.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/slip.h"
#include "dev/watchdog.h"
#include "dev/xmem.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"

#if WITH_UIP6
#include "net/ipv6/uip-ds6.h"
#endif /* WITH_UIP6 */

#include "net/rime/rime.h"

#include "sys/autostart.h"

#if UIP_CONF_ROUTER

#ifndef UIP_ROUTER_MODULE
#ifdef UIP_CONF_ROUTER_MODULE
#define UIP_ROUTER_MODULE UIP_CONF_ROUTER_MODULE
#else /* UIP_CONF_ROUTER_MODULE */
#define UIP_ROUTER_MODULE rimeroute
#endif /* UIP_CONF_ROUTER_MODULE */
#endif /* UIP_ROUTER_MODULE */

extern const struct uip_router UIP_ROUTER_MODULE;
#endif /* UIP_CONF_ROUTER */

#ifndef WITH_UIP
#define WITH_UIP 0
#endif

#if WITH_UIP
#include "net/uip.h"
#include "net/uip-fw.h"
#include "net/uip-fw-drv.h"
#include "net/uip-over-mesh.h"
static struct uip_fw_netif slipif =
  {UIP_FW_NETIF(192,168,1,2, 255,255,255,255, slip_send)};
static struct uip_fw_netif meshif =
  {UIP_FW_NETIF(172,16,0,0, 255,255,0,0, uip_over_mesh_send)};

#endif /* WITH_UIP */

#define UIP_OVER_MESH_CHANNEL 8
#if WITH_UIP
static uint8_t is_gateway;
#endif /* WITH_UIP */

#ifdef EXPERIMENT_SETUP
#include "experiment-setup.h"
#endif

void init_platform(void);
void uip_log(char *msg) { puts(msg); }

#ifndef RF_CHANNEL
#define RF_CHANNEL              26
#endif

#ifndef NODE_ID
#define NODE_ID	0x03
#endif /* NODE_ID */

static unsigned short node_id;

static void set_rime_addr(void) {
	linkaddr_t n_addr;
	int i;

	memset(&n_addr, 0, sizeof(linkaddr_t));

	// Set node address
#if UIP_CONF_IPV6
	n_addr.u8[7] = node_id & 0xff;
	n_addr.u8[6] = node_id >> 8;
#else
	n_addr.u8[0] = node_id & 0xff;
	n_addr.u8[1] = node_id >> 8;
#endif

	linkaddr_set_node_addr(&n_addr);
	printf("Rime started with address ");
	for(i = 0; i < sizeof(n_addr.u8) - 1; i++) {
		printf("%d.", n_addr.u8[i]);
	}
	printf("%d\r\n", n_addr.u8[i]);
}

#if !PROCESS_CONF_NO_PROCESS_NAMES
static void print_processes(struct process * const processes[]) {
	/*  const struct process * const * p = processes;*/
	printf("Starting");
	while(*processes != NULL) {
		printf(" '%s'", (*processes)->name);
		processes++;
	}
	printf("\r\n");
}
#endif /* !PROCESS_CONF_NO_PROCESS_NAMES */

#if WITH_UIP
static void set_gateway(void) {
	if(!is_gateway) {
		leds_on(LEDS_RED);
		//printf("%d.%d: making myself the IP network gateway.\n\n",
		//rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
		//printf("IPv4 address of the gateway: %d.%d.%d.%d\n\n",
		//uip_ipaddr_to_quad(&uip_hostaddr));
		
		/*
		uip_over_mesh_set_gateway(&rimeaddr_node_addr);
		uip_over_mesh_make_announced_gateway();
		is_gateway = 1;
		*/
	}
}
#endif /* WITH_UIP */

// Variables
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

// Private functions
static void platform_init();

int main() {
	platform_init();
	printf("Initialising\r\n");
	
	node_id = NODE_ID;

	process_init();
	process_start(&etimer_process, NULL);

	ctimer_init();

	set_rime_addr();

	cc2520_init();
	{
		uint8_t longaddr[8];
		uint16_t shortaddr;

		shortaddr = (linkaddr_node_addr.u8[0] << 8) +
			linkaddr_node_addr.u8[1];
		memset(longaddr, 0, sizeof(longaddr));
		linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);

		printf("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x \r\n",
			longaddr[0], longaddr[1], longaddr[2], longaddr[3],
			longaddr[4], longaddr[5], longaddr[6], longaddr[7]);

		cc2520_set_pan_addr(IEEE802154_PANID, shortaddr, longaddr);
	}
	
	cc2520_set_channel(RF_CHANNEL);
	cc2520_set_txpower(0xF7); // 5 dBm
	printf("TX Power: %X\r\n", cc2520_get_txpower());

	printf(CONTIKI_VERSION_STRING " started. ");
	if(node_id > 0) {
		printf("Node id is set to %u.\r\n", node_id);
	} else {
		printf("Node id is not set.\r\n");
	}

#if WITH_UIP6
	/* memcpy(&uip_lladdr.addr, ds2411_id, sizeof(uip_lladdr.addr)); */
	memcpy(&uip_lladdr.addr, linkaddr_node_addr.u8,
		UIP_LLADDR_LEN > LINKADDR_SIZE ? LINKADDR_SIZE : UIP_LLADDR_LEN);

	/* Setup nullmac-like MAC for 802.15.4 */
/*   sicslowpan_init(sicslowmac_init(&cc2520_driver)); */
/*   printf(" %s channel %u\n", sicslowmac_driver.name, RF_CHANNEL); */

	/* Setup X-MAC for 802.15.4 */
	queuebuf_init();
	NETSTACK_RDC.init();
	NETSTACK_MAC.init();
	NETSTACK_NETWORK.init();

	printf("%s %s, channel check rate %i Hz, radio channel %u\r\n",
		NETSTACK_MAC.name, NETSTACK_RDC.name,
		CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0 ? 1:
		NETSTACK_RDC.channel_check_interval()),
		RF_CHANNEL);

	process_start(&tcpip_process, NULL);

	printf("Tentative link-local IPv6 address ");
	{
		uip_ds6_addr_t *lladdr;
		int i;
		lladdr = uip_ds6_get_link_local(-1);
		for(i = 0; i < 7; ++i) {
			printf("%02x%02x:", lladdr->ipaddr.u8[i * 2],
				lladdr->ipaddr.u8[i * 2 + 1]);
		}
		printf("%02x%02x\r\n", lladdr->ipaddr.u8[14], lladdr->ipaddr.u8[15]);
	}

	if(!UIP_CONF_IPV6_RPL) {
		uip_ipaddr_t ipaddr;
		int i;
		uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
		uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
		uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
		printf("Tentative global IPv6 address ");
		for(i = 0; i < 7; ++i) {
			printf("%02x%02x:",
				ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
		}
		printf("%02x%02x\n",
			ipaddr.u8[7 * 2], ipaddr.u8[7 * 2 + 1]);
	}

#else /* WITH_UIP6 */

	NETSTACK_RDC.init();
	NETSTACK_MAC.init();
	NETSTACK_NETWORK.init();

	printf("%s %s, channel check rate %i Hz, radio channel %u\r\n",
		NETSTACK_MAC.name, NETSTACK_RDC.name,
		CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0? 1:
		NETSTACK_RDC.channel_check_interval()),
		RF_CHANNEL);
#endif /* WITH_UIP6 */

#if !WITH_UIP && !WITH_UIP6
	//uart1_set_input(serial_line_input_byte);
	//serial_line_init();
#endif

#if PROFILE_CONF_ON
	profile_init();
#endif /* PROFILE_CONF_ON */

	leds_off(LEDS_GREEN);

#if TIMESYNCH_CONF_ENABLED
	timesynch_init();
	timesynch_set_authority_level((linkaddr_node_addr.u8[0] << 4) + 16);
#endif /* TIMESYNCH_CONF_ENABLED */

#if WITH_UIP
	process_start(&tcpip_process, NULL);
	process_start(&uip_fw_process, NULL);	/* Start IP output */
	process_start(&slip_process, NULL);

	slip_set_input_callback(set_gateway);

	{
		uip_ipaddr_t hostaddr, netmask;

		uip_init();

		uip_ipaddr(&hostaddr, 172,16,
			linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
		uip_ipaddr(&netmask, 255,255,0,0);
		uip_ipaddr_copy(&meshif.ipaddr, &hostaddr);

		uip_sethostaddr(&hostaddr);
		uip_setnetmask(&netmask);
		uip_over_mesh_set_net(&hostaddr, &netmask);
		/*    uip_fw_register(&slipif);*/
		uip_over_mesh_set_gateway_netif(&slipif);
		uip_fw_default(&meshif);
		uip_over_mesh_init(UIP_OVER_MESH_CHANNEL);
		printf("uIP started with IP address %d.%d.%d.%d\r\n",
			uip_ipaddr_to_quad(&hostaddr));
	}
#endif /* WITH_UIP */

	energest_init();
	ENERGEST_ON(ENERGEST_TYPE_CPU);

	watchdog_start();
	/* Stop the watchdog */
	watchdog_stop();

#if !PROCESS_CONF_NO_PROCESS_NAMES
	print_processes(autostart_processes);
#else /* !PROCESS_CONF_NO_PROCESS_NAMES */
	printf("\r\n"); /* include putchar() */
#endif /* !PROCESS_CONF_NO_PROCESS_NAMES */
	autostart_start(autostart_processes);

	/*
	 * This is the scheduler loop.
	 */
	for(;;) {
		do {
			// Reset watchdog.
        	watchdog_periodic();
		} while(process_run() > 0);
		
		ENERGEST_OFF(ENERGEST_TYPE_CPU);
		watchdog_stop();    
		ENERGEST_ON(ENERGEST_TYPE_LPM);
		// Go to sleep until an interrupt occurs (and debug isn't enabled)
    	
#if DEBUG_BUILD != 1
		asm("wfi"::);
#endif
			
		// We are awake.
		watchdog_start();
		ENERGEST_OFF(ENERGEST_TYPE_LPM);
		ENERGEST_ON(ENERGEST_TYPE_CPU);
	}
	
	return 0;
}

#if LOG_CONF_ENABLED
void log_message(char *m1, char *m2) {
	printf("%s%s\r\n", m1, m2);
}
#endif /* LOG_CONF_ENABLED */

static void platform_init() {
	// ------------- USB -------------- //
	USBD_Init(&USB_OTG_dev,
	          USB_OTG_FS_CORE_ID,
	          &USR_desc,
	          &USBD_CDC_cb,
	          &USR_cb);
	          
	/*
	 * Disable STDOUT buffering. Otherwise nothing will be printed
	 * before a newline character or when the buffer is flushed.
	 */
	setbuf(stdout, NULL);
	
	clock_init();
	leds_init();
	
#if WITH_UIP
	slip_arch_init(115200);
#endif /* WITH_UIP */
	
	rtimer_init();
}

/*
 * Dummy function to avoid a compiler error
 */
void _init() {

}

