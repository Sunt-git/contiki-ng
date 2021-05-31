#ifndef CONTIKI_CONF_H
#define CONTIKI_CONF_H

/* Possibly standard includes here, e.g. stdint.h */
/*---------------------------------------------------------------------------*/
/* Include Project Specific conf */
#ifdef PROJECT_CONF_PATH
#include PROJECT_CONF_PATH
#endif /* PROJECT_CONF_PATH */
/*---------------------------------------------------------------------------*/
//#include "my-platform-def.h"
#include "arm-def.h"
/*---------------------------------------------------------------------------*/
/* 
 * All user configuration to go here
 */
/*---------------------------------------------------------------------------*/
#ifdef WITH_UIP6

#define LINKADDR_CONF_SIZE              8

#define UIP_CONF_LL_802154              1
#define UIP_CONF_LLH_LEN                0

#define UIP_CONF_ROUTER                 1
#ifndef UIP_CONF_IPV6_RPL
#define UIP_CONF_IPV6_RPL               1
#endif /* UIP_CONF_IPV6_RPL */

/* configure number of neighbors and routes */
#ifndef UIP_CONF_DS6_NBR_NBU
#define UIP_CONF_DS6_NBR_NBU     30
#endif /* UIP_CONF_DS6_NBR_NBU */
#ifndef UIP_CONF_DS6_ROUTE_NBU
#define UIP_CONF_DS6_ROUTE_NBU   30
#endif /* UIP_CONF_DS6_ROUTE_NBU */

#define UIP_CONF_ND6_SEND_RA		0
#define UIP_CONF_ND6_REACHABLE_TIME     600000
#define UIP_CONF_ND6_RETRANS_TIMER      10000

#define UIP_CONF_IPV6                   1
#ifndef UIP_CONF_IPV6_QUEUE_PKT
#define UIP_CONF_IPV6_QUEUE_PKT         0
#endif /* UIP_CONF_IPV6_QUEUE_PKT */
#define UIP_CONF_IPV6_CHECKS            1
#define UIP_CONF_IPV6_REASSEMBLY        0
#define UIP_CONF_NETIF_MAX_ADDRESSES    3
#define UIP_CONF_ND6_MAX_PREFIXES       3
#define UIP_CONF_ND6_MAX_NEIGHBORS      4
#define UIP_CONF_ND6_MAX_DEFROUTERS     2
#define UIP_CONF_IP_FORWARD             0
#ifndef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE		1000
#endif

#define SICSLOWPAN_CONF_COMPRESSION_IPV6        0
#define SICSLOWPAN_CONF_COMPRESSION_HC1         1
#define SICSLOWPAN_CONF_COMPRESSION_HC01        2
#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_HC06
#ifndef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG                    1
#define SICSLOWPAN_CONF_MAXAGE                  8
#endif /* SICSLOWPAN_CONF_FRAG */
#define SICSLOWPAN_CONF_CONVENTIONAL_MAC	1
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS       2
#ifndef SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS
#define SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS   5
#endif /* SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS */
#else /* WITH_UIP6 */
#define UIP_CONF_IP_FORWARD      1
#define UIP_CONF_BUFFER_SIZE     1000
#endif /* WITH_UIP6 */

#define UIP_CONF_LL_802154 0
/* Include CPU-related configuration */
/#include "stm32f4-conf.h"
/*---------------------------------------------------------------------------*/
#endif /* CONTIKI_CONF_H */
