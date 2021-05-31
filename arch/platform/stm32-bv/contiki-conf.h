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
/* Include CPU-related configuration */
/#include "stm32f4-conf.h"
/*---------------------------------------------------------------------------*/
#endif /* CONTIKI_CONF_H */
