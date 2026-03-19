/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

File    : RTT_ConfigDefaults.h
Purpose : Default configuration values of RTT

*/

#ifndef RTT_CONFIG_DEFAULTS_H
#define RTT_CONFIG_DEFAULTS_H

#ifdef __cplusplus
extern "C" {
#endif

//
// Default configuration values for RTT
//

#ifndef   SEGGER_RTT_MAX_NUM_UP_BUFFERS
  #define SEGGER_RTT_MAX_NUM_UP_BUFFERS             (2)
#endif

#ifndef   SEGGER_RTT_MAX_NUM_DOWN_BUFFERS
  #define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS           (2)
#endif

#ifdef __cplusplus
}
#endif

#endif
