
#ifndef CONFIG_H
#define CONFIG_H

#include "user_types.h"

extern const ConfigGprsType config_gprs_default[];
extern const ConfigServerType config_server_default[];

extern uint32_t ConfigPacketMaker(uint8_t* ptr);
extern ConfigCommandType sms_cmd_tbl[];
extern ConfigGprsType *gprs_ptr;
extern ConfigNavFilterType conf_nav_filter;
extern ConfigRS485SensorType conf_rs485_sensor[];
extern ConfigType conf;

#endif



