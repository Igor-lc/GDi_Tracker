

#ifndef CONTROL_H
#define CONTROL_H

#include "user_types.h"

#define PACKET_BUF_SIZE    1000
#define POINTS_TO_SEND_BUF_SIZE ((PACKET_BUF_SIZE/10)*9)
#define PAGES_BUF_ADDR    ((uint32_t)&fm_ptr->pages_buf)
#define POINTS_BUF_ADDR   ((uint32_t)&fm_ptr->points_buf)
#define POINTS_TOTAL_ADDR ((uint32_t)&fm_ptr->points_total)
#define CONFIG_GPRS_ADDR ((uint32_t)&fm_ptr->conf_gprs[0])
#define CONFIG_SERVER_ADDR         ((uint32_t)&fm_ptr->conf_server[0])
#define CONFIG_LOG_ADDR            ((uint32_t)&fm_ptr->log)
#define CONFIG_NAV_FILTER_ADDR     ((uint32_t)&fm_ptr->conf_nav_filter)
#define CONFIG_PACKET_SEND_ADDR    ((uint32_t)&fm_ptr->conf_packet_send)
#define CONFIG_RS485_SENSOR_ADDR   ((uint32_t)&fm_ptr->conf_rs485_sensor[0])
#define CONFIG_RS485_ADDR          ((uint32_t)&fm_ptr->conf_rs485)
#define CONFIG_SECUR_ADDR          ((uint32_t)&fm_ptr->conf_secur)
#define CONFIG_HARDWARE_ADDR       ((uint32_t)&fm_ptr->conf_hardware)
#define CONFIG_FIN_ADDR            ((uint32_t)&fm_ptr->conf_fin)
#define CONFIG_DOUT_ADDR           ((uint32_t)&fm_ptr->conf_dout)
#define CONFIG_SIM_ADDR           ((uint32_t)&fm_ptr->conf_sim)


							
enum ControlSignals
{
	CONTROL_SEND_PACKET_OK_SIG=MAX_PUB_SIG,
	CONTROL_SEND_PACKET_ERROR_SIG,
	CONTROL_BAD_PACKET_SIG,
	CONTROL_MODEM_READY_SIG,
	CONTROL_TIMEOUT_SIG,
	CONTROL_UPDATE_START_SIG,
	CONTROL_UPDATE_END_SIG,
	//-----------------------------
	CONTROL_DISK_READY_SIG,
  CONTROL_FRAM_ERROR_SIG,
	CONTROL_FLASH_ERROR_SIG,
	CONTROL_PRIO_SEND_SIG,
	CONTROL_MAKE_PACKET_ERROR_SIG,
	//-----------------------------------
	CONTROL_CONFIG_WRITE_OK_SIG,
	CONTROL_UPDATE_CONNECT_OK_SIG,
	CONTROL_UPD_TRANSPORT_LEVEL_ERROR_SIG,
	CONTROL_UPD_PACKET_LEVEL_ERROR_SIG,
	CONTROL_TCP_DATA_SIG,
	CONTROL_MODEM_IS_OFF_SIG,
	CONTROL_DISK_IS_OFF_SIG,
	CONTROL_ERROR_SMS_PROCESSED_SIG,
	//----------------------------------------------------
	CONTROL_UPDATE_BREAK_SIG,
	//------FATAL ERRORS---------------------------
	CONTROL_REBOOT_SIG,
};

extern QEvt const CtrlRebootEvt;
extern const FramMemoryType *fm_ptr;
extern void Control_ctor(void);
extern QActive * const AO_Control;
#endif

