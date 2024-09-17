/**
  ******************************************************************************
  * @file     vesc.c
  * @author   Du ShuYang
  * @version  V1.0
  * @date     11-23-2023
	* @brief      1. vesc_can ��������
  ******************************************************************************
  * @attention
  * 1.ע��id��ѡ��
  * 2.vesc ID��Ҫʹ����λ���趨
  * 3.��������Ҫ����λ��ͬ������
  * 
  ******************************************************************************
  */ 

#include "vesc_can.h"
#include "can.h"

static mc_values values;
static int fw_major;
static int fw_minor;
static float rotor_pos;
static mc_configuration mcconf;
static app_configuration appconf;
static float detect_cycle_int_limit;
static float detect_coupling_k;
static signed char detect_hall_table[8];
static signed char detect_hall_res;
static float dec_ppm;
static float dec_ppm_len;
static float dec_adc;
static float dec_adc_voltage;
static float dec_chuk;
vesc_motor_measure motor_bus1_vel_measure[3];
// Private functions
void send_packet_no_fwd(unsigned char *data, unsigned int len);
static void fwd_can_append(uint8_t *data, int32_t *ind);

// Function pointers
static void(*send_func)(unsigned char *data, unsigned int len) = 0;
static void(*forward_func)(unsigned char *data, unsigned int len) = 0;

// Function pointers for received data
static void(*rx_value_func)(mc_values *values) = 0;
static void(*rx_printf_func)(char *str) = 0;
static void(*rx_fw_func)(int major, int minor) = 0;
static void(*rx_rotor_pos_func)(float pos) = 0;
static void(*rx_mcconf_func)(mc_configuration *conf) = 0;
static void(*rx_appconf_func)(app_configuration *conf) = 0;
static void(*rx_detect_func)(float cycle_int_limit, float coupling_k,
		const signed char *hall_table, signed char hall_res) = 0;
static void(*rx_dec_ppm_func)(float val, float ms) = 0;
static void(*rx_dec_adc_func)(float val, float voltage) = 0;
static void(*rx_dec_chuk_func)(float val) = 0;
static void(*rx_mcconf_received_func)(void) = 0;
static void(*rx_appconf_received_func)(void) = 0;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
 CAN_filter_t defaultMask;
const unsigned short crc16_tab[] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
    0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
    0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
    0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
    0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
    0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
    0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
    0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
    0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
    0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
    0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
    0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
    0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
    0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
    0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
    0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
    0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
    0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
    0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
    0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
    0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
    0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
    0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
    0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
    0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };

		
		
		
void bldc_interface_process_packet(unsigned char *data, unsigned int len) {
	if (!len) {
		return;
	}

	if (forward_func) {
		forward_func(data, len);
		return;
	}

	int32_t ind = 0;
	int i = 0;
	unsigned char id = data[0];
	data++;
	len--;

	switch (id) {
	case COMM_FW_VERSION:
		if (len == 2) {
			ind = 0;
			fw_major = data[ind++];
			fw_minor = data[ind++];
		} else {
			fw_major = -1;
			fw_minor = -1;
		}
		break;

	case COMM_ERASE_NEW_APP:
	case COMM_WRITE_NEW_APP_DATA:
		// TODO
		break;

	case COMM_GET_VALUES:
		ind = 0;
		values.temp_mos1 = buffer_get_float16(data, 10.0, &ind);
		values.temp_mos2 = buffer_get_float16(data, 10.0, &ind);
		values.temp_mos3 = buffer_get_float16(data, 10.0, &ind);
		values.temp_mos4 = buffer_get_float16(data, 10.0, &ind);
		values.temp_mos5 = buffer_get_float16(data, 10.0, &ind);
		values.temp_mos6 = buffer_get_float16(data, 10.0, &ind);
		values.temp_pcb = buffer_get_float16(data, 10.0, &ind);
		values.current_motor = buffer_get_float32(data, 100.0, &ind);
		values.current_in = buffer_get_float32(data, 100.0, &ind);
		values.duty_now = buffer_get_float16(data, 1000.0, &ind);
		values.rpm = buffer_get_float32(data, 1.0, &ind);
		values.v_in = buffer_get_float16(data, 10.0, &ind);
		values.amp_hours = buffer_get_float32(data, 10000.0, &ind);
		values.amp_hours_charged = buffer_get_float32(data, 10000.0, &ind);
		values.watt_hours = buffer_get_float32(data, 10000.0, &ind);
		values.watt_hours_charged = buffer_get_float32(data, 10000.0, &ind);
		values.tachometer = buffer_get_int32(data, &ind);
		values.tachometer_abs = buffer_get_int32(data, &ind);
		values.fault_code = (mc_fault_code)data[ind++];

		if (rx_value_func) {
			rx_value_func(&values);
		}
		break;

	case COMM_PRINT:
		if (rx_printf_func) {
			data[len] = '\0';
			rx_printf_func((char*)data);
		}
		break;

	case COMM_SAMPLE_PRINT:
		// TODO
		break;

	case COMM_ROTOR_POSITION:
		ind = 0;
		rotor_pos = buffer_get_float32(data, 100000.0, &ind);

		if (rx_rotor_pos_func) {
			rx_rotor_pos_func(rotor_pos);
		}
		break;

	case COMM_EXPERIMENT_SAMPLE:
		// TODO
		break;

	case COMM_GET_MCCONF:
	case COMM_GET_MCCONF_DEFAULT:
		ind = 0;
		mcconf.pwm_mode = (mc_pwm_mode)data[ind++];
		mcconf.comm_mode = (mc_comm_mode)data[ind++];
		mcconf.motor_type = (mc_motor_type)data[ind++];
		mcconf.sensor_mode = (mc_sensor_mode)data[ind++];

		mcconf.l_current_max = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_current_min = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_in_current_max = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_in_current_min = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_abs_current_max = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_min_erpm = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_max_erpm = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_max_erpm_fbrake = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_max_erpm_fbrake_cc = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_min_vin = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_max_vin = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_battery_cut_start = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_battery_cut_end = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_slow_abs_current = data[ind++];
		mcconf.l_rpm_lim_neg_torque = data[ind++];
		mcconf.l_temp_fet_start = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_temp_fet_end = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_temp_motor_start = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_temp_motor_end = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_min_duty = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.l_max_duty = buffer_get_float32(data, 1000000.0, &ind);

		mcconf.sl_min_erpm = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_min_erpm_cycle_int_limit = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_max_fullbreak_current_dir_change = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_cycle_int_limit = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_phase_advance_at_br = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_cycle_int_rpm_br = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_bemf_coupling_k = buffer_get_float32(data, 1000.0, &ind);

		memcpy(mcconf.hall_table, data + ind, 8);
		ind += 8;
		mcconf.hall_sl_erpm = buffer_get_float32(data, 1000.0, &ind);

		mcconf.foc_current_kp = buffer_get_float32(data, 1e5, &ind);
		mcconf.foc_current_ki = buffer_get_float32(data, 1e5, &ind);
		mcconf.foc_f_sw = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_dt_us = buffer_get_float32(data, 1e6, &ind);
		mcconf.foc_encoder_inverted = data[ind++];
		mcconf.foc_encoder_offset = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_encoder_ratio = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sensor_mode = (mc_foc_sensor_mode)data[ind++];
		mcconf.foc_pll_kp = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_pll_ki = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_motor_l = buffer_get_float32(data, 1e8, &ind);
		mcconf.foc_motor_r = buffer_get_float32(data, 1e5, &ind);
		mcconf.foc_motor_flux_linkage = buffer_get_float32(data, 1e5, &ind);
		mcconf.foc_observer_gain = buffer_get_float32(data, 1e0, &ind);
		mcconf.foc_duty_dowmramp_kp = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_duty_dowmramp_ki = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_openloop_rpm = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sl_openloop_hyst = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sl_openloop_time = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sl_d_current_duty = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sl_d_current_factor = buffer_get_float32(data, 1e3, &ind);
		memcpy(mcconf.foc_hall_table, data + ind, 8);
		ind += 8;
		mcconf.foc_hall_sl_erpm = buffer_get_float32(data, 1000.0, &ind);

		mcconf.s_pid_kp = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.s_pid_ki = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.s_pid_kd = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.s_pid_min_erpm = buffer_get_float32(data, 1000.0, &ind);

		mcconf.p_pid_kp = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.p_pid_ki = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.p_pid_kd = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.p_pid_ang_div = buffer_get_float32(data, 1e5, &ind);

		mcconf.cc_startup_boost_duty = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.cc_min_current = buffer_get_float32(data, 1000.0, &ind);
		mcconf.cc_gain = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.cc_ramp_step_max = buffer_get_float32(data, 1000000.0, &ind);

		mcconf.m_fault_stop_time_ms = buffer_get_int32(data, &ind);
		mcconf.m_duty_ramp_step = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.m_duty_ramp_step_rpm_lim = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.m_current_backoff_gain = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.m_encoder_counts = buffer_get_uint32(data, &ind);

		if (rx_mcconf_func) {
			rx_mcconf_func(&mcconf);
		}
		break;

	case COMM_GET_APPCONF:
	case COMM_GET_APPCONF_DEFAULT:
		ind = 0;
		appconf.controller_id = data[ind++];
		appconf.timeout_msec = buffer_get_uint32(data, &ind);
		appconf.timeout_brake_current = buffer_get_float32(data, 1000.0, &ind);
		appconf.send_can_status = data[ind++];
		appconf.send_can_status_rate_hz = buffer_get_uint16(data, &ind);

		appconf.app_to_use = (app_use)data[ind++];

		appconf.app_ppm_conf.ctrl_type = (ppm_control_type)data[ind++];
		appconf.app_ppm_conf.pid_max_erpm = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_ppm_conf.hyst = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_ppm_conf.pulse_start = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_ppm_conf.pulse_end = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_ppm_conf.median_filter = data[ind++];
		appconf.app_ppm_conf.safe_start = data[ind++];
		appconf.app_ppm_conf.rpm_lim_start = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_ppm_conf.rpm_lim_end = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_ppm_conf.multi_esc = data[ind++];
		appconf.app_ppm_conf.tc = data[ind++];
		appconf.app_ppm_conf.tc_max_diff = buffer_get_float32(data, 1000.0, &ind);

		appconf.app_adc_conf.ctrl_type = (adc_control_type)data[ind++];
		appconf.app_adc_conf.hyst = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_adc_conf.voltage_start = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_adc_conf.voltage_end = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_adc_conf.use_filter = data[ind++];
		appconf.app_adc_conf.safe_start = data[ind++];
		appconf.app_adc_conf.cc_button_inverted = data[ind++];
		appconf.app_adc_conf.rev_button_inverted = data[ind++];
		appconf.app_adc_conf.voltage_inverted = data[ind++];
		appconf.app_adc_conf.rpm_lim_start = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_adc_conf.rpm_lim_end = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_adc_conf.multi_esc = data[ind++];
		appconf.app_adc_conf.tc = data[ind++];
		appconf.app_adc_conf.tc_max_diff = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_adc_conf.update_rate_hz = buffer_get_uint16(data, &ind);

		appconf.app_uart_baudrate = buffer_get_uint32(data, &ind);

		appconf.app_chuk_conf.ctrl_type = (chuk_control_type)data[ind++];
		appconf.app_chuk_conf.hyst = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_chuk_conf.rpm_lim_start = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_chuk_conf.rpm_lim_end = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_chuk_conf.ramp_time_pos = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_chuk_conf.ramp_time_neg = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_chuk_conf.stick_erpm_per_s_in_cc = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_chuk_conf.multi_esc = data[ind++];
		appconf.app_chuk_conf.tc = data[ind++];
		appconf.app_chuk_conf.tc_max_diff = buffer_get_float32(data, 1000.0, &ind);

		appconf.app_nrf_conf.speed = (NRF_SPEED)data[ind++];
		appconf.app_nrf_conf.power = (NRF_POWER)data[ind++];
		appconf.app_nrf_conf.crc_type = (NRF_CRC)data[ind++];
		appconf.app_nrf_conf.retry_delay = (NRF_RETR_DELAY)data[ind++];
		appconf.app_nrf_conf.retries = data[ind++];
		appconf.app_nrf_conf.channel = data[ind++];
		memcpy(appconf.app_nrf_conf.address, data + ind, 3);
		ind += 3;
		appconf.app_nrf_conf.send_crc_ack = data[ind++];

		if (rx_appconf_func) {
			rx_appconf_func(&appconf);
		}
		break;

	case COMM_DETECT_MOTOR_PARAM:
		ind = 0;
		detect_cycle_int_limit = buffer_get_float32(data, 1000.0, &ind);
		detect_coupling_k = buffer_get_float32(data, 1000.0, &ind);
		for (i = 0;i < 8;i++) {
			detect_hall_table[i] = (const signed char)(data[ind++]);
		}
		detect_hall_res = (const signed char)(data[ind++]);

		if (rx_detect_func) {
			rx_detect_func(detect_cycle_int_limit, detect_coupling_k,
					detect_hall_table, detect_hall_res);
		}
		break;

	case COMM_DETECT_MOTOR_R_L: {
		// TODO!
	} break;

	case COMM_DETECT_MOTOR_FLUX_LINKAGE: {
		// TODO!
	} break;

	case COMM_DETECT_ENCODER: {
		// TODO!
	} break;

	case COMM_DETECT_HALL_FOC: {
		// TODO!
	} break;

	case COMM_GET_DECODED_PPM:
		ind = 0;
		dec_ppm = buffer_get_float32(data, 1000000.0, &ind);
		dec_ppm_len = buffer_get_float32(data, 1000000.0, &ind);

		if (rx_dec_ppm_func) {
			rx_dec_ppm_func(dec_ppm, dec_ppm_len);
		}
		break;

	case COMM_GET_DECODED_ADC:
		ind = 0;
		dec_adc = buffer_get_float32(data, 1000000.0, &ind);
		dec_adc_voltage = buffer_get_float32(data, 1000000.0, &ind);
		// TODO for adc2

		if (rx_dec_adc_func) {
			rx_dec_adc_func(dec_adc, dec_adc_voltage);
		}
		break;

	case COMM_GET_DECODED_CHUK:
		ind = 0;
		dec_chuk = buffer_get_float32(data, 1000000.0, &ind);

		if (rx_dec_chuk_func) {
			rx_dec_chuk_func(dec_chuk);
		}
		break;

	case COMM_SET_MCCONF:
		// This is a confirmation that the new mcconf is received.
		if (rx_mcconf_received_func) {
			rx_mcconf_received_func();
		}
		break;

	case COMM_SET_APPCONF:
		// This is a confirmation that the new appconf is received.
		if (rx_appconf_received_func) {
			rx_appconf_received_func();
		}
		break;

	default:
		break;
	}
}
		
int can_id = 0;
uint8_t rx_buffer[RX_BUFFER_SIZE];
unsigned int rx_buffer_last_id;

can_status_msg stat_msgs[CAN_STATUS_MSGS_TO_STORE];

void vesc_can_begin() {
  HAL_CAN_Start(&hcan2);
}


uint8_t buffer[4];
void vesc_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}



void comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}



void comm_can_set_current_brake(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}



void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}



void comm_can_get_values(uint8_t controller_id) {
  can_id= 0;
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer[send_index++] = 1;
  buffer[send_index++] = 0;        
  buffer[send_index++] = COMM_GET_VALUES;
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), buffer, send_index);
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{vesc_can_read();}

  uint8_t dframe[8];
int vesc_can_read() {
  CAN_message_t  inMsg;

  int32_t ind = 0;
  unsigned int rxbuf_len;
  unsigned int rxbuf_ind;
  uint8_t crc_low;
  uint8_t crc_high;

	CAN_RxHeaderTypeDef re_header;
  int ijk;
  can_status_msg *stat_tmp;
  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &re_header, dframe) != HAL_OK) {
    return 0; 
  }
  if (inMsg.ext == 1) {
    uint8_t id = inMsg.id & 0xFF;
    CAN_PACKET_ID cmd = (CAN_PACKET_ID) (inMsg.id >> 8); 
    switch (cmd) {
      case CAN_PACKET_SET_DUTY:
        ind = 0;

        break;

      case CAN_PACKET_SET_CURRENT:
        ind = 0;

        break;

      case CAN_PACKET_SET_CURRENT_BRAKE:
        ind = 0;
        break;

      case CAN_PACKET_SET_RPM:
        ind = 0;
        break;

      case CAN_PACKET_SET_POS:
        ind = 0;
        break;
      case CAN_PACKET_FILL_RX_BUFFER:
        memcpy(rx_buffer + inMsg.buf[0], inMsg.buf + 1, inMsg.len - 1);
        break;
      case CAN_PACKET_FILL_RX_BUFFER_LONG:
        rxbuf_ind = (unsigned int)inMsg.buf[0] << 8;
        rxbuf_ind |= inMsg.buf[1];
        if (rxbuf_ind < RX_BUFFER_SIZE) {
          memcpy(rx_buffer + rxbuf_ind, inMsg.buf + 2, inMsg.len - 2);
        }
        break;

      case CAN_PACKET_PROCESS_RX_BUFFER:
        ind = 0;
        rx_buffer_last_id = inMsg.buf[ind++];
        inMsg.buf[ind++];
        rxbuf_len = (unsigned int)inMsg.buf[ind++] << 8;
        rxbuf_len |= (unsigned int)inMsg.buf[ind++];

        if (rxbuf_len > RX_BUFFER_SIZE) {
          break;
        }

        crc_high = inMsg.buf[ind++];
        crc_low = inMsg.buf[ind++];

        if (crc16(rx_buffer, rxbuf_len)
            == ((unsigned short) crc_high << 8
                | (unsigned short) crc_low)) {

          //                  if (commands_send) {
          //                     send_packet_wrapper(rx_buffer, rxbuf_len);
          //                  } else {
          can_process_packet(rx_buffer, rxbuf_len);
          //                  }
        }
        break;

      case CAN_PACKET_PROCESS_SHORT_BUFFER:
        ind = 0;
        rx_buffer_last_id = inMsg.buf[ind++];
        inMsg.buf[ind++];
        can_process_packet(inMsg.buf + ind, inMsg.len - ind);
        break;

      case CAN_PACKET_STATUS:
        for (ijk = 0; ijk < CAN_STATUS_MSGS_TO_STORE; ijk++) {
          stat_tmp = &stat_msgs[ijk];
          if (stat_tmp->id == id || stat_tmp->id == -1) {
            ind = 0;
            stat_tmp->id = id;
            stat_tmp->rpm = (float)buffer_get_int32(inMsg.buf, &ind);
            stat_tmp->current = (float)buffer_get_int16(inMsg.buf, &ind) / 10.0;
            stat_tmp->duty = (float)buffer_get_int16(inMsg.buf, &ind) / 1000.0;
            break;
          }
          
        }
        

      default:
        break;

    }


  }
  return 1;



}



bool sendPacket(uint8_t id, uint8_t packet[], int32_t len) {
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.ExtId= id;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC =0x04;
//	send_data[0] =0x00;
//	send_data[1] =0x00;
//	send_data[2] =0x03;
//	send_data[3] =0xE8;
	while ( !(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) ){} 
	HAL_CAN_AddTxMessage(&hcan2 , &TxHeader, packet,&send_mail_box);
//	CAN_TxHeaderTypeDef TxHeader;
//	TxHeader.ExtId= id;
//	TxHeader.IDE = CAN_ID_EXT;
//	TxHeader.RTR = CAN_RTR_DATA;
//	TxHeader.DLC =0x04;
//	
//	while ( !(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) ){} 
//	uint32_t send_mail_box;
//	HAL_CAN_AddTxMessage(&hcan2 , &TxHeader, packet, &send_mail_box);
//  return (bool)  HAL_CAN_AddTxMessage(&hcan2 , &TxHeader, packet, &send_mail_box);
}

void can_process_packet(unsigned char *data, unsigned int len){
   bldc_interface_process_packet(data,len);
   can_id=1;
}

unsigned short crc16(unsigned char *buf, unsigned int len) {
  unsigned int i;
  unsigned short cksum = 0;
  for (i = 0; i < len; i++) {
    cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
  }
  return cksum;
}

//const vesc_motor_measure *get_vesc_motorvel_pointer(can_bus can, uint8_t i)
//{
//	return &motor_bus1_vel_measure[(i & 0x03)];
//}
//void vesc_int(vesc_motor_t *motor, can_bus can, can_vesc_id_e id)
//{
//  motor->id = id;
//  motor->can = can;
//  uint8_t i = 0;
//  motor->measure_vesc = get_vesc_motorvel_pointer(can, i);
//// pid_init(&motor->position_pid);
////    motor->position_pid.f_param_init(&motor->position_pid, PID_Position, PITCH_MOTOR_POS_PID_MAX_OUT, PITCH_MOTOR_POS_PID_MAX_IOUT, 
////                  PITCH_MOTOR_POS_PID_DEADBAND, 0, 0, 0, 
////                  PITCH_MOTOR_POS_PID_KP, PITCH_MOTOR_POS_PID_KI, PITCH_MOTOR_POS_PID_KD,PID_Ramp); //位置环pid赋值
//      
////    pid_init(&motor->velocity_pid);
////    motor->velocity_pid.f_param_init(&motor->velocity_pid, PID_Speed, VESC_MOTOR_SPEED_PID_MAX_OUT, VESC_MOTOR_SPEED_PID_MAX_IOUT,
////                  PITCH_MOTOR_SPEED_PID_DEADBAND, 0, 800,0, 
////                  PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD,PID_IMPROVE_NONE); //速度环pid赋值

//}
//void vesc_motor_pos_control_set(vesc_motor_t *motor)
//{

//    // 位置环
//    motor->position_pid.target = motor->angle_set;
//    motor->position_pid.f_cal_pid(&motor->position_pid, motor->measure->total_angle);
//    // 速度环
//    motor->velocity_pid.target = motor->position_pid.f_cal_pid(&motor->position_pid, motor->measure->total_angle);
//    motor->velocity_pid.f_cal_pid(&motor->velocity_pid, motor->measure->speed_rpm); //根据设定值进行PID计算。
//    motor->speed_rpm_set = motor->velocity_pid.output;
//  }        
    
 

//void vesc_motor_vel_control_set(vesc_motor_t *motor)
//{


//    // 速度环
//    motor->velocity_pid.target = motor->speed_rpm_set;
//    motor->velocity_pid.f_cal_pid(&motor->velocity_pid, motor->measure_vesc->speed_rpm); //根据设定值进行PID计算。
//    motor->current = motor->velocity_pid.output;
//  }
    


void vesc_spm(vesc_motor_t *motor)
{
	
	
	for(int i=0;i < 3;i++)
    {	
		
//	comm_can_set_current(motor[i].id, motor[i].current );
		comm_can_set_rpm(motor[i].id, motor[i].speed_rpm_set);
	 }
 }

void vesc_get_date1(vesc_motor_measure *ptr, uint8_t *Data)
{

//	ptr->last_angle = ptr->angle;
//	ptr->angle = (uint16_t)(Data[0] << 8 | Data[1]) ;
	ptr->speed_rpm  = (int32_t)(Data[0] << 24 | Data[1] << 16| Data[2] <<8 | Data[3]);
	ptr->Total_current = (int16_t)(Data[4] << 8 | Data[5]) * 100;
	ptr->Duty_Cycle = (int16_t)(Data[6] << 8 | Data[7]) / 1000;
	

}	
void vesc_get_date2(vesc_motor_measure *ptr, uint8_t *Data)
{

//	ptr->last_angle = ptr->angle;
//	ptr->angle = (uint16_t)(Data[0] << 8 | Data[1]) ;
	ptr->PID_POS  = (int16_t)(Data[6] << 8 | Data[7]) / 50;
	ptr->current_in = (int16_t)(Data[4] << 8 | Data[5]) * 100;
	ptr->Motor_Temp = (int16_t)(Data[3] << 8 | Data[2]) / 10;
	ptr->FET_Temp = (int16_t)(Data[1] << 8 | Data[0]) / 10;

}

	
	



