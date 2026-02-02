/*******************************************************************************
 * Credit: [sirojudinMunir]
 * Source: https://github.com/sirojudinMunir/stm32-FOC/blob/master/lib/Controller_app/controller_app.c
 * License: MIT License
 * * Description: controller app  usb interface
 * Adapted by: [DevJ1n]
 * Date: 2025-12-25
 ******************************************************************************/

#include "controller_app.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>


extern uint16_t controller_mode;
extern float sp_input;
extern PI_Controller pi_current;
extern PI_Controller pi_voltage;


char usb_send_buff[128];

void send_data_float(const float* values, uint8_t count) {
	if (count == 0 || values == NULL) return;

	// Total frame: 2 byte header + 4*count float + 2 byte footer
	uint16_t total_size = 2 + count * 4 + 2;
	uint8_t frame[256];

	if (total_size > sizeof(frame)) return;

	// Header: 0x55, 0xAA
	frame[0] = 0x55;
	frame[1] = 0xAA;

	// Copy floats
	for (uint8_t i = 0; i < count; i++) {
		union { float f; uint8_t b[4]; } u;
		u.f = values[i];
		frame[2 + i * 4 + 0] = u.b[0];
		frame[2 + i * 4 + 1] = u.b[1];
		frame[2 + i * 4 + 2] = u.b[2];
		frame[2 + i * 4 + 3] = u.b[3];
	}

	// Footer: 0xAA, 0x55
	frame[2 + count * 4 + 0] = 0xAA;
	frame[2 + count * 4 + 1] = 0x55;

	// Transmit
//	CDC_Transmit_FS(frame, total_size);
//	HAL_UART_Transmit_DMA(&huart1,frame,total_size);
	HAL_UART_Transmit(&huart1, frame, total_size, 2);
}

void change_legend(uint8_t index, const char *str) {
	uint8_t frame[64];
	uint16_t total_size = 0;

	frame[0] = 0x03;
	frame[1] = index;

	size_t len = strlen(str);
	if (len > sizeof(frame) - 2) len = sizeof(frame) - 2;
	memcpy(&frame[2], str, len);

	total_size = 2 + len;

//	CDC_Transmit_FS(frame, total_size);
//	HAL_UART_Transmit_DMA(&huart1,frame,total_size);
	HAL_UART_Transmit(&huart1, frame, total_size, 2);
}

void change_title(const char *str) {
	uint8_t frame[64];
	uint16_t total_size = 0;

	frame[0] = 0x04;
	frame[1] = 0x00;

	size_t len = strlen(str);
	if (len > sizeof(frame) - 2) len = sizeof(frame) - 2;
	memcpy(&frame[2], str, len);

	total_size = 2 + len;

//	CDC_Transmit_FS(frame, total_size);
//	HAL_UART_Transmit_DMA(&huart1,frame,total_size);
	HAL_UART_Transmit(&huart1, frame, total_size, 2);
}

void erase_graph(void) {
	uint8_t frame[2];
	frame[0] = 0x02;
	frame[1] = 0x00;
//	CDC_Transmit_FS(frame, 2);
//	HAL_UART_Transmit_DMA(&huart1,frame,2);
	HAL_UART_Transmit(&huart1, frame, 2, 2);
}

/******************************************************************************************* */


int parse_float_value(const char *input, char separator, float *out_value) {
    if (!input || !out_value) return 0;

    // Cari posisi separator
    const char *sep_pos = strchr(input, separator);
    if (!sep_pos) return 0;

    // Lompat ke karakter setelah separator
    const char *val_str = sep_pos + 1;

    // Lewati whitespace
    while (isspace((unsigned char)*val_str)) {
        val_str++;
    }

    // Cek apakah setelah spasi masih ada karakter valid
    if (*val_str == '\0') return 0;

    // Parsing float
    char *endptr;
    float value = strtof(val_str, &endptr);

    if (val_str == endptr) {
        // Tidak ada konversi yang terjadi
        return 0;
    }

    *out_value = value;
    return 1;
}

int parse_int_value(const char *input, char separator, int *out_value) {
    if (!input || !out_value) return 0;

    const char *sep_pos = strchr(input, separator);
    if (!sep_pos) return 0;

    const char *val_str = sep_pos + 1;

    while (isspace((unsigned char)*val_str)) {
        val_str++;
    }

    if (*val_str == '\0') return 0;

    char *endptr;
    long value = strtol(val_str, &endptr, 10);

    // Validasi: endptr harus berada di akhir angka atau hanya whitespace
    while (isspace((unsigned char)*endptr)) {
        endptr++;
    }
    if (*endptr != '\0') return 0; // Ada karakter tidak valid setelah angka

    *out_value = (int)value;
    return 1;
}


void get_pid_param(void) {
	char write_buffer[128];
	uint16_t len = 0;

	switch (controller_mode) {
	case VOLTAGE_CONTROL_MODE:
		len = sprintf(write_buffer, "Current Control param:\n"
                                "Kp(Id):%f\n"
                                "Ki(Id):%f\n"
                                "Kp(Iq):%f\n"
                                "Ki(Iq):%f\n"
                                "Deadband:%f\n"
                                "Max output:%f\n",  
								pi_voltage.Kp,  pi_voltage.Ki,
								pi_voltage.Kp, pi_voltage.Ki,
                                0.0f, pi_voltage.out_max);
		break;
	case CURRENT_CONTROL_MODE:
		len = sprintf(write_buffer, "Speed Control param:\n"
                                "Kp:%f\n"
                                "Ki:%f\n"
                                "Deadband:%f\n"
                                "Max output:%f\n",
								pi_current.Kp, pi_current.Ki, 0.0f, pi_current.out_max);
		break;
	default:
		len = sprintf(write_buffer, "Wrong Mode\n");
		break;
	}
//	CDC_Transmit_FS((uint8_t*)write_buffer, len);
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)write_buffer, len);
	HAL_UART_Transmit(&huart1, (uint8_t*)write_buffer , len, 2);
}

void print_mode(CONTROL_MODE mode) {
	char write_buffer[64];
	uint16_t len = 0;

	switch (mode) {
	case VOLTAGE_CONTROL_MODE:
		len = sprintf(write_buffer, "Control Mode[0]: Voltage Control\n");
		break;
	case CURRENT_CONTROL_MODE:
		len = sprintf(write_buffer, "Control Mode[1]: Current Control\n");
		break;
	case CAL_SENSOR:
		len = sprintf(write_buffer, "Control Mode[3]: Calibration\n");
		break;

	default:
		len = sprintf(write_buffer, "Wrong Mode\n");
		break;
	}
//	CDC_Transmit_FS((uint8_t*)write_buffer, len);
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)write_buffer, len);
	HAL_UART_Transmit(&huart1, (uint8_t*)write_buffer , len, 10);
}
uint8_t check_cmd[64] = {0};
void parse_command(char *cmd) {
	_Bool set_pid_detected = 0;
//	strncpy((char*)check_cmd, cmd, sizeof(check_cmd) - 1);
//	check_cmd[sizeof(check_cmd) - 1] = '\0';

	if (strstr(cmd, "cal")) {
    usb_print("start callibration\r\n");
//    hfoc.control_mode = CALIBRATION_MODE;
//    calibration_flag = 1;
	}
	else if (strstr(cmd, "test")) {
		usb_print("start test pulse response\r\n");
//    	hfoc.control_mode = TEST_MODE;
//    	test_bw_flag = 1;
	}
	else if (strstr(cmd, "get_bandwidth")) {
		usb_print("Current Bandwidth:%.2f\r\n", 0.0f);
	}
	else if (strstr(cmd, "set_bandwidth")) {
		float bandwidth;
		parse_float_value(cmd, '=', &bandwidth);
//    	hfoc.I_ctrl_bandwidth = bandwidth;
//    	m_config.I_ctrl_bandwidth = bandwidth;
//    	flash_auto_tuning_torque_control(&m_config);
//   	hfoc.id_ctrl.kp = m_config.id_kp;
//    	hfoc.id_ctrl.ki = m_config.id_ki;
//    	hfoc.iq_ctrl.kp = m_config.iq_kp;
//    	hfoc.iq_ctrl.ki = m_config.iq_ki;

		usb_print("New Bandwidth:%.2f\r\n", bandwidth);
	}
	else if (strstr(cmd, "sp")) {
		parse_float_value(cmd, '=', &sp_input);
	}
	else if (strstr(cmd, "kp")) {
		float kp;
		parse_float_value(cmd, '=', &kp);

		switch (controller_mode) {
		case VOLTAGE_CONTROL_MODE:
			pi_voltage.Kp = kp;

			break;
		case CURRENT_CONTROL_MODE:
			pi_current.Kp = kp;

			break;

		default:
			break;
		}
		set_pid_detected = 1;
	}
	else if (strstr(cmd, "ki")) {
		float ki;
		parse_float_value(cmd, '=', &ki);
    
		switch (controller_mode) {
		case VOLTAGE_CONTROL_MODE:
			pi_voltage.Ki = ki;

			break;
		case CURRENT_CONTROL_MODE:
			pi_current.Ki = ki;
			break;

		default:
			break;
		}
		set_pid_detected = 1;
	}
	else if (strstr(cmd, "kd")) {
		float kd;
		parse_float_value(cmd, '=', &kd);
    
		switch (controller_mode) {

		default:
			break;
		}
		set_pid_detected = 1;
	}
	else if (strstr(cmd, "deadband")) {
		float db;
		parse_float_value(cmd, '=', &db);
    
		switch (controller_mode) {
		case VOLTAGE_CONTROL_MODE:
//        	hfoc.id_ctrl.e_deadband = db;
//        	m_config.id_e_deadband = db;
//        	hfoc.iq_ctrl.e_deadband = db;
//       	 m_config.iq_e_deadband = db;
			break;
		case CURRENT_CONTROL_MODE:
//        	hfoc.speed_ctrl.e_deadband = db;
//        	m_config.speed_e_deadband = db;
			break;

		default:
			break;
		}
		set_pid_detected = 1;
	}
	else if (strstr(cmd, "max")) {
		float max;
		parse_float_value(cmd, '=', &max);
    
		switch (controller_mode) {
		case VOLTAGE_CONTROL_MODE:
			pi_voltage.out_max = max;

			break;
		case CURRENT_CONTROL_MODE:
			pi_current.out_max = max;
			break;

		default:
			break;
		}
		set_pid_detected = 1;
	}
	else if (strstr(cmd, "mode") != NULL) {
		CONTROL_MODE mode = VOLTAGE_CONTROL_MODE;

		parse_int_value(cmd, '=', (int*)&mode);
		print_mode(mode);

		if (mode <= UNKNOW_CONTROL_MODE) {
    	controller_mode = mode;
    	sp_input = 0;
		}
	}
//	else if (strstr(cmd, "ratio") != NULL) {
//		float ratio;
//		parse_float_value(cmd, '=', &ratio);
////    	hfoc.gear_ratio = ratio;
//	}
	else if (strstr(cmd, "set_zero") != NULL) {
		usb_print("success set zero offset\r\n");
	}
	else if (strstr(cmd, "save") != NULL) {
//    	flash_save_config(&m_config);
		usb_print("success save configuration\r\n");
	}
	else if (strstr(cmd, "set_default") != NULL) {
//    	flash_default_config(&m_config);
		usb_print("success reset configuration\r\n");
	}
	else if (strstr(cmd, "get_param") != NULL) {
		get_pid_param();
	}

	if (set_pid_detected) {
		get_pid_param();
	}
}
