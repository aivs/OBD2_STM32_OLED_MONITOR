/*
 * obd2.c
 *
 *  Created on: Feb 8, 2024
 *      Author: aivs
 */

/* Private includes ----------------------------------------------------------*/
#include "obd2.h"
#include "printf.h"

/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan; // CAN Data Transmit Setup
static uint32_t last_request_time = 0;

/* Private function prototypes -----------------------------------------------*/
void obd2_SendFrame(uint16_t address, uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6, uint8_t byte7);
uint8_t obd2_GetParamIndex(uint8_t pid);

/* Private user code ---------------------------------------------------------*/
void obd2_NextRequest()
{
	static uint8_t num = 0;
	obd2_SendFrame(0x7DF, 0x02, 0x01, params[num++], 0x55, 0x55, 0x55, 0x55, 0x55);
	num = (num == GET_SIZE(params)) ? 0 : num;

	// TEST VAG UDS VOLTAGE
	// obd2_SendFrame(0x7E1, 0x03, 0x22, 0xF4, 0x42, 0x55, 0x55, 0x55, 0x55);
	// obd2_SendFrame(0x7DF, 0x02, 0x09, 0x02, 0x55, 0x55, 0x55, 0x55, 0x55);
}

void obd2_Handler(uint8_t RxData[], uint8_t len)
{
	//uint8_t length = RxData[0];
	//uint8_t status = RxData[1];
	uint16_t pid  = RxData[2];
	uint8_t data1 = RxData[3];
	uint8_t data2 = RxData[4];

	int16_t value;

	switch (pid) {
	case PID_RPM:
	case PID_EVAP_SYS_VAPOR_PRESSURE: // kPa
		value = (data2 | data1 << 8) / 4;
		break;
	case PID_FUEL_PRESSURE: // kPa
		value = data1 * 3;
		break;
	case PID_COOLANT_TEMP:
	case PID_INTAKE_TEMP:
	case PID_AMBIENT_TEMP:
	case PID_ENGINE_OIL_TEMP:
		value = data1 - 40;
		break;
	case PID_THROTTLE:
	case PID_COMMANDED_EGR:
	case PID_COMMANDED_EVAPORATIVE_PURGE:
	case PID_FUEL_LEVEL:
	case PID_RELATIVE_THROTTLE_POS:
	case PID_ABSOLUTE_THROTTLE_POS_B:
	case PID_ABSOLUTE_THROTTLE_POS_C:
	case PID_ACC_PEDAL_POS_D:
	case PID_ACC_PEDAL_POS_E:
	case PID_ACC_PEDAL_POS_F:
	case PID_COMMANDED_THROTTLE_ACTUATOR:
	case PID_ENGINE_LOAD:
	case PID_ABSOLUTE_ENGINE_LOAD:
	case PID_ETHANOL_FUEL:
	case PID_HYBRID_BATTERY_PERCENTAGE:
		value = data1 * 100 / 255;
		break;
	case PID_MAF_FLOW: // grams/sec
		value = (data2 | data1 << 8) / 100;
		break;
	case PID_TIMING_ADVANCE:
		value = (data1 / 2) - 64;
		break;
	case PID_DISTANCE: // km
	case PID_DISTANCE_WITH_MIL: // km
	case PID_TIME_WITH_MIL: // minute
	case PID_TIME_SINCE_CODES_CLEARED: // minute
	case PID_RUNTIME: // second
	case PID_FUEL_RAIL_PRESSURE: // kPa
	case PID_ENGINE_REF_TORQUE: // Nm
		value = (data2 | data1 << 8);
		break;
	case PID_CONTROL_MODULE_VOLTAGE: // V
		value = (data2 | data1 << 8) / 1000;
		break;
	case PID_ENGINE_FUEL_RATE: // L/h
		value = (data2 | data1 << 8) / 20;
		break;
	case PID_ENGINE_TORQUE_DEMANDED: // %
	case PID_ENGINE_TORQUE_PERCENTAGE: // %
		value = data1 - 125;
		break;
	case PID_SHORT_TERM_FUEL_TRIM_1:
	case PID_LONG_TERM_FUEL_TRIM_1:
	case PID_SHORT_TERM_FUEL_TRIM_2:
	case PID_LONG_TERM_FUEL_TRIM_2:
	case PID_EGR_ERROR:
		value = (data1 * 100 / 128) - 100;
		break;
	case PID_FUEL_INJECTION_TIMING:
		value = ((data2 | data1 << 8) / 128) - 210;
		break;
	case PID_CATALYST_TEMP_B1S1:
	case PID_CATALYST_TEMP_B2S1:
	case PID_CATALYST_TEMP_B1S2:
	case PID_CATALYST_TEMP_B2S2:
		value = ((data2 | data1 << 8) / 10) - 40;
		break;
	case PID_AIR_FUEL_EQUIV_RATIO: // 0~200
		value = (data2 | data1 << 8) * 2 / 65536;
		break;
	default:
		value = data1;
	}

	valueHandler(obd2_GetParamIndex(pid), value);
}

void obd2_SendFrame(uint16_t address, uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6, uint8_t byte7)
{
	// CAN Data Transmit Setup
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			TxMailbox;
	uint8_t				TxData[8];

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = address;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxData[0] = byte0;
	TxData[1] = byte1;
	TxData[2] = byte2;
	TxData[3] = byte3;
	TxData[4] = byte4;
	TxData[5] = byte5;
	TxData[6] = byte6;
	TxData[7] = byte7;

	#ifdef DEBUG
	printf("%08lu TxData: %X [%d] ",HAL_GetTick(), TxHeader.StdId, TxHeader.DLC);
	for (int i = 0; i < TxHeader.DLC; i++) {
		printf("%02X ",TxData[i]);
	}
	printf("\n");
	#endif

	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
	last_request_time = HAL_GetTick();
}

uint8_t obd2_GetParamIndex(uint8_t pid)
{
	for (uint8_t i = 0; i < GET_SIZE(params); i++) {
		if (params[i] == pid) {
			return i;
		}
	}
	return 0;
}

uint32_t obd2_getLastRequestTime()
{
	return last_request_time;
}
