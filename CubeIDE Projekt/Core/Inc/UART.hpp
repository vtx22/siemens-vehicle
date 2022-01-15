#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <array>
#include "PRINTER.hpp"
#include "INA226.hpp"
#include "DS18B20.hpp"

const static uint8_t RX_BUF_SIZE = 12;
const static uint8_t TX_BUF_SIZE = 12;

class UART
{
	public:

		UART(UART_HandleTypeDef* huart, INA226* ina, PRINTER* print, DS18B20* _tempPCB);


		std::array<uint8_t, TX_BUF_SIZE> statusMessage(uint8_t codes);
		std::array<uint8_t, TX_BUF_SIZE> batteryStatus(uint16_t voltage, int16_t current);
		std::array<uint8_t, TX_BUF_SIZE> tempStatus(uint16_t tempPCB, uint16_t tempIOT, uint16_t tempOUT, uint16_t humidity);

		//== TRANSMITTING DATA ==//
		void sendStatus(uint8_t code);
		void sendBATstatus();
		void sendTEMPstatus();

		template<size_t SIZE>
		void sendBytes(std::array<uint8_t, SIZE>& data)
		{
			HAL_UART_Transmit(_huart, data.data(), data.size(), 100);
		}
		//====================//

		//== RECEIVING DATA ==//
		void startReceiveIT();
		void parseMessage();
		void parseMSG1();
		uint8_t RXBuffer[RX_BUF_SIZE];

		//====================//

		const uint8_t _id = 0x69;
		bool _globalEnable = true;
	private:

		uint8_t calculateChecksum(uint8_t* msg);
		template<std::size_t SIZE>
		uint8_t calculateChecksum(std::array<uint8_t, SIZE>& complete_array)
		{
			uint8_t checksum = 0x00;
			for(auto &runner : complete_array)
			{
				checksum ^= runner;
			}

			return checksum;
		}



		UART_HandleTypeDef* _huart;
		INA226* _ina;
		PRINTER* _print;
		DS18B20* _tempPCB;

};
