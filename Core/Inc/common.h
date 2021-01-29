/*
 * common.h
 *
 *  Created on: Jan 13, 2021
 *      Author: energia
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include <stdint.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"

							// Возможные типы состояний контроллера
typedef enum { enUnknown, enReady, enSend, enSendWait, enErrVersion, enErrFormat, enErrCRC, enErrType, enIsDoubleStartByte, enIsDoubleEndByte, enReset, enCount } tpState ;
typedef enum { rstSoftware, rstErr, rstHardware } tpReset ;
typedef enum {
	cmdCheckAddr = 0xFE,
	cmdChangeAddr = 0x01,
	cmdSetSN = 0x00,
	cmdSetDateTime = 0xFD,
	cmdReset = 0xFC,
	cmdGetData= 0x50,
	cmdError = 0x51
} tpCommand;

typedef enum {
	errUnknown = 0,
	errCRC = 1,
	errType = 2,
	errVersion = 3,
	errWriteSN = 4,
	errWriteAddr = 5,
	errWriteDateTime = 6,
	errFormat = 7
} tpProtocolErr ;

#define defVersionSW 0x01					// Версия ПО. Ver 0.1 - SHT, 0.2 - HIH, 0.3 - HTU

#define defBlinkLedPort GPIOC			// Порт для зелёного светодиода
#define defBlinkLedPin GPIO_PIN_13		// Пин для зелёного светодиода

#define defBufRxLen 40					// Размер буфера для приёма команд
#define defBufTxLen 40					// Размер буфера для передачи ответа

#define defSHT3adress 0x44				// Адрес датчика по щине I2C
#define defSHT3adressWrite (defSHT3adress << 1)
#define defSHT3adressRead ((defSHT3adress << 1) + 1)

#define defI2cTimeOut 100				// Таймаут для чтения данных по I2C

#define defPosStartByte 0				// Позиция стартового байта
#define defPosAddrReceive 1				// Позиция адреса получателя
#define defPosAddrHost 2				// Позиция адреса источника
#define defPosVersion 3					// Позиция версии протокола
#define defPosType 4					// Позиция поля TYPE
#define defPosData 5					// Позиция началя блока DATA

#define defVersion 0x24					// Версия протокола

#define defStartByte 0x1F				// Значение стартового байта
#define defStopByte 0x55				// Значение стопового байта
#define defEndByte 0x2F					// Значение признака конца пакета

#define dfeFlashNumPage 63				// Номер страницы куда будем записывать данные
#define defPtrFlash 0x08000000 + dfeFlashNumPage * FLASH_PAGE_SIZE	// Адрес страницы флешака
#define defPtrFlashSN defPtrFlash 		// Указатель на серийный номер на флешке
#define defPtrFlashAddr defPtrFlash + 2 // Указатель на адрес устройства

extern tpState stState ;				// Текущее состояние
extern uint16_t *stPtrSN ;				// Указатель на серийный номер на флешке
extern uint16_t *stPtrAddr ;			// Указатель на адрес устройства

extern uint8_t stSHT3Write [2] ;		// Команда на запрос данных с датчика SHT3x-DIS
extern uint8_t stSHT3reset [2] ;		// Команда софтового сброса
extern uint8_t stSHT3clear [2] ;		// Команда очистки регистра статуса
extern uint8_t stSHT3Data [6] ;			// Буфер с данными полученными от датчика SHT3x-DIS

extern HAL_StatusTypeDef stErrorI2c ;	// Флаг ошибки при работе по шине I2C

extern RTC_TimeTypeDef stTime ;			// Дата и время последнего успешного чтения данных с датчика
extern RTC_DateTypeDef stDate ;

extern uint8_t stBufRx [defBufRxLen] ;	// Буфер для приёма команд
extern uint8_t stBufRxCrc [defBufRxLen] ; // Буфер для приёма команд с не удалёными служебными символами. Нужен для расчёта CRC
extern uint8_t stBufTx [defBufTxLen] ;	// Буфер для отправки ответа на команду
extern uint8_t stBufRxId ;				// Индекс текущей позиции в буфере приёма команд
extern uint8_t stBufRxCrcId ;			// Индекс текущей позиции в буфере приёма команд с не удалёными служебными символами.
extern uint8_t stBufTxId ;				// Индекс текущей позиции в буфере отправки ответа на команду

HAL_StatusTypeDef getSHT () ;			// Получение данных с датчика SHT
void receiveByte (uint8_t) ;			// Обработка полученного байта
bool checkByte (uint8_t) ;				// Устранение дублирования стартового байта и признака окончания пакета
void sendAnswer () ;					// Отправка ответа на команду
uint16_t checkCRC (uint8_t *, uint8_t) ;// Рассчёт CRC16 CCITT-FALSE с вектором инициализации 0
void makeHeader (tpCommand) ;			// Формирование заголовка протокола
void makeTxBuf (tpCommand, uint8_t*) ;	// Формируем буфер для передачи
void writeTxByte (uint8_t) ;			// В буфер для передачи записываются данные. Если встречается служебный байт, то он дублируется
bool writeFlash (uint16_t, uint8_t) ;	// Запись данных не флешку SN и адреса. При ошибке записи возвращается false

uint8_t crcSHT(uint8_t *, uint32_t) ; 	// Подсчёт CRC для датчика SHT


#endif /* INC_COMMON_H_ */
