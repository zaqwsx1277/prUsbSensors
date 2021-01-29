/*
 * common.c
 *
 *  Created on: Jan 13, 2021
 *      Author: AAL
 */

#include <rtc.h>
#include <tim.h>
#include <i2c.h>

#include "common.h"

uint8_t stSHT3Write [2] = { 0x2C, 0x10 } ;
uint8_t stSHT3reset [2] = { 0x30, 0xA2 } ;
uint8_t stSHT3clear [2] = { 0x30, 0xA2 } ;
uint8_t stSHT3Data [6] = { 0, } ;
uint32_t stSHT3humidity = { 0 } ;
uint32_t stSHT3temperature = { 0 } ;

HAL_StatusTypeDef stErrorI2c = { HAL_OK } ;

RTC_TimeTypeDef stTime = {0, 0, 0};
RTC_DateTypeDef stDate = {0, 0, 0, 0};

uint8_t stBufRx [defBufRxLen] = { 0, } ;
uint8_t stBufRxCrc [defBufRxLen] = { 0, } ;
uint8_t stBufTx [defBufTxLen] = { 0, } ;
uint8_t stBufRxId = { 0 } ;
uint8_t stBufRxCrcId = { 0 } ;
uint8_t stBufTxId = { 0 } ;

uint16_t *stPtrSN = defPtrFlashSN ;
uint16_t *stPtrAddr = defPtrFlashAddr ;

tpState stState = { enUnknown } ;
//-----------------------------------------------------------------------------------------------
void receiveByte (uint8_t inByte)
{

	if (stBufRxId == defBufRxLen) {
		stBufRxId = 0 ;	// Кол-во байт превышает размер буфера приёма
		stBufRxCrcId = 0 ;
	}

	if (stBufRxId != 0) {
	  stBufRxCrc [stBufRxCrcId++] = inByte ;		// Заполняем буфер для расчёта CRC
	  if (checkByte (inByte)) return ;				// Проверка повтора служебных символов. Для первого символа проверка не выполняется
	}
	  else stBufRxCrcId = 0 ;

	switch (stBufRxId) {
	  case defPosStartByte:							// Ждём стартовый байт протокола
		if (inByte != defStartByte) return ;
		stBufTxId = 0 ;
	  break;

	  case defPosAddrReceive:
		if (inByte != 0 && inByte != (uint8_t) *stPtrAddr) { // Проверяем адресата команды
		  stBufRxId = 0 ;							// Команда не для этого устройства
		  return ;
		}
	  break ;

	  case defPosVersion:
		if (inByte != defVersion) stState = enErrVersion ;	// Проверяем версию протокола
	  break ;

	  default:
	  break;
	}

	stBufRx [stBufRxId++] = inByte ;						// Т.к. при дубликате мы сюда не попадаем, то без проблем записываем принятый байт в буфер
	if (stBufRxId > 4)										// Проверяем завершение команды
		if (inByte == defStopByte && stBufRx [stBufRxId - 2] == defEndByte) {
		  if (*((uint16_t *)&stBufRx [stBufRxId - 4]) != checkCRC (stBufRxCrc, stBufRxCrcId - 4)) stState = enErrCRC ;
		  sendAnswer () ;
		}
}
//---------------------------------------------------------------------------------------------------
/*!
 *  Проверка дублирования служебного символа
 */
bool checkByte (uint8_t inByte)
{
	bool retVal = false  ;
	if (stState == enReady || stState == enIsDoubleStartByte || stState == enIsDoubleEndByte) {	// В других состояниях смысла проверять дубликаты нет
		switch (inByte) {
		  case defStartByte:
			// Проверяем предыдущий символ и оставляем возможность обработки нескольких подряд служебных символов (1F 1F 1F 1F -> 1F 1F)
			if (stBufRx [stBufRxId - 1] == defStartByte && stState == enIsDoubleStartByte) {
				retVal = true ;
				stState = enReady ;
			}
			  else stState = enIsDoubleStartByte ;
		  break;

		  case defStopByte:
			if (stBufRx [stBufRxId - 1] == defStopByte && stState == enIsDoubleEndByte) {
				retVal = true ;
				stState = enReady ;
			}
			  else stState = enIsDoubleEndByte ;
			break;

		  default:
			stState = enReady ;
		  break;
		}
	}
	return retVal ;
}

//---------------------------------------------------------------------------------------------------
void sendAnswer ()
{
	switch (stState) {
	  case enErrVersion:
	  case enErrCRC:
		makeTxBuf (cmdError, &stState) ;
		stState = enSend ;
		HAL_TIM_Base_Start_IT(&htim4);
	  break;

	  case enReady:
	  case enIsDoubleEndByte:
	  case enIsDoubleStartByte:
		makeTxBuf (stBufRx [defPosType], NULL) ;
		stState = enSend ;
		HAL_TIM_Base_Stop_IT(&htim4);		// Запускаем таймер для задержки на 20 мСек
		HAL_TIM_Base_Start_IT(&htim4);
//		while (stState != enReady) { } ;
//		CDC_Transmit_FS(stBufTx, stBufTxId) ;
	  break ;

	  default:
	  break;
	}
}
//---------------------------------------------------------------------------------------------------
uint16_t checkCRC (uint8_t * inPtrBuf, uint8_t inLen)
{
    uint8_t i;
    uint16_t wCrc = 0 ; // 0xffff;
    while (inLen--) {
        wCrc ^= *(unsigned char *)inPtrBuf++ << 8;
        for (i=0; i < 8; i++)
            wCrc = wCrc & 0x8000 ? (wCrc << 1) ^ 0x1021 : wCrc << 1;
    }

    return wCrc ; //  (wCrc << 8) + ((wCrc & 0xff00) >> 8);
}
//---------------------------------------------------------------------------------------------------
/*!
 * !!! Я ни коем образом не проверяю правильность заполнения поля inData и поэтому возможны косяки!!!
 * 	И мне стыдно за такой маразматический код, но это голый С и иначе не сделать :(
 */
void makeTxBuf (tpCommand inCommand, uint8_t* inData)
{

	makeHeader (inCommand) ;		// Вормируем заголовок

	switch (inCommand) {			// Формируем поле DATA
	  case cmdReset: {
		switch (stBufRx [defPosData]) {
		  uint8_t success = 1 ;
		  case rstSoftware:
			if (HAL_I2C_Master_Transmit(&hi2c2, defSHT3adressWrite, stSHT3reset, sizeof (stSHT3reset), defI2cTimeOut) != HAL_OK)
				success = 0 ;
			writeTxByte (success) ;
		  break ;

		  case rstErr:
		  case rstSoftware:
			if (HAL_I2C_Master_Transmit(&hi2c2, defSHT3adressWrite, stSHT3clear, sizeof (stSHT3clear), defI2cTimeOut) != HAL_OK)
				success = 0 ;
			writeTxByte (success) ;
		  break ;

		  case rstHardware:
			writeTxByte (1) ;
		  break ;

		  default :
			makeHeader (cmdError) ;
			writeTxByte (stBufRx [defPosType]) ;
			writeTxByte (errFormat) ;
		  break ;
		}
	  }
	  break ;

	  case cmdGetData:
		  writeTxByte (stDate.Year) ;
		  writeTxByte (stDate.Month) ;
		  writeTxByte (stDate.Date) ;
		  writeTxByte (stTime.Hours) ;
		  writeTxByte (stTime.Minutes) ;
		  writeTxByte (stTime.Seconds) ;
		  writeTxByte (defVersionSW) ;
		  writeTxByte (0) ;
		  writeTxByte (0) ;
		  if (crcSHT (&stSHT3Data [0], 2) == stSHT3Data [2] && crcSHT (&stSHT3Data [3], 2) == stSHT3Data [5]) {
			  uint32_t temperature = stSHT3Data [0] ;
			  temperature = (temperature << 8) + stSHT3Data [1] ;		// температура усноженная на 100 (что бы передать дробное значение)
			  temperature = ((17500 * temperature) / 65535) - 4500 ;
			  writeTxByte (((uint16_t) temperature) >> 8) ;
			  writeTxByte (((uint16_t) temperature) & 0xFF) ;
			  uint32_t humidity = stSHT3Data [3] ;						// Влажность передаётся только целое значение. Дробная часть тупо откидывается
			  humidity = (humidity << 8) + stSHT3Data [4] ;
			  humidity = (100 * humidity) / 65535 ;
			  writeTxByte (humidity & 0xFF) ;
		  }
		    else {
			  writeTxByte (0xFF) ;
			  writeTxByte (0xFF) ;
			  writeTxByte (0xFF) ;
		    }
	  break ;

	  case cmdError:
		  writeTxByte (stBufRx [defPosType]) ;
		  writeTxByte (*inData) ;
	  break ;

	  case cmdCheckAddr:
		writeTxByte (*stPtrSN >> 8) ;
		writeTxByte (*stPtrSN & 0xFF) ;
	  break ;

	  case cmdChangeAddr: {
		uint16_t sn = (stBufRx [defPosData] << 8) + stBufRx [defPosData + 1] ;	// Формируем полученный серийный номер
		if (writeFlash (sn, stBufRx [defPosData + 2])) {
			makeHeader (inCommand) ;	// Т.к. сменился адрес формируем заголовок по новой
			writeTxByte (*stPtrSN >> 8) ;
			writeTxByte (*stPtrSN & 0xFF) ;
		}
		  else {					// При ошибке записи на флешку вместо стандартного формируем ответ с ошибкой
			makeHeader (cmdError) ;
			writeTxByte (stBufRx [defPosType]) ;
			writeTxByte (errWriteAddr) ;
		  }
	  }
	  break ;

	  case cmdSetSN: {
			uint16_t sn = (stBufRx [defPosData] << 8) + stBufRx [defPosData + 1] ;	// Формируем полученный серийный номер
			if (writeFlash (sn, *stPtrAddr)) {
				writeTxByte (*stPtrSN >> 8) ;
				writeTxByte (*stPtrSN & 0xFF) ;
			}
			  else {					// При ошибке записи на флешку вместо стандартного формируем ответ с ошибкой
				makeHeader (cmdError) ;
				writeTxByte (stBufRx [defPosType]) ;
				writeTxByte (errWriteSN) ;
			  }
	  }
	  break ;

	  case cmdSetDateTime : {
		RTC_TimeTypeDef time ;
		RTC_DateTypeDef date ;

		date.Year = stBufRx [defPosData] ;
		date.Month = stBufRx [defPosData + 1] ;
		date.Date = stBufRx [defPosData + 2] ;
		time.Hours = stBufRx [defPosData + 3] ;
		time.Minutes = stBufRx [defPosData + 4] ;
		time.Seconds = stBufRx [defPosData + 5] ;

		if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) == HAL_OK && HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) == HAL_OK) {
			writeTxByte (stBufRx [defPosData]) ;
			writeTxByte (stBufRx [defPosData + 1]) ;
			writeTxByte (stBufRx [defPosData + 2]) ;
			writeTxByte (stBufRx [defPosData + 3]) ;
			writeTxByte (stBufRx [defPosData + 4]) ;
			writeTxByte (stBufRx [defPosData + 5]) ;
		}
	      else {					// При ошибке записи на флешку вместо стандартного формируем ответ с ошибкой
	    	makeHeader (cmdError) ;
	    	writeTxByte (stBufRx [defPosType]) ;
	    	writeTxByte (errWriteDateTime) ;
	      }
	  }
	  break ;

	  default:
		makeHeader (cmdError) ;		// Формируем ошибку по неправильной команде
		writeTxByte (stBufRx [defPosType]) ;
		writeTxByte (errType) ;
	  break;
	}
												// Рассчитаваем CRC и полностью формируем буфер для передачи
	uint16_t crc = checkCRC (&stBufTx [1], stBufTxId - 1) ;
	writeTxByte (crc & 0xFF) ;
	writeTxByte (crc >> 8) ;
	stBufTx [stBufTxId++] = defEndByte ;
	stBufTx [stBufTxId++] = defStopByte ;		// stBufTxId увеличиваем на единицу, что бы он правильно показывал размер передаваемого буфера
}
//---------------------------------------------------------------------------------------------------
void writeTxByte (uint8_t inByte)
{
	stBufTx [stBufTxId++] = inByte ;
	if (inByte == defStartByte || inByte == defEndByte) stBufTx [stBufTxId++] = inByte ;
}
//---------------------------------------------------------------------------------------------------
bool writeFlash (uint16_t inSN, uint8_t inAddr)
{
	bool retVal = true ;

	FLASH_EraseInitTypeDef eraseHndl ;
	uint32_t errPage = 0 ;

	eraseHndl.TypeErase = FLASH_TYPEERASE_PAGES ;
	eraseHndl.PageAddress = defPtrFlash ;
	eraseHndl.NbPages = 1 ;

	HAL_FLASH_Unlock() ;

	if (HAL_FLASHEx_Erase(&eraseHndl, &errPage) == HAL_OK) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t) stPtrSN, inSN) != HAL_OK ||
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t) stPtrAddr, inAddr) != HAL_OK) retVal = false ;
	}
	  else retVal = false ;

#ifndef DEBUG
	HAL_FLASH_Lock () ;
#endif

	return retVal ;
}
//---------------------------------------------------------------------------------------------------
void makeHeader (tpCommand inCommand)
{
	stBufTxId = defPosStartByte ;
	stBufTx [stBufTxId++] = defStartByte ;
	writeTxByte (stBufRx [defPosAddrHost]) ;			// Адрес устройства с которого пришла команда
	writeTxByte (*stPtrAddr) ;
	writeTxByte (defVersion) ;
	writeTxByte (inCommand) ;
}
//---------------------------------------------------------------------------------------------------
uint8_t crcSHT(uint8_t *pcBlock, uint32_t len)
{
	uint8_t crc = 0xFF;
	uint32_t i;

    while (len--)
    {
        crc ^= *pcBlock++;
        for (i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }

    return crc;
}
//------------------------------------------------------------------------
/*!
 * @brief
 */
HAL_StatusTypeDef getSHT ()
{
	HAL_StatusTypeDef ret ;
	ret = HAL_I2C_Master_Transmit(&hi2c2, defSHT3adressWrite, stSHT3Write, sizeof (stSHT3Write), defI2cTimeOut) ;
	if (ret != HAL_OK) return ret ;
	HAL_Delay(5) ;
	ret = HAL_I2C_Master_Receive(&hi2c2, defSHT3adressWrite, stSHT3Data, sizeof (stSHT3Data), defI2cTimeOut) ;
	if (ret != HAL_OK) return ret ;
	HAL_RTC_GetTime(&hrtc, &stTime, RTC_FORMAT_BIN);	// При успешном чтении данных фиксируем дату и время
	HAL_RTC_GetDate(&hrtc, &stDate, RTC_FORMAT_BIN);
	return ret ;
}
//---------------------------------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
		switch (stState) {
		  case enSend:
			CDC_Transmit_FS(stBufTx, stBufTxId) ;
			stBufRxId = 0 ;				// Т.к. вроде как всё что надо отправили, то подготавливаемся к приёму следущей команды
			stBufTxId = 0 ;
			stBufRxCrcId = 0 ;
			stState = enReady ;
		  break;

		  default:
		  break;
		}
	}
}
//----------------------------------------------------------------------------
