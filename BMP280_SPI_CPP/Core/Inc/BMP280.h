#ifndef BMP280_H_
#define BMP280_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "BMP280_defines.h"

typedef struct
{
    float temperature;
    float pressure;
    float altitude;
} BMP280_Measurement;

typedef struct
{
    SPI_HandleTypeDef *spiHandle;
    float p_reference;
    int32_t t_fine;

    struct {
        uint16_t dig_t1;
        int16_t dig_t2;
        int16_t dig_t3;
        uint16_t dig_p1;
        int16_t dig_p2;
        int16_t dig_p3;
        int16_t dig_p4;
        int16_t dig_p5;
        int16_t dig_p6;
        int16_t dig_p7;
        int16_t dig_p8;
        int16_t dig_p9;
    } compensationParameters;

    BMP280_Measurement measurement;
} BMP280_HandleTypeDef;

/* API */
void     BMP280_Init(BMP280_HandleTypeDef *dev, SPI_HandleTypeDef *hspi);
uint8_t  BMP280_Initialize(BMP280_HandleTypeDef *dev);
void     BMP280_Measure(BMP280_HandleTypeDef *dev);

/* Low-level helpers */
void     BMP280_Reset(BMP280_HandleTypeDef *dev);
uint8_t  BMP280_GetID(BMP280_HandleTypeDef *dev);
void     BMP280_ReadCompensationParameters(BMP280_HandleTypeDef *dev);
void     BMP280_SetReferencePressure(BMP280_HandleTypeDef *dev, uint16_t samples, uint8_t delay);
void     BMP280_SetPressureOversampling(BMP280_HandleTypeDef *dev, Oversampling osrs_p);
void     BMP280_SetTemperatureOversampling(BMP280_HandleTypeDef *dev, Oversampling osrs_t);
void     BMP280_SetPowerMode(BMP280_HandleTypeDef *dev, PowerMode mode);
void     BMP280_SetStandbyTime(BMP280_HandleTypeDef *dev, StandbyTime t_sb);
void     BMP280_SetFilterCoefficient(BMP280_HandleTypeDef *dev, FilterSetting filter);

int32_t  BMP280_CompensateTemperature(BMP280_HandleTypeDef *dev, int32_t adc_T);
uint32_t BMP280_CompensatePressure(BMP280_HandleTypeDef *dev, int32_t adc_P);

uint8_t  BMP280_ReadRegister(BMP280_HandleTypeDef *dev, uint8_t address);
void     BMP280_WriteRegister(BMP280_HandleTypeDef *dev, uint8_t address, uint8_t value);
void     BMP280_ReadMBRegister(BMP280_HandleTypeDef *dev, uint8_t address, uint8_t *values, uint8_t length);

uint8_t  BMP280_SPIReadWrite(BMP280_HandleTypeDef *dev, uint8_t tx_message);
void     BMP280_SPICSNHigh(void);
void     BMP280_SPICSNLow(void);
void     BMP280_DelayMs(uint32_t ms);

#endif /* BMP280_H_ */
