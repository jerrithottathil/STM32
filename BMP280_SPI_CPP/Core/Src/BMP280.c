#include "BMP280.h"
#include <math.h>
#include <string.h>
#include "main.h"

/* ---------------- Init ---------------- */
void BMP280_Init(BMP280_HandleTypeDef *dev, SPI_HandleTypeDef *hspi) {
    dev->spiHandle = hspi;
    dev->p_reference = 0;
    dev->t_fine = 0;
    memset(&dev->measurement, 0, sizeof(dev->measurement));
}

uint8_t BMP280_Initialize(BMP280_HandleTypeDef *dev) {
    if (BMP280_GetID(dev) != BMP280_CHIP_ID) {
        return 1;
    }
    BMP280_Reset(dev);
    BMP280_DelayMs(500);

    BMP280_SetPressureOversampling(dev, oversampling_x16);
    BMP280_SetTemperatureOversampling(dev, oversampling_x2);
    BMP280_SetPowerMode(dev, mode_normal);
    BMP280_SetFilterCoefficient(dev, filter_coeff_16);
    BMP280_SetStandbyTime(dev, standby_time_500us);

    BMP280_ReadCompensationParameters(dev);
    BMP280_SetReferencePressure(dev, 100, 50);

    return 0;
}

/* ---------------- Measurement ---------------- */
void BMP280_Measure(BMP280_HandleTypeDef *dev) {
    uint8_t data[6];
    BMP280_ReadMBRegister(dev, BMP280_REG_DATA, data, 6);

    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    dev->measurement.temperature = (float)BMP280_CompensateTemperature(dev, adc_T) / 100.0f;
    dev->measurement.pressure    = (float)BMP280_CompensatePressure(dev, adc_P) / 256.0f;

    if (dev->p_reference > 0) {
        dev->measurement.altitude =
            (1.0 - powf(dev->measurement.pressure / dev->p_reference, 0.1903f)) * 44330.76f;
    }
}

/* ---------------- Basic functions ---------------- */
void BMP280_Reset(BMP280_HandleTypeDef *dev) {
    BMP280_WriteRegister(dev, BMP280_REG_RESET, BMP280_RESET_VALUE);
}

uint8_t BMP280_GetID(BMP280_HandleTypeDef *dev) {
    return BMP280_ReadRegister(dev, BMP280_REG_ID);
}

void BMP280_ReadCompensationParameters(BMP280_HandleTypeDef *dev) {
    uint8_t buf[24];
    BMP280_ReadMBRegister(dev, BMP280_REG_CALIB, buf, 24);

    dev->compensationParameters.dig_t1 = (buf[1] << 8) | buf[0];
    dev->compensationParameters.dig_t2 = (buf[3] << 8) | buf[2];
    dev->compensationParameters.dig_t3 = (buf[5] << 8) | buf[4];
    dev->compensationParameters.dig_p1 = (buf[7] << 8) | buf[6];
    dev->compensationParameters.dig_p2 = (buf[9] << 8) | buf[8];
    dev->compensationParameters.dig_p3 = (buf[11] << 8) | buf[10];
    dev->compensationParameters.dig_p4 = (buf[13] << 8) | buf[12];
    dev->compensationParameters.dig_p5 = (buf[15] << 8) | buf[14];
    dev->compensationParameters.dig_p6 = (buf[17] << 8) | buf[16];
    dev->compensationParameters.dig_p7 = (buf[19] << 8) | buf[18];
    dev->compensationParameters.dig_p8 = (buf[21] << 8) | buf[20];
    dev->compensationParameters.dig_p9 = (buf[23] << 8) | buf[22];
}

void BMP280_SetReferencePressure(BMP280_HandleTypeDef *dev, uint16_t samples, uint8_t delay) {
    float sum = 0;
    BMP280_DelayMs(500);
    for (uint16_t i = 0; i < samples; i++) {
        BMP280_Measure(dev);
        sum += dev->measurement.pressure;
        BMP280_DelayMs(delay);
    }
    dev->p_reference = sum / samples;
}

/* ---------------- Register Configuration ---------------- */
void BMP280_SetPressureOversampling(BMP280_HandleTypeDef *dev, Oversampling osrs_p) {
    uint8_t ctrl = BMP280_ReadRegister(dev, BMP280_REG_CTRL_MEAS);
    ctrl = (ctrl & 0b11100011) | (osrs_p << 2);
    BMP280_WriteRegister(dev, BMP280_REG_CTRL, ctrl);
}

void BMP280_SetTemperatureOversampling(BMP280_HandleTypeDef *dev, Oversampling osrs_t) {
    uint8_t ctrl = BMP280_ReadRegister(dev, BMP280_REG_CTRL_MEAS);
    ctrl = (ctrl & 0b00011111) | (osrs_t << 5);
    BMP280_WriteRegister(dev, BMP280_REG_CTRL, ctrl);
}

void BMP280_SetPowerMode(BMP280_HandleTypeDef *dev, PowerMode mode) {
    uint8_t ctrl = BMP280_ReadRegister(dev, BMP280_REG_CTRL_MEAS);
    ctrl = (ctrl & 0b11111100) | mode;
    BMP280_WriteRegister(dev, BMP280_REG_CTRL, ctrl);
}

void BMP280_SetStandbyTime(BMP280_HandleTypeDef *dev, StandbyTime t_sb) {
    uint8_t conf = BMP280_ReadRegister(dev, BMP280_REG_CONFIG);
    conf = (conf & 0b00011111) | (t_sb << 5);
    BMP280_WriteRegister(dev, BMP280_REG_CONFIG, conf);
}

void BMP280_SetFilterCoefficient(BMP280_HandleTypeDef *dev, FilterSetting filter) {
    uint8_t conf = BMP280_ReadRegister(dev, BMP280_REG_CONFIG);
    conf = (conf & 0b11100011) | (filter << 2);
    BMP280_WriteRegister(dev, BMP280_REG_CONFIG, conf);
}

/* ---------------- Compensation ---------------- */
int32_t BMP280_CompensateTemperature(BMP280_HandleTypeDef *dev, int32_t uncomp_temp) {
    int32_t var1, var2;
    var1 = ((((uncomp_temp / 8) - ((int32_t)dev->compensationParameters.dig_t1 << 1))) *
           ((int32_t)dev->compensationParameters.dig_t2)) / 2048;
    var2 = (((((uncomp_temp / 16) - ((int32_t)dev->compensationParameters.dig_t1)) *
             ((uncomp_temp / 16) - ((int32_t)dev->compensationParameters.dig_t1))) / 4096) *
           ((int32_t)dev->compensationParameters.dig_t3)) / 16384;
    dev->t_fine = var1 + var2;
    return (dev->t_fine * 5 + 128) / 256;
}

uint32_t BMP280_CompensatePressure(BMP280_HandleTypeDef *dev, int32_t uncomp_pres) {
    int64_t var1, var2, p;

    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->compensationParameters.dig_p6;
    var2 = var2 + ((var1 * (int64_t)dev->compensationParameters.dig_p5) * 131072);
    var2 = var2 + (((int64_t)dev->compensationParameters.dig_p4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)dev->compensationParameters.dig_p3) / 256) +
           ((var1 * (int64_t)dev->compensationParameters.dig_p2) * 4096);
    var1 = ((INT64_C(0x800000000000) + var1) *
           ((int64_t)dev->compensationParameters.dig_p1)) / 8589934592;
    if (var1 == 0) return 0;
    p = 1048576 - uncomp_pres;
    p = (((((p * 2147483648U)) - var2) * 3125) / var1);
    var1 = (((int64_t)dev->compensationParameters.dig_p9) * (p / 8192) * (p / 8192)) / 33554432;
    var2 = (((int64_t)dev->compensationParameters.dig_p8) * p) / 524288;
    p = ((p + var1 + var2) / 256) + (((int64_t)dev->compensationParameters.dig_p7) * 16);
    return (uint32_t)p;
}

/* ---------------- Hardware SPI Helpers ---------------- */
uint8_t BMP280_ReadRegister(BMP280_HandleTypeDef *dev, uint8_t address) {
    BMP280_SPICSNLow();
    BMP280_SPIReadWrite(dev, address);
    uint8_t value = BMP280_SPIReadWrite(dev, 0);
    BMP280_SPICSNHigh();
    return value;
}

void BMP280_WriteRegister(BMP280_HandleTypeDef *dev, uint8_t address, uint8_t value) {
    BMP280_SPICSNLow();
    BMP280_SPIReadWrite(dev, address & BMP280_SPI_MASK_WRITE);
    BMP280_SPIReadWrite(dev, value);
    BMP280_SPICSNHigh();
}

void BMP280_ReadMBRegister(BMP280_HandleTypeDef *dev, uint8_t address, uint8_t *values, uint8_t length) {
    BMP280_SPICSNLow();
    BMP280_SPIReadWrite(dev, address);
    while (length--) {
        *values++ = BMP280_SPIReadWrite(dev, 0);
    }
    BMP280_SPICSNHigh();
}

uint8_t BMP280_SPIReadWrite(BMP280_HandleTypeDef *dev, uint8_t tx_message) {
    uint8_t rx_message = 0xFF;
    HAL_SPI_TransmitReceive(dev->spiHandle, &tx_message, &rx_message, 1, HAL_MAX_DELAY);
    return rx_message;
}

void BMP280_SPICSNHigh(void) {
    HAL_GPIO_WritePin(BMP280_CSN_Pin_GPIO_Port, BMP280_CSN_Pin_Pin, GPIO_PIN_SET);
}

void BMP280_SPICSNLow(void) {
    HAL_GPIO_WritePin(BMP280_CSN_Pin_GPIO_Port, BMP280_CSN_Pin_Pin, GPIO_PIN_RESET);
}

void BMP280_DelayMs(uint32_t ms) {
    HAL_Delay(ms);
}
