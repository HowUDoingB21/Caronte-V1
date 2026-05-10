#include "ADXL375.h"
#include <math.h>

// =============================================================================
// ADXL375 — Implementación
// =============================================================================

ADXL375::ADXL375(uint8_t i2c_addr)
    : _addr(i2c_addr), _connected(false),
      _ox(0.0f), _oy(0.0f), _oz(0.0f) {}

// -----------------------------------------------------------------------------
bool ADXL375::begin() {
    // Verificar ID de dispositivo
    uint8_t who = _readReg(ADXL375_REG_DEVID);
    if (who != ADXL375_WHO_AM_I) {
        Serial.print("[ADXL375] WHO_AM_I fallido: 0x");
        Serial.println(who, HEX);
        return false;
    }

    // DATA_FORMAT: ±200g full resolution, right-justified, 13-bit
    // Bit7=0 (self-test off), Bit6=0 (SPI 4-wire, irrelevante), Bit5=0,
    // Bit4=0 (INT active high), Bit3=1 (full resolution), Bit1:0=11 (±200g)
    if (!_writeReg(ADXL375_REG_DATA_FORMAT, 0x0B)) return false;

    // BW_RATE: ODR = 200Hz, no low-power
    if (!_writeReg(ADXL375_REG_BW_RATE, ADXL375_BW_200HZ)) return false;

    // POWER_CTL: modo medición activo (bit3=1)
    if (!_writeReg(ADXL375_REG_POWER_CTL, 0x08)) return false;

    _connected = true;
    Serial.print("[ADXL375] OK — addr=0x");
    Serial.print(_addr, HEX);
    Serial.println("  rango=±200g  ODR=200Hz");
    return true;
}

// -----------------------------------------------------------------------------
bool ADXL375::read(ADXL375_Data &out) {
    out.valid = false;

    uint8_t buf[6];
    if (!_readBurst(ADXL375_REG_DATAX0, buf, 6)) return false;

    // Los datos son little-endian de 16 bits con signo,
    // pero el ADXL375 usa solo 13 bits (bits 15-13 son extensión de signo)
    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    out.accel_x  = (float)raw_x * ADXL375_SCALE_MS2 - _ox;
    out.accel_y  = (float)raw_y * ADXL375_SCALE_MS2 - _oy;
    out.accel_z  = (float)raw_z * ADXL375_SCALE_MS2 - _oz;
    out.magnitude = sqrtf(out.accel_x * out.accel_x +
                          out.accel_y * out.accel_y +
                          out.accel_z * out.accel_z);
    out.valid = true;
    return true;
}

// -----------------------------------------------------------------------------
void ADXL375::setOffsets(float ox_ms2, float oy_ms2, float oz_ms2) {
    _ox = ox_ms2;
    _oy = oy_ms2;
    _oz = oz_ms2;
}

// =============================================================================
// I2C helpers
// =============================================================================

bool ADXL375::_writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

uint8_t ADXL375::_readReg(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

bool ADXL375::_readBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(_addr, len);
    if (Wire.available() < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}
