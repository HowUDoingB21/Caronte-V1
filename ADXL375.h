#pragma once
#include <Arduino.h>
#include <Wire.h>

// =============================================================================
// ADXL375 — Driver I2C para Caronte V1
// =============================================================================
// Acelerómetro de alto rango ±200g, resolución 13 bits (49 mg/LSB)
// Comparte el bus I2C con ICM-45686 y MS5611.
//
// Rol en el sistema:
//   Sensor de respaldo de aceleración para cuando el ICM-45686 se satura
//   (>20g). En vuelo nominal, el ICM proporciona mayor precisión. Cuando el
//   módulo de fusión detecta saturación (|a| > ICM_SATURATION_G × g), el
//   estimador conmuta automáticamente al ADXL375 para la aceleración lineal.
//
// Conexión (esquemático REV 12.0):
//   SDO → GND  → I2C addr = 0x53
//   CS  → 3.3V → modo I2C activo
//   SCL → PB6, SDA → PB7 (bus compartido)
//
// Registros usados:
//   0x00 DEVID       — ID de dispositivo (0xE5 esperado)
//   0x2C BW_RATE     — frecuencia de muestreo
//   0x2D POWER_CTL   — modo medición
//   0x31 DATA_FORMAT — ±200g, self-test off, 13-bit
//   0x32 DATAX0      — bloque de 6 bytes: X0,X1,Y0,Y1,Z0,Z1
// =============================================================================

// Resolución en m/s²: 49 mg/LSB × 9.80665 m/s²/g / 1000
#define ADXL375_MG_PER_LSB     49.0f
#define ADXL375_SCALE_MS2      (ADXL375_MG_PER_LSB * 9.80665f / 1000.0f)
#define ADXL375_WHO_AM_I       0xE5

// Dirección I2C según SDO:
//   SDO = GND → 0x53  (default en el esquemático)
//   SDO = VCC → 0x1D
#define ADXL375_ADDR_SDO_LOW   0x53
#define ADXL375_ADDR_SDO_HIGH  0x1D

// Registros
#define ADXL375_REG_DEVID       0x00
#define ADXL375_REG_BW_RATE     0x2C
#define ADXL375_REG_POWER_CTL   0x2D
#define ADXL375_REG_DATA_FORMAT 0x31
#define ADXL375_REG_DATAX0      0x32

// BW_RATE: ODR 200Hz (0x0B) — suficiente para 100Hz del loop
#define ADXL375_BW_200HZ        0x0B

// --- Estructura de datos de salida ---
struct ADXL375_Data {
    float accel_x;   // [m/s²]  — eje X
    float accel_y;   // [m/s²]  — eje Y
    float accel_z;   // [m/s²]  — eje Z
    float magnitude; // [m/s²]  — magnitud del vector
    bool  valid;     // lectura exitosa
};


// =============================================================================
class ADXL375 {
public:
    explicit ADXL375(uint8_t i2c_addr = ADXL375_ADDR_SDO_LOW);

    // Inicialización — retorna true si el sensor responde
    bool begin();

    // Lee los tres ejes en una ráfaga I2C de 6 bytes
    bool read(ADXL375_Data &out);

    // Aplica calibración simple (offset de fábrica / montaje)
    // Llama opcionalmente antes del despegue para restar el bias estático
    void setOffsets(float ox_ms2, float oy_ms2, float oz_ms2);

    bool isConnected() const { return _connected; }

private:
    uint8_t _addr;
    bool    _connected;
    float   _ox, _oy, _oz;   // offsets [m/s²]

    bool     _writeReg(uint8_t reg, uint8_t val);
    uint8_t  _readReg(uint8_t reg);
    bool     _readBurst(uint8_t reg, uint8_t *buf, uint8_t len);
};
