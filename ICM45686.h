#pragma once
#include <Arduino.h>
#include <Wire.h>

// =============================================================================
// DRIVER ICM-45686 — IMU (Acelerómetro + Giroscopio)
// Interfaz: I2C
// Módulo: GY-601N1
// =============================================================================
// Nota: El ICM-45686 es parte de la familia ICM-4x de TDK InvenSense.
// Su mapa de registros es similar al ICM-42688-P pero con diferencias en
// algunos registros de configuración. Verifica con el datasheet oficial si
// el WHO_AM_I devuelve un valor inesperado.
// =============================================================================


// --- Registros principales (Bank 0) ---
// Banco 0 — registros principales
#define ICM_REG_BANK_SEL        0x76   // Selector de banco — escribir 0x00 para Banco 0
#define ICM_REG_WHO_AM_I        0x72   // Debe retornar 0xE9 para ICM-45686 (solo Banco 0)
#define ICM_REG_DEVICE_CONFIG   0x11   // Soft reset (bit 0)
#define ICM_REG_PWR_MGMT0       0x10   // Control de energía accel+gyro
#define ICM_REG_GYRO_CONFIG0    0x20   // FS y ODR del giróscopo
#define ICM_REG_ACCEL_CONFIG0   0x21   // FS y ODR del acelerómetro
#define ICM_REG_INT_CONFIG      0x14
#define ICM_REG_INT_STATUS      0x2D

// --- Registros de datos ---
// Bloque de datos — el driver usa burst read desde TEMP_MSB
// Orden: TEMP(2) ACCEL_XYZ(6) GYRO_XYZ(6) = 14 bytes
#define ICM_REG_TEMP_MSB        0x1D   // Inicio del burst de 14 bytes
// Nota: no declarar ACCEL_X_LSB=0x20 ni ACCEL_Y_MSB=0x21
//       para evitar colisión con GYRO_CONFIG0 y ACCEL_CONFIG0

// --- Configuración de rango ---
// Giroscopio - GYRO_CONFIG0[7:5]
#define ICM_GYRO_FS_2000DPS     0x00    // ±2000 °/s — máxima dinámica
#define ICM_GYRO_FS_1000DPS     0x20
#define ICM_GYRO_FS_500DPS      0x40
#define ICM_GYRO_FS_250DPS      0x60

// Acelerómetro - ACCEL_CONFIG0[7:5]
#define ICM_ACCEL_FS_16G        0x00    // ±16g — máxima dinámica para cohete
#define ICM_ACCEL_FS_8G         0x20
#define ICM_ACCEL_FS_4G         0x40
#define ICM_ACCEL_FS_2G         0x60

// ODR (Output Data Rate) - bits [3:0] de CONFIG0
#define ICM_ODR_1KHZ            0x06
#define ICM_ODR_500HZ           0x07
#define ICM_ODR_200HZ           0x08
#define ICM_ODR_100HZ           0x09

// WHO_AM_I esperado
#define ICM45686_WHO_AM_I       0xE9


// --- Estructura de datos de salida ---
struct IMU_Data {
    float accel_x;      // [m/s²]
    float accel_y;      // [m/s²]
    float accel_z;      // [m/s²]
    float gyro_x;       // [rad/s] — Roll rate
    float gyro_y;       // [rad/s] — Pitch rate
    float gyro_z;       // [rad/s] — Yaw rate
    float temperature;  // [°C]
    bool  valid;        // true si la lectura fue exitosa

    // ── ADXL375 — aceleración de alto rango ──────────────────────────────
    // Cuando imu_saturated=true, el Estimator usa estos valores en lugar
    // de accel_x/y/z para la predicción del EKF. El giróscopo del ICM
    // siempre se usa (no se satura antes de 2000 dps).
    float accel_hg_x;      // [m/s²] ADXL375 — válido cuando imu_saturated
    float accel_hg_y;      // [m/s²] ADXL375
    float accel_hg_z;      // [m/s²] ADXL375
    bool  imu_saturated;   // true = ICM cerca o en saturación, usar accel_hg_*
    bool  adxl_valid;      // true = ADXL375 disponible y leyendo correctamente
};


// =============================================================================
class ICM45686 {
public:
    ICM45686(uint8_t i2c_addr = 0x68);

    // Inicialización — retorna true si el sensor responde correctamente
    bool begin();

    // Lee todos los datos del sensor en una sola ráfaga de I2C
    bool read(IMU_Data &data);

    // Calibración de offsets en reposo (llamar antes del vuelo)
    void calibrate(uint16_t samples = 500);

    // Retorna el último error como string (para debugging)
    const char* getLastError() const { return _lastError; }

private:
    uint8_t  _addr;
    float    _accel_scale;      // LSB a m/s²
    float    _gyro_scale;       // LSB a rad/s

    // Offsets de calibración
    float    _gyro_offset_x = 0;
    float    _gyro_offset_y = 0;
    float    _gyro_offset_z = 0;
    float    _accel_offset_x = 0;
    float    _accel_offset_y = 0;
    float    _accel_offset_z = 0;

    const char* _lastError = "None";

    // Primitivas I2C
    void    writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    void    readBurst(uint8_t reg, uint8_t *buf, uint8_t len);

    // Convierte raw int16 a float con escala
    float   rawToFloat(int16_t raw, float scale);
};
