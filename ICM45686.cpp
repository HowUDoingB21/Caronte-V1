#include "ICM45686.h"

// =============================================================================
// ICM-45686 — Implementación del driver
// =============================================================================

ICM45686::ICM45686(uint8_t i2c_addr) : _addr(i2c_addr) {}

// -----------------------------------------------------------------------------
bool ICM45686::begin() {
    Wire.begin();

    // Pequeña espera al encendido
    delay(100);

    // Seleccionar Banco 0 explícitamente antes de cualquier lectura.
    // El ICM-45686 tiene registros organizados en bancos. Si el chip
    // quedó en un banco diferente (por reset previo o fallo de bus),
    // todos los registros devuelven 0x00 incluyendo WHO_AM_I.
    // BANK_SEL (0x76) = 0x00 → Banco 0
    writeReg(ICM_REG_BANK_SEL, 0x00);
    delay(10);

    // Verificar identidad del chip
    uint8_t whoami = readReg(ICM_REG_WHO_AM_I);
    if (whoami != ICM45686_WHO_AM_I) {
        _lastError = "WHO_AM_I mismatch — verifica direccion I2C y datasheet";
        Serial.print("[ICM] WHO_AM_I esperado: 0x");
        Serial.print(ICM45686_WHO_AM_I, HEX);
        Serial.print(" | recibido: 0x");
        Serial.println(whoami, HEX);
        return false;
    }

    // Asegurar Banco 0 antes de configurar
    writeReg(ICM_REG_BANK_SEL, 0x00);

    // Salir de modo sleep, habilitar accel y gyro
    // PWR_MGMT0: bits [3:2] = gyro mode, bits [1:0] = accel mode
    // 0x0F = Accel + Gyro en modo Low Noise
    writeReg(ICM_REG_PWR_MGMT0, 0x0F);
    delay(50);  // Esperar arranque del sensor

    // Configurar Giroscopio: ±2000 dps, ODR 1kHz
    writeReg(ICM_REG_GYRO_CONFIG0, ICM_GYRO_FS_2000DPS | ICM_ODR_1KHZ);

    // Configurar Acelerómetro: ±16g, ODR 1kHz
    writeReg(ICM_REG_ACCEL_CONFIG0, ICM_ACCEL_FS_16G | ICM_ODR_1KHZ);
    delay(10);

    // Escala acelerómetro: ±16g → sensibilidad 2048 LSB/g → factor a m/s²
    _accel_scale = (16.0f * 9.80665f) / 32768.0f;

    // Escala giroscopio: ±2000 dps → sensibilidad 16.4 LSB/(°/s) → a rad/s
    _gyro_scale  = (2000.0f * DEG_TO_RAD) / 32768.0f;

    Serial.println("[ICM] ICM-45686 inicializado correctamente");
    return true;
}

// -----------------------------------------------------------------------------
bool ICM45686::read(IMU_Data &data) {
    // Asegurar Banco 0 antes de leer datos
    writeReg(ICM_REG_BANK_SEL, 0x00);

    // Leer 14 bytes comenzando en TEMP_MSB (0x1D):
    // [0-1]=Temp, [2-7]=Accel XYZ, [8-13]=Gyro XYZ
    uint8_t buf[14];
    readBurst(ICM_REG_TEMP_MSB, buf, 14);

    // Temperatura
    int16_t raw_temp = ((int16_t)buf[0] << 8) | buf[1];
    data.temperature = (float)raw_temp / 132.48f + 25.0f;

    // Acelerómetro
    int16_t raw_ax = ((int16_t)buf[2] << 8) | buf[3];
    int16_t raw_ay = ((int16_t)buf[4] << 8) | buf[5];
    int16_t raw_az = ((int16_t)buf[6] << 8) | buf[7];

    data.accel_x = rawToFloat(raw_ax, _accel_scale) - _accel_offset_x;
    data.accel_y = rawToFloat(raw_ay, _accel_scale) - _accel_offset_y;
    data.accel_z = rawToFloat(raw_az, _accel_scale) - _accel_offset_z;

    // Giroscopio
    int16_t raw_gx = ((int16_t)buf[8]  << 8) | buf[9];
    int16_t raw_gy = ((int16_t)buf[10] << 8) | buf[11];
    int16_t raw_gz = ((int16_t)buf[12] << 8) | buf[13];

    data.gyro_x = rawToFloat(raw_gx, _gyro_scale) - _gyro_offset_x;
    data.gyro_y = rawToFloat(raw_gy, _gyro_scale) - _gyro_offset_y;
    data.gyro_z = rawToFloat(raw_gz, _gyro_scale) - _gyro_offset_z;

    data.valid = true;
    return true;
}

// -----------------------------------------------------------------------------
void ICM45686::calibrate(uint16_t samples) {
    Serial.println("[ICM] Calibrando... mantener el cohete estático");
    delay(500);

    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    double sum_ax = 0, sum_ay = 0, sum_az = 0;

    IMU_Data d;
    for (uint16_t i = 0; i < samples; i++) {
        read(d);
        sum_gx += d.gyro_x;
        sum_gy += d.gyro_y;
        sum_gz += d.gyro_z;
        sum_ax += d.accel_x;
        sum_ay += d.accel_y;
        sum_az += d.accel_z;
        delay(5);
    }

    _gyro_offset_x  = (float)(sum_gx / samples);
    _gyro_offset_y  = (float)(sum_gy / samples);
    _gyro_offset_z  = (float)(sum_gz / samples);
    _accel_offset_x = (float)(sum_ax / samples);
    _accel_offset_y = (float)(sum_ay / samples);
    // Para Z no quitamos el offset de gravedad (g ≈ 9.80665 m/s² en reposo)
    _accel_offset_z = (float)(sum_az / samples) - 9.80665f;

    Serial.println("[ICM] Calibración completada");
    Serial.print("  Gyro offset XYZ [rad/s]: ");
    Serial.print(_gyro_offset_x, 5); Serial.print(", ");
    Serial.print(_gyro_offset_y, 5); Serial.print(", ");
    Serial.println(_gyro_offset_z, 5);
}

// =============================================================================
// PRIMITIVAS I2C PRIVADAS
// =============================================================================

void ICM45686::writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t ICM45686::readReg(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

void ICM45686::readBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        buf[i] = Wire.read();
    }
}

float ICM45686::rawToFloat(int16_t raw, float scale) {
    return (float)raw * scale;
}
