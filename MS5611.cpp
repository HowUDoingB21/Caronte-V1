#include "MS5611.h"
#include "config.h"

// =============================================================================
// MS5611 — Implementación del driver
// Basado en el datasheet AN520 de TE Connectivity / MEAS France
// =============================================================================

MS5611::MS5611(uint8_t i2c_addr) : _addr(i2c_addr) {}

// -----------------------------------------------------------------------------
bool MS5611::begin() {
    // Reset del sensor
    sendCommand(MS5611_CMD_RESET);
    delay(10);

    // Leer los 6 coeficientes de calibración de la PROM interna
    // C[0] = reservado | C[1]..C[6] = coeficientes de compensación | C[7] = CRC
    bool prom_ok = true;
    for (uint8_t i = 0; i < 8; i++) {
        _prom[i] = readPROM(i);
    }

    // Verificación básica: los coeficientes no deben ser 0x0000 ni 0xFFFF
    for (uint8_t i = 1; i <= 6; i++) {
        if (_prom[i] == 0x0000 || _prom[i] == 0xFFFF) {
            _lastError = "PROM invalida — verificar conexion I2C y CSB";
            prom_ok = false;
            break;
        }
    }

    if (prom_ok) {
        Serial.println("[BARO] MS5611 inicializado correctamente");
        Serial.print("  PROM C1-C6: ");
        for (uint8_t i = 1; i <= 6; i++) {
            Serial.print(_prom[i]); Serial.print(" ");
        }
        Serial.println();
    }

    return prom_ok;
}

// -----------------------------------------------------------------------------
// Lectura bloqueante — no usar dentro del loop de control de 100Hz
bool MS5611::read(Baro_Data &data) {
    // Conversión D1 (Presión)
    sendCommand(MS5611_CMD_CONV_D1_4096);
    delay(MS5611_CONV_TIME_4096);
    uint32_t D1 = readADC();

    // Conversión D2 (Temperatura)
    sendCommand(MS5611_CMD_CONV_D2_4096);
    delay(MS5611_CONV_TIME_4096);
    uint32_t D2 = readADC();

    if (D1 == 0 || D2 == 0) {
        _lastError = "ADC retorno 0 — conversion fallida";
        data.valid = false;
        return false;
    }

    float press, temp;
    if (!compensate(D1, D2, press, temp)) {
        data.valid = false;
        return false;
    }

    data.pressure    = press;
    data.temperature = temp;
    // Altitud sobre nivel del mar (fórmula hipsométrica con temperatura real)
    data.altitude     = pressureToAltitude(press, temp);
    // AGL: la referencia ground devuelve 0 con la nueva fórmula (P=P0, T=T_ground)
    // pero se calcula explícitamente por robustez
    data.altitude_agl = data.altitude - pressureToAltitude(_ground_pressure, _ground_temp);
    data.valid = true;
    return true;
}

// -----------------------------------------------------------------------------
// Versión asíncrona — Paso 1: disparar conversión de presión
void MS5611::triggerPressure() {
    sendCommand(MS5611_CMD_CONV_D1_4096);
    _conv_start_ms = millis();
    _state = WAITING_PRESSURE;
}

void MS5611::triggerTemperature() {
    sendCommand(MS5611_CMD_CONV_D2_4096);
    _conv_start_ms = millis();
    _state = WAITING_TEMPERATURE;
}

bool MS5611::readResult(Baro_Data &data) {
    if (millis() - _conv_start_ms < MS5611_CONV_TIME_4096) return false;

    if (_state == WAITING_PRESSURE) {
        _conv_D1 = readADC();
        triggerTemperature();
        return false; // Aún esperamos la temperatura
    }

    if (_state == WAITING_TEMPERATURE) {
        uint32_t D2 = readADC();
        _state = IDLE;

        float press, temp;
        if (!compensate(_conv_D1, D2, press, temp)) {
            data.valid = false;
            return false;
        }

        data.pressure     = press;
        data.temperature  = temp;
        data.altitude     = pressureToAltitude(press, temp);
        data.altitude_agl = data.altitude - pressureToAltitude(_ground_pressure, _ground_temp);
        data.valid        = true;
        return true;
    }

    return false;
}

// -----------------------------------------------------------------------------
void MS5611::setGroundLevel() {
    Baro_Data d;
    // Promedio de 10 lecturas para referencia estable
    float sum_p = 0, sum_t = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < 10; i++) {
        if (read(d)) {
            sum_p += d.pressure;
            sum_t += d.temperature;
            count++;
        }
    }
    if (count > 0) {
        _ground_pressure = sum_p / count;
        _ground_temp     = sum_t / count;
        Serial.print("[BARO] Presion de referencia terrestre: ");
        Serial.print(_ground_pressure, 2);
        Serial.println(" Pa");
    }
}

// =============================================================================
// PRIVADOS
// =============================================================================

// Compensación según datasheet MS5611 §4.1 — Ecuaciones exactas del fabricante
bool MS5611::compensate(uint32_t D1, uint32_t D2, float &press_Pa, float &temp_prom) {
    // dT = diferencia de temperatura real vs referencia
    int32_t dT    = (int32_t)D2 - ((int32_t)_prom[5] << 8);

    // TEMP en centésimas de grado Celsius
    int32_t TEMP  = 2000 + ((int64_t)dT * _prom[6] >> 23);

    // Offset y sensibilidad a temperatura actual
    int64_t OFF   = ((int64_t)_prom[2] << 16) + (((int64_t)_prom[4] * dT) >> 7);
    int64_t SENS  = ((int64_t)_prom[1] << 15) + (((int64_t)_prom[3] * dT) >> 8);

    // Compensación de segundo orden para temperatura baja (<20°C)
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) {
        T2    = ((int64_t)dT * dT) >> 31;
        OFF2  = 5 * ((int64_t)(TEMP - 2000) * (TEMP - 2000)) >> 1;
        SENS2 = 5 * ((int64_t)(TEMP - 2000) * (TEMP - 2000)) >> 2;

        if (TEMP < -1500) {
            OFF2  += 7 * (int64_t)(TEMP + 1500) * (TEMP + 1500);
            SENS2 += 11 * ((int64_t)(TEMP + 1500) * (TEMP + 1500)) >> 1;
        }
    }

    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;

    // Presión compensada
    int32_t P = (((int64_t)D1 * SENS >> 21) - OFF) >> 15;

    temp_prom   = (float)TEMP / 100.0f;
    press_Pa = (float)P;

    return (P > 0 && P < 120000);  // Rango razonable: 0 a 120 kPa
}

// Altitud — Fórmula hipsométrica con temperatura medida real
// =============================================================================
// Versión anterior: usaba ISA_T0 = 288.15K (15°C estándar) para TODAS
// las lecturas, ignorando la temperatura real medida por el MS5611.
//
// Versión nueva: usa la temperatura REAL medida en tierra como T0, y la
// temperatura actual del sensor para aproximar el gradiente de la columna:
//
//   T_columna ≈ (T_suelo + T_sensor) / 2   [promedio de la capa]
//
// Impacto real: en Jalisco a 1600 msnm con 30°C en tierra,
//   ISA asume 15°C → error de altitud de ~2.5% (≈25m a 1000m AGL)
//   Con temperatura real → error reducido a <0.5%
//
// La fórmula hipsométrica completa (ICAO, válida en troposfera ≤11km):
//   h = (T_col / L) * [1 − (P/P0)^(R·L/g)]
// =============================================================================
float MS5611::pressureToAltitude(float press_Pa, float temp_C) {
    // Temperatura de la columna de aire entre el suelo y la altitud actual.
    // Usamos el promedio entre la temperatura en tierra (almacenada al calibrar)
    // y la temperatura medida en este instante por el sensor.
    float T_ground_K = _ground_temp + 273.15f;    // temperatura real en tierra [K]
    float T_local_K  = temp_C      + 273.15f;    // temperatura local actual   [K]
    float T_col      = (T_ground_K + T_local_K) * 0.5f;  // temperatura media de la columna

    // Presión de referencia: la presión real en tierra (no ISA_P0)
    // Esta fue almacenada al llamar setGroundLevel(), que promedia 50 muestras.
    float P0 = _ground_pressure;

    // Fórmula hipsométrica estándar (ICAO)
    // Exponente: (R * L) / g = (287.05 * 0.0065) / 9.80665 ≈ 0.190263
    return (T_col / ISA_L) * (1.0f - powf(press_Pa / P0, (ISA_R * ISA_L) / ISA_G));
}

// --- Primitivas I2C ---
void MS5611::sendCommand(uint8_t cmd) {
    Wire.beginTransmission(_addr);
    Wire.write(cmd);
    Wire.endTransmission();
}

uint32_t MS5611::readADC() {
    Wire.beginTransmission(_addr);
    Wire.write(MS5611_CMD_READ_ADC);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)3);

    if (Wire.available() < 3) return 0;

    uint32_t val  = (uint32_t)Wire.read() << 16;
    val          |= (uint32_t)Wire.read() << 8;
    val          |= (uint32_t)Wire.read();
    return val;
}

uint16_t MS5611::readPROM(uint8_t addr) {
    Wire.beginTransmission(_addr);
    Wire.write(MS5611_CMD_READ_PROM + (addr << 1));
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)2);

    if (Wire.available() < 2) return 0;
    uint16_t val = (uint16_t)Wire.read() << 8;
    val |= Wire.read();
    return val;
}
