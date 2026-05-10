#pragma once
#include <Arduino.h>
#include <Wire.h>

// =============================================================================
// DRIVER MS5611-01BA03 — Barómetro de alta resolución
// Interfaz: I2C (CSB = VCC → dirección 0x77)
// =============================================================================
// El MS5611 requiere leer coeficientes de calibración de su PROM interna
// (8 registros de 16 bits) al inicio, y usarlos en cada conversión para
// compensar temperatura y presión según la secuencia del datasheet oficial.
// =============================================================================


// --- Comandos ---
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_CONV_D1_256  0x40    // Presión OSR=256  (0.9ms)
#define MS5611_CMD_CONV_D1_512  0x42    // Presión OSR=512  (1.2ms)
#define MS5611_CMD_CONV_D1_1024 0x44    // Presión OSR=1024 (2.3ms)
#define MS5611_CMD_CONV_D1_2048 0x46    // Presión OSR=2048 (4.6ms)
#define MS5611_CMD_CONV_D1_4096 0x48    // Presión OSR=4096 (9.1ms) — máx resolución
#define MS5611_CMD_CONV_D2_256  0x50    // Temperatura OSR=256
#define MS5611_CMD_CONV_D2_512  0x52
#define MS5611_CMD_CONV_D2_1024 0x54
#define MS5611_CMD_CONV_D2_2048 0x56
#define MS5611_CMD_CONV_D2_4096 0x58
#define MS5611_CMD_READ_ADC     0x00
#define MS5611_CMD_READ_PROM    0xA0    // + (addr << 1) para cada coeficiente

// Tiempos de conversión [ms] — depende del OSR seleccionado
#define MS5611_CONV_TIME_4096   10      // 9.1ms + margen de seguridad


// --- Estructura de datos de salida ---
struct Baro_Data {
    float pressure;     // [Pa] Presión absoluta
    float temperature;  // [°C] Temperatura compensada
    float altitude;     // [m]  Altitud estimada sobre nivel del mar (ISA)
    float altitude_agl; // [m]  Altitud sobre el punto de lanzamiento
    bool  valid;
};


// =============================================================================
class MS5611 {
public:
    MS5611(uint8_t i2c_addr = 0x77);

    // Inicialización — lee PROM y retorna true si OK
    bool begin();

    // Lee presión y temperatura compensadas
    // ATENCIÓN: Esta función tarda ~20ms (2 conversiones ADC)
    // Para uso en loop de control, usar las versiones asíncronas (ver abajo)
    bool read(Baro_Data &data);

    // Versión asíncrona para no bloquear el loop principal
    // Llama triggerPressure(), espera 10ms, luego readResult()
    void  triggerPressure();
    void  triggerTemperature();
    bool  readResult(Baro_Data &data);

    // Establece la referencia de altitud 0 (calibrar en tierra antes del vuelo)
    void setGroundLevel();

    const char* getLastError() const { return _lastError; }

private:
    uint8_t  _addr;
    uint16_t _prom[8];      // Coeficientes de calibración PROM
    float    _ground_pressure = 101325.0f;
    float    _ground_temp     = 288.15f;

    // Estado de la conversión asíncrona
    enum ConvState { IDLE, WAITING_PRESSURE, WAITING_TEMPERATURE };
    ConvState   _state = IDLE;
    uint32_t    _conv_D1 = 0;  // Valor ADC de presión
    unsigned long _conv_start_ms = 0;

    const char* _lastError = "None";

    // Fórmulas de compensación según datasheet MS5611 §4.1
    bool compensate(uint32_t D1, uint32_t D2, float &press_Pa, float &temp_C);

    // Altitud por modelo ISA simplificado
    float pressureToAltitude(float press_Pa, float temp_C);

    // Primitivas I2C
    void    sendCommand(uint8_t cmd);
    uint32_t readADC();
    uint16_t readPROM(uint8_t addr);
};
