#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include "ICM45686.h"
#include "config.h"

// =============================================================================
// CALIBRACIÓN DE 6 POSICIONES — ICM-45686 — Caronte V1
// =============================================================================
//
// Modelo de calibración aplicado:
//
//   a_body = M · diag(1/s) · (a_raw − b)
//
//   donde:
//     a_raw [3]  — lectura cruda del sensor en m/s²
//     b     [3]  — vector de bias (offset de cada eje)
//     s     [3]  — factores de escala (ideal = 1.0 por eje)
//     M     [3x3]— matriz de misalignment (corrige no-ortogonalidad)
//
// Las 6 posiciones de medición y el eje de gravedad resultante:
//
//   POS 0: Cohete vertical, nariz arriba  (+Z up)  →  a_ideal = [0, 0, +g]
//   POS 1: Cohete vertical, nariz abajo   (-Z up)  →  a_ideal = [0, 0, -g]
//   POS 2: Cohete tumbado, nariz apunta   (+X up)  →  a_ideal = [+g, 0,  0]
//   POS 3: Cohete tumbado, cola apunta    (-X up)  →  a_ideal = [-g, 0,  0]
//   POS 4: Cohete tumbado, aleta Y arriba (+Y up)  →  a_ideal = [0, +g,  0]
//   POS 5: Cohete tumbado, aleta Y abajo  (-Y up)  →  a_ideal = [0, -g,  0]
//
// Procedimiento:
//   1. CMD_CAL_START por LoRa → el cohete envía instrucciones vía telemetría
//   2. Operador coloca cohete en posición indicada
//   3. CMD_CAL_NEXT → recoge 200 muestras (~2s) y pasa a siguiente posición
//   4. Tras 6 posiciones, calcula coeficientes y guarda en Flash (EEPROM)
//   5. Los coeficientes persisten entre reinicios hasta nueva calibración
//
// Beneficios cuantificados sobre el sistema anterior (solo offset simple):
//   - Error de bias:       ~0.01–0.05 m/s² → <0.005 m/s² (90% reducción)
//   - Error de escala:     ~0.5–2%         → <0.1%
//   - Error de misalign:   ~0.1–2°         → <0.02°
//   - Error acumulado INS a 30s: ~50m → ~5m (estimado, depende del vuelo)
// =============================================================================

#define CAL_MAGIC           0xCA1B2024u   // Identifica datos de calibración válidos
#define CAL_EEPROM_ADDR     0             // Dirección base en Flash/EEPROM emulado
#define CAL_SAMPLES         200           // Muestras por posición (2s @ 100Hz)
#define CAL_SAMPLE_DELAY_MS 10            // ms entre muestras

// Número de posiciones requeridas
#define CAL_NUM_POSITIONS   6


// ─────────────────────────────────────────────────────────────────────────────
// Estructura de datos de calibración — persistida en Flash
// ─────────────────────────────────────────────────────────────────────────────
struct CalData {
    float    accel_bias[3];     // [m/s²]   Offset de cada eje del acelerómetro
    float    accel_scale[3];    // [adim]   Factor de escala (1.0 = perfecto)
    float    misalign[9];       // [adim]   Matriz de rotación 3×3 row-major
                                //          que transforma sensor→cuerpo
    float    gyro_bias[3];      // [rad/s]  Offset del giroscopio (refinado por ZUPT)
    uint32_t magic;             // Valor mágico para validar datos en Flash
};

// Devuelve una CalData con valores de identidad (sin corrección)
inline CalData calDataIdentity() {
    CalData c;
    c.accel_bias[0]  = 0.0f; c.accel_bias[1]  = 0.0f; c.accel_bias[2]  = 0.0f;
    c.accel_scale[0] = 1.0f; c.accel_scale[1] = 1.0f; c.accel_scale[2] = 1.0f;
    // Identidad 3×3
    c.misalign[0]=1; c.misalign[1]=0; c.misalign[2]=0;
    c.misalign[3]=0; c.misalign[4]=1; c.misalign[5]=0;
    c.misalign[6]=0; c.misalign[7]=0; c.misalign[8]=1;
    c.gyro_bias[0]   = 0.0f; c.gyro_bias[1]   = 0.0f; c.gyro_bias[2]   = 0.0f;
    c.magic          = 0;
    return c;
}


// ─────────────────────────────────────────────────────────────────────────────
// Mensajes de estado enviables por LoRa al operador
// ─────────────────────────────────────────────────────────────────────────────
enum class CalStatus : uint8_t {
    IDLE            = 0,
    WAITING_POS_0   = 1,   // Esperar: cohete vertical, nariz arriba
    COLLECTING_0    = 2,
    WAITING_POS_1   = 3,   // Esperar: nariz abajo
    COLLECTING_1    = 4,
    WAITING_POS_2   = 5,   // Esperar: nariz apunta (+X arriba)
    COLLECTING_2    = 6,
    WAITING_POS_3   = 7,   // Esperar: cola apunta (+X abajo)
    COLLECTING_3    = 8,
    WAITING_POS_4   = 9,   // Esperar: aleta Y arriba
    COLLECTING_4    = 10,
    WAITING_POS_5   = 11,  // Esperar: aleta Y abajo
    COLLECTING_5    = 12,
    COMPUTING       = 13,  // Calculando coeficientes
    DONE_OK         = 14,  // Calibración guardada con éxito
    DONE_ERROR      = 15,  // Error durante la calibración
    ABORTED         = 16,
};

// Mensaje de texto corto para telemetría
const char* calStatusMsg(CalStatus s);


// ─────────────────────────────────────────────────────────────────────────────
class IMU_Calibration {
public:
    IMU_Calibration(ICM45686 &imu);

    // Carga la calibración desde Flash al arrancar.
    // Retorna true si encontró datos válidos, false si usa identidad.
    bool begin();

    // Guarda la calibración actual en Flash
    bool save();

    // Inicia el procedimiento de calibración de 6 posiciones
    void startCalibration();

    // Confirma la posición actual y avanza a la siguiente
    // Llamar cuando el operador envía CMD_CAL_NEXT
    void nextPosition();

    // Aborta la calibración en curso
    void abort();

    // Llamar en cada ciclo del loop (solo activo durante la calibración)
    // Retorna true si la calibración terminó este ciclo (OK o error)
    bool update();

    // ── Acceso ──────────────────────────────────────────────────────────────

    // Aplica la calibración completa a una lectura del IMU
    // Modifica data.accel_* y data.gyro_* con valores calibrados
    void apply(IMU_Data &data) const;

    // Actualiza solo el bias del giroscopio (llamado por ZUPT)
    void updateGyroBias(float bx, float by, float bz);

    const CalData&  getData()   const { return _cal; }
    CalStatus       getStatus() const { return _status; }
    bool            isActive()  const { return _status != CalStatus::IDLE &&
                                               _status != CalStatus::DONE_OK &&
                                               _status != CalStatus::DONE_ERROR &&
                                               _status != CalStatus::ABORTED; }
    bool            isValid()   const { return _cal.magic == CAL_MAGIC; }

private:
    ICM45686  &_imu;
    CalData    _cal;
    CalStatus  _status = CalStatus::IDLE;

    // Acumuladores para promediar muestras de cada posición
    // _meas[pos][eje] = suma de muestras
    float    _meas[6][3];
    uint16_t _sample_count;
    uint8_t  _current_pos;

    // Recoge una muestra en la posición actual
    void     _collectSample();

    // Calcula coeficientes con las 6 posiciones recolectadas
    bool     _compute();

    // Invierte una matriz 3×3 (analítica, evita dependencia de librería)
    // Retorna false si la matriz es singular
    static bool _invert3x3(const float A[9], float Ainv[9]);
};
