#pragma once
#include <Arduino.h>
#include "config.h"
#include "ICM45686.h"
#include "MS5611.h"
#include "IMU_Calibration.h"
#include "EKF9.h"
#include "GPS_INS.h"
#include "GPS_NEO6M.h"
#include "ADXL375.h"

// =============================================================================
// ESTIMADOR DE ESTADO — Caronte V1  [Fase 3: GPS/INS loosely-coupled]
// =============================================================================
//
// Arquitectura completa:
//
//   IMU_Data (crudo)
//       │
//       ▼
//   IMU_Calibration.apply()   ← 6-posiciones: bias + escala + misalignment
//       │
//       ▼
//   ZUPT (solo PAD)           ← refina bias del giróscopo antes del despegue
//       │
//       ▼
//   EKF9.predict()            ← actitud + altitud @ 100Hz
//       │
//       ├─── (baro.valid) → EKF9.update()        ← fusión baro + accel ref
//       │
//       ├─── (gps.fix, ~1Hz) → EKF9.updateGPSAlt()  ← ancla altitud absoluta
//       │                                             (anti-drift barométrico)
//       │
//       └─── (gps.fix, ~1Hz) → GPS_INS.updateGPS()  ← corrige velocidad horizontal
//                GPS_INS.predict() @ 100Hz         ← propaga vN, vE con INS
//
//   State ← lectura de EKF9 + GPS_INS → FSM / PID / telemetría
//
// Mejoras sobre Fase 2:
//   - vN, vE ya no son integración pura (unbounded drift)
//   - Kalman 2D corrige velocidad horizontal con GPS a cada fix
//   - Altitud GPS ancla el EKF contra deriva barométrica de largo plazo
//   - State incluye sigma_vH para cuantificar confianza en velocidad horizontal
// =============================================================================

struct State {
    // Actitud [rad]
    float roll;
    float pitch;
    float yaw;

    // Velocidades angulares corregidas [rad/s]
    float roll_rate;
    float pitch_rate;
    float yaw_rate;

    // Altitud y cinemática vertical
    float altitude_agl;
    float vertical_speed;
    float accel_vertical;

    // Velocidad 3D inercial [m/s]
    float vx;
    float vy;
    float speed;

    // Densidad del aire [kg/m³]
    float air_density;

    // Flags
    bool  attitude_valid;
    bool  altitude_valid;
    bool  high_accel;        // true cuando |accel| > ACCEL_THRESHOLD
    bool  imu_saturated;     // true cuando ICM-45686 está saturado (>18g)
                             // → Estimator usa ADXL375 para aceleración lineal

    // EKF — diagnóstico / telemetría
    float alpha_eq;          // α equivalente del EKF
    float var_altitude;      // Varianza de altitud [m²]
    float var_roll;          // Varianza de roll [rad²]
    float var_pitch;         // Varianza de pitch [rad²]

    // GPS/INS Fase 3
    float sigma_vH;          // Desviación estándar de velocidad horizontal [m/s]
    uint16_t gps_updates;    // Número de correcciones GPS recibidas

    // ZUPT
    bool  zupt_active;
    float zupt_stable_s;
};


class StateEstimator {
public:
    explicit StateEstimator(IMU_Calibration *imuCal = nullptr,
                           ADXL375        *adxl    = nullptr);

    void begin();
    // update() recibe imu ya con campos accel_hg_* e imu_saturated rellenos
    // por el bucle principal ANTES de llamar a update().
    void update(const IMU_Data &imu, const Baro_Data &baro, float dt);

    const State& getState() const { return _state; }

    void setPadPhase(bool is_pad) { _in_pad = is_pad; }
    void resetYaw()      { _ekf.resetYaw(0.0f); _state.yaw = 0.0f; }
    void resetAttitude();

    void getGyroBias(float &bx, float &by, float &bz) const;

    // ── Fase 3: GPS ──────────────────────────────────────────────────────
    // Llamar cada vez que GPS_NEO6M.newDataAvailable() == true.
    // Internamente decide si la calidad del fix es suficiente para
    // actualizar el GPS_INS y/o el EKF de altitud.
    void updateGPS(const GPS_Data &gps);

    // Llamar una vez en setup() cuando el GPS tiene fix estable en la rampa.
    // Registra la altitud MSL como referencia para convertir GPS MSL → AGL.
    void setGroundAltMSL(float alt_msl) {
        _ekf.setGroundAltMSL(alt_msl);
    }

    // Acceso al EKF y GPS_INS para debug/logging
    const EKF9&    ekf()    const { return _ekf; }
    const GPS_INS& gpsIns() const { return _gps_ins; }

    // ── ZUPT — aprobación explícita del operador ─────────────────────────
    //
    // getZUPTBias(): devuelve el bias residual acumulado por el ZUPT desde
    //   el arranque. La estación terrena lo muestra al operador antes de
    //   confirmar. Valores típicos esperados: < 0.01 rad/s por eje.
    //   Si algún componente supera ~0.05 rad/s el sensor puede estar
    //   degradado o el cohete no estaba quieto durante la convergencia.
    //
    // commitZUPTBias(): aplica _zupt_gyro_bias a _cal.gyro_bias en RAM
    //   (sin escribir a Flash) y resetea el acumulador del ZUPT a cero.
    //   Solo debe llamarse cuando zupt_stable_s es suficiente (≥ 10s) y
    //   el operador ha confirmado que el cohete está estático en la rampa.
    //   El EKF ya tiene la corrección inyectada — este commit solo actualiza
    //   la calibración RAM para que apply() reste el bias correcto en vuelo.
    //
    // saveZUPTBias(): llama a commitZUPTBias() y además persiste en Flash.
    //   Usar con precaución — sobreescribe la calibración de giróscopo de la
    //   calibración de 6 posiciones con el bias refinado por ZUPT.
    //
    void getZUPTBias(float &bx, float &by, float &bz) const {
        bx = _zupt_gyro_bias[0];
        by = _zupt_gyro_bias[1];
        bz = _zupt_gyro_bias[2];
    }
    bool commitZUPTBias();   // aplica a RAM, resetea acumulador
    bool saveZUPTBias();     // commitZUPTBias() + guarda en Flash

private:
    State            _state;
    IMU_Calibration *_cal;
    EKF9             _ekf;
    GPS_INS          _gps_ins;

    // ── ZUPT ─────────────────────────────────────────────────────────────
    bool     _in_pad           = false;

    static constexpr float    ZUPT_GYRO_THRESH  = 0.015f;
    static constexpr float    ZUPT_ACCEL_THRESH = 0.5f;
    static constexpr uint16_t ZUPT_MIN_STABLE   = 100;
    static constexpr float    ZUPT_LR           = 0.005f;

    uint16_t _zupt_stable_count = 0;
    float    _zupt_stable_s     = 0.0f;
    float    _zupt_gyro_bias[3] = {0.0f, 0.0f, 0.0f};

    void _updateZUPT(const IMU_Data &imu, float dt);

    // ── INS horizontal ────────────────────────────────────────────────────
    float _vx = 0.0f;
    float _vy = 0.0f;

    void  _projectAccelInertial(const IMU_Data &imu,
                                float &ax_out, float &ay_out, float &az_out);

    float _estimateAirDensity(float pressure_Pa, float temperature_C);

    static constexpr float ACCEL_THRESHOLD = 15.0f;   // ~1.5g — flag high_accel

    ADXL375 *_adxl;          // nullptr si no está instalado
};
