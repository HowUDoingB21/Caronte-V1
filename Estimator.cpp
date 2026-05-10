#include "Estimator.h"
#include "IMU_Calibration.h"
#include <math.h>

// =============================================================================
// ESTIMADOR DE ESTADO — Implementación Fase 3 (EKF9 + GPS_INS)
// =============================================================================

StateEstimator::StateEstimator(IMU_Calibration *imuCal, ADXL375 *adxl)
    : _cal(imuCal), _adxl(adxl) {
    memset(&_state, 0, sizeof(_state));
}

// -----------------------------------------------------------------------------
void StateEstimator::begin() {
    memset(&_state, 0, sizeof(_state));
    _vx = 0.0f;
    _vy = 0.0f;
    _zupt_stable_count = 0;
    _zupt_stable_s     = 0.0f;
    memset(_zupt_gyro_bias, 0, sizeof(_zupt_gyro_bias));

    _ekf.begin(0.0f, 0.0f, 0.0f, 0.0f);
    _gps_ins.reset();

    Serial.println("[EST] Estimador Fase 3 (EKF9 + GPS_INS) inicializado");
}

// -----------------------------------------------------------------------------
void StateEstimator::resetAttitude() {
    _ekf.begin(0.0f, 0.0f, 0.0f, _ekf.altitude());
    _gps_ins.reset();
    _state.vx = _state.vy = _state.speed = 0.0f;
    Serial.println("[EST] Actitud e INS horizontal reseteados");
}

// -----------------------------------------------------------------------------
void StateEstimator::getGyroBias(float &bx, float &by, float &bz) const {
    bx = _zupt_gyro_bias[0] + _ekf.gyroBiasX();
    by = _zupt_gyro_bias[1] + _ekf.gyroBiasY();
    bz = _zupt_gyro_bias[2] + _ekf.gyroBiasZ();
}

// =============================================================================
// UPDATE PRINCIPAL — 100 Hz
// =============================================================================
void StateEstimator::update(const IMU_Data &imuRaw, const Baro_Data &baro, float dt) {

    // ── Paso 0: Calibración de 6 posiciones ─────────────────────────────
    IMU_Data imu = imuRaw;
    if (_cal && _cal->isValid()) {
        _cal->apply(imu);
    }

    // Guardar velocidades angulares calibradas (antes del ZUPT, para referencia)
    _state.roll_rate  = imu.gyro_x;
    _state.pitch_rate = imu.gyro_y;
    _state.yaw_rate   = imu.gyro_z;

    // ── Paso 0b: Detección de saturación ICM y handoff al ADXL375 ────────
    // El bucle principal ya rellenó imu.accel_hg_* e imu.imu_saturated.
    // Aquí decidimos qué aceleración usar en el EKF:
    //   - imu_saturated=false  → accel ICM (alta precisión)
    //   - imu_saturated=true   → accel ADXL375 (alto rango)
    // El giróscopo del ICM NUNCA se sustituye — no se satura antes de
    // la destrucción física del cohete (rango ±2000 dps).
    //
    // Estrategia de fusión en EKF:
    //   Cuando el ADXL375 está activo, inyectamos su aceleración como si
    //   fuera la del ICM, pero aumentamos Q (ruido de proceso) para reflejar
    //   la menor precisión del sensor (49 mg/LSB vs ~0.1 mg del ICM).
    //   El EKF se adapta automáticamente via la covarianza P.

    _state.imu_saturated = imu.imu_saturated;

    // Seleccionar fuente de aceleración para EKF
    IMU_Data imu_ekf = imu;  // copia — giróscopo siempre del ICM
    if (imu.imu_saturated && imu.adxl_valid) {
        imu_ekf.accel_x = imu.accel_hg_x;
        imu_ekf.accel_y = imu.accel_hg_y;
        imu_ekf.accel_z = imu.accel_hg_z;
        // Aumentar covarianza de proceso del EKF para reflejar menor precisión
        // del ADXL375 (el EKF bajará el peso de la predicción aceleromética)
        _ekf.setHighGMode(true);
    } else {
        _ekf.setHighGMode(false);
    }

    // ── Paso 1: ZUPT (solo en PAD) ────────────────────────────────────────
    if (_in_pad) {
        _updateZUPT(imu, dt);
    } else {
        _state.zupt_active = false;
    }

    // ── Paso 2: EKF — predicción ──────────────────────────────────────────
    if (imu_ekf.valid) {
        _ekf.predict(imu_ekf, dt);
        _state.attitude_valid = true;

        // Flag high_accel para FSM — usar magnitud de la fuente activa
        float a_mag = sqrtf(imu_ekf.accel_x*imu_ekf.accel_x +
                            imu_ekf.accel_y*imu_ekf.accel_y +
                            imu_ekf.accel_z*imu_ekf.accel_z);
        _state.high_accel = (a_mag > ACCEL_THRESHOLD);

    } else {
        _state.attitude_valid = false;
    }

    // ── Paso 3: EKF — actualización con barométrica ───────────────────────
    if (baro.valid && imu_ekf.valid) {
        _ekf.update(baro.altitude_agl, imu_ekf);
        _state.altitude_valid = true;
    } else {
        _state.altitude_valid = false;
    }

    // ── Paso 4: Leer estado estimado del EKF ──────────────────────────────
    _state.roll          = _ekf.roll();
    _state.pitch         = _ekf.pitch();
    _state.yaw           = _ekf.yaw();
    _state.altitude_agl  = _ekf.altitude();
    _state.vertical_speed= _ekf.verticalSpeed();
    _state.alpha_eq      = _ekf.alphaEquivalent();
    _state.var_altitude  = _ekf.varAltitude();
    _state.var_roll      = _ekf.varRoll();
    _state.var_pitch     = _ekf.varPitch();
    _state.zupt_stable_s = _zupt_stable_s;
    _state.gps_updates   = _gps_ins.gpsUpdates();

    // ── Paso 5: Cinemática vertical e INS horizontal ──────────────────────
    if (baro.valid && imu_ekf.valid) {
        float ax_in, ay_in, az_in;
        _projectAccelInertial(imu_ekf, ax_in, ay_in, az_in);
        _state.accel_vertical = az_in;

        if (!_in_pad) {
            // GPS_INS: propagar velocidad horizontal con aceleración inercial
            _gps_ins.predict(ax_in, ay_in, dt);
        } else {
            _gps_ins.reset();
        }

        _state.vx     = _gps_ins.vN();
        _state.vy     = _gps_ins.vE();
        _state.speed  = sqrtf(_gps_ins.vN() * _gps_ins.vN() +
                               _gps_ins.vE() * _gps_ins.vE() +
                               _state.vertical_speed * _state.vertical_speed);
        _state.sigma_vH  = sqrtf(_gps_ins.sigmaVN() * _gps_ins.sigmaVN() +
                                  _gps_ins.sigmaVE() * _gps_ins.sigmaVE()) * 0.707f;
        _state.gps_updates = _gps_ins.gpsUpdates();

        _state.air_density = _estimateAirDensity(baro.pressure, baro.temperature);
    }
}

// =============================================================================
// ZUPT — Zero Velocity Update
// =============================================================================
// Detecta periodos de quietud en la rampa y refina el bias residual del gyro.
// En lugar de corregirlo directamente en el IMU_Data, lo inyecta al EKF
// como corrección del estado x[5-7] (bias residual del gyro).
//
// La diferencia respecto a Fase 1:
//   - Fase 1: el ZUPT ajustaba _zupt_gyro_bias y se restaba en update()
//   - Fase 2: el ZUPT inyecta la corrección en el EKF vía injectGyroBiasCorrection()
//             El EKF también estima su propio bias (x[5-7]) simultáneamente.
//             Los dos mecanismos coexisten — el ZUPT actúa más rápido (EMA),
//             el EKF actúa de forma óptima durante el vuelo.
// =============================================================================
void StateEstimator::_updateZUPT(const IMU_Data &imu, float dt) {

    float a_mag = sqrtf(imu.accel_x*imu.accel_x +
                        imu.accel_y*imu.accel_y +
                        imu.accel_z*imu.accel_z);

    bool gyro_still  = (fabsf(imu.gyro_x) < ZUPT_GYRO_THRESH &&
                        fabsf(imu.gyro_y) < ZUPT_GYRO_THRESH &&
                        fabsf(imu.gyro_z) < ZUPT_GYRO_THRESH);
    bool accel_still = (fabsf(a_mag - ISA_G) < ZUPT_ACCEL_THRESH);

    if (gyro_still && accel_still) {
        _zupt_stable_count++;
        _zupt_stable_s += dt;
    } else {
        _zupt_stable_count = 0;
    }

    if (_zupt_stable_count >= ZUPT_MIN_STABLE) {
        // Corrección incremental del bias por EMA
        float delta[3];
        delta[0] = ZUPT_LR * (imu.gyro_x - _zupt_gyro_bias[0]);
        delta[1] = ZUPT_LR * (imu.gyro_y - _zupt_gyro_bias[1]);
        delta[2] = ZUPT_LR * (imu.gyro_z - _zupt_gyro_bias[2]);

        _zupt_gyro_bias[0] += delta[0];
        _zupt_gyro_bias[1] += delta[1];
        _zupt_gyro_bias[2] += delta[2];

        // Inyectar corrección incremental al EKF (x[5-7] = bias residual del gyro).
        // Factor 0.5 para no sobre-corregir — el EKF también estima este bias
        // simultáneamente a través de su propio proceso de predicción/update.
        // El ZUPT actúa más rápido (EMA directa) mientras el EKF converge
        // de forma óptima durante el vuelo.
        //
        // IMPORTANTE: NO se toca _cal->gyro_bias aquí.
        // La calibración de 6 posiciones es una medición estática y no debe
        // ser modificada automáticamente por el estimador en tiempo real.
        // Razones:
        //   1. Doble-conteo: apply() ya resta _cal.gyro_bias al dato crudo.
        //      Si el ZUPT también lo modifica, la misma corrección se aplica
        //      dos veces (una en apply(), otra en el EKF x[5-7]).
        //   2. Contaminación: vibración ambiental (viento, grúa de lanzamiento)
        //      durante PAD puede hacer converger _zupt_gyro_bias a un valor
        //      incorrecto. Si ese valor se escribe en _cal, afecta el vuelo
        //      entero sin posibilidad de recovery.
        //   3. Aprobación explícita: el operador puede revisar el delta ZUPT
        //      en la estación terrena y confirmar con CMD_ZUPT_COMMIT solo
        //      cuando el cohete está quieto y estable en la rampa.
        _ekf.injectGyroBiasCorrection(delta[0] * 0.5f,
                                       delta[1] * 0.5f,
                                       delta[2] * 0.5f);

        _state.zupt_active = true;
    } else {
        _state.zupt_active = false;
    }

    _state.zupt_stable_s = _zupt_stable_s;
    _state.gps_updates   = _gps_ins.gpsUpdates();
}

// =============================================================================
// PROYECCIÓN DE ACELERACIÓN AL MARCO INERCIAL (para INS horizontal)
// =============================================================================
// Usa la actitud del EKF para la rotación.
// =============================================================================
void StateEstimator::_projectAccelInertial(const IMU_Data &imu,
                                            float &ax_out, float &ay_out,
                                            float &az_out) {
    const float phi   = _ekf.roll();
    const float theta = _ekf.pitch();
    const float psi   = _ekf.yaw();

    float sr = sinf(phi),   cr = cosf(phi);
    float sp = sinf(theta), cp = cosf(theta);
    float sy = sinf(psi),   cy = cosf(psi);

    float ax = imu.accel_x;
    float ay = imu.accel_y;
    float az = imu.accel_z;

    ax_out =  ax*(cp*cy) + ay*(sr*sp*cy - cr*sy) + az*(cr*sp*cy + sr*sy);
    ay_out =  ax*(cp*sy) + ay*(sr*sp*sy + cr*cy) + az*(cr*sp*sy - sr*cy);
    az_out = -ax*sp      + ay*(sr*cp)             + az*(cr*cp)   - ISA_G;
}

// =============================================================================
float StateEstimator::_estimateAirDensity(float pressure_Pa, float temperature_C) {
    return pressure_Pa / (ISA_R * (temperature_C + 273.15f));
}

// =============================================================================
// ACTUALIZACIÓN GPS — Fase 3
// =============================================================================
// Llamar desde el loop principal cuando GPS_NEO6M.newDataAvailable() == true.
//
// Realiza dos correcciones independientes:
//   1. GPS_INS.updateGPS()  → corrige vN, vE del Kalman horizontal
//   2. EKF9.updateGPSAlt()  → ancla altitud AGL contra drift barométrico
//
// Calidad mínima requerida:
//   - gps.fix == true   (tiene al menos 4 satélites)
//   - gps.valid == true (HDOP < 5.0, definido en GPS_NEO6M)
// =============================================================================
void StateEstimator::updateGPS(const GPS_Data &gps) {

    if (!gps.fix || !gps.valid) return;

    // ── 1. Corrección de velocidad horizontal ────────────────────────────
    _gps_ins.updateGPS(gps.speed_ms, gps.course_deg, gps.hdop);

    // ── 2. Corrección de altitud GPS → EKF9 ──────────────────────────────
    // Solo cuando el GPS tiene buena señal y hay referencia terrestre
    if (_ekf.groundAltMSL() > 0.0f && gps.altitude_msl > -500.0f) {
        _ekf.updateGPSAlt(gps.altitude_msl, gps.hdop);
    }

    Serial.print("[GPS] Update #");
    Serial.print(_gps_ins.gpsUpdates());
    Serial.print(" | vN="); Serial.print(_gps_ins.vN(), 1);
    Serial.print(" vE="); Serial.print(_gps_ins.vE(), 1);
    Serial.print(" | σvH="); Serial.print(_state.sigma_vH, 2);
    Serial.print(" m/s | HDOP="); Serial.println(gps.hdop, 1);
}

// =============================================================================
// ZUPT COMMIT — aprobación explícita del operador
// =============================================================================

// Aplica el bias acumulado por ZUPT a la calibración en RAM y resetea el
// acumulador. No escribe en Flash.
//
// Flujo de datos después del commit:
//   raw IMU → apply() (resta _cal.gyro_bias actualizado) → EKF
//
// El EKF ya tiene las correcciones inyectadas incrementalmente durante PAD.
// Al commitear, apply() pasa a restar el bias completo desde el primer ciclo
// del vuelo, y el acumulador ZUPT arranca desde cero para posibles refinamientos
// adicionales si el cohete permanece en rampa más tiempo.
//
// Retorna false si no hay calibración válida o si el ZUPT no ha convergido
// (zupt_stable_count < ZUPT_MIN_STABLE).
bool StateEstimator::commitZUPTBias() {
    if (!_cal || !_cal->isValid()) {
        Serial.println("[ZUPT] WARN: commit rechazado — sin calibracion base valida");
        return false;
    }
    if (_zupt_stable_count < ZUPT_MIN_STABLE) {
        Serial.println("[ZUPT] WARN: commit rechazado — ZUPT aun no convergio");
        return false;
    }

    // Aplicar delta acumulado a la calibración RAM
    const CalData& base = _cal->getData();
    float new_bx = base.gyro_bias[0] + _zupt_gyro_bias[0];
    float new_by = base.gyro_bias[1] + _zupt_gyro_bias[1];
    float new_bz = base.gyro_bias[2] + _zupt_gyro_bias[2];
    _cal->updateGyroBias(new_bx, new_by, new_bz);

    Serial.print("[ZUPT] Bias commiteado a RAM: bx=");
    Serial.print(_zupt_gyro_bias[0] * 1000.0f, 2); Serial.print(" mrad/s  by=");
    Serial.print(_zupt_gyro_bias[1] * 1000.0f, 2); Serial.print(" mrad/s  bz=");
    Serial.print(_zupt_gyro_bias[2] * 1000.0f, 2); Serial.println(" mrad/s");

    // Resetear acumulador — el ZUPT parte de cero desde la nueva base
    _zupt_gyro_bias[0] = 0.0f;
    _zupt_gyro_bias[1] = 0.0f;
    _zupt_gyro_bias[2] = 0.0f;
    _zupt_stable_count = 0;

    return true;
}

// commit + guardar en Flash
bool StateEstimator::saveZUPTBias() {
    if (!commitZUPTBias()) return false;
    bool ok = _cal->save();
    if (ok) {
        Serial.println("[ZUPT] Bias guardado en Flash");
    } else {
        Serial.println("[ZUPT] ERROR: fallo al guardar en Flash");
    }
    return ok;
}
