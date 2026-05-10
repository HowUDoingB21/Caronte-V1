#include "IMU_Calibration.h"
#include <math.h>

// =============================================================================
// Mensajes de estado para telemetría
// =============================================================================
const char* calStatusMsg(CalStatus s) {
    switch(s) {
        case CalStatus::IDLE:          return "CAL:IDLE";
        case CalStatus::WAITING_POS_0: return "CAL:PON COHETE VERTICAL NARIZ ARRIBA (+Z)";
        case CalStatus::COLLECTING_0:  return "CAL:RECOGIENDO POS0...";
        case CalStatus::WAITING_POS_1: return "CAL:PON COHETE VERTICAL NARIZ ABAJO (-Z)";
        case CalStatus::COLLECTING_1:  return "CAL:RECOGIENDO POS1...";
        case CalStatus::WAITING_POS_2: return "CAL:PON NARIZ APUNTANDO HACIA ARRIBA (+X)";
        case CalStatus::COLLECTING_2:  return "CAL:RECOGIENDO POS2...";
        case CalStatus::WAITING_POS_3: return "CAL:PON COLA APUNTANDO HACIA ARRIBA (-X)";
        case CalStatus::COLLECTING_3:  return "CAL:RECOGIENDO POS3...";
        case CalStatus::WAITING_POS_4: return "CAL:PON ALETA Y APUNTANDO HACIA ARRIBA (+Y)";
        case CalStatus::COLLECTING_4:  return "CAL:RECOGIENDO POS4...";
        case CalStatus::WAITING_POS_5: return "CAL:PON ALETA Y OPUESTA HACIA ARRIBA (-Y)";
        case CalStatus::COLLECTING_5:  return "CAL:RECOGIENDO POS5...";
        case CalStatus::COMPUTING:     return "CAL:CALCULANDO COEFICIENTES...";
        case CalStatus::DONE_OK:       return "CAL:CALIBRACION GUARDADA OK";
        case CalStatus::DONE_ERROR:    return "CAL:ERROR EN CALIBRACION";
        case CalStatus::ABORTED:       return "CAL:ABORTADA";
        default:                       return "CAL:DESCONOCIDO";
    }
}

// =============================================================================
IMU_Calibration::IMU_Calibration(ICM45686 &imu) : _imu(imu) {
    _cal = calDataIdentity();
    memset(_meas, 0, sizeof(_meas));
    _sample_count = 0;
    _current_pos  = 0;
}

// -----------------------------------------------------------------------------
bool IMU_Calibration::begin() {
    CalData loaded;
    EEPROM.get(CAL_EEPROM_ADDR, loaded);

    if (loaded.magic == CAL_MAGIC) {
        _cal = loaded;
        Serial.println("[CAL] Calibracion cargada desde Flash:");
        Serial.print  ("  Bias accel [m/s2]: ");
        Serial.print(_cal.accel_bias[0], 4); Serial.print(", ");
        Serial.print(_cal.accel_bias[1], 4); Serial.print(", ");
        Serial.println(_cal.accel_bias[2], 4);
        Serial.print  ("  Scale accel:       ");
        Serial.print(_cal.accel_scale[0], 4); Serial.print(", ");
        Serial.print(_cal.accel_scale[1], 4); Serial.print(", ");
        Serial.println(_cal.accel_scale[2], 4);
        Serial.print  ("  Bias gyro [rad/s]: ");
        Serial.print(_cal.gyro_bias[0], 5); Serial.print(", ");
        Serial.print(_cal.gyro_bias[1], 5); Serial.print(", ");
        Serial.println(_cal.gyro_bias[2], 5);
        return true;
    }

    Serial.println("[CAL] Sin calibracion en Flash — usando identidad");
    _cal = calDataIdentity();
    return false;
}

// -----------------------------------------------------------------------------
bool IMU_Calibration::save() {
    _cal.magic = CAL_MAGIC;
    EEPROM.put(CAL_EEPROM_ADDR, _cal);
    Serial.println("[CAL] Calibracion guardada en Flash");
    return true;
}

// -----------------------------------------------------------------------------
void IMU_Calibration::startCalibration() {
    memset(_meas, 0, sizeof(_meas));
    _sample_count = 0;
    _current_pos  = 0;
    _status = CalStatus::WAITING_POS_0;
    Serial.println("[CAL] Calibracion iniciada — 6 posiciones requeridas");
    Serial.println(calStatusMsg(_status));
}

// -----------------------------------------------------------------------------
void IMU_Calibration::nextPosition() {
    // Solo avanza si estamos en un estado de espera
    bool is_waiting = (_status == CalStatus::WAITING_POS_0 ||
                       _status == CalStatus::WAITING_POS_1 ||
                       _status == CalStatus::WAITING_POS_2 ||
                       _status == CalStatus::WAITING_POS_3 ||
                       _status == CalStatus::WAITING_POS_4 ||
                       _status == CalStatus::WAITING_POS_5);
    if (!is_waiting) return;

    // Pasar al estado de recolección de la posición actual
    _sample_count = 0;
    _status = (CalStatus)((uint8_t)_status + 1);  // WAITING→COLLECTING
    Serial.print("[CAL] Recogiendo posicion ");
    Serial.println(_current_pos);
}

// -----------------------------------------------------------------------------
void IMU_Calibration::abort() {
    _status = CalStatus::ABORTED;
    Serial.println("[CAL] Calibracion abortada");
}

// -----------------------------------------------------------------------------
// Llamar en cada ciclo del loop mientras isActive() == true
// Retorna true cuando la calibración termina (OK o error)
bool IMU_Calibration::update() {
    if (!isActive()) return false;

    // Solo actuar en estados de recolección
    bool is_collecting = (_status == CalStatus::COLLECTING_0 ||
                          _status == CalStatus::COLLECTING_1 ||
                          _status == CalStatus::COLLECTING_2 ||
                          _status == CalStatus::COLLECTING_3 ||
                          _status == CalStatus::COLLECTING_4 ||
                          _status == CalStatus::COLLECTING_5);
    if (!is_collecting) return false;

    _collectSample();
    _sample_count++;

    if (_sample_count < CAL_SAMPLES) return false;

    // Posición completada — promediar
    _meas[_current_pos][0] /= CAL_SAMPLES;
    _meas[_current_pos][1] /= CAL_SAMPLES;
    _meas[_current_pos][2] /= CAL_SAMPLES;

    Serial.print("[CAL] Pos ");
    Serial.print(_current_pos);
    Serial.print(" promedio [m/s2]: ");
    Serial.print(_meas[_current_pos][0], 4); Serial.print(", ");
    Serial.print(_meas[_current_pos][1], 4); Serial.print(", ");
    Serial.println(_meas[_current_pos][2], 4);

    _current_pos++;

    if (_current_pos < CAL_NUM_POSITIONS) {
        // Avanzar al siguiente WAITING
        // Los estados son: W0,C0,W1,C1,...,W5,C5 = 1,2,3,4,...,11,12
        // Después de C_k terminamos en W_{k+1}
        _sample_count = 0;
        _status = (CalStatus)((uint8_t)_status + 1);  // COLLECTING→WAITING_next
        Serial.println(calStatusMsg(_status));
        return false;
    }

    // Todas las posiciones recolectadas → calcular
    _status = CalStatus::COMPUTING;
    if (_compute()) {
        save();
        _status = CalStatus::DONE_OK;
        Serial.println("[CAL] Exito — coeficientes guardados");
    } else {
        _status = CalStatus::DONE_ERROR;
        Serial.println("[CAL] Error en el calculo de coeficientes");
    }
    return true;
}

// -----------------------------------------------------------------------------
// Recoge y acumula una muestra cruda (sin calibración de offset del driver)
void IMU_Calibration::_collectSample() {
    IMU_Data d;
    _imu.read(d);
    if (!d.valid) return;

    // Acumular sin aplicar la calibración preexistente del driver
    // (la calibración de 6 posiciones reemplaza al offset simple)
    _meas[_current_pos][0] += d.accel_x;
    _meas[_current_pos][1] += d.accel_y;
    _meas[_current_pos][2] += d.accel_z;
}

// =============================================================================
// CÁLCULO DE COEFICIENTES
// =============================================================================
// Convención de posiciones:
//   [0]: +Z up → ideal [0,  0, +g]
//   [1]: -Z up → ideal [0,  0, -g]
//   [2]: +X up → ideal [+g, 0,  0]
//   [3]: -X up → ideal [-g, 0,  0]
//   [4]: +Y up → ideal [0, +g,  0]
//   [5]: -Y up → ideal [0, -g,  0]
//
// Paso 1: Bias por eje (promedio de los dos extremos cancela la gravedad)
//   bias_x = (m2[x] + m3[x]) / 2
//   bias_y = (m4[y] + m5[y]) / 2
//   bias_z = (m0[z] + m1[z]) / 2
//
// Paso 2: Escala por eje (diferencia de los extremos normalizada a 2g)
//   scale_x = (m2[x] - m3[x]) / (2 * g)
//   scale_y = (m4[y] - m5[y]) / (2 * g)
//   scale_z = (m0[z] - m1[z]) / (2 * g)
//
// Paso 3: Vectores de eje corregidos (tras bias y scale)
//   e_x = (m2 - bias) / (scale * g)   → columna X del marco sensor
//   e_y = (m4 - bias) / (scale * g)   → columna Y
//   e_z = (m0 - bias) / (scale * g)   → columna Z
//
// Paso 4: Matriz de misalignment M = inv([e_x | e_y | e_z])
//   Si el sensor fuera perfecto, M = I.
//   La desviación de la identidad es el misalignment a corregir.
// =============================================================================
bool IMU_Calibration::_compute() {
    const float g = ISA_G;

    // ── Paso 1: Bias ──────────────────────────────────────────────────────
    float bx = (_meas[2][0] + _meas[3][0]) * 0.5f;
    float by = (_meas[4][1] + _meas[5][1]) * 0.5f;
    float bz = (_meas[0][2] + _meas[1][2]) * 0.5f;

    // ── Paso 2: Escala ───────────────────────────────────────────────────
    float sx = (_meas[2][0] - _meas[3][0]) / (2.0f * g);
    float sy = (_meas[4][1] - _meas[5][1]) / (2.0f * g);
    float sz = (_meas[0][2] - _meas[1][2]) / (2.0f * g);

    // Guardar la escala calculada antes de comprobarla
    Serial.print("[CAL] Scale x/y/z: ");
    Serial.print(sx, 4); Serial.print(" / ");
    Serial.print(sy, 4); Serial.print(" / ");
    Serial.println(sz, 4);

    // Sanidad: scale debe estar cerca de 1.0 (±20%)
    if (fabsf(sx - 1.0f) > 0.2f || fabsf(sy - 1.0f) > 0.2f || fabsf(sz - 1.0f) > 0.2f) {
        Serial.println("[CAL] Error: escala fuera de rango — verificar orientaciones");
        return false;
    }

    // ── Paso 3: Vectores de eje normalizados (e_i = col_i de A) ─────────
    // Para cada posición positiva, restamos bias y dividimos por (scale * g)
    // El resultado son los ejes del sensor expresados en el marco del cuerpo
    float ex[3], ey[3], ez[3];
    for (int i = 0; i < 3; i++) {
        float bias[3] = {bx, by, bz};
        float scale[3] = {sx, sy, sz};

        ex[i] = (_meas[2][i] - bias[i]) / (scale[i] * g);   // +X up
        ey[i] = (_meas[4][i] - bias[i]) / (scale[i] * g);   // +Y up
        ez[i] = (_meas[0][i] - bias[i]) / (scale[i] * g);   // +Z up
    }

    // ── Paso 4: Misalignment = inv([ex | ey | ez]) ──────────────────────
    // La matriz A tiene ex, ey, ez como COLUMNAS:
    //   A = [ex[0] ey[0] ez[0]]
    //       [ex[1] ey[1] ez[1]]
    //       [ex[2] ey[2] ez[2]]
    float A[9] = {
        ex[0], ey[0], ez[0],
        ex[1], ey[1], ez[1],
        ex[2], ey[2], ez[2]
    };

    float Ainv[9];
    if (!_invert3x3(A, Ainv)) {
        Serial.println("[CAL] Error: matriz singular — posiciones incorrectas");
        return false;
    }

    // ── Guardar resultados ───────────────────────────────────────────────
    _cal.accel_bias[0]  = bx;
    _cal.accel_bias[1]  = by;
    _cal.accel_bias[2]  = bz;
    _cal.accel_scale[0] = sx;
    _cal.accel_scale[1] = sy;
    _cal.accel_scale[2] = sz;
    memcpy(_cal.misalign, Ainv, sizeof(Ainv));
    // El bias del giroscopio lo mantiene ZUPT — no se modifica aquí

    return true;
}

// =============================================================================
// APLICAR CALIBRACIÓN A UNA LECTURA DEL IMU
// =============================================================================
// Secuencia:
//   1. a_corrected_i = (a_raw_i - bias_i) / scale_i   [bias + escala]
//   2. a_body        = M · a_corrected                 [misalignment]
//   3. gyro_body_i   = gyro_raw_i - gyro_bias_i        [solo bias del gyro]
// =============================================================================
void IMU_Calibration::apply(IMU_Data &data) const {
    // ── Acelerómetro ─────────────────────────────────────────────────────
    float cx = (data.accel_x - _cal.accel_bias[0]) / _cal.accel_scale[0];
    float cy = (data.accel_y - _cal.accel_bias[1]) / _cal.accel_scale[1];
    float cz = (data.accel_z - _cal.accel_bias[2]) / _cal.accel_scale[2];

    const float *M = _cal.misalign;
    data.accel_x = M[0]*cx + M[1]*cy + M[2]*cz;
    data.accel_y = M[3]*cx + M[4]*cy + M[5]*cz;
    data.accel_z = M[6]*cx + M[7]*cy + M[8]*cz;

    // ── Giroscopio ───────────────────────────────────────────────────────
    data.gyro_x -= _cal.gyro_bias[0];
    data.gyro_y -= _cal.gyro_bias[1];
    data.gyro_z -= _cal.gyro_bias[2];
}

// -----------------------------------------------------------------------------
void IMU_Calibration::updateGyroBias(float bx, float by, float bz) {
    _cal.gyro_bias[0] = bx;
    _cal.gyro_bias[1] = by;
    _cal.gyro_bias[2] = bz;
}

// =============================================================================
// INVERSA ANALÍTICA DE MATRIZ 3×3 (Regla de Cramer)
// =============================================================================
// A = [a0 a1 a2]   (row-major: fila i, col j → A[3*i+j])
//     [a3 a4 a5]
//     [a6 a7 a8]
// =============================================================================
bool IMU_Calibration::_invert3x3(const float A[9], float Ainv[9]) {
    float a=A[0],b=A[1],c=A[2];
    float d=A[3],e=A[4],f=A[5];
    float g=A[6],h=A[7],i=A[8];

    float det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);

    if (fabsf(det) < 1e-6f) return false;

    float inv_det = 1.0f / det;

    Ainv[0] =  (e*i - f*h) * inv_det;
    Ainv[1] = -(b*i - c*h) * inv_det;
    Ainv[2] =  (b*f - c*e) * inv_det;
    Ainv[3] = -(d*i - f*g) * inv_det;
    Ainv[4] =  (a*i - c*g) * inv_det;
    Ainv[5] = -(a*f - c*d) * inv_det;
    Ainv[6] =  (d*h - e*g) * inv_det;
    Ainv[7] = -(a*h - b*g) * inv_det;
    Ainv[8] =  (a*e - b*d) * inv_det;

    return true;
}
