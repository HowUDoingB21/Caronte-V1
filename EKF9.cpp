#include "config.h"
#include "EKF9.h"
#include <math.h>
#include <string.h>

// =============================================================================
// EKF9 — Implementación
// =============================================================================

EKF9::EKF9() : _high_g_mode(false) {
    memset(_x, 0, sizeof(_x));
    memset(_Pcov, 0, sizeof(_Pcov));
    _alpha_eq = 0.9997f;
}

// -----------------------------------------------------------------------------
void EKF9::begin(float roll0, float pitch0, float yaw0, float alt0) {
    _high_g_mode = false;
    memset(_x, 0, sizeof(_x));
    _x[0] = alt0;
    _x[2] = roll0;
    _x[3] = pitch0;
    _x[4] = yaw0;

    // Covarianza inicial: refleja nuestra incertidumbre de arranque
    memset(_Pcov, 0, sizeof(_Pcov));
    _Pcov[0][0] = 5.0f;       // altitud: ±2.2m (baro sin calibrar)
    _Pcov[1][1] = 1.0f;       // velocidad vertical: ±1m/s
    _Pcov[2][2] = 0.01f;      // roll:  ±0.1 rad (~6°)
    _Pcov[3][3] = 0.01f;      // pitch: ±0.1 rad
    _Pcov[4][4] = 0.1f;       // yaw:   ±0.3 rad (~18°, gyro integra desde 0)
    _Pcov[5][5] = 1e-4f;      // bias gyro X: ±0.01 rad/s
    _Pcov[6][6] = 1e-4f;      // bias gyro Y
    _Pcov[7][7] = 1e-4f;      // bias gyro Z
    _Pcov[8][8] = 0.25f;      // bias accel vertical: ±0.5 m/s²

    _alpha_eq = 0.9997f;

    Serial.println("[EKF9] Inicializado — Fase 2 activa");
}

// =============================================================================
// PREDICCIÓN — 100 Hz
// =============================================================================
// Modelo de proceso (no lineal):
//
//   ωx = gyro_x − x[5]   (velocidad angular corregida de bias)
//   ωy = gyro_y − x[6]
//   ωz = gyro_z − x[7]
//
//   az_inercial = Rz(ψ)Ry(θ)Rx(φ) · a_body − g − x[8]
//              = −ax·sin(θ) + ay·sin(φ)·cos(θ) + az·cos(φ)·cos(θ) − g − x[8]
//
//   x[0]' = x[0] + x[1]·dt + ½·az·dt²
//   x[1]' = x[1] + az·dt
//   x[2]' = x[2] + ωx·dt
//   x[3]' = x[3] + ωy·dt
//   x[4]' = x[4] + ωz·dt
//   x[5-8]' = x[5-8]              (bias: random walk, impulsado solo por Q)
//
// Jacobiano F = ∂f/∂x (sparse, solo entradas no triviales):
//
//   ∂az/∂φ = ay·cos(φ)·cos(θ) − az·sin(φ)·cos(θ)
//   ∂az/∂θ = −ax·cos(θ) − ay·sin(φ)·sin(θ) − az·cos(φ)·sin(θ)
//
//   F[0][1] = dt
//   F[0][2] = ∂az/∂φ · ½dt²       F[0][3] = ∂az/∂θ · ½dt²   F[0][8] = −½dt²
//   F[1][2] = ∂az/∂φ · dt         F[1][3] = ∂az/∂θ · dt     F[1][8] = −dt
//   F[2][5] = −dt
//   F[3][6] = −dt
//   F[4][7] = −dt
//   F[i][i] = 1 para todo i
// =============================================================================
void EKF9::predict(const IMU_Data &imu, float dt) {

    const float dt2 = dt * dt;

    // Actitud actual
    const float phi   = _x[2];  // roll
    const float theta = _x[3];  // pitch

    const float sp = sinf(theta), cp = cosf(theta);
    const float sr = sinf(phi),   cr = cosf(phi);

    // Velocidades angulares corregidas de bias EKF
    const float wx = imu.gyro_x - _x[5];
    const float wy = imu.gyro_y - _x[6];
    const float wz = imu.gyro_z - _x[7];

    // Aceleración vertical en marco inercial (componente Z tras rotación body→NED)
    const float ax = imu.accel_x;
    const float ay = imu.accel_y;
    const float az = imu.accel_z;

    const float az_in = (-ax * sp + ay * sr * cp + az * cr * cp) - ISA_G - _x[8];

    // Derivadas parciales de az_in respecto a actitud (para el Jacobiano)
    const float daz_dphi   = (ay * cr * cp - az * sr * cp);
    const float daz_dtheta = (-ax * cp - ay * sr * sp - az * cr * sp);

    // ── Propagación del estado ────────────────────────────────────────────
    _x[0] += _x[1] * dt + 0.5f * az_in * dt2;
    _x[1] += az_in * dt;
    _x[2]  = _wrapAngle(_x[2] + wx * dt);
    _x[3]  = _wrapAngle(_x[3] + wy * dt);
    _x[4]  = _wrapAngle(_x[4] + wz * dt);
    // x[5-8]: bias, solo Q los mueve

    // ── Construcción del Jacobiano F (sparse → solo no-triviales) ─────────
    // F se inicializa como identidad; solo almacenamos la parte que se desvía
    float Fx[EKF_N][EKF_N];
    memset(Fx, 0, sizeof(Fx));
    for (int i = 0; i < EKF_N; i++) Fx[i][i] = 1.0f;

    Fx[0][1] = dt;
    Fx[0][2] = daz_dphi   * 0.5f * dt2;
    Fx[0][3] = daz_dtheta * 0.5f * dt2;
    Fx[0][8] = -0.5f * dt2;

    Fx[1][2] = daz_dphi   * dt;
    Fx[1][3] = daz_dtheta * dt;
    Fx[1][8] = -dt;

    Fx[2][5] = -dt;
    Fx[3][6] = -dt;
    Fx[4][7] = -dt;

    // ── Propagación de covarianza P = F·P·Fᵀ + Q ─────────────────────────
    // Q diagonal: ruido de proceso
    float Q[EKF_N] = {
        Q_ALT, Q_VZ,
        Q_ATT, Q_ATT, Q_ATT,
        Q_GBIAS, Q_GBIAS, Q_GBIAS,
        Q_ABIAS
    };
    // En modo high-G (ADXL375 activo), aumentar ruido de proceso de aceleración
    // para que el EKF confíe menos en la predicción y más en el barómetro.
    // Factor ×25 refleja la diferencia de resolución ADXL375 vs ICM (49 mg vs ~2 mg).
    if (_high_g_mode) {
        Q[1] *= 25.0f;   // velocidad vertical (afectada por accel menos precisa)
        Q[8] *= 5.0f;    // bias de aceleración — más incertidumbre al cambiar sensor
    }

    _covariancePredict(Fx, Q);
}

// =============================================================================
// ACTUALIZACIÓN — cuando hay dato barométrico nuevo (~50Hz o cada ciclo)
// =============================================================================
// Mediciones:
//   z[0] = baro_alt_agl                              → H[0] = [1,0,…,0]
//   z[1] = atan2(ay, az)                             → H[1] = [0,0,1,0,…,0]
//   z[2] = atan2(−ax, √(ay²+az²))                   → H[2] = [0,0,0,1,0,…,0]
//
// Nota sobre z[1] y z[2]:
//   Solo son válidos cuando el acelerómetro mide la gravedad (|a| ≈ g).
//   En lugar de descartarlos, inflamos R[1,1] y R[2,2] con la desviación
//   de 1g → el EKF los ignora automáticamente cuando el cohete acelera.
//   Esto es equivalente al filtro complementario adaptativo, pero óptimo.
// =============================================================================
void EKF9::update(float baro_alt_agl, const IMU_Data &imu) {

    // ── Calcular R adaptativo para actitud ──────────────────────────────
    const float ax = imu.accel_x;
    const float ay = imu.accel_y;
    const float az = imu.accel_z;

    const float a_mag     = sqrtf(ax*ax + ay*ay + az*az);
    const float deviation = fabsf(a_mag - ISA_G) / ISA_G;  // 0=1g exacto, >1=propulsión
    const float t         = fminf(deviation / R_ATT_DEV, 1.0f);
    const float r_att     = R_ATT_MIN + t * (R_ATT_MAX - R_ATT_MIN);

    // α equivalente para telemetría (comparable con la versión anterior)
    _alpha_eq = 1.0f - (R_ATT_MIN / r_att) * (1.0f - 0.95f);

    // Ruido de medición R (diagonal del 3×3)
    const float R[EKF_M] = { R_BARO, r_att, r_att };

    // ── Modelo de medición h(x) ──────────────────────────────────────────
    const float h0 = _x[0];                         // altitud estimada
    const float h1 = atan2f(ay, az);                // roll del acelerómetro
    const float h2 = atan2f(-ax, sqrtf(ay*ay + az*az)); // pitch del acelerómetro

    // Innovación y = z - h(x)
    float y[EKF_M];
    y[0] = baro_alt_agl - h0;
    y[1] = _wrapAngle(h1 - _x[2]);   // diferencia angular envuelta
    y[2] = _wrapAngle(h2 - _x[3]);

    // ── Jacobiano de medición H (3×9) — sparse ──────────────────────────
    float H[EKF_M][EKF_N];
    memset(H, 0, sizeof(H));
    H[0][0] = 1.0f;   // z[0] mide x[0] (altitud)
    H[1][2] = 1.0f;   // z[1] mide x[2] (roll)
    H[2][3] = 1.0f;   // z[2] mide x[3] (pitch)

    // ── S = H·P·Hᵀ + R ──────────────────────────────────────────────────
    // Dado que H es sparse (filas 0,1,2 seleccionan columnas 0,2,3),
    // H·P·Hᵀ es una submatriz 3×3 de P más R diagonal.
    float S[3][3];
    // S[i][j] = Σ_k Σ_l H[i][k] * P[k][l] * H[j][l]
    // Por la estructura de H: H[0] extrae fila/col 0; H[1] extrae 2; H[2] extrae 3
    int idx[3] = {0, 2, 3};  // columnas de P que cada fila de H selecciona
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = _Pcov[idx[i]][idx[j]];
        }
        S[i][i] += R[i];   // + R diagonal
    }

    // ── Invertir S (analítica 3×3) ───────────────────────────────────────
    float Sinv[3][3];
    if (!_invert3x3(S, Sinv)) {
        // S es singular — no actualizar (caso muy raro, solo en arranque)
        return;
    }

    // ── Ganancia de Kalman K = P·Hᵀ·S⁻¹ (9×3) ──────────────────────────
    // Hᵀ selecciona columnas 0, 2, 3 de P respectivamente para cada medición.
    // K[i][m] = Σ_k P[i][k] * H[m][k] * Sinv término
    // Con H sparse: K[i][0] usa col 0 de P; K[i][1] usa col 2; K[i][2] usa col 3
    float PHt[EKF_N][3];  // P · Hᵀ
    for (int i = 0; i < EKF_N; i++) {
        PHt[i][0] = _Pcov[i][idx[0]];
        PHt[i][1] = _Pcov[i][idx[1]];
        PHt[i][2] = _Pcov[i][idx[2]];
    }

    float K[EKF_N][EKF_M];
    for (int i = 0; i < EKF_N; i++) {
        for (int m = 0; m < EKF_M; m++) {
            float sum = 0.0f;
            for (int j = 0; j < 3; j++) {
                sum += PHt[i][j] * Sinv[j][m];
            }
            K[i][m] = sum;
        }
    }

    // ── Actualización del estado x = x + K·y ────────────────────────────
    for (int i = 0; i < EKF_N; i++) {
        float corr = 0.0f;
        for (int m = 0; m < EKF_M; m++) {
            corr += K[i][m] * y[m];
        }
        _x[i] += corr;
    }

    // Envolver ángulos después de la corrección
    _x[2] = _wrapAngle(_x[2]);
    _x[3] = _wrapAngle(_x[3]);
    _x[4] = _wrapAngle(_x[4]);

    // ── Actualización de covarianza P = (I − K·H)·P ──────────────────────
    _updateCovariance(K, H, R);
}

// =============================================================================
// PROPAGACIÓN DE COVARIANZA P = F·P·Fᵀ + Q
// =============================================================================
// F es casi identidad — aprovechamos la estructura sparse para eficiencia.
// En lugar de F·P·Fᵀ completo (9³ = 729 ops), hacemos:
//   1. Tmp = F · P
//   2. P' = Tmp · Fᵀ
// Con los ceros de F, ~200 operaciones en lugar de 729.
// =============================================================================
void EKF9::_covariancePredict(const float Fx[EKF_N][EKF_N], const float Q[EKF_N]) {

    // Paso 1: Tmp = F · P  (9×9)
    float Tmp[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            float sum = 0.0f;
            for (int k = 0; k < EKF_N; k++) {
                sum += Fx[i][k] * _Pcov[k][j];
            }
            Tmp[i][j] = sum;
        }
    }

    // Paso 2: P' = Tmp · Fᵀ + Q (diagonal)
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            float sum = 0.0f;
            for (int k = 0; k < EKF_N; k++) {
                sum += Tmp[i][k] * Fx[j][k];  // Fᵀ[k][j] = F[j][k]
            }
            _Pcov[i][j] = sum;
        }
        _Pcov[i][i] += Q[i];  // + Q diagonal
    }

    _symmetrizeP();
}

// =============================================================================
// ACTUALIZACIÓN DE COVARIANZA P = (I − K·H) · P
// =============================================================================
void EKF9::_updateCovariance(const float K[EKF_N][EKF_M],
                              const float H[EKF_M][EKF_N],
                              const float R[EKF_M]) {
    // Calcular I − K·H  (9×9)
    float IKH[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            float kh = 0.0f;
            for (int m = 0; m < EKF_M; m++) {
                kh += K[i][m] * H[m][j];
            }
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - kh;
        }
    }

    // P = (I − KH) · P
    float Pnew[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            float sum = 0.0f;
            for (int k = 0; k < EKF_N; k++) {
                sum += IKH[i][k] * _Pcov[k][j];
            }
            Pnew[i][j] = sum;
        }
    }

    // Forzar mínimo en la diagonal (evitar P no positiva definida)
    for (int i = 0; i < EKF_N; i++) {
        Pnew[i][i] = fmaxf(Pnew[i][i], 1e-9f);
    }

    memcpy(_Pcov, Pnew, sizeof(_Pcov));
    _symmetrizeP();
}

// =============================================================================
// HELPERS
// =============================================================================
bool EKF9::_invert3x3(const float A[3][3], float Ainv[3][3]) {
    float a=A[0][0], b=A[0][1], c=A[0][2];
    float d=A[1][0], e=A[1][1], f=A[1][2];
    float g=A[2][0], h=A[2][1], ii=A[2][2];

    float det = a*(e*ii - f*h) - b*(d*ii - f*g) + c*(d*h - e*g);
    if (fabsf(det) < 1e-12f) return false;

    float inv = 1.0f / det;
    Ainv[0][0] =  (e*ii - f*h)  * inv;
    Ainv[0][1] = -(b*ii - c*h)  * inv;
    Ainv[0][2] =  (b*f  - c*e)  * inv;
    Ainv[1][0] = -(d*ii - f*g)  * inv;
    Ainv[1][1] =  (a*ii - c*g)  * inv;
    Ainv[1][2] = -(a*f  - d*c)  * inv;
    Ainv[2][0] =  (d*h  - e*g)  * inv;
    Ainv[2][1] = -(a*h  - b*g)  * inv;
    Ainv[2][2] =  (a*e  - b*d)  * inv;
    return true;
}

float EKF9::_wrapAngle(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void EKF9::_symmetrizeP() {
    for (int i = 0; i < EKF_N; i++) {
        for (int j = i+1; j < EKF_N; j++) {
            float avg = 0.5f * (_Pcov[i][j] + _Pcov[j][i]);
            _Pcov[i][j] = avg;
            _Pcov[j][i] = avg;
        }
    }
}

void EKF9::injectGyroBiasCorrection(float dbx, float dby, float dbz) {
    _x[5] += dbx;
    _x[6] += dby;
    _x[7] += dbz;
}

void EKF9::resetYaw(float yaw_rad) {
    _x[4] = yaw_rad;
    _Pcov[4][4] = 0.1f;  // Reiniciar incertidumbre de yaw
}

// =============================================================================
// ACTUALIZACIÓN GPS ALTITUD — Fase 3
// =============================================================================
// Fusión barométrica (baja latencia, deriva) + GPS (alta latencia, absoluta).
//
// Modelo de medición:
//   z = gps_alt_msl − ground_alt_msl   →   altitud AGL desde GPS
//   H = [1, 0, 0, ..., 0]             →   solo observa x[0] = alt AGL
//
// R escala con HDOP²: con HDOP=1 R=100m², HDOP=2 R=400m², HDOP=4 R=1600m².
// Esto hace que el GPS solo corrija significativamente cuando la señal es limpia.
//
// La actualización es un Kalman 1D trivial (H sparse, solo toca x[0]):
//   innov = gps_alt_agl − x[0]
//   S     = P[0][0] + R
//   K[i]  = P[i][0] / S       (columna 0 de P, escalada por S)
//   x[i] += K[i] * innov
//   P    -= K * H * P          (Joseph form simplificada, 1D)
// =============================================================================
void EKF9::updateGPSAlt(float gps_alt_msl, float hdop) {

    if (hdop > GPS_HDOP_MAX) return;
    if (_ground_alt_msl == 0.0f) return;  // Sin referencia terrestre todavía

    float gps_alt_agl = gps_alt_msl - _ground_alt_msl;

    // Sanity check: GPS altitude debe estar dentro de ±500m de la estimación
    // actual (evita corrupción por latencia GPS extrema o fix falso)
    if (fabsf(gps_alt_agl - _x[0]) > 500.0f) return;

    float hdop_c = fmaxf(hdop, 1.0f);
    float R = R_GPS_ALT_BASE * hdop_c * hdop_c;

    float innov = gps_alt_agl - _x[0];
    float S     = _Pcov[0][0] + R;
    float inv_S = 1.0f / S;

    // Ganancia de Kalman — columna 0 de P / S
    float K[EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        K[i] = _Pcov[i][0] * inv_S;
    }

    // Actualización del estado
    for (int i = 0; i < EKF_N; i++) {
        _x[i] += K[i] * innov;
    }
    _x[2] = _wrapAngle(_x[2]);
    _x[3] = _wrapAngle(_x[3]);
    _x[4] = _wrapAngle(_x[4]);

    // Actualización de covarianza P = (I − K·H) · P  (H = e_0^T)
    // Solo la columna 0 de H es no nula (H[0][0]=1, resto=0)
    // → P'[i][j] = P[i][j] − K[i] · P[0][j]
    for (int i = 0; i < EKF_N; i++) {
        float ki = K[i];
        for (int j = 0; j < EKF_N; j++) {
            _Pcov[i][j] -= ki * _Pcov[0][j];
        }
    }

    // Clamp diagonal positiva
    for (int i = 0; i < EKF_N; i++) {
        _Pcov[i][i] = fmaxf(_Pcov[i][i], 1e-9f);
    }

    _symmetrizeP();
}
