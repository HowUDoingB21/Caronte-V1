#include "GPS_INS.h"
#include <math.h>

// =============================================================================
// GPS_INS — Implementación
// =============================================================================

GPS_INS::GPS_INS() { reset(); }

void GPS_INS::reset() {
    _vN         = 0.0f;
    _vE         = 0.0f;
    _Pdiag[0]       = 100.0f;   // σ inicial ≈ 10 m/s (desconocida)
    _Pdiag[1]       = 100.0f;
    _gps_updates = 0;
}

// =============================================================================
// PREDICCIÓN — 100 Hz
// =============================================================================
// Modelo: velocidad integrada con aceleración inercial horizontal.
// La covarianza crece con el ruido de proceso Q en cada paso.
//
//   vN' = vN + aN · dt
//   vE' = vE + aE · dt
//   P[i]' = P[i] + Q · dt
//
// Q·dt representa la incertidumbre que añade un ciclo de integración de IMU.
// =============================================================================
void GPS_INS::predict(float aN, float aE, float dt) {
    _vN += aN * dt;
    _vE += aE * dt;
    _Pdiag[0] += Q_VEL * dt;
    _Pdiag[1] += Q_VEL * dt;
}

// =============================================================================
// ACTUALIZACIÓN GPS — ~1 Hz
// =============================================================================
// Kalman 1D independiente para cada eje (P es diagonal, H = I):
//
//   z[0] = vN_gps = speed · cos(course)
//   z[1] = vE_gps = speed · sin(course)
//
//   S[i] = P[i] + R[i]
//   K[i] = P[i] / S[i]
//   vN'  = vN + K[0] · (z[0] − vN)
//   vE'  = vE + K[1] · (z[1] − vE)
//   P[i]'= (1 − K[i]) · P[i]
//
// La ganancia de Kalman K converge a ~0 cuando el GPS es muy ruidoso (HDOP alto)
// y a ~1 cuando el GPS es fiable y P es grande (mucha incertidumbre INS).
// =============================================================================
bool GPS_INS::updateGPS(float speed_ms, float course_deg, float hdop) {

    // No actualizar si la velocidad es insuficiente para fiarse del curso
    if (speed_ms < GPS_MIN_SPEED) return false;

    // Convertir speed + course → vN, vE
    float course_rad = course_deg * (float)DEG_TO_RAD;
    float vN_gps = speed_ms * cosf(course_rad);
    float vE_gps = speed_ms * sinf(course_rad);

    // Ruido de medición — escala cuadráticamente con HDOP
    float hdop_clamped = fmaxf(hdop, 1.0f);  // HDOP < 1 no es físicamente posible
    float R = R_GPS_BASE * hdop_clamped * hdop_clamped;

    // Actualización Kalman (ejes independientes)
    float K0 = _Pdiag[0] / (_Pdiag[0] + R);
    float K1 = _Pdiag[1] / (_Pdiag[1] + R);

    _vN    += K0 * (vN_gps - _vN);
    _vE    += K1 * (vE_gps - _vE);
    _Pdiag[0]   = (1.0f - K0) * _Pdiag[0];
    _Pdiag[1]   = (1.0f - K1) * _Pdiag[1];

    // Clamp mínimo para evitar que P colapse a cero (singularidad numérica)
    _Pdiag[0]   = fmaxf(_Pdiag[0], 1e-4f);
    _Pdiag[1]   = fmaxf(_Pdiag[1], 1e-4f);

    _gps_updates++;
    return true;
}
