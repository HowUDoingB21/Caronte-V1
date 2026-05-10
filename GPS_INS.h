#pragma once
#include <Arduino.h>
#include "config.h"

// =============================================================================
// GPS_INS — Filtro de Kalman 2D horizontal (loosely-coupled)  [Fase 3]
// =============================================================================
//
// Problema que resuelve:
//   La INS horizontal pura integra la aceleración dos veces. En 30s de vuelo
//   un bias de acelerómetro de 0.05 m/s² acumula ~22m de error en posición
//   y ~1.5 m/s en velocidad. Sin corrección externa, los datos de velocidad
//   horizontal se vuelven inútiles pasados los primeros segundos.
//
// Solución: Kalman 2D [vN, vE]
//   - Predicción @ 100Hz:  propagación con aceleración horizontal del cuerpo
//                          proyectada al marco NED usando la actitud del EKF9.
//   - Actualización @ ~1Hz: medición de velocidad GPS cuando hay fix válido.
//
// Por qué NO ampliar EKF9 a 11 estados:
//   Los estados de actitud y los de velocidad horizontal están débilmente
//   acoplados en un cohete (la dinámica es mayoritariamente vertical).
//   Un filtro 2×2 separado tiene el mismo efecto con 1/25 del costo
//   computacional de ampliar la covarianza a 11×11.
//
// GPS → velocidad NED:
//   El NEO-6M reporta speed_ms (módulo horizontal) y course_deg (0=Norte).
//   vN_gps = speed_ms · cos(course_rad)
//   vE_gps = speed_ms · sin(course_rad)
//   Restricción: solo válido cuando speed_ms > GPS_MIN_SPEED (el curso es ruido
//   a velocidades bajas).
//
// Costo computacional:
//   Predict:  ~20 FLOPS @ 100Hz = 2000 FLOPS/s   → < 0.02 ms/ciclo
//   Update:   ~30 FLOPS @ 1Hz   = 30   FLOPS/s   → negligible
// =============================================================================

class GPS_INS {
public:
    GPS_INS();

    // Resetear estado — llamar en PAD o al reiniciar
    void reset();

    // Predicción — llamar a 100Hz con aceleración horizontal inercial [m/s²]
    // aN, aE: componentes Norte y Este de la aceleración tras restar gravedad
    void predict(float aN, float aE, float dt);

    // Actualización GPS — llamar cuando hay nuevo fix válido
    // speed_ms:  módulo de velocidad horizontal del GPS [m/s]
    // course_deg: rumbo de la velocidad (0=Norte, 90=Este) [°]
    // hdop:      dilución horizontal de precisión (escala el ruido de medición)
    // Retorna false si la velocidad es demasiado baja para confiar en el curso
    bool updateGPS(float speed_ms, float course_deg, float hdop);

    // Acceso al estado estimado
    float vN()        const { return _vN; }
    float vE()        const { return _vE; }
    float speedH()    const { return sqrtf(_vN*_vN + _vE*_vE); }

    // Incertidumbre (desviación estándar) [m/s]
    float sigmaVN()   const { return sqrtf(_Pdiag[0]); }
    float sigmaVE()   const { return sqrtf(_Pdiag[1]); }

    // Número de actualizaciones GPS recibidas (para diagnóstico)
    uint16_t gpsUpdates() const { return _gps_updates; }

private:
    float    _vN;      // Velocidad Norte estimada [m/s]
    float    _vE;      // Velocidad Este estimada  [m/s]
    float    _Pdiag[2];    // Varianza diagonal [P_vN, P_vE] (covarianza diagonal)

    uint16_t _gps_updates;

    // ── Ruido de proceso ─────────────────────────────────────────────────
    // Incertidumbre en la aceleración horizontal: error de proyección + bias
    // 0.5 m/s² equivale a ~3° de error de actitud a 10 m/s²
    static constexpr float Q_VEL      = 0.25f;   // [(m/s)²/s]

    // ── Ruido de medición GPS ─────────────────────────────────────────────
    // NEO-6M: ~0.5 m/s RMS en velocidad. R escala con HDOP².
    // R_base × HDOP² → con HDOP=1: R=0.5 m²/s², HDOP=3: R=4.5 m²/s²
    static constexpr float R_GPS_BASE = 0.5f;    // [m²/s²] con HDOP=1

    // Velocidad mínima para confiar en el curso del GPS
    // Por debajo de esto el NEO-6M no reporta course con confianza
    static constexpr float GPS_MIN_SPEED = 0.5f; // [m/s]
};
