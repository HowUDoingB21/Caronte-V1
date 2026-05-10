// =============================================================================
// aero_table.h — Tabla aerodinámica para ServoController — Caronte V1
//
// PLACEHOLDER — Regenerar con AeroSim Analyzer tras el análisis CFD:
//   1. Ejecutar simscale_sweep_gui.py para el barrido paramétrico
//   2. Cargar el CSV en aero_analysis_gui.py
//   3. Ajustar superficie con método "Spline CL(alpha)×q" o "Físico"
//   4. Sección "Control AOA" → "Exportar aero_table.h para FC"
//   5. Copiar el archivo generado aquí y recompilar
//
// Este placeholder usa una aproximación LINEAL:
//   CL(alpha) = CL_SLOPE × alpha  [alpha en grados]
// válida solo para ángulos pequeños y como orden de magnitud.
//
// REEMPLAZAR con la tabla real antes del vuelo.
// =============================================================================
#pragma once
#include <Arduino.h>

// Área de referencia de la aleta [m²] — confirmar desde CAD
// Valor típico para aleta de ~15cm × 8cm
#define AERO_SREF    0.0120f

// LUT inversa: 32 puntos, CL uniformemente espaciado → AOA [°]
// Placeholder lineal: CL = 0.055 × alpha  (pendiente típica para aleta plana)
#define AERO_TABLE_N 32
#define AERO_CL_MIN  0.00000000f
#define AERO_CL_MAX  1.65000000f     // CL_max @ alpha=30°

// AOA [°] para cada CL: invertida de CL = 0.055 × alpha
// AOA[i] = (CL_MIN + i*(CL_MAX-CL_MIN)/(N-1)) / 0.055
static const float AERO_AOA_TABLE[AERO_TABLE_N] = {
     0.000000f,  0.967742f,  1.935484f,  2.903226f,
     3.870968f,  4.838710f,  5.806452f,  6.774194f,
     7.741935f,  8.709677f,  9.677419f, 10.645161f,
    11.612903f, 12.580645f, 13.548387f, 14.516129f,
    15.483871f, 16.451613f, 17.419355f, 18.387097f,
    19.354839f, 20.322581f, 21.290323f, 22.258065f,
    23.225806f, 24.193548f, 25.161290f, 26.129032f,
    27.096774f, 28.064516f, 29.032258f, 30.000000f
};

// =============================================================================
// aero_cl_to_aoa() — Invierte CL → AOA [°] por interpolación lineal en LUT
// =============================================================================
static inline float aero_cl_to_aoa(float cl) {
    if (cl <= AERO_CL_MIN) return AERO_AOA_TABLE[0];
    if (cl >= AERO_CL_MAX) return AERO_AOA_TABLE[AERO_TABLE_N - 1];
    float t = (cl - AERO_CL_MIN) / (AERO_CL_MAX - AERO_CL_MIN)
              * (float)(AERO_TABLE_N - 1);
    int   i = (int)t;
    float f = t - (float)i;
    if (i >= AERO_TABLE_N - 1) return AERO_AOA_TABLE[AERO_TABLE_N - 1];
    return AERO_AOA_TABLE[i] * (1.0f - f) + AERO_AOA_TABLE[i + 1] * f;
}

// =============================================================================
// aero_aoa_for_force() — AOA [°] necesario para producir force_N con q_Pa
// Entrada : force_N — fuerza lateral deseada [N]  (se toma |force_N|)
//           q_Pa    — presión dinámica ½ρV² [Pa]
// Salida  : AOA en grados ≥ 0  (el signo lo aplica ServoController)
// =============================================================================
static inline float aero_aoa_for_force(float force_N, float q_Pa) {
    if (q_Pa < 1.0f) return 0.0f;
    float cl_cmd = fabsf(force_N) / (q_Pa * AERO_SREF);
    return aero_cl_to_aoa(cl_cmd);
}
