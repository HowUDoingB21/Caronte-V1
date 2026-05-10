#pragma once
#include <Arduino.h>
#include "ICM45686.h"
#include "MS5611.h"

// =============================================================================
// EKF9 — Extended Kalman Filter de 9 estados para Caronte V1  [Fase 2]
// =============================================================================
//
// Estado x[9]:
//   x[0]  altitud AGL          [m]
//   x[1]  velocidad vertical Vz [m/s]
//   x[2]  roll  φ              [rad]
//   x[3]  pitch θ              [rad]
//   x[4]  yaw   ψ              [rad]
//   x[5]  bias residual gyro X [rad/s]  ← residual tras IMU_Cal + ZUPT
//   x[6]  bias residual gyro Y [rad/s]
//   x[7]  bias residual gyro Z [rad/s]
//   x[8]  bias aceleración vertical inercial [m/s²]
//
// Entradas u (no modeladas en el estado, vienen del IMU):
//   gyro_{x,y,z}   velocidades angulares calibradas
//   accel_{x,y,z}  aceleraciones calibradas
//
// Mediciones z[3]:
//   z[0]  altitud barométrica AGL
//   z[1]  roll de referencia  = atan2(ay, az)          ← solo cuando |a| ≈ g
//   z[2]  pitch de referencia = atan2(-ax, √(ay²+az²)) ← solo cuando |a| ≈ g
//
// La incertidumbre del acelerómetro como referencia de actitud (R[1,1] y R[2,2])
// se infla adaptativamente cuando | |a|−g | es grande (propulsión / caída libre).
// Esto reemplaza el filtro complementario adaptativo por un estimador óptimo.
//
// Costo computacional estimado en STM32F405 (168MHz, FPU):
//   Predicción (F·P·Fᵀ + Q):   ~350 FLOPS  → < 0.3 ms
//   Actualización (K, P, x):   ~250 FLOPS  → < 0.2 ms
//   Total por ciclo @ 100Hz:   ~600 FLOPS  → ~0.5 ms  (~5% del presupuesto)
// =============================================================================

// Dimensiones
#define EKF_N   9    // Tamaño del vector de estado
#define EKF_M   3    // Número de mediciones


class EKF9 {
public:
    EKF9();

    // Inicializar con actitud y altitud de arranque
    void begin(float roll0 = 0.0f, float pitch0 = 0.0f,
               float yaw0 = 0.0f, float alt0 = 0.0f);

    // Paso de predicción — ejecutar a cada ciclo de control (100Hz)
    // imu: datos del IMU ya calibrados (IMU_Calibration.apply())
    void predict(const IMU_Data &imu, float dt);

    // Paso de actualización con barométrica + actitud del acelerómetro
    // Llamar cuando hay nueva lectura válida del barométrico
    void update(float baro_alt_agl, const IMU_Data &imu);

    // ── Acceso al estado ─────────────────────────────────────────────────
    float altitude()       const { return _x[0]; }
    float verticalSpeed()  const { return _x[1]; }
    float roll()           const { return _x[2]; }
    float pitch()          const { return _x[3]; }
    float yaw()            const { return _x[4]; }
    float gyroBiasX()      const { return _x[5]; }
    float gyroBiasY()      const { return _x[6]; }
    float gyroBiasZ()      const { return _x[7]; }
    float accelBiasZ()     const { return _x[8]; }

    // Incertidumbre (varianza diagonal de P) — útil para telemetría / monitor
    float varAltitude()    const { return _Pcov[0][0]; }
    float varRoll()        const { return _Pcov[2][2]; }
    float varPitch()       const { return _Pcov[3][3]; }

    // Conmuta entre modo ICM (alta precisión) y modo ADXL375 (alto rango).
    // En modo high-G, el ruido de proceso Q de aceleración se incrementa
    // para reflejar la menor resolución del ADXL375 (49 mg/LSB vs ~0.1 mg).
    void setHighGMode(bool enable) { _high_g_mode = enable; }
    bool isHighGMode()       const { return _high_g_mode; }

    // α equivalente del EKF — para compatibilidad con telemetría
    float alphaEquivalent() const { return _alpha_eq; }

    // Inyectar bias de gyro desde ZUPT (se suma al estado actual x[5-7])
    void injectGyroBiasCorrection(float dbx, float dby, float dbz);

    // Resetear yaw sin tocar el resto del estado
    void resetYaw(float yaw_rad = 0.0f);

    // Acceso directo al estado completo (para logging)
    const float* stateVector() const { return _x; }

    // ── GPS Fase 3 ────────────────────────────────────────────────────────

    // Registrar altitud MSL de tierra al momento del lanzamiento.
    // Necesario para convertir GPS altitude_msl → AGL.
    // Llamar desde Estimator cuando se establece el nivel del suelo.
    void setGroundAltMSL(float alt_msl_m) { _ground_alt_msl = alt_msl_m; }
    float groundAltMSL() const { return _ground_alt_msl; }

    // Actualización con altitud GPS — correctivo de largo plazo del bias barométrico.
    //
    // El barométrico es preciso en el corto plazo (baja latencia, bajo ruido)
    // pero deriva con cambios de presión atmosférica. El GPS no deriva pero
    // tiene más ruido (~5-15m RMS en el NEO-6M) y latencia.
    //
    // Esta actualización fusiona ambos: el EKF pondera GPS vs baro según
    // sus covarianzas actuales. Con HDOP alto, el GPS casi no corrige.
    // Con HDOP bajo y mucho tiempo de vuelo (P_alt creciente), el GPS
    // ancla la altitud absoluta.
    //
    // Restricción: solo llamar cuando gps.fix == true, gps.hdop < GPS_HDOP_MAX,
    //              y la velocidad vertical no sea extrema (evitar latencia GPS).
    void updateGPSAlt(float gps_alt_msl, float hdop);

private:
    bool  _high_g_mode;        // true → Q aceleración incrementado (ADXL375)
    float _x[EKF_N];          // Vector de estado
    float _Pcov[EKF_N][EKF_N];   // Matriz de covarianza

    float _alpha_eq;           // α equivalente (para telemetría)

    // ── Ruido de proceso Q (diagonal) ────────────────────────────────────
    // Cuánto permitimos que derive cada estado entre ciclos.
    // Valores conservadores — se ajustan con datos reales de vuelo.
    static constexpr float Q_ALT     = 0.005f;   // [m²/s]
    static constexpr float Q_VZ      = 0.10f;    // [(m/s)²/s]
    static constexpr float Q_ATT     = 1e-5f;    // [rad²/s]    actitud
    static constexpr float Q_GBIAS   = 1e-8f;    // [(rad/s)²/s] bias gyro (muy lento)
    static constexpr float Q_ABIAS   = 2e-6f;    // [(m/s²)²/s] bias accel vertical

    // ── Ruido de medición R ───────────────────────────────────────────────
    static constexpr float R_BARO    = 0.25f;    // [m²]  varianza altitud baro
    static constexpr float R_ATT_MIN = 0.01f;    // [rad²] actitud accel (bajo ruido)
    static constexpr float R_ATT_MAX = 1e6f;     // [rad²] actitud accel (propulsión)
    // Desviación de 1g a partir de la cual R_att satura en R_ATT_MAX
    static constexpr float R_ATT_DEV = 1.0f;     // normalizado: ||a|-g|/g

    // GPS altitude update — ruido base del NEO-6M en altitud [m²]
    // RMS ~10m → varianza base ~100 m². Escala con HDOP².
    static constexpr float R_GPS_ALT_BASE = 100.0f;  // [m²] con HDOP=1
    static constexpr float GPS_HDOP_MAX   = 4.0f;    // No usar GPS si HDOP > esto

    float _ground_alt_msl = 0.0f;  // Altitud MSL del punto de lanzamiento [m]

    // ── Helpers de álgebra matricial ─────────────────────────────────────
    // Todas las operaciones son inline para evitar overhead de llamada

    // Producto F·P·Fᵀ: aprovecha la estructura sparse de F (no es densa)
    void _covariancePredict(const float Fx[EKF_N][EKF_N],
                            const float Q[EKF_N]);

    // Actualización de Joseph (numéricamente estable):
    //   P = (I - K·H) · P · (I - K·H)ᵀ + K·R·Kᵀ
    // Aquí usamos la forma simplificada P = (I-KH)P por eficiencia,
    // con verificación de simetría al final.
    void _updateCovariance(const float K[EKF_N][EKF_M],
                           const float H[EKF_M][EKF_N],
                           const float R[EKF_M]);

    // Inversión analítica 3×3 (regla de Cramer)
    static bool _invert3x3(const float A[3][3], float Ainv[3][3]);

    // Envuelve un ángulo a [−π, π]
    static float _wrapAngle(float a);

    // Simetría forzada de P (evita acumulación de error numérico)
    void _symmetrizeP();
};
