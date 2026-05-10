#pragma once
#include <Arduino.h>
#include "config.h"
#include "Estimator.h"

// =============================================================================
// CONTROLADOR PID — 3 EJES (Roll, Pitch, Yaw)
// =============================================================================
// Implementa la cadena completa de control por eje:
//
//   PASO 1 — Error:
//     e(t) = θ_deseado - θ_actual
//     ė(t) = 0 - ω_actual   (derivada del error = negativo de la vel. angular)
//
//   PASO 2 — PID discreto → aceleración angular comandada:
//     θ̈_cmd = Kp·e + Ki·Σ(e·Δt) + Kd·ė
//
//   PASO 3 — Inversión dinámica → torque requerido:
//     τ_req = I·θ̈_cmd + ω×(I·ω)   ← término giroscópico
//
//     El término ω×(I·ω) es el acoplamiento entre ejes causado por la
//     rotación del cohete. Con la matriz de inercia completa (términos
//     cruzados no nulos) este término no puede ignorarse.
//
//   PASO 4 — Fuerza requerida (se pasa al módulo de inversión aerodinámica):
//     L_req = τ_req / d
//     donde d = distancia CG → CP de aletas
//
// NOTA SOBRE LA MATRIZ DE INERCIA:
//   Se usa el tensor completo 3×3. Los términos cruzados (Ixy, Ixz, Iyz)
//   están presentes porque el cohete no es perfectamente simétrico.
//   La ecuación de Euler completa es:
//     τ = I·α + ω×(I·ω)
//   donde I·ω se calcula con el tensor completo.
// =============================================================================


// --- Resultado del PID para los 3 ejes ---
struct PID_Output {
    float torque_roll;      // [N·m] Torque requerido eje roll
    float torque_pitch;     // [N·m] Torque requerido eje pitch
    float torque_yaw;       // [N·m] Torque requerido eje yaw

    float lift_roll;        // [N] Fuerza aerodinámica requerida — roll
    float lift_pitch;       // [N] Fuerza aerodinámica requerida — pitch
    float lift_yaw;         // [N] Fuerza aerodinámica requerida — yaw

    bool  saturated_roll;   // true si el integral fue limitado (anti-windup)
    bool  saturated_pitch;
    bool  saturated_yaw;
};

// --- Estructura interna de un canal PID ---
struct PID_Channel {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    bool  initialized;
};


// =============================================================================
class PIDController {
public:
    PIDController();

    // Inicializar con ganancias desde config.h
    void begin();

    // Actualizar los 3 ejes — llamar a 100Hz
    // Entrada: estado estimado actual
    // Salida:  torques y fuerzas requeridas
    void update(const State &state, float dt, PID_Output &out);

    // Resetear integrales (llamar al inicio del vuelo)
    void resetIntegrals();

    // Ajuste de ganancias en vuelo (para futura sintonización por telemetría)
    void setGainsPitch(float Kp, float Ki, float Kd);
    void setGainsYaw  (float Kp, float Ki, float Kd);
    void setGainsRoll (float Kp, float Ki, float Kd);

private:
    PID_Channel _roll;
    PID_Channel _pitch;
    PID_Channel _yaw;

    // Calcula un canal PID y retorna la aceleración angular comandada θ̈_cmd
    float computeChannel(PID_Channel &ch, float error, float error_dot, float dt);

    // Inversión dinámica: convierte θ̈_cmd a torque usando tensor de inercia
    // τ = I·θ̈_cmd + ω×(I·ω)
    void dynamicInversion(float alpha_roll, float alpha_pitch, float alpha_yaw,
                          const State &state,
                          float &tau_roll, float &tau_pitch, float &tau_yaw);

    // Producto vectorial a×b
    static void cross(const float a[3], const float b[3], float out[3]);

    // Multiplicación tensor 3×3 × vector 3
    static void matVecMul(const float M[3][3], const float v[3], float out[3]);
};
