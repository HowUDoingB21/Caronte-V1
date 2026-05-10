#include "PIDController.h"
#include <math.h>

// =============================================================================
// PID CONTROLLER — Implementación
// =============================================================================

PIDController::PIDController() {
    memset(&_roll,  0, sizeof(PID_Channel));
    memset(&_pitch, 0, sizeof(PID_Channel));
    memset(&_yaw,   0, sizeof(PID_Channel));
}

// -----------------------------------------------------------------------------
void PIDController::begin() {
    // Cargar ganancias desde config.h
    _roll.Kp  = PID_ROLL_KP;
    _roll.Ki  = PID_ROLL_KI;
    _roll.Kd  = PID_ROLL_KD;

    _pitch.Kp = PID_PITCH_KP;
    _pitch.Ki = PID_PITCH_KI;
    _pitch.Kd = PID_PITCH_KD;

    _yaw.Kp   = PID_YAW_KP;
    _yaw.Ki   = PID_YAW_KI;
    _yaw.Kd   = PID_YAW_KD;

    resetIntegrals();

    Serial.println("[PID] Controlador inicializado");
    Serial.print("  Pitch  Kp:"); Serial.print(_pitch.Kp);
    Serial.print(" Ki:"); Serial.print(_pitch.Ki);
    Serial.print(" Kd:"); Serial.println(_pitch.Kd);
    Serial.print("  Yaw    Kp:"); Serial.print(_yaw.Kp);
    Serial.print(" Ki:"); Serial.print(_yaw.Ki);
    Serial.print(" Kd:"); Serial.println(_yaw.Kd);
    Serial.print("  Roll   Kp:"); Serial.print(_roll.Kp);
    Serial.print(" Ki:"); Serial.print(_roll.Ki);
    Serial.print(" Kd:"); Serial.println(_roll.Kd);
}

// -----------------------------------------------------------------------------
void PIDController::resetIntegrals() {
    _roll.integral        = 0.0f;
    _pitch.integral       = 0.0f;
    _yaw.integral         = 0.0f;
    _roll.initialized     = false;
    _pitch.initialized    = false;
    _yaw.initialized      = false;
    Serial.println("[PID] Integrales reseteados");
}

// =============================================================================
// UPDATE PRINCIPAL — Llamar a 100Hz
// =============================================================================
void PIDController::update(const State &state, float dt, PID_Output &out) {

    // -------------------------------------------------------------------------
    // PASO 1: Calcular errores de actitud
    // Objetivo: ascenso perfectamente vertical → todos los ángulos = 0
    // -------------------------------------------------------------------------
    float e_roll  = TARGET_ROLL  - state.roll;
    float e_pitch = TARGET_PITCH - state.pitch;
    float e_yaw   = TARGET_YAW   - state.yaw;

    // Normalizar errores de actitud a [-π, π] para evitar el "camino largo"
    // (por ejemplo que corrija 350° en lugar de -10°)
    while (e_roll  >  M_PI) e_roll  -= 2.0f * M_PI;
    while (e_roll  < -M_PI) e_roll  += 2.0f * M_PI;
    while (e_pitch >  M_PI) e_pitch -= 2.0f * M_PI;
    while (e_pitch < -M_PI) e_pitch += 2.0f * M_PI;
    while (e_yaw   >  M_PI) e_yaw   -= 2.0f * M_PI;
    while (e_yaw   < -M_PI) e_yaw   += 2.0f * M_PI;

    // Derivada del error = -velocidad angular actual
    // (porque la derivada de θ_deseado=0 es cero)
    float edot_roll  = -state.roll_rate;
    float edot_pitch = -state.pitch_rate;
    float edot_yaw   = -state.yaw_rate;

    // -------------------------------------------------------------------------
    // PASO 2: PID → aceleración angular comandada [rad/s²]
    // -------------------------------------------------------------------------
    float alpha_roll  = computeChannel(_roll,  e_roll,  edot_roll,  dt);
    float alpha_pitch = computeChannel(_pitch, e_pitch, edot_pitch, dt);
    float alpha_yaw   = computeChannel(_yaw,   e_yaw,   edot_yaw,   dt);

    // -------------------------------------------------------------------------
    // PASO 3: Inversión dinámica → torque requerido [N·m]
    // Ecuación de Euler completa: τ = I·α + ω×(I·ω)
    // -------------------------------------------------------------------------
    float tau_roll, tau_pitch, tau_yaw;
    dynamicInversion(alpha_roll, alpha_pitch, alpha_yaw,
                     state,
                     tau_roll, tau_pitch, tau_yaw);

    out.torque_roll  = tau_roll;
    out.torque_pitch = tau_pitch;
    out.torque_yaw   = tau_yaw;

    // -------------------------------------------------------------------------
    // PASO 4: Fuerza aerodinámica requerida [N]
    // L = τ / d   donde d = distancia CG → CP de aletas
    // -------------------------------------------------------------------------
    out.lift_roll  = tau_roll  / LEVER_ARM_D;
    out.lift_pitch = tau_pitch / LEVER_ARM_D;
    out.lift_yaw   = tau_yaw   / LEVER_ARM_D;

    // Flags de saturación (seteados dentro de computeChannel vía anti-windup)
    out.saturated_roll  = (fabsf(_roll.integral)  >= PID_INTEGRAL_LIMIT);
    out.saturated_pitch = (fabsf(_pitch.integral) >= PID_INTEGRAL_LIMIT);
    out.saturated_yaw   = (fabsf(_yaw.integral)   >= PID_INTEGRAL_LIMIT);
}

// =============================================================================
// PID DISCRETO — Un canal
// =============================================================================
float PIDController::computeChannel(PID_Channel &ch, float error,
                                     float error_dot, float dt) {
    // Inicialización: primer ciclo no tiene derivada histórica válida
    if (!ch.initialized) {
        ch.prev_error  = error;
        ch.initialized = true;
    }

    // Término proporcional
    float P = ch.Kp * error;

    // Término integral con anti-windup por saturación
    // Si el integral supera el límite, dejamos de acumular en esa dirección
    ch.integral += error * dt;
    ch.integral  = constrain(ch.integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    float I = ch.Ki * ch.integral;

    // Término derivativo usando la velocidad angular directamente
    // (más limpio que diferenciar el error, evita derivative kick en setpoint)
    float D = ch.Kd * error_dot;

    ch.prev_error = error;

    // θ̈_cmd [rad/s²]
    return P + I + D;
}

// =============================================================================
// INVERSIÓN DINÁMICA — Euler con tensor de inercia completo
// =============================================================================
// τ = I·α + ω×(I·ω)
//
// Donde:
//   I  = tensor de inercia 3×3 (incluye términos cruzados)
//   α  = [alpha_roll, alpha_pitch, alpha_yaw]' aceleración angular comandada
//   ω  = [roll_rate, pitch_rate, yaw_rate]'    velocidad angular actual
//   ω×(I·ω) = término giroscópico (acoplamiento entre ejes)
//
void PIDController::dynamicInversion(float alpha_roll,  float alpha_pitch,
                                      float alpha_yaw,   const State &state,
                                      float &tau_roll,   float &tau_pitch,
                                      float &tau_yaw) {

    // Tensor de inercia completo desde config.h [kg·m²]
    // Nota: los valores de config.h ya deben estar en kg·m² (convertidos de g·mm²)
    const float I[3][3] = {
        { INERTIA_IXX, INERTIA_IXY, INERTIA_IXZ },
        { INERTIA_IYX, INERTIA_IYY, INERTIA_IYZ },
        { INERTIA_IZX, INERTIA_IZY, INERTIA_IZZ }
    };

    // Vector de aceleración angular comandada α
    float alpha[3] = { alpha_roll, alpha_pitch, alpha_yaw };

    // Vector de velocidad angular actual ω
    float omega[3] = { state.roll_rate, state.pitch_rate, state.yaw_rate };

    // --- Término I·α ---
    float I_alpha[3];
    matVecMul(I, alpha, I_alpha);

    // --- Término giroscópico ω×(I·ω) ---
    float I_omega[3];
    matVecMul(I, omega, I_omega);

    float gyro_coupling[3];
    cross(omega, I_omega, gyro_coupling);

    // --- τ = I·α + ω×(I·ω) ---
    tau_roll  = I_alpha[0] + gyro_coupling[0];
    tau_pitch = I_alpha[1] + gyro_coupling[1];
    tau_yaw   = I_alpha[2] + gyro_coupling[2];
}

// =============================================================================
// AJUSTE DE GANANCIAS EN VUELO
// =============================================================================
void PIDController::setGainsPitch(float Kp, float Ki, float Kd) {
    _pitch.Kp = Kp; _pitch.Ki = Ki; _pitch.Kd = Kd;
}
void PIDController::setGainsYaw(float Kp, float Ki, float Kd) {
    _yaw.Kp = Kp; _yaw.Ki = Ki; _yaw.Kd = Kd;
}
void PIDController::setGainsRoll(float Kp, float Ki, float Kd) {
    _roll.Kp = Kp; _roll.Ki = Ki; _roll.Kd = Kd;
}

// =============================================================================
// ÁLGEBRA LINEAL — Privados
// =============================================================================

// Producto vectorial: out = a × b
void PIDController::cross(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

// Multiplicación matriz 3×3 por vector 3: out = M·v
void PIDController::matVecMul(const float M[3][3], const float v[3], float out[3]) {
    for (int i = 0; i < 3; i++) {
        out[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            out[i] += M[i][j] * v[j];
        }
    }
}
