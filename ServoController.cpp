#include "ServoController.h"
#include <math.h>

// =============================================================================
// CONTROLADOR DE SERVOS — Implementación
// =============================================================================

ServoController::ServoController() {}

// -----------------------------------------------------------------------------
void ServoController::begin() {
    _servo1.attach(PIN_SERVO1, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US);
    _servo2.attach(PIN_SERVO2, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US);
    _servo3.attach(PIN_SERVO3, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US);
    _servo4.attach(PIN_SERVO4, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US);

    setNeutral();

    Serial.println("[SERVO] Servos inicializados en posicion neutra");
    Serial.print("  PWM rango: "); Serial.print(SERVO_PWM_MIN_US);
    Serial.print(" - ");          Serial.print(SERVO_PWM_MAX_US);
    Serial.println(" us");
    Serial.print("  Delta max: "); Serial.print(DELTA_MAX_DEG); Serial.println(" deg");
}

// =============================================================================
// UPDATE PRINCIPAL — Llamar a 100Hz
// =============================================================================
void ServoController::update(const PID_Output &pid_out,
                              const State      &state,
                              bool              control_active,
                              ServoAngles      &angles_out) {

    // Si el control no está activo (en tierra, apogeo, descenso) → neutro
    if (!control_active) {
        setNeutral();
        angles_out = {0, 0, 0, 0, false};
        return;
    }

    // Velocidad total del cohete — magnitud del vector de velocidad inercial
    // calculada por el estimador INS a partir de la integración del IMU.
    float velocity_ms = state.speed;

    // No actuar si la velocidad es insuficiente para tener autoridad aerodinámica
    if (velocity_ms < MIN_CONTROL_SPEED_MS) {
        setNeutral();
        angles_out = {0, 0, 0, 0, false};
        return;
    }

    // -------------------------------------------------------------------------
    // INVERSIÓN AERODINÁMICA
    // Convierte fuerzas requeridas [N] en ángulos de deflexión [rad]
    // *** PLACEHOLDER — Reemplazar con función CFD cuando esté disponible ***
    // -------------------------------------------------------------------------
    float delta_pitch = aeroInverse(pid_out.lift_pitch,
                                    state.air_density,
                                    velocity_ms);

    float delta_yaw   = aeroInverse(pid_out.lift_yaw,
                                    state.air_density,
                                    velocity_ms);

    float delta_roll  = aeroInverse(pid_out.lift_roll,
                                    state.air_density,
                                    velocity_ms);

    // -------------------------------------------------------------------------
    // MATRIZ DE MEZCLA (MIXING)
    // Distribuye pitch + yaw + roll en deflexiones individuales
    // -------------------------------------------------------------------------
    mixingMatrix(delta_pitch, delta_yaw, delta_roll, angles_out);

    // -------------------------------------------------------------------------
    // ENVIAR A SERVOS
    // -------------------------------------------------------------------------
    _servo1.writeMicroseconds(angleToPWM(angles_out.delta1));
    _servo2.writeMicroseconds(angleToPWM(angles_out.delta2));
    _servo3.writeMicroseconds(angleToPWM(angles_out.delta3));
    _servo4.writeMicroseconds(angleToPWM(angles_out.delta4));
}

// =============================================================================
// INVERSIÓN AERODINÁMICA — LUT CL(α) de AeroSim Analyzer
// =============================================================================
// Flujo de cálculo:
//   1. q = ½ρV²                           presión dinámica [Pa]
//   2. CL_cmd = |lift_N| / (q × AERO_SREF) coeficiente requerido
//   3. δ [°] = aero_cl_to_aoa(CL_cmd)     inversión LUT (aero_table.h)
//   4. δ [rad] = δ[°] × π/180 × sign(lift_N)
//
// aero_table.h es generado por AeroSim Analyzer tras el análisis CFD.
// Hasta tener la tabla real, el placeholder en aero_table.h usa una
// aproximación lineal CL = 0.055 × α válida para ángulos pequeños.
// =============================================================================
float ServoController::aeroInverse(float lift_N, float air_density, float velocity_ms) {
    float q = 0.5f * air_density * velocity_ms * velocity_ms;
    if (q < 1.0f) return 0.0f;

    // Obtener AOA requerido desde la LUT (siempre positivo)
    float aoa_deg = aero_aoa_for_force(lift_N, q);

    // Convertir a radianes y aplicar signo de la fuerza
    float delta = aoa_deg * (float)(PI / 180.0);
    if (lift_N < 0.0f) delta = -delta;

    return saturate(delta);
}

// =============================================================================
// MATRIZ DE MEZCLA
// =============================================================================
void ServoController::mixingMatrix(float delta_pitch, float delta_yaw,
                                    float delta_roll,  ServoAngles &out) {
    // Distribución en cruz:
    //   δ1 (superior)  = +pitch + roll
    //   δ2 (derecha)   = +yaw   - roll
    //   δ3 (inferior)  = -pitch + roll
    //   δ4 (izquierda) = -yaw   - roll
    out.delta1 = saturate(+delta_pitch + delta_roll);
    out.delta2 = saturate(+delta_yaw   - delta_roll);
    out.delta3 = saturate(-delta_pitch + delta_roll);
    out.delta4 = saturate(-delta_yaw   - delta_roll);

    // Verificar si alguna aleta fue saturada
    float raw1 = +delta_pitch + delta_roll;
    float raw2 = +delta_yaw   - delta_roll;
    float raw3 = -delta_pitch + delta_roll;
    float raw4 = -delta_yaw   - delta_roll;

    out.saturated = (fabsf(raw1) > DELTA_MAX_RAD ||
                     fabsf(raw2) > DELTA_MAX_RAD ||
                     fabsf(raw3) > DELTA_MAX_RAD ||
                     fabsf(raw4) > DELTA_MAX_RAD);
}

// =============================================================================
// AUXILIARES
// =============================================================================

void ServoController::setNeutral() {
    _servo1.writeMicroseconds(SERVO_PWM_MID_US);
    _servo2.writeMicroseconds(SERVO_PWM_MID_US);
    _servo3.writeMicroseconds(SERVO_PWM_MID_US);
    _servo4.writeMicroseconds(SERVO_PWM_MID_US);
}

void ServoController::selfTest() {
    Serial.println("[SERVO] Iniciando self-test...");
    const int steps = 20;
    const float step_rad = DELTA_MAX_RAD / steps;

    // Barrer de neutro a +δmax
    for (int i = 0; i <= steps; i++) {
        float delta = i * step_rad;
        _servo1.writeMicroseconds(angleToPWM(delta));
        _servo2.writeMicroseconds(angleToPWM(delta));
        _servo3.writeMicroseconds(angleToPWM(delta));
        _servo4.writeMicroseconds(angleToPWM(delta));
        delay(30);
    }
    // Barrer de +δmax a -δmax
    for (int i = steps; i >= -steps; i--) {
        float delta = i * step_rad;
        _servo1.writeMicroseconds(angleToPWM(delta));
        _servo2.writeMicroseconds(angleToPWM(delta));
        _servo3.writeMicroseconds(angleToPWM(delta));
        _servo4.writeMicroseconds(angleToPWM(delta));
        delay(30);
    }
    // Volver a neutro
    setNeutral();
    Serial.println("[SERVO] Self-test completado");
}

// -----------------------------------------------------------------------------
void ServoController::sweepSingle(uint8_t servo_idx, uint32_t duration_ms) {
    Servo *servo = nullptr;
    switch (servo_idx) {
        case 0: servo = &_servo1; break;
        case 1: servo = &_servo2; break;
        case 2: servo = &_servo3; break;
        case 3: servo = &_servo4; break;
        default: return;
    }

    uint32_t steps     = duration_ms / 10;
    float    delta_rad = 0.0f;

    // -δmax → +δmax → neutro
    for (uint32_t s = 0; s <= steps; s++) {
        float t = (float)s / steps;         // 0 → 1
        // Barrido completo: -max → +max usando seno
        delta_rad = sinf(t * M_PI - M_PI / 2.0f) * DELTA_MAX_RAD;
        servo->writeMicroseconds(angleToPWM(delta_rad));
        delay(10);
    }
    servo->writeMicroseconds(SERVO_PWM_MID_US);
}

float ServoController::saturate(float delta) {
    if (delta >  DELTA_MAX_RAD) return  DELTA_MAX_RAD;
    if (delta < -DELTA_MAX_RAD) return -DELTA_MAX_RAD;
    return delta;
}

// Mapeo lineal: [-δmax, +δmax] → [PWM_MIN, PWM_MAX]
int ServoController::angleToPWM(float delta_rad) {
    // Normalizar δ al rango [-1, +1]
    float normalized = delta_rad / DELTA_MAX_RAD;
    normalized = constrain(normalized, -1.0f, 1.0f);

    // Mapear a microsegundos
    float pwm = SERVO_PWM_MID_US + normalized * (float)(SERVO_PWM_MAX_US - SERVO_PWM_MID_US);
    return (int)pwm;
}
