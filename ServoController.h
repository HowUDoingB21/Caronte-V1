#pragma once
#include <Arduino.h>
#include <Servo.h>
#include "config.h"
#include "PIDController.h"
#include "FlightFSM.h"
#include "aero_table.h"     // LUT CL->AOA generada por AeroSim Analyzer

// =============================================================================
// CONTROLADOR DE SERVOS — Caronte V1
// =============================================================================
// Recibe las fuerzas aerodinámicas requeridas del PID y, una vez que el
// módulo de inversión aerodinámica esté disponible, las convierte en
// ángulos físicos de deflexión δ para cada aleta.
//
// GEOMETRÍA DE LAS 4 ALETAS:
// Las aletas están distribuidas en cruz a 90° entre sí.
// Convención de ejes (vista desde atrás del cohete):
//
//         Aleta 1 (arriba)
//              |
//   Aleta 4 ───┼─── Aleta 2
//  (izquierda) |   (derecha)
//         Aleta 3 (abajo)
//
// Mezcla de señales (mixing):
//   Pitch (cabeceo +): Aleta1 +δ,  Aleta3 -δ,  Aleta2=0,   Aleta4=0
//   Yaw   (guiñada +): Aleta2 +δ,  Aleta4 -δ,  Aleta1=0,   Aleta3=0
//   Roll  (rodadura +): Todas las aletas con δ rotacional (diferencial)
//
//   Combinando pitch + yaw + roll:
//     δ1 = +pitch_delta + roll_delta
//     δ2 = +yaw_delta   - roll_delta
//     δ3 = -pitch_delta + roll_delta
//     δ4 = -yaw_delta   - roll_delta
//
// PLACEHOLDER AERODINÁMICA:
//   Hasta tener la función de CFD, se usa una aproximación lineal:
//   δ = L_req / (AERO_K * q)
//   donde q = 0.5 * ρ * V² es la presión dinámica
//   y AERO_K es una constante de eficiencia aerodinámica placeholder.
//   REEMPLAZAR con la función CFD real cuando esté disponible.
// =============================================================================


// Velocidad mínima para activar control aerodinámico [m/s]
#define MIN_CONTROL_SPEED_MS    30.0f


// --- Ángulos de deflexión calculados para cada aleta ---
struct ServoAngles {
    float delta1;   // [rad] Aleta 1 — superior
    float delta2;   // [rad] Aleta 2 — derecha
    float delta3;   // [rad] Aleta 3 — inferior
    float delta4;   // [rad] Aleta 4 — izquierda
    bool  saturated; // true si alguna aleta fue limitada a δmax
};


// =============================================================================
class ServoController {
public:
    ServoController();

    // Inicializar servos y posicionar en neutro
    void begin();

    // Actualizar posiciones de servos — llamar a 100Hz
    // Si el control no está activo, mueve todos a posición neutra
    void update(const PID_Output &pid_out,
                const State      &state,
                bool              control_active,
                ServoAngles      &angles_out);

    // Mover todos los servos a posición neutra inmediatamente
    void setNeutral();

    // Test de movimiento en tierra (barrido de -δmax a +δmax, todos)
    void selfTest();

    // Barrer un servo individual de -δmax a +δmax (para test de instalación)
    void sweepSingle(uint8_t servo_idx, uint32_t duration_ms = 1000);

private:
    Servo _servo1, _servo2, _servo3, _servo4;

    // Inversión aerodinámica — usa LUT de aero_table.h
    // Convierte fuerza requerida [N] en ángulo de deflexión [rad]
    // La LUT se genera con AeroSim Analyzer tras el análisis CFD en SimScale.
    float aeroInverse(float lift_N, float air_density, float velocity_ms);

    // Mezcla pitch + yaw + roll en deflexiones individuales por aleta
    void mixingMatrix(float delta_pitch, float delta_yaw, float delta_roll,
                      ServoAngles &out);

    // Saturación de ángulos a ±δmax
    float saturate(float delta);

    // Convierte ángulo [rad] a pulso PWM [µs]
    int angleToPWM(float delta_rad);
};
