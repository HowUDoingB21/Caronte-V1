#pragma once
#include <Arduino.h>
#include "config.h"
#include "Estimator.h"
#include "LED_Utils.h"

// =============================================================================
// MÁQUINA DE ESTADOS DEL VUELO — Caronte V1
// =============================================================================
//
// PERFIL DE VUELO REAL:
//
//  PAD ──(despegue)──► POWERED_1
//       Aletas: NEUTRO
//
//  POWERED_1 ──(burnout motor 1)──► STAGING
//       Acción: Dispara Pyro4 (separa etapas + enciende motor 2)
//       Aletas: NEUTRO
//       Inicia monitoreo de Ret1/Ret2
//
//  STAGING ──(Ret1 Y Ret2 dejan de leer HIGH)──► POWERED_2
//       Ret1/Ret2 = HIGH → etapas físicamente unidas
//       Ret1/Ret2 = LOW  → etapas separadas (confirmación física)
//       Pines configurados como INPUT_PULLDOWN → flotan a LOW al desconectarse
//       Timeout de seguridad: si tras STAGING_TIMEOUT_MS no se confirma,
//       se asume separación exitosa para no bloquear el vuelo
//       Aletas: NEUTRO hasta confirmar separación
//
//  POWERED_2 ──(burnout motor 2)──► COASTING
//       Aletas: CONTROL ACTIVO desde aquí
//
//  COASTING ──(apogeo detectado)──► APOGEE
//       Aletas: CONTROL ACTIVO
//
//  APOGEE ──(dwell ≥ APOGEE_MIN_DWELL_MS)──► DESCENT
//       Acción: Aletas a NEUTRO + Dispara Pyro1 (recuperación)
//       Pyro2 y Pyro3 NO se usan en este proyecto
//
//  DESCENT ──(velocidad ≈ 0 sostenida)──► LANDED
//       Aletas: NEUTRO
//
// DIAGRAMA DE PINES DE SEPARACIÓN (U23):
//   Sep1 (OUTPUT) → envía HIGH → cable físico → Ret1 (INPUT_PULLDOWN)
//   Sep2 (OUTPUT) → envía HIGH → cable físico → Ret2 (INPUT_PULLDOWN)
//   Unido:    Ret1=HIGH, Ret2=HIGH  (señal de Sep1/Sep2 llega)
//   Separado: Ret1=LOW,  Ret2=LOW   (pulldown interno tira a GND)
// =============================================================================


// --- Fases del vuelo ---
enum FlightPhase {
    PHASE_PAD       = 0,    // En el riel, esperando despegue
    PHASE_POWERED_1 = 1,    // Motor 1 encendido, aletas neutro
    PHASE_STAGING   = 2,    // Pyro4 disparado, esperando separación física
    PHASE_POWERED_2 = 3,    // Motor 2 encendido, control activo
    PHASE_COASTING  = 4,    // Motor 2 apagado, control activo
    PHASE_APOGEE    = 5,    // Apogeo detectado, Pyro1 disparado
    PHASE_DESCENT   = 6,    // Descenso
    PHASE_LANDED    = 7     // En tierra
};

static const char* PHASE_NAMES[] = {
    "PAD", "POWERED_1", "STAGING", "POWERED_2",
    "COASTING", "APOGEE", "DESCENT", "LANDED"
};


// --- Umbrales de detección ---
#define LAUNCH_ACCEL_THRESHOLD_MS2      20.0f   // ~2g para confirmar despegue
#define LAUNCH_CONFIRM_MS               50

#define BURNOUT1_ACCEL_THRESHOLD_MS2    12.0f   // Motor 1 apagado (~1.2g = solo drag)
#define BURNOUT1_CONFIRM_MS             100

// Delay entre confirmación de burnout del motor 1 y disparo de Pyro4 [ms]
// 0 = disparo inmediato.
// Útil para dejar que el cohete decelere antes de separar, reduciendo las
// fuerzas axiales en el momento de la separación y el riesgo de colisión
// entre etapas. Mover a config.h cuando se tenga el valor definitivo.
// Valor típico: 0–500 ms según simulación de trayectoria.
#define STAGING_DELAY_MS                0       // *** AJUSTAR SEGÚN SIMULACIÓN ***

#define STAGING_TIMEOUT_MS              3000    // Timeout seguridad separación

#define BURNOUT2_ACCEL_THRESHOLD_MS2    12.0f   // Motor 2 apagado
#define BURNOUT2_CONFIRM_MS             100

// Detección de apogeo — doble confirmación:
//
//   1. COASTING → APOGEE:
//      La velocidad vertical debe mantenerse por debajo de
//      APOGEE_SPEED_THRESHOLD_MS durante APOGEE_CONFIRM_MS consecutivos,
//      Y la varianza de altitud del EKF debe ser menor que
//      APOGEE_EKF_VAR_ALT_MAX (el estimador está en buen estado).
//
//      Si el EKF tiene alta incertidumbre (spike de aceleración en burnout2)
//      no se acepta la detección aunque la velocidad parezca negativa —
//      se espera a que el estimador converja.
//
//   2. APOGEE → DESCENT:
//      Una vez en PHASE_APOGEE (Pyro1 ya disparado), la FSM permanece
//      al menos APOGEE_MIN_DWELL_MS antes de transicionar a descenso.
//      Esto actúa como salvaguarda contra rebotes del estimador en la
//      cúspide de la trayectoria.
//
#define APOGEE_SPEED_THRESHOLD_MS      -2.0f    // Velocidad vertical [m/s] para candidato
#define APOGEE_CONFIRM_MS               300     // Tiempo mínimo sostenido [ms] (30 ciclos @ 100Hz)
#define APOGEE_EKF_VAR_ALT_MAX          25.0f   // Varianza máxima aceptable [m²] → σ ≤ 5 m
#define APOGEE_MIN_DWELL_MS             300     // Tiempo mínimo en PHASE_APOGEE antes de → DESCENT

#define LANDED_SPEED_THRESHOLD_MS       1.5f
#define LANDED_CONFIRM_MS               3000

#define PYRO_FIRE_DURATION_MS           500     // Duración pulso de ignición [ms]


// --- Estado de vuelo ---
struct FlightState {
    FlightPhase phase;
    uint32_t    phase_start_ms;
    uint32_t    time_in_phase_ms;
    uint32_t    flight_time_ms;
    float       max_altitude_agl;
    float       max_speed;
    bool        pyro4_fired;            // Separación etapas + ignición motor 2
    bool        pyro1_fired;            // Recuperación (apogeo)
    bool        separation_confirmed;   // Confirmado por Ret1/Ret2 = LOW
    bool        control_active;         // PID activo solo en POWERED_2 y COASTING
};

// --- Canal pirotécnico activo (para disparo no bloqueante) ---
// El pin se activa en firePyro() y se desactiva automáticamente en
// _updatePyros() cuando han transcurrido PYRO_FIRE_DURATION_MS.
// Soporta hasta PYRO_MAX_ACTIVE canales simultáneos (normalmente 1).
#define PYRO_MAX_ACTIVE  2

struct PyroActive {
    uint8_t  pin;
    uint8_t  led_pin;
    uint32_t start_ms;
    bool     active;
};


// =============================================================================
class FlightFSM {
public:
    FlightFSM();

    void begin();

    // Actualizar FSM — llamar a 100Hz
    void update(const State &s);

    const FlightState& getFlightState() const { return _fs; }
    FlightPhase        getPhase()        const { return _fs.phase; }
    bool               isControlActive() const { return _fs.control_active; }

    // Forzar fase (testing en tierra)
    void forcePhase(FlightPhase phase);

    // true si algún canal pirotécnico sigue activo (pulso en curso)
    bool isPyroActive() const;

    bool isArmed() const { return _armed; }
    void setArmed(bool v) { _armed = v; }

private:
    FlightState _fs;
    uint32_t    _launch_time_ms      = 0;
    bool        _armed               = false;

    // Candidatos y timestamps de confirmación
    bool     _launch_candidate       = false;
    bool     _burnout1_candidate     = false;
    bool     _burnout2_candidate     = false;
    bool     _apogee_candidate       = false;
    bool     _landed_candidate       = false;
    uint32_t _launch_confirm_start   = 0;
    uint32_t _burnout1_confirm_start = 0;
    uint32_t _burnout2_confirm_start = 0;
    uint32_t _apogee_confirm_start   = 0;
    uint32_t _landed_confirm_start   = 0;

    // ── Pirotécnicos no bloqueantes ───────────────────────────────────────
    // Array de canales activos. firePyro() activa el pin y registra la hora.
    // _updatePyros() (llamado al inicio de update()) los apaga al vencer el tiempo.
    PyroActive _pyros[PYRO_MAX_ACTIVE];

    // Activa un canal pirotécnico sin bloquear el loop.
    // El pin permanece HIGH durante PYRO_FIRE_DURATION_MS y luego se apaga solo.
    void firePyro(uint8_t pin, uint8_t led_pin, const char* name);

    // Comprueba y apaga los canales cuyo pulso ha terminado.
    // Debe llamarse al inicio de cada update() — coste: < 1 µs.
    void _updatePyros();

    // Retorna true si Ret1 Y Ret2 leen LOW (etapas físicamente separadas)
    bool isSeparationConfirmed();

    void transitionTo(FlightPhase new_phase);

    // Acciones de entrada por fase
    void onEnterPowered1();
    void onEnterStaging();
    void onEnterPowered2();
    void onEnterCoasting();
    void onEnterApogee();
    void onEnterDescent();
    void onEnterLanded();
};
