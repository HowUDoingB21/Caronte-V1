#include "FlightFSM.h"
#include "Buzzer.h"

// Buzzer instanciado en Caronte_V1.ino — referencia externa
extern Buzzer buzzer;

// =============================================================================
// MÁQUINA DE ESTADOS — Implementación
// =============================================================================

FlightFSM::FlightFSM() {
    memset(&_fs,    0, sizeof(FlightState));
    memset(_pyros,  0, sizeof(_pyros));
    _fs.phase = PHASE_PAD;
}

// -----------------------------------------------------------------------------
void FlightFSM::begin() {
    memset(&_fs,    0, sizeof(FlightState));
    memset(_pyros,  0, sizeof(_pyros));
    _fs.phase          = PHASE_PAD;
    _fs.control_active = false;
    _fs.phase_start_ms = millis();

    // Pines pirotécnicos — LOW por defecto (seguridad)
    pinMode(PIN_PYRO1, OUTPUT); digitalWrite(PIN_PYRO1, LOW);
    pinMode(PIN_PYRO4, OUTPUT); digitalWrite(PIN_PYRO4, LOW);
    pinMode(PIN_PYRO2, OUTPUT); digitalWrite(PIN_PYRO2, LOW);
    pinMode(PIN_PYRO3, OUTPUT); digitalWrite(PIN_PYRO3, LOW);

    // Ret1/Ret2: INPUT_PULLDOWN
    // La señal HIGH viene ahora del rail 3.3V del conector SEP (REV 9.0)
    // HIGH = etapas unidas | LOW = etapas separadas (pulldown tira a GND)
    pinMode(PIN_RET1, INPUT_PULLDOWN);
    pinMode(PIN_RET2, INPUT_PULLDOWN);

    Serial.println("[FSM] Maquina de estados inicializada — PHASE_PAD");
    Serial.print("[FSM] Continuidad separacion: Ret1=");
    Serial.print(digitalRead(PIN_RET1) ? "HIGH(unido)" : "LOW(separado)");
    Serial.print(" Ret2=");
    Serial.println(digitalRead(PIN_RET2) ? "HIGH(unido)" : "LOW(separado)");
}

// =============================================================================
// UPDATE PRINCIPAL — 100Hz
// =============================================================================
void FlightFSM::update(const State &s) {
    uint32_t now = millis();

    // Apagar pirotécnicos cuyo pulso haya terminado — siempre lo primero
    _updatePyros();
    _fs.time_in_phase_ms = now - _fs.phase_start_ms;

    // Actualizar records de vuelo
    if (_fs.phase >= PHASE_POWERED_1) {
        _fs.flight_time_ms = now - _launch_time_ms;
        if (s.altitude_agl > _fs.max_altitude_agl)
            _fs.max_altitude_agl = s.altitude_agl;
        if (s.vertical_speed > _fs.max_speed)
            _fs.max_speed = s.vertical_speed;
    }

    switch (_fs.phase) {

    // -------------------------------------------------------------------------
    case PHASE_PAD:
        // Detectar despegue por aceleración vertical sostenida
        if (s.accel_vertical > LAUNCH_ACCEL_THRESHOLD_MS2 &&
            s.vertical_speed >= 0.0f) {
            if (!_launch_candidate) {
                _launch_candidate     = true;
                _launch_confirm_start = now;
            } else if ((now - _launch_confirm_start) >= LAUNCH_CONFIRM_MS) {
                transitionTo(PHASE_POWERED_1);
            }
        } else {
            _launch_candidate = false;
        }
        break;

    // -------------------------------------------------------------------------
    case PHASE_POWERED_1:
        // Detectar burnout motor 1 — aceleración cae sostenidamente
        if (s.accel_vertical < BURNOUT1_ACCEL_THRESHOLD_MS2) {
            if (!_burnout1_candidate) {
                _burnout1_candidate     = true;
                _burnout1_confirm_start = now;
            } else if ((now - _burnout1_confirm_start) >= BURNOUT1_CONFIRM_MS) {
                transitionTo(PHASE_STAGING);
            }
        } else {
            _burnout1_candidate = false;
        }
        break;

    // -------------------------------------------------------------------------
    case PHASE_STAGING:
        // Esperar el delay de separación antes de disparar Pyro4
        if (!_fs.pyro4_fired) {
            if (_fs.time_in_phase_ms >= STAGING_DELAY_MS) {
                firePyro(PIN_PYRO4, PIN_LED4, "PYRO4 (Separacion + Motor 2)");
                _fs.pyro4_fired = true;
                Serial.println("[FSM] Esperando confirmacion de separacion por Ret1/Ret2...");
            } else {
                // Mostrar cuenta regresiva cada 100ms durante el delay
                static uint32_t _last_countdown_ms = 0;
                if ((millis() - _last_countdown_ms) >= 100) {
                    uint32_t remaining = STAGING_DELAY_MS - _fs.time_in_phase_ms;
                    Serial.print("[FSM] Staging en: "); Serial.print(remaining); Serial.println(" ms");
                    _last_countdown_ms = millis();
                }
            }
        }
        // Monitorear separación física (solo después de haber disparado)
        else if (isSeparationConfirmed()) {
            _fs.separation_confirmed = true;
            Serial.println("[FSM] Separacion confirmada por Ret1/Ret2");
            transitionTo(PHASE_POWERED_2);
        }
        // Timeout de seguridad — si no se confirma, asumir separación exitosa
        else if (_fs.time_in_phase_ms >= (STAGING_DELAY_MS + STAGING_TIMEOUT_MS)) {
            Serial.println("[FSM] WARN: Timeout de separacion — asumiendo exitosa");
            _fs.separation_confirmed = false;
            transitionTo(PHASE_POWERED_2);
        }
        break;

    // -------------------------------------------------------------------------
    case PHASE_POWERED_2:
        // Detectar burnout motor 2
        if (s.accel_vertical < BURNOUT2_ACCEL_THRESHOLD_MS2) {
            if (!_burnout2_candidate) {
                _burnout2_candidate     = true;
                _burnout2_confirm_start = now;
            } else if ((now - _burnout2_confirm_start) >= BURNOUT2_CONFIRM_MS) {
                transitionTo(PHASE_COASTING);
            }
        } else {
            _burnout2_candidate = false;
        }
        break;

    // -------------------------------------------------------------------------
    case PHASE_COASTING:
        // Detectar apogeo — velocidad vertical sostenida por debajo del umbral
        // con EKF en buen estado (var_altitude dentro del límite aceptable).
        //
        // El spike de aceleración al final de POWERED_2 puede elevar
        // temporalmente s.var_altitude — en ese caso se resetea el candidato
        // y se espera a que el estimador converja antes de aceptar el apogeo.
        // Esto evita falsos positivos que dispararían Pyro1 prematuramente.
        if (s.vertical_speed < APOGEE_SPEED_THRESHOLD_MS) {
            if (s.var_altitude > APOGEE_EKF_VAR_ALT_MAX) {
                // EKF degradado — resetear ventana y esperar convergencia
                _apogee_candidate = false;
            } else if (!_apogee_candidate) {
                _apogee_candidate     = true;
                _apogee_confirm_start = now;
            } else if ((now - _apogee_confirm_start) >= APOGEE_CONFIRM_MS) {
                transitionTo(PHASE_APOGEE);
            }
        } else {
            _apogee_candidate = false;
        }
        break;

    // -------------------------------------------------------------------------
    case PHASE_APOGEE:
        // Las acciones (aletas neutro + Pyro1) se ejecutaron en onEnterApogee().
        //
        // Se permanece en esta fase al menos APOGEE_MIN_DWELL_MS antes de
        // transicionar a DESCENT. Razones:
        //
        //   1. El pulso pirotécnico de Pyro1 dura PYRO_FIRE_DURATION_MS (500ms)
        //      — transicionar antes no afecta al pyro (sigue activo en _pyros[])
        //      pero mantener la fase correcta es coherente con el estado real.
        //
        //   2. Guarda frente a rebotes del estimador: si Vz oscila alrededor
        //      de cero justo en la cúspide, el dwell asegura que PHASE_APOGEE
        //      dura al menos 300ms antes de que el descenso se registre.
        //
        //   3. La estación terrena y el DataLogger registran la duración real
        //      de la fase — un dwell de 10ms es indetectable, 300ms es útil.
        if (_fs.time_in_phase_ms >= APOGEE_MIN_DWELL_MS) {
            transitionTo(PHASE_DESCENT);
        }
        break;

    // -------------------------------------------------------------------------
    case PHASE_DESCENT:
        if (fabsf(s.vertical_speed) < LANDED_SPEED_THRESHOLD_MS) {
            if (!_landed_candidate) {
                _landed_candidate     = true;
                _landed_confirm_start = now;
            } else if ((now - _landed_confirm_start) >= LANDED_CONFIRM_MS) {
                transitionTo(PHASE_LANDED);
            }
        } else {
            _landed_candidate = false;
        }
        break;

    // -------------------------------------------------------------------------
    case PHASE_LANDED:
        // Estado terminal
        break;
    }
}

// =============================================================================
// DETECCIÓN DE SEPARACIÓN FÍSICA
// =============================================================================
bool FlightFSM::isSeparationConfirmed() {
    // Ambos pines deben leer LOW para confirmar separación
    // Un solo pin LOW podría ser un falso positivo (cable dañado)
    bool ret1_low = !digitalRead(PIN_RET1);
    bool ret2_low = !digitalRead(PIN_RET2);
    return (ret1_low && ret2_low);
}

// =============================================================================
// TRANSICIÓN DE FASE
// =============================================================================
void FlightFSM::transitionTo(FlightPhase new_phase) {
    Serial.print("[FSM] ");
    Serial.print(PHASE_NAMES[_fs.phase]);
    Serial.print(" → ");
    Serial.print(PHASE_NAMES[new_phase]);
    if (_launch_time_ms > 0) {
        Serial.print("  T+");
        Serial.print(_fs.flight_time_ms / 1000.0f, 2);
        Serial.print("s");
    }
    Serial.println();

    _fs.phase          = new_phase;
    _fs.phase_start_ms = millis();

    switch (new_phase) {
        case PHASE_POWERED_1: onEnterPowered1(); break;
        case PHASE_STAGING:   onEnterStaging();  break;
        case PHASE_POWERED_2: onEnterPowered2(); break;
        case PHASE_COASTING:  onEnterCoasting(); break;
        case PHASE_APOGEE:    onEnterApogee();   break;
        case PHASE_DESCENT:   onEnterDescent();  break;
        case PHASE_LANDED:    onEnterLanded();   break;
        default: break;
    }
}

// =============================================================================
// ACCIONES DE ENTRADA POR FASE
// =============================================================================

void FlightFSM::onEnterPowered1() {
    _launch_time_ms    = millis();
    _fs.control_active = false;
    buzzer.launchDetected();
    Serial.println("[FSM] Motor 1 encendido — aletas en NEUTRO");
}

void FlightFSM::onEnterStaging() {
    _fs.control_active = false;
    buzzer.staging();
    if (STAGING_DELAY_MS == 0) {
        Serial.println("[FSM] Burnout motor 1 — disparo Pyro4 inmediato");
    } else {
        Serial.print("[FSM] Burnout motor 1 — disparo Pyro4 en ");
        Serial.print(STAGING_DELAY_MS); Serial.println(" ms");
    }
}

void FlightFSM::onEnterPowered2() {
    _fs.control_active = true;
    buzzer.separation();
    Serial.println("[FSM] Segunda etapa — control por aletas ACTIVADO");
}

void FlightFSM::onEnterCoasting() {
    _fs.control_active = true;
    Serial.print("[FSM] Burnout motor 2. Alt: ");
    Serial.print(_fs.max_altitude_agl, 1); Serial.println(" m");
}

void FlightFSM::onEnterApogee() {
    _fs.control_active = false;
    buzzer.apogee();
    Serial.print("[FSM] APOGEO. Alt max: ");
    Serial.print(_fs.max_altitude_agl, 1); Serial.println(" m");
    if (!_fs.pyro1_fired) {
        firePyro(PIN_PYRO1, PIN_LED1, "PYRO1 (Recuperacion)");
        _fs.pyro1_fired = true;
    }
}

void FlightFSM::onEnterDescent() {
    _fs.control_active = false;
    Serial.println("[FSM] Descenso iniciado");
}

void FlightFSM::onEnterLanded() {
    _fs.control_active = false;
    buzzer.landed();
    Serial.println("[FSM] ATERRIZAJE DETECTADO");
    Serial.print("[FSM] Alt max: ");  Serial.print(_fs.max_altitude_agl, 1); Serial.println(" m");
    Serial.print("[FSM] Vel max: ");  Serial.print(_fs.max_speed, 1);        Serial.println(" m/s");
    Serial.print("[FSM] T vuelo: ");  Serial.print(_fs.flight_time_ms / 1000.0f, 2); Serial.println(" s");
}

// =============================================================================
// PIROTÉCNICOS NO BLOQUEANTES
// =============================================================================
//
// firePyro(): activa el pin inmediatamente y registra la hora de inicio.
//             Retorna en < 1 µs — no bloquea el loop de control.
//
// _updatePyros(): llamado al inicio de cada update(). Recorre el array
//                 de canales activos y apaga los que ya cumplieron su
//                 duración. Coste: < 1 µs cuando no hay canales activos.
//
// Capacidad: PYRO_MAX_ACTIVE canales simultáneos (actualmente 2).
//            En Caronte V1 nunca se disparan dos pirotécnicos al mismo
//            tiempo, pero el array permite hacerlo si fuera necesario.
//
// NOTA DE SEGURIDAD: Si la FC se reinicia mientras un canal estaba activo,
// begin() hace memset del array y no queda ningún canal "huérfano" en HIGH,
// porque begin() también hace digitalWrite(LOW) a todos los pines pirotécnicos.
// =============================================================================

void FlightFSM::_updatePyros() {
    uint32_t now = millis();
    for (uint8_t i = 0; i < PYRO_MAX_ACTIVE; i++) {
        if (_pyros[i].active &&
            (now - _pyros[i].start_ms) >= PYRO_FIRE_DURATION_MS) {
            digitalWrite(_pyros[i].pin,     LOW);
            digitalWrite(_pyros[i].led_pin, LOW);
            _pyros[i].active = false;
            Serial.print("[PYRO] Pulso terminado — pin "); Serial.println(_pyros[i].pin);
        }
    }
}

void FlightFSM::firePyro(uint8_t pin, uint8_t led_pin, const char* name) {
    // Buscar una ranura libre en el array
    for (uint8_t i = 0; i < PYRO_MAX_ACTIVE; i++) {
        if (!_pyros[i].active) {
            _pyros[i].pin      = pin;
            _pyros[i].led_pin  = led_pin;
            _pyros[i].start_ms = millis();
            _pyros[i].active   = true;

            digitalWrite(led_pin, HIGH);
            digitalWrite(pin,     HIGH);

            Serial.print("[PYRO] DISPARANDO: "); Serial.println(name);
            Serial.print("[PYRO] Pulso activo — duracion: ");
            Serial.print(PYRO_FIRE_DURATION_MS); Serial.println(" ms");
            return;
        }
    }
    // Si no hay ranura libre (no debería ocurrir), forzar disparo bloqueante
    // como última salvaguarda
    Serial.println("[PYRO] WARN: sin ranura libre — disparo bloqueante de emergencia");
    digitalWrite(led_pin, HIGH);
    digitalWrite(pin,     HIGH);
    delay(PYRO_FIRE_DURATION_MS);
    digitalWrite(pin,     LOW);
    digitalWrite(led_pin, LOW);
}

bool FlightFSM::isPyroActive() const {
    for (uint8_t i = 0; i < PYRO_MAX_ACTIVE; i++) {
        if (_pyros[i].active) return true;
    }
    return false;
}

// -----------------------------------------------------------------------------
void FlightFSM::forcePhase(FlightPhase phase) {
    Serial.print("[FSM] FORZANDO FASE (testing): ");
    Serial.println(PHASE_NAMES[phase]);
    transitionTo(phase);
}
