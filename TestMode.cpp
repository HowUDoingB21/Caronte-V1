#include "TestMode.h"
#include <math.h>
#include <stdio.h>

// =============================================================================
// MODO DE PRUEBA — Implementación
// Toda la interacción FC ↔ operador pasa por LoRa.
// Serial.print() es solo debug pasivo.
// =============================================================================

TestMode::TestMode(ICM45686        &imu,
                   MS5611          &baro,
                   GPS_NEO6M       &gps,
                   StateEstimator  &estimator,
                   PIDController   &pid,
                   ServoController &servos,
                   LoRaTelemetry   &lora,
                   Buzzer          &buzzer)
    : _imu(imu), _baro(baro), _gps(gps),
      _estimator(estimator), _pid(pid),
      _servos(servos), _lora(lora), _buzzer(buzzer) {}

// =============================================================================
// ACTIVACIÓN
// =============================================================================
bool TestMode::enter() {
    enforcePyroSafety();

    if (!initSensors()) {
        respond("WARN: sensores degradados, continuando", TEST_EVT_ERROR);
    }

    _servos.begin();
    _pid.begin();
    _estimator.begin();
    _estimator.resetAttitude();

    _active       = true;
    _current_test = 0;
    _last_loop_us = micros();

    _buzzer.armed();
    Serial.println("[TEST] Modo de prueba activado via LoRa");

    sendMenu();
    return true;
}

// =============================================================================
// PROCESAMIENTO DE COMANDOS (llamado desde loop principal)
// =============================================================================
bool TestMode::handleCommand(const UplinkCmd &cmd) {
    if (!_active) return false;

    enforcePyroSafety();

    switch (cmd.type) {

        case CMD_TEST_SELECT: {
            uint8_t sel = cmd.data[0];
            if (sel == 0) {
                // Salir del modo de prueba — reset completo
                respond("Saliendo del modo de prueba. Reiniciando...",
                        TEST_EVT_TEST_END);
                _buzzer.startup();
                delay(1000);
                NVIC_SystemReset();
            } else {
                startTest(sel);
            }
            return true;
        }

        case CMD_TEST_SET_VELOCITY: {
            // Velocidad en cm/s como uint16 little-endian
            uint16_t vel_cms = (uint16_t)cmd.data[0] | ((uint16_t)cmd.data[1] << 8);
            _sim_velocity = vel_cms / 100.0f;
            char buf[40];
            snprintf(buf, sizeof(buf), "Velocidad simulada: %.0f m/s", _sim_velocity);
            respond(buf, TEST_EVT_DATA);
            Serial.print("[TEST] "); Serial.println(buf);
            return true;
        }

        case CMD_TEST_NEXT: {
            if (_current_test == 2) handleServoSweepNext();
            else if (_current_test == 6) handleDryRunNext();
            return true;
        }

        case CMD_TEST_EXIT: {
            stopCurrentTest();
            return true;
        }

        default:
            return false;  // No es un comando de test — dejar que el loop lo procese
    }
}

// =============================================================================
// UPDATE — Llamar a 100Hz desde el loop principal cuando _active == true
// =============================================================================
void TestMode::update() {
    if (!_active) return;
    enforcePyroSafety();

    // Leer sensores en cada ciclo independientemente del test activo
    uint32_t now_us = micros();
    float dt = (now_us - _last_loop_us) * 1e-6f;
    if (dt < CONTROL_LOOP_DT * 0.5f) return;   // Limitar a ~100Hz
    _last_loop_us = now_us;

    _imu.read(_imu_data);
    _baro.readResult(_baro_data);
    if (_baro_data.valid) _baro.triggerPressure();
    _gps.update();
    _estimator.update(_imu_data, _baro_data, dt);

    // Ejecutar ciclo del test activo
    switch (_current_test) {
        case 1: updateAttitudeControl(); break;
        case 3: updateSensorDump();      break;
        case 6: updateDryRun();          break;
        default: break;  // Tests 2/4/5 son event-driven, no tienen update loop
    }
}

// =============================================================================
// TEST 1 — CONTROL DE ACTITUD (loop continuo)
// =============================================================================
void TestMode::updateAttitudeControl() {
    const State &state = _estimator.getState();

    // Estado con velocidad simulada inyectada
    State sim_state = state;
    sim_state.speed = _sim_velocity;

    // PID y servos activos — aletas se mueven físicamente
    _pid.update(sim_state, CONTROL_LOOP_DT, _pid_out);
    _servos.update(_pid_out, sim_state, true, _servo_angles);

    // Enviar telemetría de actitud+servos a la estación a 5Hz
    uint32_t now = millis();
    if ((now - _last_tx_ms) < TEST_ATTITUDE_TX_INTERVAL) return;
    _last_tx_ms = now;

    // Formatear mensaje compacto: R/P/Y en décimas de grado + deltas
    char buf[42];
    snprintf(buf, sizeof(buf), "R%.1fP%.1fY%.1f d%.1f%.1f%.1f%.1f",
        state.roll  * RAD_TO_DEG,
        state.pitch * RAD_TO_DEG,
        state.yaw   * RAD_TO_DEG,
        _servo_angles.delta1 * RAD_TO_DEG,
        _servo_angles.delta2 * RAD_TO_DEG,
        _servo_angles.delta3 * RAD_TO_DEG,
        _servo_angles.delta4 * RAD_TO_DEG);

    _lora.sendTestResponse(1, TEST_EVT_DATA, buf);

    // Debug por Serial (pasivo)
    Serial.print("[T1] "); Serial.println(buf);
}

// =============================================================================
// TEST 3 — SENSOR DUMP (loop continuo)
// =============================================================================
void TestMode::updateSensorDump() {
    uint32_t now = millis();
    if ((now - _last_tx_ms) < TEST_SENSOR_TX_INTERVAL) return;
    _last_tx_ms = now;

    const State &s = _estimator.getState();

    // Paquete 1: actitud + velocidades angulares
    char buf[42];
    snprintf(buf, sizeof(buf), "R%.1f P%.1f Y%.1f | rr%.3f pr%.3f",
        s.roll * RAD_TO_DEG, s.pitch * RAD_TO_DEG, s.yaw * RAD_TO_DEG,
        s.roll_rate, s.pitch_rate);
    _lora.sendTestResponse(3, TEST_EVT_DATA, buf);

    // Paquete 2: altitud + velocidad + densidad
    snprintf(buf, sizeof(buf), "Alt%.1fm Vz%.2f rho%.4f",
        s.altitude_agl, s.vertical_speed, s.air_density);
    _lora.sendTestResponse(3, TEST_EVT_DATA, buf);

    // Paquete 3: GPS
    GPS_Data gps_data;
    _gps.read(gps_data);
    if (gps_data.fix) {
        snprintf(buf, sizeof(buf), "GPS %.6f,%.6f %dsats",
            gps_data.latitude, gps_data.longitude, gps_data.satellites);
    } else {
        snprintf(buf, sizeof(buf), "GPS sin fix");
    }
    _lora.sendTestResponse(3, TEST_EVT_DATA, buf);

    Serial.print("[T3] Alt="); Serial.print(s.altitude_agl, 1);
    Serial.print(" R="); Serial.print(s.roll * RAD_TO_DEG, 1);
    Serial.print(" P="); Serial.println(s.pitch * RAD_TO_DEG, 1);
}

// =============================================================================
// TEST 6 — DRY RUN (loop continuo entre fases)
// =============================================================================
// Tabla de fases sintéticas
static const struct {
    const char  *name;
    uint8_t      phase_id;
    float        alt;
    float        vz;
    float        speed;
    uint32_t     duration_ms;
} DRY_RUN_PHASES[] = {
    { "PAD",       0, 0.0f,   0.0f,   0.0f,  2000  },
    { "POWERED_1", 1, 100.0f, 80.0f,  80.0f, 3000  },
    { "STAGING",   2, 300.0f, 60.0f,  62.0f, 1500  },
    { "POWERED_2", 3, 400.0f, 100.0f, 102.0f,3000  },
    { "COASTING",  4, 700.0f, 40.0f,  42.0f, 2000  },
    { "APOGEE",    5, 850.0f, 0.0f,   5.0f,  500   },
    { "DESCENT",   6, 700.0f,-15.0f,  17.0f, 3000  },
    { "LANDED",    7, 0.0f,   0.0f,   0.0f,  2000  },
};
static const uint8_t DRY_RUN_N = 8;

void TestMode::updateDryRun() {
    if (_dryrun_waiting || _dryrun_phase_idx >= DRY_RUN_N) return;

    auto &ph = DRY_RUN_PHASES[_dryrun_phase_idx];
    static uint32_t phase_start_ms = 0;
    static bool     phase_entered  = false;

    if (!phase_entered) {
        phase_start_ms = millis();
        phase_entered  = true;

        // Notificar fase al operador
        char buf[42];
        snprintf(buf, sizeof(buf), "FASE %s Alt=%.0f Vz=%.0f",
                 ph.name, ph.alt, ph.vz);
        _lora.sendTestResponse(6, TEST_EVT_DATA, buf);
        Serial.print("[T6] >>> "); Serial.println(buf);

        // Buzzer según fase
        switch (ph.phase_id) {
            case 1: _buzzer.launchDetected(); break;
            case 2: _buzzer.staging();        break;
            case 3: _buzzer.separation();     break;
            case 5: _buzzer.apogee();         break;
            case 7: _buzzer.landed();         break;
            default: break;
        }
    }

    // Inyectar estado sintético
    State sim;
    memset(&sim, 0, sizeof(sim));
    sim.altitude_agl   = ph.alt;
    sim.vertical_speed = ph.vz;
    sim.speed          = ph.speed;
    sim.air_density    = 1.225f;
    sim.altitude_valid = true;
    sim.attitude_valid = true;
    // Pequeña perturbación para que los servos tengan algo que corregir
    sim.pitch = sinf(millis() * 0.001f) * 0.04f;
    sim.yaw   = cosf(millis() * 0.0013f) * 0.025f;

    bool ctrl = (ph.phase_id == 3 || ph.phase_id == 4);
    _pid.update(sim, CONTROL_LOOP_DT, _pid_out);
    _servos.update(_pid_out, sim, ctrl, _servo_angles);

    // Enviar telemetría de servos a 5Hz durante dry run
    uint32_t now = millis();
    if ((now - _last_tx_ms) >= 200) {
        _last_tx_ms = now;
        char buf[42];
        snprintf(buf, sizeof(buf), "%s t=%.1fs d=%.1f%.1f%.1f%.1f",
            ph.name,
            (now - phase_start_ms) / 1000.0f,
            _servo_angles.delta1 * RAD_TO_DEG,
            _servo_angles.delta2 * RAD_TO_DEG,
            _servo_angles.delta3 * RAD_TO_DEG,
            _servo_angles.delta4 * RAD_TO_DEG);
        _lora.sendTestResponse(6, TEST_EVT_DATA, buf);
    }

    // Fase completada: esperar confirmación del operador para avanzar
    if ((now - phase_start_ms) >= ph.duration_ms) {
        phase_entered    = false;
        _dryrun_waiting  = true;

        if (_dryrun_phase_idx < DRY_RUN_N - 1) {
            _lora.sendTestResponse(6, TEST_EVT_DATA,
                "Fase completada. CMD_TEST_NEXT para continuar.");
        } else {
            _lora.sendTestResponse(6, TEST_EVT_TEST_END,
                "Dry run completo.");
            _dryrun_phase_idx = 0;
            _dryrun_waiting   = false;
            _current_test     = 0;
            _servos.setNeutral();
            _pid.resetIntegrals();
            _buzzer.sensorsOK();
            sendMenu();
        }
    }
}

// =============================================================================
// ACCIONES EVENT-DRIVEN
// =============================================================================

void TestMode::startTest(uint8_t test_id) {
    stopCurrentTest();
    _current_test = test_id;
    _last_tx_ms   = 0;

    char buf[42];
    snprintf(buf, sizeof(buf), "Iniciando test %d", test_id);
    _lora.sendTestResponse(test_id, TEST_EVT_TEST_START, buf);
    Serial.print("[TEST] "); Serial.println(buf);
    _buzzer.beep(800, 100);

    switch (test_id) {
        case 1:
            snprintf(buf, sizeof(buf), "Mueve el cohete. V=%.0fm/s", _sim_velocity);
            respond(buf, TEST_EVT_DATA);
            _estimator.resetAttitude();
            _pid.resetIntegrals();
            _servos.setNeutral();
            break;

        case 2:
            _sweep_idx   = 0;
            _sweep_ready = false;
            respond("Servo sweep. CMD_TEST_NEXT para cada servo.", TEST_EVT_DATA);
            handleServoSweepNext();  // Empezar con servo 0 directamente
            break;

        case 3:
            respond("Sensor dump activo a 2Hz.", TEST_EVT_DATA);
            _estimator.resetAttitude();
            break;

        case 4:
            handlePyroContinuity();  // Acción única, no tiene loop
            _current_test = 0;
            sendMenu();
            break;

        case 5: {
            // LoRa link: enviar ping numerados — la estación responde
            respond("LoRa link test. Enviando pings. CMD_TEST_EXIT para salir.",
                    TEST_EVT_DATA);
            for (uint8_t i = 1; i <= 5; i++) {
                char ping[42];
                snprintf(ping, sizeof(ping), "PING #%d", i);
                _lora.sendTestResponse(5, TEST_EVT_DATA, ping);
                Serial.print("[T5] "); Serial.println(ping);
                delay(500);
            }
            respond("5 pings enviados.", TEST_EVT_TEST_END);
            _current_test = 0;
            sendMenu();
            break;
        }

        case 6:
            _dryrun_phase_idx = 0;
            _dryrun_waiting   = false;
            respond("Dry run iniciado. CMD_TEST_NEXT entre fases.", TEST_EVT_DATA);
            break;

        default:
            respond("Test no reconocido", TEST_EVT_ERROR);
            _current_test = 0;
            sendMenu();
            break;
    }
}

void TestMode::stopCurrentTest() {
    if (_current_test == 0) return;
    char buf[42];
    snprintf(buf, sizeof(buf), "Test %d detenido", _current_test);
    _lora.sendTestResponse(_current_test, TEST_EVT_TEST_END, buf);
    _servos.setNeutral();
    _pid.resetIntegrals();
    _current_test = 0;
    sendMenu();
}

void TestMode::handleServoSweepNext() {
    const char *nombres[] = { "Servo1", "Servo2", "Servo3", "Servo4" };

    if (_sweep_idx >= 4) {
        respond("Sweep completo.", TEST_EVT_TEST_END);
        _buzzer.sensorsOK();
        _current_test = 0;
        sendMenu();
        return;
    }

    char buf[42];
    snprintf(buf, sizeof(buf), "Barriendo %s...", nombres[_sweep_idx]);
    respond(buf, TEST_EVT_DATA);
    Serial.print("[T2] "); Serial.println(buf);

    _buzzer.beep(600, 80);
    _servos.sweepSingle(_sweep_idx, 800);
    delay(300);
    _servos.setNeutral();

    _sweep_idx++;
    if (_sweep_idx < 4) {
        snprintf(buf, sizeof(buf), "OK. CMD_TEST_NEXT para %s.",
                 nombres[_sweep_idx]);
        respond(buf, TEST_EVT_DATA);
    } else {
        respond("Sweep completo. CMD_TEST_NEXT para finalizar.",
                TEST_EVT_DATA);
    }
}

void TestMode::handleDryRunNext() {
    if (!_dryrun_waiting) return;
    _dryrun_waiting = false;
    _dryrun_phase_idx++;
}

void TestMode::handlePyroContinuity() {
    struct {
        const char *name;
        uint8_t     pin;
        uint8_t     led;
    } ch[] = {
        { "Pyro1(Recup)", PIN_PYRO1, PIN_LED1 },
        { "Pyro2       ", PIN_PYRO2, PIN_LED2 },
        { "Pyro3       ", PIN_PYRO3, PIN_LED3 },
        { "Pyro4(Sep)  ", PIN_PYRO4, PIN_LED4 },
    };

    respond("Verificando continuidad pirotecnica...", TEST_EVT_TEST_START);

    for (uint8_t i = 0; i < 4; i++) {
        enforcePyroSafety();

        pinMode(ch[i].pin, INPUT);
        delayMicroseconds(10);
        bool hi = digitalRead(ch[i].pin);
        pinMode(ch[i].pin, OUTPUT);
        digitalWrite(ch[i].pin, LOW);

        char buf[42];
        snprintf(buf, sizeof(buf), "%s: %s",
                 ch[i].name,
                 !hi ? "OK continuidad" : "ABIERTO sin carga");
        respond(buf, TEST_EVT_DATA);
        Serial.print("[T4] "); Serial.println(buf);

        if (!hi) {
            _buzzer.beep(1000, 60);
            digitalWrite(ch[i].led, HIGH);
            delay(200);
            digitalWrite(ch[i].led, LOW);
        } else {
            _buzzer.beep(300, 200);
        }
        delay(100);
    }

    // LEDs apagados al terminar
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, LOW);
    digitalWrite(PIN_LED3, LOW);
    digitalWrite(PIN_LED4, LOW);
    respond("Continuidad completada.", TEST_EVT_TEST_END);
}

// =============================================================================
// HELPERS
// =============================================================================

void TestMode::sendMenu() {
    _lora.sendTestResponse(0, TEST_EVT_MENU,
        "1=Actitud 2=Sweep 3=Dump");
    _lora.sendTestResponse(0, TEST_EVT_MENU,
        "4=Pyro 5=Link 6=DryRun 0=Reset");
    Serial.println("[TEST] Menu enviado via LoRa");
}

void TestMode::respond(const char *msg, uint8_t event) {
    _lora.sendTestResponse(_current_test, event, msg);
}

void TestMode::enforcePyroSafety() {
    digitalWrite(PIN_PYRO1, LOW);
    digitalWrite(PIN_PYRO2, LOW);
    digitalWrite(PIN_PYRO3, LOW);
    digitalWrite(PIN_PYRO4, LOW);
}

bool TestMode::initSensors() {
    Wire.begin();
    Wire.setClock(400000);
    bool ok = true;
    if (!_imu.begin()) { Serial.println("[TEST] WARN: IMU no disponible"); ok = false; }
    else _imu.calibrate(100);
    if (!_baro.begin()) { Serial.println("[TEST] WARN: Baro no disponible"); ok = false; }
    else { _baro.setGroundLevel(); _baro.triggerPressure(); }
    _gps.begin(GPS_BAUD);
    return ok;
}
