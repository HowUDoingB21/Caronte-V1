#pragma once
#include <Arduino.h>
#include "config.h"
#include "ICM45686.h"
#include "MS5611.h"
#include "GPS_NEO6M.h"
#include "Estimator.h"
#include "PIDController.h"
#include "ServoController.h"
#include "LoRaTelemetry.h"
#include "Buzzer.h"

// =============================================================================
// MODO DE PRUEBA — Caronte V1
// =============================================================================
// Toda la interacción es a través de la estación de tierra vía LoRa.
// El Serial USB solo imprime debug pasivo (nunca se lee).
//
// ACTIVACIÓN:
//   Enviar CMD_ENTER_TEST (0xB0) desde la estación de tierra en cualquier
//   momento mientras la FC está en PHASE_PAD.
//   La FC responde con PKT_TEST_RESPONSE confirmando el modo y envía el menú.
//
// COMANDOS DESDE LA ESTACIÓN:
//   CMD_TEST_SELECT (0xB1)      data[0] = número de test (1-6, 0 = salir/reset)
//   CMD_TEST_SET_VELOCITY (0xB2) data[0..1] = velocidad en cm/s (uint16 LE)
//   CMD_TEST_NEXT (0xB3)        Avanzar paso / confirmar
//   CMD_TEST_EXIT (0xB4)        Abortar test activo, volver al menú
//
// TESTS DISPONIBLES:
//   1 — Control de actitud: mueve el cohete, las aletas responden en tiempo real.
//       La FC envía datos de actitud+servos a 5Hz por LoRa.
//   2 — Servo sweep individual con confirmación entre cada servo.
//   3 — Sensor dump a 2Hz por LoRa (IMU, baro, GPS).
//   4 — Continuidad pirotécnica (sin fuego).
//   5 — LoRa link test: paquetes de ping/pong.
//   6 — Full dry run: simulación completa de todas las fases.
//
// SEGURIDAD:
//   PIN_PYRO1-4 se mantienen forzados a LOW en cada ciclo del modo de prueba.
// =============================================================================

#define TEST_SIM_VELOCITY_DEFAULT   80.0f   // [m/s]
#define TEST_ATTITUDE_TX_INTERVAL   200     // Enviar actitud a la estación [ms]
#define TEST_SENSOR_TX_INTERVAL     500     // Enviar sensor dump [ms]


class TestMode {
public:
    TestMode(ICM45686        &imu,
             MS5611          &baro,
             GPS_NEO6M       &gps,
             StateEstimator  &estimator,
             PIDController   &pid,
             ServoController &servos,
             LoRaTelemetry   &lora,
             Buzzer          &buzzer);

    // Activar modo de prueba — llamar cuando se recibe CMD_ENTER_TEST
    // Solo tiene efecto en PHASE_PAD. Retorna false si no es seguro activar.
    bool enter();

    // ¿Está activo el modo de prueba?
    bool isActive() const { return _active; }

    // Procesar un comando uplink — llamar desde el loop principal
    // cuando lora.hasCommand() retorna true y _active == true.
    // Retorna true si el comando fue consumido por el test mode.
    bool handleCommand(const UplinkCmd &cmd);

    // Actualizar el test activo — llamar en el loop a 100Hz cuando _active.
    // Envía telemetría de test periódicamente y actualiza servos.
    void update();

private:
    ICM45686        &_imu;
    MS5611          &_baro;
    GPS_NEO6M       &_gps;
    StateEstimator  &_estimator;
    PIDController   &_pid;
    ServoController &_servos;
    LoRaTelemetry   &_lora;
    Buzzer          &_buzzer;

    bool    _active      = false;
    uint8_t _current_test = 0;      // 0 = menú, 1-6 = test activo
    float   _sim_velocity = TEST_SIM_VELOCITY_DEFAULT;

    // Estado interno de cada test
    uint8_t  _sweep_idx   = 0;      // Test 2: servo actual
    bool     _sweep_ready = false;  // Test 2: esperando CMD_TEST_NEXT
    uint32_t _dryrun_phase_idx = 0; // Test 6: índice de fase actual
    bool     _dryrun_waiting   = false;

    uint32_t _last_tx_ms   = 0;
    uint32_t _last_loop_us = 0;

    // Datos de sensores compartidos
    IMU_Data    _imu_data;
    Baro_Data   _baro_data;
    ServoAngles _servo_angles;
    PID_Output  _pid_out;

    void enforcePyroSafety();
    void sendMenu();
    void respond(const char *msg, uint8_t event = TEST_EVT_DATA);

    // Ciclos de actualización de cada test (llamados desde update())
    void updateAttitudeControl();
    void updateSensorDump();
    void updateDryRun();

    // Acciones únicas de tests (llamadas desde handleCommand())
    void startTest(uint8_t test_id);
    void stopCurrentTest();
    void handleServoSweepNext();
    void handleDryRunNext();
    void handlePyroContinuity();

    bool initSensors();
};

