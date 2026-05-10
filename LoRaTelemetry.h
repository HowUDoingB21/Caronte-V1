#pragma once
#include <Arduino.h>
#include "config.h"
#include "Estimator.h"
#include "FlightFSM.h"
#include "PIDController.h"
#include "ServoController.h"
#include "GPS_NEO6M.h"

// =============================================================================
// TELEMETRÍA LoRa — Ebyte E32-900T20D — Caronte V1
// =============================================================================
// El E32 es un módulo LoRa con interfaz UART transparente. El STM32 envía
// bytes por UART y el E32 los transmite por LoRa automáticamente.
// En tierra se usa otro E32 conectado a una PC como receptor.
//
// MODOS DE OPERACIÓN (M0, M1):
//   M0=0, M1=0 → Modo normal    — transmisión/recepción transparente (UART)
//   M0=1, M1=0 → Modo wake-up   — envía preámbulo para despertar receptores
//   M0=0, M1=1 → Modo ahorro    — solo recibe, sleep entre paquetes
//   M0=1, M1=1 → Modo sleep     — configuración vía AT commands por UART
//
// AUX: HIGH = módulo listo para enviar/recibir. Siempre verificar antes de TX.
//
// PROTOCOLO DE PAQUETES (idéntico al del SX1276):
//   [0xCA][TYPE][SEQ][PAYLOAD...]
//   Mismos tipos y formatos — compatible con estación de tierra existente.
//
// FRECUENCIA DE TRANSMISIÓN adaptativa por fase de vuelo:
//   PAD/LANDED:  1 paquete/s
//   POWERED/COASTING: 10 paquetes/s
//   DESCENT:     2 paquetes/s
//
// UPLINK: tierra → FC
//   CMD_ARM, CMD_DISARM, CMD_FORCE_PHASE, CMD_SET_GAINS
// =============================================================================

// Protocolo
#define TELEM_SYNC_BYTE         0xCA
#define TELEM_MAX_PAYLOAD       48
#define TELEM_MAX_PACKET        (3 + TELEM_MAX_PAYLOAD)

// Tipos de paquete downlink
#define PKT_STATUS              0x01
#define PKT_ATTITUDE            0x02
#define PKT_POSITION            0x03
#define PKT_PYRO_EVENT          0x04
#define PKT_HEARTBEAT           0x05
#define PKT_TEST_RESPONSE       0x10    // Respuesta de test hacia la estación

// Comandos uplink — vuelo
#define CMD_ARM                 0xA1
#define CMD_DISARM              0xA2
#define CMD_FORCE_PHASE         0xA3
#define CMD_SET_GAINS           0xA4

// Comandos uplink — modo de prueba
// Estos comandos solo tienen efecto cuando la FC está en modo de prueba.
// La FC ignora CMD_TEST_* durante el vuelo real.
#define CMD_ENTER_TEST          0xB0    // Activar modo de prueba remotamente
#define CMD_TEST_SELECT         0xB1    // Seleccionar test: data[0]=número (1-6, 0=salir)
#define CMD_TEST_SET_VELOCITY   0xB2    // Cambiar velocidad simulada: data[0-1]=vel_cms (uint16)
#define CMD_TEST_NEXT           0xB3    // Avanzar al siguiente paso (equivale a ENTER)
#define CMD_TEST_EXIT           0xB4    // Abortar test actual, volver al menú

// Comandos RunCam (grabación de video)
#define CMD_RUNCAM_START        0xC0    // Iniciar grabación
#define CMD_RUNCAM_STOP         0xC1    // Detener grabación

// Comandos de calibración IMU (6 posiciones)
// Solo válidos cuando el cohete está en FASE_PAD y desarmado.
// Secuencia: CMD_CAL_START → (6×) CMD_CAL_NEXT → calibración guardada en Flash
#define CMD_CAL_START           0xD0    // Inicia el procedimiento de calibración
#define CMD_CAL_NEXT            0xD1    // Confirma posición actual, avanza a la siguiente
#define CMD_CAL_ABORT           0xD2    // Aborta la calibración en curso
#define CMD_ZUPT_COMMIT         0xD3    // Aplica bias ZUPT a cal. RAM + Flash (solo en PAD+desarmado)
                                        // data[0]: 0=solo RAM, 1=RAM+Flash

// Timeout para esperar que AUX suba a HIGH antes de transmitir [ms]
#define E32_AUX_TIMEOUT_MS      100

// Tiempo de transición de modo al cambiar M0/M1 [ms] (datasheet: ≥2ms)
#define E32_MODE_SWITCH_MS      10


// --- Estructuras de paquetes (empaquetadas para minimizar tamaño) ---
#pragma pack(push, 1)

struct PktStatus {
    uint8_t  flight_phase;
    uint8_t  flags;         // b0=imu_ok b1=baro_ok b2=gps_fix b3=armed
                            // b4=ctrl_active b5=sat_pitch b6=sat_yaw b7=sat_roll
                            // b8=zupt_active (en flags2)
    uint16_t flight_time_s;
    float    max_altitude;
    int8_t   rssi;
    uint8_t  seq_rx;
    uint8_t  flags2;        // b0=zupt_active  b1=ekf_healthy
    uint8_t  _pad[1];
};

struct PktAttitude {
    int16_t  roll_cdeg;
    int16_t  pitch_cdeg;
    int16_t  yaw_cdeg;
    int16_t  roll_rate_mrd;
    int16_t  pitch_rate_mrd;
    int16_t  yaw_rate_mrd;
    int16_t  delta1_cdeg;
    int16_t  delta2_cdeg;
    int16_t  delta3_cdeg;
    int16_t  speed_cms;         // Velocidad total × 100 [cm/s]
    // EKF — diagnóstico (reemplaza padding anterior)
    uint8_t  ekf_var_alt_dm2;   // sqrt(var_altitude) × 10  [dm]   — resolución 0.1m
    uint8_t  ekf_var_att_mrad;  // sqrt(var_roll|pitch) × 1000 [mrad] — resolución 1mrad
};

struct PktPosition {
    float    altitude_agl;
    float    vertical_speed;
    float    air_density;
    int32_t  lat_1e7;
    int32_t  lon_1e7;
    int16_t  gps_alt_m;
    uint8_t  gps_sats;
    uint8_t  sigma_vH_dm;   // σ velocidad horizontal × 10 [dm/s] — resolución 0.1 m/s
    uint8_t  gps_updates;   // Número de correcciones GPS acumuladas (mod 256)
    uint8_t  _pad[3];
};

struct PktPyroEvent {
    uint8_t  channel;
    uint8_t  event_type;
    uint16_t flight_time_s;
};

// Respuesta genérica del modo de prueba hacia la estación de tierra.
// msg es texto ASCII terminado en '\0' — máximo 44 caracteres útiles.
struct PktTestResponse {
    uint8_t  test_id;       // Número de test activo (0 = menú)
    uint8_t  event_code;    // Código de evento (ver TEST_EVT_*)
    char     msg[42];       // Mensaje legible para la estación de tierra
};

// Códigos de evento para PktTestResponse
#define TEST_EVT_MENU           0x00    // Menú principal enviado
#define TEST_EVT_TEST_START     0x01    // Test iniciado
#define TEST_EVT_DATA           0x02    // Datos periódicos del test
#define TEST_EVT_TEST_END       0x03    // Test finalizado
#define TEST_EVT_ERROR          0xFF    // Error

#pragma pack(pop)

struct UplinkCmd {
    uint8_t  type;
    uint8_t  data[8];
    bool     valid;
};


// =============================================================================
class LoRaTelemetry {
public:
    LoRaTelemetry();

    bool begin();

    void update(const State        &state,
                const FlightState  &flight,
                const PID_Output   &pid_out,
                const ServoAngles  &servos,
                const GPS_Data     &gps,
                bool imu_ok, bool baro_ok);

    void sendPyroEvent(uint8_t channel, uint8_t event_type, uint16_t flight_time_s);

    // Enviar respuesta de test hacia la estación de tierra
    void sendTestResponse(uint8_t test_id, uint8_t event_code, const char *msg);

    bool      hasCommand()    const { return _pending_cmd.valid; }
    UplinkCmd getCommand();
    bool      isInitialized() const { return _initialized; }

private:
    bool      _initialized = false;
    uint8_t   _seq_tx      = 0;
    uint8_t   _seq_rx      = 0;
    bool      _armed       = false;
    UplinkCmd _pending_cmd;

    uint32_t _last_status_ms    = 0;
    uint32_t _last_attitude_ms  = 0;
    uint32_t _last_position_ms  = 0;
    uint32_t _last_heartbeat_ms = 0;

    // Control de modos del E32
    void setMode(uint8_t m0, uint8_t m1);
    bool waitAUX(uint32_t timeout_ms = E32_AUX_TIMEOUT_MS);

    // Intervalos de TX según fase
    uint32_t txIntervalStatus  (FlightPhase phase);
    uint32_t txIntervalAttitude(FlightPhase phase);
    uint32_t txIntervalPosition(FlightPhase phase);

    // Envío de paquetes
    void sendStatus  (const State &s, const FlightState &f,
                      const PID_Output &p, bool imu_ok, bool baro_ok);
    void sendAttitude(const State &s, const ServoAngles &srv);
    void sendPosition(const State &s, const GPS_Data &gps);
    void sendHeartbeat();

    bool transmit(uint8_t pkt_type, const uint8_t *payload, uint8_t len);

    // Recepción uplink
    void checkRx();
    void processCommand(const uint8_t *buf, uint8_t len);
};
