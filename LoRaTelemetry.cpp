#include "LoRaTelemetry.h"
#include <string.h>

// =============================================================================
// TELEMETRÍA LoRa E32 — Implementación
// =============================================================================

LoRaTelemetry::LoRaTelemetry() {
    memset(&_pending_cmd, 0, sizeof(UplinkCmd));
}

// =============================================================================
// INICIALIZACIÓN
// =============================================================================
bool LoRaTelemetry::begin() {
    // Configurar pines de control del E32
    pinMode(PIN_LORA_M0,  OUTPUT);
    pinMode(PIN_LORA_M1,  OUTPUT);
    pinMode(PIN_LORA_AUX, INPUT);

    // Poner en modo normal (M0=0, M1=0) para transmisión transparente
    setMode(0, 0);

    // Iniciar UART
    LORA_SERIAL.begin(LORA_BAUD);

    // Esperar a que AUX suba (módulo listo) con timeout de 2 segundos
    uint32_t start = millis();
    while (!digitalRead(PIN_LORA_AUX)) {
        if ((millis() - start) > 2000) {
            Serial.println("[LORA] WARN: E32 no responde en AUX — continuando sin radio");
            // No retornamos false: el vuelo continúa sin telemetría
            _initialized = false;
            return false;
        }
        delay(10);
    }

    _initialized = true;
    Serial.println("[LORA] E32-900T20D listo");
    Serial.print("  UART: "); Serial.print(LORA_BAUD); Serial.println(" baud");
    Serial.println("  Modo: transparente (M0=0, M1=0)");
    return true;
}

// =============================================================================
// UPDATE PRINCIPAL
// =============================================================================
void LoRaTelemetry::update(const State       &state,
                            const FlightState &flight,
                            const PID_Output  &pid_out,
                            const ServoAngles &srv,
                            const GPS_Data    &gps,
                            bool imu_ok, bool baro_ok) {
    if (!_initialized) return;

    uint32_t now = millis();
    FlightPhase phase = (FlightPhase)flight.phase;

    // Verificar uplink entrante primero
    checkRx();

    if ((now - _last_status_ms) >= txIntervalStatus(phase)) {
        sendStatus(state, flight, pid_out, imu_ok, baro_ok);
        _last_status_ms = now;
    }
    if ((now - _last_attitude_ms) >= txIntervalAttitude(phase)) {
        sendAttitude(state, srv);
        _last_attitude_ms = now;
    }
    if ((now - _last_position_ms) >= txIntervalPosition(phase)) {
        sendPosition(state, gps);
        _last_position_ms = now;
    }
    if ((now - _last_heartbeat_ms) >= 1000) {
        if (phase == PHASE_PAD || phase == PHASE_LANDED) {
            sendHeartbeat();
        }
        _last_heartbeat_ms = now;
    }
}

// =============================================================================
// INTERVALOS DE TX POR FASE
// =============================================================================
uint32_t LoRaTelemetry::txIntervalStatus(FlightPhase phase) {
    switch (phase) {
        case PHASE_POWERED_1:
        case PHASE_POWERED_2:
        case PHASE_COASTING:  return 100;
        case PHASE_STAGING:
        case PHASE_DESCENT:   return 500;
        case PHASE_LANDED:    return 5000;
        default:              return 1000;
    }
}

uint32_t LoRaTelemetry::txIntervalAttitude(FlightPhase phase) {
    switch (phase) {
        case PHASE_POWERED_1:
        case PHASE_POWERED_2:
        case PHASE_COASTING:  return 100;
        case PHASE_DESCENT:   return 500;
        default:              return 2000;
    }
}

uint32_t LoRaTelemetry::txIntervalPosition(FlightPhase phase) {
    switch (phase) {
        case PHASE_POWERED_1:
        case PHASE_POWERED_2:
        case PHASE_COASTING:  return 200;
        case PHASE_DESCENT:   return 500;
        default:              return 2000;
    }
}

// =============================================================================
// CONSTRUCCIÓN Y ENVÍO DE PAQUETES
// =============================================================================
void LoRaTelemetry::sendStatus(const State &s, const FlightState &f,
                                const PID_Output &p, bool imu_ok, bool baro_ok) {
    PktStatus pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.flight_phase  = (uint8_t)f.phase;
    pkt.flight_time_s = (uint16_t)(f.flight_time_ms / 1000);
    pkt.max_altitude  = f.max_altitude_agl;
    pkt.seq_rx        = _seq_rx;
    pkt.flags = 0;
    if (imu_ok)           pkt.flags |= (1 << 0);
    if (baro_ok)          pkt.flags |= (1 << 1);
    if (s.altitude_valid) pkt.flags |= (1 << 2);
    if (_armed)           pkt.flags |= (1 << 3);
    if (f.control_active) pkt.flags |= (1 << 4);
    if (p.saturated_pitch)pkt.flags |= (1 << 5);
    if (p.saturated_yaw)  pkt.flags |= (1 << 6);
    if (p.saturated_roll) pkt.flags |= (1 << 7);
    if (s.zupt_active)    pkt.flags2 |= (1 << 0);
    // EKF healthy = varianza de altitud < 5m² y varianza de actitud < 0.01 rad²
    bool ekf_ok = (s.var_altitude < 5.0f) && (s.var_roll < 0.01f);
    if (ekf_ok)           pkt.flags2 |= (1 << 1);
    transmit(PKT_STATUS, (uint8_t*)&pkt, sizeof(pkt));
}

void LoRaTelemetry::sendAttitude(const State &s, const ServoAngles &srv) {
    PktAttitude pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.roll_cdeg      = (int16_t)(s.roll  * RAD_TO_DEG * 100.0f);
    pkt.pitch_cdeg     = (int16_t)(s.pitch * RAD_TO_DEG * 100.0f);
    pkt.yaw_cdeg       = (int16_t)(s.yaw   * RAD_TO_DEG * 100.0f);
    pkt.roll_rate_mrd  = (int16_t)(s.roll_rate  * 1000.0f);
    pkt.pitch_rate_mrd = (int16_t)(s.pitch_rate * 1000.0f);
    pkt.yaw_rate_mrd   = (int16_t)(s.yaw_rate   * 1000.0f);
    pkt.delta1_cdeg    = (int16_t)(srv.delta1 * RAD_TO_DEG * 100.0f);
    pkt.delta2_cdeg    = (int16_t)(srv.delta2 * RAD_TO_DEG * 100.0f);
    pkt.delta3_cdeg    = (int16_t)(srv.delta3 * RAD_TO_DEG * 100.0f);
    pkt.speed_cms      = (int16_t)(s.speed * 100.0f);
    // Diagnóstico EKF — incertidumbre como desviación estándar comprimida
    float std_alt = sqrtf(s.var_altitude);
    float std_att = sqrtf(fmaxf(s.var_roll, s.var_pitch));
    pkt.ekf_var_alt_dm2   = (uint8_t)fminf(std_alt * 10.0f,   255.0f);
    pkt.ekf_var_att_mrad  = (uint8_t)fminf(std_att * 1000.0f, 255.0f);
    transmit(PKT_ATTITUDE, (uint8_t*)&pkt, sizeof(pkt));
}

void LoRaTelemetry::sendPosition(const State &s, const GPS_Data &gps) {
    PktPosition pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.altitude_agl   = s.altitude_agl;
    pkt.vertical_speed = s.vertical_speed;
    pkt.air_density    = s.air_density;
    if (gps.fix) {
        pkt.lat_1e7  = (int32_t)(gps.latitude  * 1e7);
        pkt.lon_1e7  = (int32_t)(gps.longitude * 1e7);
        pkt.gps_alt_m = (int16_t)gps.altitude_msl;
        pkt.gps_sats  = gps.satellites;
    }
    pkt.sigma_vH_dm  = (uint8_t)fminf(s.sigma_vH * 10.0f, 255.0f);
    pkt.gps_updates  = (uint8_t)(s.gps_updates & 0xFF);
    transmit(PKT_POSITION, (uint8_t*)&pkt, sizeof(pkt));
}

void LoRaTelemetry::sendHeartbeat() {
    uint8_t payload[2] = { 0xBE, 0xA7 };
    transmit(PKT_HEARTBEAT, payload, 2);
}

void LoRaTelemetry::sendPyroEvent(uint8_t channel, uint8_t event_type,
                                   uint16_t flight_time_s) {
    if (!_initialized) return;
    PktPyroEvent pkt = { channel, event_type, flight_time_s };
    transmit(PKT_PYRO_EVENT, (uint8_t*)&pkt, sizeof(pkt));
}

void LoRaTelemetry::sendTestResponse(uint8_t test_id, uint8_t event_code,
                                      const char *msg) {
    if (!_initialized) return;
    PktTestResponse pkt;
    pkt.test_id    = test_id;
    pkt.event_code = event_code;
    memset(pkt.msg, 0, sizeof(pkt.msg));
    if (msg) strncpy(pkt.msg, msg, sizeof(pkt.msg) - 1);
    transmit(PKT_TEST_RESPONSE, (uint8_t*)&pkt, sizeof(pkt));
}

// =============================================================================
// TRANSMISIÓN — El E32 en modo transparente envía todo lo que llega por UART
// =============================================================================
bool LoRaTelemetry::transmit(uint8_t pkt_type, const uint8_t *payload, uint8_t len) {
    if (!_initialized) return false;

    // Esperar a que el módulo esté listo
    if (!waitAUX()) return false;

    uint8_t packet[TELEM_MAX_PACKET];
    uint8_t total = 3 + len;
    if (total > TELEM_MAX_PACKET) return false;

    packet[0] = TELEM_SYNC_BYTE;
    packet[1] = pkt_type;
    packet[2] = _seq_tx++;
    memcpy(&packet[3], payload, len);

    LORA_SERIAL.write(packet, total);
    return true;
}

// =============================================================================
// RECEPCIÓN UPLINK
// =============================================================================
void LoRaTelemetry::checkRx() {
    if (!LORA_SERIAL.available()) return;

    uint8_t buf[32];
    uint8_t len = 0;

    // Leer bytes disponibles con timeout corto
    uint32_t start = millis();
    while (LORA_SERIAL.available() && len < 32 && (millis() - start) < 5) {
        buf[len++] = LORA_SERIAL.read();
    }

    if (len < 3 || buf[0] != TELEM_SYNC_BYTE) return;

    _seq_rx = buf[2];
    processCommand(buf, len);
}

void LoRaTelemetry::processCommand(const uint8_t *buf, uint8_t len) {
    uint8_t cmd_type = buf[1];
    Serial.print("[LORA] CMD recibido: 0x"); Serial.println(cmd_type, HEX);

    _pending_cmd.type  = cmd_type;
    _pending_cmd.valid = true;
    uint8_t data_len = min((uint8_t)(len - 3), (uint8_t)8);
    memset(_pending_cmd.data, 0, 8);
    if (data_len > 0) memcpy(_pending_cmd.data, &buf[3], data_len);

    if      (cmd_type == CMD_ARM)    { _armed = true;  Serial.println("[LORA] ARMADO"); }
    else if (cmd_type == CMD_DISARM) { _armed = false; Serial.println("[LORA] DESARMADO"); }
}

UplinkCmd LoRaTelemetry::getCommand() {
    UplinkCmd cmd = _pending_cmd;
    _pending_cmd.valid = false;
    return cmd;
}

// =============================================================================
// CONTROL DE MODOS DEL E32
// =============================================================================
void LoRaTelemetry::setMode(uint8_t m0, uint8_t m1) {
    digitalWrite(PIN_LORA_M0, m0 ? HIGH : LOW);
    digitalWrite(PIN_LORA_M1, m1 ? HIGH : LOW);
    delay(E32_MODE_SWITCH_MS);
}

bool LoRaTelemetry::waitAUX(uint32_t timeout_ms) {
    uint32_t start = millis();
    while (!digitalRead(PIN_LORA_AUX)) {
        if ((millis() - start) >= timeout_ms) return false;
    }
    return true;
}
