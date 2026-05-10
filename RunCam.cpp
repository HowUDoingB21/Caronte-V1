#include "RunCam.h"

// =============================================================================
// RunCam Device Protocol — Implementación
// =============================================================================

RunCam::RunCam(Stream &serial) : _serial(serial) {}

// begin() solo espera el boot de la cámara.
// La inicialización del SoftwareSerial se hace en Caronte_V1.ino
// antes de llamar a camera.begin().
void RunCam::begin() {
    Serial.println("[CAM] Esperando arranque de RunCam Split 4...");
    delay(RUNCAM_BOOT_TIME_MS);
    _ready       = true;
    _recording   = false;
    _last_cmd_ms = millis();
    Serial.println("[CAM] RunCam lista");
}

// =============================================================================
// CONTROL DE GRABACIÓN
// =============================================================================

void RunCam::startRecording() {
    if (!_ready || _recording) return;

    Serial.println("[CAM] Iniciando grabacion");
    sendCommand(RC_ACTION_WIFI_BTN);
    _recording = true;
}

void RunCam::stopRecording() {
    if (!_ready || !_recording) return;

    Serial.println("[CAM] Deteniendo grabacion");
    sendCommand(RC_ACTION_WIFI_BTN);
    _recording = false;
}

void RunCam::toggleRecording() {
    if (_recording) stopRecording();
    else            startRecording();
}

void RunCam::powerOff() {
    if (!_ready) return;

    if (_recording) stopRecording();
    delay(RUNCAM_CMD_DELAY_MS);

    Serial.println("[CAM] Apagando RunCam");
    sendCommand(RC_ACTION_POWER_BTN);
    _ready     = false;
    _recording = false;
}

// =============================================================================
// PROTOCOLO
// =============================================================================

// Trama: [0xCC] [CMD=0x01] [ACTION] [CRC8 de los 3 bytes anteriores]
void RunCam::sendCommand(uint8_t action) {
    waitForReady();

    uint8_t frame[4];
    frame[0] = RC_HEADER;
    frame[1] = RC_CMD_CAMERA_CONTROL;
    frame[2] = action;
    frame[3] = crc8(frame, 3);

    _serial.write(frame, 4);
    _last_cmd_ms = millis();

    Serial.print("[CAM] CMD enviado: 0x");
    Serial.print(frame[1], HEX);
    Serial.print(" action=0x");
    Serial.print(frame[2], HEX);
    Serial.print(" crc=0x");
    Serial.println(frame[3], HEX);
}

// CRC-8 con polinomio 0xD5 (estándar RunCam Device Protocol)
uint8_t RunCam::crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else            crc <<= 1;
        }
    }
    return crc;
}

void RunCam::waitForReady() {
    uint32_t elapsed = millis() - _last_cmd_ms;
    if (elapsed < RUNCAM_CMD_DELAY_MS) {
        delay(RUNCAM_CMD_DELAY_MS - elapsed);
    }
}
