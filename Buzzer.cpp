#include "Buzzer.h"

// =============================================================================
// BUZZER — Implementación
// =============================================================================

Buzzer::Buzzer() {}

void Buzzer::begin() {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
}

// -----------------------------------------------------------------------------
// PRIMITIVA BASE
// -----------------------------------------------------------------------------

void Buzzer::playNote(uint16_t freq, uint16_t duration_ms, uint16_t pause_ms) {
    tone(PIN_BUZZER, freq, duration_ms);
    delay(duration_ms);
    noTone(PIN_BUZZER);
    if (pause_ms > 0) delay(pause_ms);
}

void Buzzer::beep(uint16_t freq_hz, uint16_t duration_ms) {
    playNote(freq_hz, duration_ms, 0);
}

void Buzzer::silence() {
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, LOW);
}

// =============================================================================
// SECUENCIAS DE VUELO
// =============================================================================

// Arranque del sistema — escala ascendente corta
// Do - Mi - Sol  (acorde mayor ascendente)
void Buzzer::startup() {
    playNote(NOTE_C4, 120, 60);
    playNote(NOTE_E4, 120, 60);
    playNote(NOTE_G4, 200, 0);
}

// Sensores OK — 2 pitidos cortos y agudos
void Buzzer::sensorsOK() {
    playNote(NOTE_C6, 80, 80);
    playNote(NOTE_C6, 80, 0);
}

// Error de sensores — 3 notas descendentes + tono de alerta
void Buzzer::sensorsError() {
    playNote(NOTE_G4, 200, 80);
    playNote(NOTE_E4, 200, 80);
    playNote(NOTE_C4, 200, 80);
    // Tono de alerta: pitidos lentos graves
    for (uint8_t i = 0; i < 4; i++) {
        playNote(NOTE_C4, 400, 300);
    }
}

// Sistema armado — secuencia de 4 notas ascendentes
// Do - Mi - Sol - Do5  (confirmación positiva)
void Buzzer::armed() {
    playNote(NOTE_C4, 100, 50);
    playNote(NOTE_E4, 100, 50);
    playNote(NOTE_G4, 100, 50);
    playNote(NOTE_C5, 300, 0);
}

// Despegue detectado — chirp corto y agudo
void Buzzer::launchDetected() {
    playNote(NOTE_G6, 60, 0);
}

// Burnout motor 1, preparando separación — doble pitido grave
void Buzzer::staging() {
    playNote(NOTE_C4, 150, 100);
    playNote(NOTE_C4, 150, 0);
}

// Separación física confirmada — triple pitido ascendente
void Buzzer::separation() {
    playNote(NOTE_C5, 100, 60);
    playNote(NOTE_E5, 100, 60);
    playNote(NOTE_G5, 200, 0);
}

// Apogeo detectado — tono largo y agudo
void Buzzer::apogee() {
    playNote(NOTE_A5, 600, 0);
}

// Aterrizaje detectado — celebración + aviso de localización
void Buzzer::landed() {
    // Secuencia de celebración: Do-Mi-Sol-Mi-Do5
    playNote(NOTE_C4,  80, 40);
    playNote(NOTE_E4,  80, 40);
    playNote(NOTE_G4,  80, 40);
    playNote(NOTE_E4,  80, 40);
    playNote(NOTE_C5, 300, 100);
    // Tres pitidos de localización iniciales
    for (uint8_t i = 0; i < 3; i++) {
        playNote(NOTE_A4, 200, 200);
    }
    _last_beep_ms = millis();
}

// =============================================================================
// PITIDOS PERIÓDICOS — Llamar en el loop, gestionan su propio timing
// =============================================================================

// Durante el descenso: pitido regular cada 2s para localización en aire
void Buzzer::updateRecovery() {
    uint32_t now = millis();
    if ((now - _last_beep_ms) >= RECOVERY_BEEP_INTERVAL_MS) {
        playNote(NOTE_A4, 120, 0);
        _last_beep_ms = now;
    }
}

// Después de aterrizar: pitido lento cada 5s para localización en tierra
void Buzzer::updateLanded() {
    uint32_t now = millis();
    if ((now - _last_beep_ms) >= LANDED_BEEP_INTERVAL_MS) {
        playNote(NOTE_A4, 200, 80);
        playNote(NOTE_A4, 200, 0);
        _last_beep_ms = now;
    }
}
