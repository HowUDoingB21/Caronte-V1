#pragma once
#include <Arduino.h>
#include "config.h"

// =============================================================================
// UTILIDADES LED4 — Canal exclusivo de comunicación de estado de la FC
// Separado en su propio archivo para que FlightFSM y otros módulos
// puedan llamar estas funciones sin depender del sketch principal.
// =============================================================================

// Parpadeos simples: N veces con periodo 'ms' encendido + 'ms' apagado
inline void led4Blink(uint8_t times, uint16_t ms) {
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(PIN_LED4, HIGH); delay(ms);
        digitalWrite(PIN_LED4, LOW);  delay(ms);
    }
}

// SOS: · · · — — — · · ·  (error crítico)
inline void led4SOS() {
    for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(PIN_LED4, HIGH); delay(150);
        digitalWrite(PIN_LED4, LOW);  delay(150);
    }
    delay(200);
    for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(PIN_LED4, HIGH); delay(450);
        digitalWrite(PIN_LED4, LOW);  delay(150);
    }
    delay(200);
    for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(PIN_LED4, HIGH); delay(150);
        digitalWrite(PIN_LED4, LOW);  delay(150);
    }
}
