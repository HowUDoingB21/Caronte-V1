#pragma once
#include <Arduino.h>
#include "config.h"

// =============================================================================
// BUZZER — Caronte V1
// =============================================================================
// Buzzer pasivo controlado por Q3 (2N7000) en PA7 (TIM14_CH1).
// Usa tone() del STM32duino para generar frecuencias variables.
//
// SECUENCIAS DEFINIDAS:
//
//   startup()         — Al encender la FC. Escala ascendente de 3 notas.
//                       Indica que el sistema está arrancando.
//
//   sensorsOK()       — Todos los sensores inicializados correctamente.
//                       2 pitidos cortos y agudos.
//
//   sensorsError()    — Fallo en uno o más sensores.
//                       3 pitidos descendentes + tono de alerta continuo.
//
//   armed()           — Sistema pirotécnico armado (comando uplink recibido).
//                       Secuencia ascendente de 4 notas — similar a "listo".
//
//   launchDetected()  — Despegue confirmado. Chirp corto.
//
//   staging()         — Burnout motor 1, preparando separación.
//                       Doble pitido grave.
//
//   separation()      — Separación física confirmada por Ret1/Ret2.
//                       Triple pitido ascendente.
//
//   apogee()          — Apogeo detectado, Pyro1 disparado.
//                       Tono largo y agudo.
//
//   recovery()        — Llamada periódica durante el descenso.
//                       Pitido regular cada ~2 segundos para localización.
//
//   landed()          — Aterrizaje detectado.
//                       Secuencia de celebración + pitidos periódicos lentos
//                       para ayudar a localizar el cohete en campo.
//
//   beep()            — Pitido genérico configurable.
// =============================================================================

// Frecuencias de notas musicales [Hz]
#define NOTE_C4     262
#define NOTE_D4     294
#define NOTE_E4     330
#define NOTE_F4     349
#define NOTE_G4     392
#define NOTE_A4     440
#define NOTE_B4     494
#define NOTE_C5     523
#define NOTE_D5     587
#define NOTE_E5     659
#define NOTE_G5     784
#define NOTE_A5     880
#define NOTE_C6    1047
#define NOTE_G6    1568

// Intervalo entre pitidos de recuperación/aterrizaje [ms]
#define RECOVERY_BEEP_INTERVAL_MS    2000
#define LANDED_BEEP_INTERVAL_MS      5000


class Buzzer {
public:
    Buzzer();

    void begin();

    // Secuencias de vuelo
    void startup();
    void sensorsOK();
    void sensorsError();
    void armed();
    void launchDetected();
    void staging();
    void separation();
    void apogee();
    void landed();

    // Llamar periódicamente durante descenso — gestiona su propio timing
    void updateRecovery();

    // Llamar periódicamente después de aterrizar — pitidos lentos de localización
    void updateLanded();

    // Pitido genérico
    void beep(uint16_t freq_hz, uint16_t duration_ms);

    // Silencio
    void silence();

private:
    uint32_t _last_beep_ms = 0;

    void playNote(uint16_t freq, uint16_t duration_ms, uint16_t pause_ms = 50);
};
