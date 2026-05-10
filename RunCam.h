#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "config.h"

// =============================================================================
// DRIVER RunCam Split 4 V2 — RunCam Device Protocol v1
// =============================================================================
// La RunCam Split 4 V2 implementa el RunCam Device Protocol sobre UART
// a 115200 baud. Permite controlar grabación, modo y configuración desde
// el STM32 sin necesidad de intervención física.
//
// PROTOCOLO (RunCam Device Protocol v1):
//   Trama: [HEADER] [COMMAND] [CRC8]
//   HEADER: siempre 0xCC
//   CRC8:   polinomio 0xD5, calculado sobre [HEADER, COMMAND]
//
// INTEGRACIÓN CON EL VUELO:
//   PRE-LAUNCH (PAD):   Iniciar grabación → captura el despegue completo
//   LANDED:             Detener grabación → archivo cerrado limpiamente
//
//   De esta forma el video cubre desde el pad hasta el aterrizaje.
//   Si hay un fallo de energía la RunCam guarda lo que alcanzó a grabar.
//
// COMANDOS IMPLEMENTADOS:
//   startRecording()  — Simula presión del botón WiFi (inicia grabación)
//   stopRecording()   — Simula presión del botón WiFi (detiene grabación)
//   toggleRecording() — Alterna estado de grabación
//   powerOff()        — Simula botón de poder (apagado limpio)
//
// NOTA SOBRE LA ALIMENTACIÓN:
//   La RunCam está alimentada por 6V del UBEC con C1=470µF de filtro.
//   Tarda ~3 segundos en arrancar tras encenderse. El código tiene en cuenta
//   este tiempo de inicio antes de enviar el primer comando.
// =============================================================================


// --- Bytes del protocolo ---
#define RC_HEADER               0xCC

// --- Comandos de control de cámara ---
#define RC_CMD_CAMERA_CONTROL   0x01

// --- Acciones dentro de CAMERA_CONTROL ---
#define RC_ACTION_WIFI_BTN      0x00    // Simular botón WiFi (start/stop record)
#define RC_ACTION_POWER_BTN     0x01    // Simular botón de poder
#define RC_ACTION_CHANGE_MODE   0x02    // Cambiar modo (video/foto/config)

// Tiempo de arranque de la RunCam tras encendido [ms]
#define RUNCAM_BOOT_TIME_MS     4000

// Tiempo de espera entre comandos [ms] (evitar flooding)
#define RUNCAM_CMD_DELAY_MS     500


// =============================================================================
class RunCam {
public:
    // Acepta Stream& para ser compatible con HardwareSerial y SoftwareSerial.
    // La inicialización del serial (begin()) se hace externamente en el sketch.
    RunCam(Stream &serial);

    // Esperar boot de la cámara — NO llama begin() en el serial
    void begin();

    // Control de grabación
    void startRecording();
    void stopRecording();
    void toggleRecording();
    void powerOff();

    bool isRecording() const { return _recording; }
    bool isReady()     const { return _ready; }

private:
    Stream  &_serial;
    bool            _recording  = false;
    bool            _ready      = false;
    uint32_t        _last_cmd_ms = 0;

    // Calcular CRC8 con polinomio 0xD5
    uint8_t crc8(const uint8_t *data, uint8_t len);

    // Enviar trama de control
    void sendCommand(uint8_t action);

    // Respetar delay mínimo entre comandos
    void waitForReady();
};
