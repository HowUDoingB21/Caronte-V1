#pragma once
#include <Arduino.h>
#include "config.h"
#include "Estimator.h"
#include "FlightFSM.h"
#include "PIDController.h"
#include "ServoController.h"
#include "GPS_NEO6M.h"

// =============================================================================
// DATA LOGGER — MicroSD vía SDIO — Caronte V1
// =============================================================================
// Escribe un archivo CSV por vuelo en la tarjeta microSD integrada de la
// WeAct STM32F405. Usa el periférico SDIO del STM32F405 a través de la
// librería STM32duino FatFS.
//
// LIBRERÍA REQUERIDA:
//   STM32duino FatFS — incluida en STM32duino, activar en:
//   Tools → USB Support → (none) y verificar que "SD (via SDIO)" esté
//   disponible. Alternativamente usar: #include <STM32SD.h>
//
// FORMATO DE ARCHIVO:
//   Nombre:  FLT_XXXX.CSV  donde XXXX es un contador incremental
//   Header:  columnas en la primera línea
//   Filas:   una por iteración del loop de control (100Hz)
//             → ~100 filas/segundo → ~6000 filas/minuto
//             Un vuelo de 2 min ≈ 12,000 filas ≈ ~2.5 MB
//
// ESTRATEGIA DE ESCRITURA:
//   No se hace flush en cada fila (demasiado lento para 100Hz).
//   Se acumula en buffer de RAM y se hace flush cada FLUSH_INTERVAL_MS.
//   En caso de reset/crash los últimos FLUSH_INTERVAL_MS de datos se pierden.
//   Si se detecta apogeo o aterrizaje se fuerza un flush inmediato.
//
// COLUMNAS DEL CSV:
//   t_ms, phase, alt_agl_m, vz_ms, ax_ms2, ay_ms2, az_ms2,
//   roll_deg, pitch_deg, yaw_deg, roll_rate, pitch_rate, yaw_rate,
//   rho, delta1_deg, delta2_deg, delta3_deg, delta4_deg,
//   tau_pitch, tau_yaw, tau_roll,
//   gps_lat, gps_lon, gps_alt_m, gps_sats
// =============================================================================

#include <STM32SD.h>    // STM32duino SDIO driver

// Intervalo de flush al disco [ms] — balance entre seguridad y rendimiento
#define FLUSH_INTERVAL_MS       2000

// Tamaño del buffer de escritura en RAM [bytes]
// A 100Hz cada fila es ~150 bytes → 2s × 100Hz × 150B = 30KB
// El STM32F405 tiene 192KB RAM, 32KB es razonable
#define LOG_BUFFER_SIZE         32768

// Máximo número de archivos de vuelo (contador 0000-9999)
#define LOG_MAX_FILES           9999


// =============================================================================
class DataLogger {
public:
    DataLogger();

    // Inicializar SDIO y preparar sistema de archivos
    // Retorna true si la SD está lista
    bool begin();

    // Abrir un nuevo archivo de vuelo — llamar al detectar despegue
    bool openFlightFile();

    // Escribir una fila de datos — llamar a 100Hz
    void log(uint32_t       t_ms,
             const State    &state,
             const FlightState &flight,
             const PID_Output  &pid,
             const ServoAngles &servos,
             const GPS_Data    &gps);

    // Forzar escritura al disco — llamar en apogeo y aterrizaje
    void flush();

    // Cerrar el archivo limpiamente
    void close();

    bool isReady()    const { return _sd_ready; }
    bool isLogging()  const { return _logging; }
    const char* getFilename() const { return _filename; }
    uint32_t    getRowCount() const { return _row_count; }

private:
    bool    _sd_ready  = false;
    bool    _logging   = false;
    char    _filename[16];
    uint32_t _row_count     = 0;
    uint32_t _last_flush_ms = 0;

    File    _file;

    // Buscar el siguiente número de archivo disponible (FLT_0001.CSV, etc.)
    bool findNextFilename();

    // Escribir cabecera CSV
    void writeHeader();

    // Formatear una fila en el buffer y escribirla
    void writeRow(uint32_t t_ms,
                  const State       &state,
                  const FlightState &flight,
                  const PID_Output  &pid,
                  const ServoAngles &servos,
                  const GPS_Data    &gps);
};
