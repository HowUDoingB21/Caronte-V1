#include "DataLogger.h"
#include <stdio.h>

// =============================================================================
// DATA LOGGER — Implementación
// =============================================================================

// Instancia global del objeto SD (requerido por STM32SD)
#ifndef SD_DETECT_PIN
  #define SD_DETECT_PIN     PC15   // Pin de detección de tarjeta en WeAct F405
#endif

DataLogger::DataLogger() {
    memset(_filename, 0, sizeof(_filename));
}

// =============================================================================
// INICIALIZACIÓN
// =============================================================================
bool DataLogger::begin() {
    Serial.println("[LOG] Inicializando microSD via SDIO...");

    // STM32SD usa SDIO automáticamente en el STM32F405
    // Los pines PC8-PC12 + PD2 son manejados por el hardware SDIO
    if (!SD.begin(SD_DETECT_PIN)) {
        Serial.println("[LOG] ERROR: No se detectó tarjeta SD");
        _sd_ready = false;
        return false;
    }

    Serial.println("[LOG] SD lista");
    _sd_ready = true;
    return true;
}

// =============================================================================
// ABRIR ARCHIVO DE VUELO
// =============================================================================
bool DataLogger::openFlightFile() {
    if (!_sd_ready) {
        Serial.println("[LOG] ERROR: SD no inicializada");
        return false;
    }

    if (!findNextFilename()) {
        Serial.println("[LOG] ERROR: No se pudo crear nombre de archivo");
        return false;
    }

    _file = SD.open(_filename, FILE_WRITE);
    if (!_file) {
        Serial.print("[LOG] ERROR: No se pudo abrir ");
        Serial.println(_filename);
        return false;
    }

    writeHeader();
    _row_count     = 0;
    _last_flush_ms = millis();
    _logging       = true;

    Serial.print("[LOG] Archivo de vuelo abierto: ");
    Serial.println(_filename);
    return true;
}

// =============================================================================
// LOG — Llamar a 100Hz
// =============================================================================
void DataLogger::log(uint32_t          t_ms,
                     const State       &state,
                     const FlightState &flight,
                     const PID_Output  &pid,
                     const ServoAngles &servos,
                     const GPS_Data    &gps) {

    if (!_logging || !_file) return;

    writeRow(t_ms, state, flight, pid, servos, gps);
    _row_count++;

    // Flush periódico al disco
    if ((millis() - _last_flush_ms) >= FLUSH_INTERVAL_MS) {
        _file.flush();
        _last_flush_ms = millis();
    }
}

// =============================================================================
// FLUSH — Forzar escritura inmediata (apogeo, aterrizaje)
// =============================================================================
void DataLogger::flush() {
    if (!_logging || !_file) return;
    _file.flush();
    _last_flush_ms = millis();
    Serial.print("[LOG] Flush forzado. Filas guardadas: ");
    Serial.println(_row_count);
}

// =============================================================================
// CLOSE
// =============================================================================
void DataLogger::close() {
    if (!_logging || !_file) return;
    _file.flush();
    _file.close();
    _logging = false;
    Serial.print("[LOG] Archivo cerrado: ");
    Serial.print(_filename);
    Serial.print(" — ");
    Serial.print(_row_count);
    Serial.println(" filas");
}

// =============================================================================
// PRIVADOS
// =============================================================================

// Buscar FLT_0001.CSV, FLT_0002.CSV... hasta encontrar uno que no exista
bool DataLogger::findNextFilename() {
    for (uint16_t i = 1; i <= LOG_MAX_FILES; i++) {
        snprintf(_filename, sizeof(_filename), "FLT_%04d.CSV", i);
        if (!SD.exists(_filename)) {
            return true;
        }
    }
    return false;  // Todos los nombres ocupados
}

// -----------------------------------------------------------------------------
void DataLogger::writeHeader() {
    _file.println(
        "t_ms,"
        "phase,"
        "alt_agl_m,"
        "vz_ms,"
        "accel_vert_ms2,"
        "ax_ms2,"
        "ay_ms2,"
        "az_ms2,"
        "roll_deg,"
        "pitch_deg,"
        "yaw_deg,"
        "roll_rate_rads,"
        "pitch_rate_rads,"
        "yaw_rate_rads,"
        "air_density_kgm3,"
        "delta1_deg,"
        "delta2_deg,"
        "delta3_deg,"
        "delta4_deg,"
        "servo_saturated,"
        "tau_roll_Nm,"
        "tau_pitch_Nm,"
        "tau_yaw_Nm,"
        "ctrl_active,"
        "high_accel,"
        "gps_fix,"
        "gps_lat,"
        "gps_lon,"
        "gps_alt_m,"
        "gps_sats"
    );
}

// -----------------------------------------------------------------------------
void DataLogger::writeRow(uint32_t          t_ms,
                           const State       &s,
                           const FlightState &f,
                           const PID_Output  &pid,
                           const ServoAngles &srv,
                           const GPS_Data    &gps) {

    // Usar un buffer local para formatear la fila antes de escribirla
    // Esto minimiza el número de llamadas a File.print() que son lentas
    char buf[256];

    snprintf(buf, sizeof(buf),
        "%lu,"       // t_ms
        "%u,"        // phase
        "%.2f,"      // alt_agl_m
        "%.3f,"      // vz_ms
        "%.3f,"      // accel_vert_ms2
        "%.3f,"      // ax_ms2
        "%.3f,"      // ay_ms2
        "%.3f,"      // az_ms2
        "%.2f,"      // roll_deg
        "%.2f,"      // pitch_deg
        "%.2f,"      // yaw_deg
        "%.4f,"      // roll_rate
        "%.4f,"      // pitch_rate
        "%.4f,"      // yaw_rate
        "%.4f,"      // air_density
        "%.2f,"      // delta1_deg
        "%.2f,"      // delta2_deg
        "%.2f,"      // delta3_deg
        "%.2f,"      // delta4_deg
        "%u,"        // servo_saturated
        "%.4f,"      // tau_roll
        "%.4f,"      // tau_pitch
        "%.4f,"      // tau_yaw
        "%u,"        // ctrl_active
        "%u,"        // high_accel
        "%u,"        // gps_fix
        "%.7f,"      // gps_lat
        "%.7f,"      // gps_lon
        "%.1f,"      // gps_alt_m
        "%u",        // gps_sats
        (unsigned long)t_ms,
        (uint8_t)f.phase,
        s.altitude_agl,
        s.vertical_speed,
        s.accel_vertical,
        0.0f,        // ax — placeholder (agregar al State si se necesita)
        0.0f,        // ay
        0.0f,        // az
        s.roll  * RAD_TO_DEG,
        s.pitch * RAD_TO_DEG,
        s.yaw   * RAD_TO_DEG,
        s.roll_rate,
        s.pitch_rate,
        s.yaw_rate,
        s.air_density,
        srv.delta1 * RAD_TO_DEG,
        srv.delta2 * RAD_TO_DEG,
        srv.delta3 * RAD_TO_DEG,
        srv.delta4 * RAD_TO_DEG,
        (uint8_t)srv.saturated,
        pid.torque_roll,
        pid.torque_pitch,
        pid.torque_yaw,
        (uint8_t)f.control_active,
        (uint8_t)s.high_accel,
        (uint8_t)gps.fix,
        gps.fix ? gps.latitude  : 0.0,
        gps.fix ? gps.longitude : 0.0,
        gps.fix ? (double)gps.altitude_msl : 0.0,
        gps.fix ? (unsigned)gps.satellites : 0u
    );

    _file.println(buf);
}
