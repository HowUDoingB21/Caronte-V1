#include "GPS_NEO6M.h"

// =============================================================================
// GPS NEO-6M — Implementación
// =============================================================================

GPS_NEO6M::GPS_NEO6M(HardwareSerial &serial) : _serial(serial) {}

// -----------------------------------------------------------------------------
void GPS_NEO6M::begin(uint32_t baud) {
    _serial.begin(baud);
    delay(100);

    memset(&_last_data, 0, sizeof(_last_data));
    _last_data.fix   = false;
    _last_data.valid = false;

    Serial.println("[GPS] NEO-6M inicializado en UART1");
    Serial.println("[GPS] Esperando fix satelital... (puede tardar 30-90s en frío)");
}

// -----------------------------------------------------------------------------
// Llamar cada iteración del loop — procesa todos los bytes disponibles
void GPS_NEO6M::update() {
    bool updated = false;

    while (_serial.available() > 0) {
        char c = _serial.read();
        if (_gps.encode(c)) {
            updated = true;
        }
    }

    if (updated && _gps.location.isUpdated()) {
        updateDataStruct();
        _new_data = true;
    }
}

// -----------------------------------------------------------------------------
bool GPS_NEO6M::newDataAvailable() {
    return _new_data;
}

// -----------------------------------------------------------------------------
void GPS_NEO6M::read(GPS_Data &data) {
    data      = _last_data;
    _new_data = false;
}

// -----------------------------------------------------------------------------
// Estimación de velocidad vertical por diferenciación numérica de altitud AGL
float GPS_NEO6M::estimateVerticalSpeed(float alt_agl, float dt) {
    float vz = (alt_agl - _prev_altitude_agl) / dt;
    _prev_altitude_agl = alt_agl;
    return vz;
}

// =============================================================================
// PRIVADOS
// =============================================================================

void GPS_NEO6M::updateDataStruct() {
    if (_gps.location.isValid()) {
        _last_data.latitude   = _gps.location.lat();
        _last_data.longitude  = _gps.location.lng();
        _last_data.fix        = true;
    } else {
        _last_data.fix = false;
    }

    if (_gps.altitude.isValid()) {
        _last_data.altitude_msl = (float)_gps.altitude.meters();
    }

    if (_gps.speed.isValid()) {
        _last_data.speed_ms = (float)_gps.speed.mps();
    }

    if (_gps.course.isValid()) {
        _last_data.course_deg = (float)_gps.course.deg();
    }

    if (_gps.satellites.isValid()) {
        _last_data.satellites = (uint8_t)_gps.satellites.value();
    }

    if (_gps.hdop.isValid()) {
        _last_data.hdop = (float)_gps.hdop.hdop();
    }

    if (_gps.time.isValid()) {
        _last_data.hour   = _gps.time.hour();
        _last_data.minute = _gps.time.minute();
        _last_data.second = _gps.time.second();
    }

    // La data se considera válida si tiene fix y el HDOP es aceptable (<5.0)
    _last_data.valid = _last_data.fix && (_last_data.hdop < 5.0f);
}

// =============================================================================
// Fase 3 — Referencia de altitud MSL en tierra
// =============================================================================
bool GPS_NEO6M::setGroundAltMSL(uint8_t num_samples) {
    float   sum   = 0.0f;
    uint8_t count = 0;

    for (uint8_t attempt = 0; attempt < num_samples * 3 && count < num_samples; attempt++) {
        // Procesar datos disponibles
        update();

        if (_new_data && _last_data.fix && _last_data.valid &&
            _last_data.altitude_msl > -500.0f) {   // Sanity: sobre el nivel del mar
            sum += _last_data.altitude_msl;
            count++;
            _new_data = false;
        }
        delay(150);  // Dar tiempo al GPS entre lecturas
    }

    if (count >= 3) {
        _ground_alt_msl   = sum / count;
        _ground_alt_valid = true;
        Serial.print("[GPS] Altitud MSL terrestre establecida: ");
        Serial.print(_ground_alt_msl, 1);
        Serial.print(" m (");
        Serial.print(count);
        Serial.println(" muestras)");
        return true;
    }

    Serial.println("[GPS] No se pudo establecer altitud terrestre — sin fix suficiente");
    return false;
}

// =============================================================================
// Velocidad en marco NED desde la lectura más reciente
// =============================================================================
void GPS_NEO6M::getVelocityNED(float &vN, float &vE) const {
    if (!_last_data.valid || _last_data.speed_ms < GPS_MIN_SPEED) {
        vN = 0.0f;
        vE = 0.0f;
        return;
    }
    float cr = _last_data.course_deg * (float)(DEG_TO_RAD);
    vN = _last_data.speed_ms * cosf(cr);
    vE = _last_data.speed_ms * sinf(cr);
}
