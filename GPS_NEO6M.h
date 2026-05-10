#pragma once
#include <Arduino.h>
#include <TinyGPS++.h>

// =============================================================================
// DRIVER GPS NEO-6M (GY-NEO6MV2)
// Interfaz: UART (Serial1 en STM32F405 → PA9/PA10)
// Librería requerida: TinyGPS++ (instalar desde Library Manager)
// =============================================================================
// El NEO-6M transmite tramas NMEA a 9600 baud por defecto.
// Usamos TinyGPS++ para parsear NMEA y extraer los datos relevantes.
// =============================================================================


// --- Estructura de datos de salida ---
struct GPS_Data {
    double  latitude;       // [°] Latitud decimal
    double  longitude;      // [°] Longitud decimal
    float   altitude_msl;   // [m] Altitud sobre nivel del mar (GPS)
    float   speed_ms;       // [m/s] Velocidad horizontal sobre tierra
    float   course_deg;     // [°] Rumbo sobre tierra (0=Norte, 90=Este)
    uint8_t satellites;     // Número de satélites en vista
    float   hdop;           // Dilución horizontal de precisión (menor = mejor)
    bool    fix;            // true si hay fix GPS válido
    bool    valid;          // true si los datos son frescos (<2s)

    // Timestamp UTC
    uint8_t hour, minute, second;
};


// =============================================================================
class GPS_NEO6M {
public:
    GPS_NEO6M(HardwareSerial &serial);

    // Inicialización del puerto UART
    void begin(uint32_t baud = 9600);

    // Procesar bytes disponibles en el buffer UART
    // Llamar en cada iteración del loop principal — NO bloquea
    void update();

    // Retorna true si llegó una nueva posición desde el último read()
    bool newDataAvailable();

    // Copia los datos más recientes a la estructura destino
    void read(GPS_Data &data);

    // ── Fase 3: referencia de altitud terrestre ───────────────────────────

    // Registrar la altitud MSL actual como referencia del punto de lanzamiento.
    // Llamar cuando la FC está en fase PAD con fix GPS estable.
    // Promedia N lecturas válidas para reducir ruido puntual.
    // Retorna true cuando la referencia quedó establecida.
    bool setGroundAltMSL(uint8_t num_samples = 10);

    // Altitud MSL del punto de lanzamiento registrada [m]
    float groundAltMSL() const { return _ground_alt_msl; }

    // true si la referencia de altitud terrestre fue establecida
    bool groundAltValid() const { return _ground_alt_valid; }

    // Velocidad en marco NED [m/s] desde la última lectura GPS válida.
    // Solo válidos si speed_ms > GPS_MIN_SPEED (0.5 m/s).
    // vN = speed * cos(course),  vE = speed * sin(course)
    void getVelocityNED(float &vN, float &vE) const;

    // Retorna velocidad 3D estimada combinando GPS + barómetro
    float estimateVerticalSpeed(float alt_agl, float dt);

private:
    HardwareSerial  &_serial;
    TinyGPSPlus      _gps;
    GPS_Data         _last_data;
    bool             _new_data = false;

    float    _prev_altitude_agl = 0.0f;

    // Fase 3 — referencia altitud terrestre
    float    _ground_alt_msl    = 0.0f;
    bool     _ground_alt_valid  = false;

    static constexpr float GPS_MIN_SPEED = 0.5f;  // [m/s]

    void updateDataStruct();
};
