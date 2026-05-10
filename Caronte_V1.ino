// =============================================================================
// CARONTE V1 — Computadora de Vuelo
// Orbital Dynamics | Rev 9.0
// =============================================================================
// Módulo actual: LoRa E32 + Buzzer + REV 9.0
// Siguiente módulo: Inversión Aerodinámica (CFD)
//
// LIBRERÍAS REQUERIDAS (instalar via Library Manager):
//   - TinyGPS++        by Mikal Hart
//
// PLATAFORMA: Arduino framework para STM32 (STM32duino)
//   - Board: "Generic STM32F4 series"
//   - Board part number: "Generic F405RGTx"
// =============================================================================

// =============================================================================
// DECLARACIONES UART — STM32duino requiere instanciar HardwareSerial
// explícitamente indicando los pines RX, TX
// =============================================================================
// GPS:    USART1 → PA10=RX, PA9=TX
// LoRa:   USART6 → PC7=RX,  PC6=TX  (E32-900T20D)
// RunCam: SoftwareSerial → PC2=RX, PC1=TX
HardwareSerial Serial1(PA10, PA9);    // GPS
HardwareSerial Serial3(PC7,  PC6);    // LoRa E32 (USART6)
#include <SoftwareSerial.h>
SoftwareSerial runcamSerial(PC2, PC1); // RunCam: RX=PC2, TX=PC1

#include <Wire.h>
#include <IWatchdog.h>      // IWDG — STM32duino built-in, no instalar
#include "config.h"
#include "ICM45686.h"
#include "IMU_Calibration.h"
#include "MS5611.h"
#include "GPS_NEO6M.h"
#include "Estimator.h"
#include "PIDController.h"
#include "FlightFSM.h"
#include "ServoController.h"
#include "LoRaTelemetry.h"
#include "LED_Utils.h"
#include "Buzzer.h"
#include "RunCam.h"
#include "TestMode.h"
#if ENABLE_SD_LOGGING
  #include "DataLogger.h"
#endif

// --- Forward declarations ---
void printDebug();

// --- Instancias ---
ICM45686         imu(IMU_I2C_ADDR);
IMU_Calibration  imuCal(imu);           // Calibración de 6 posiciones
ADXL375          adxl(ADXL375_I2C_ADDR); // Alto rango ±200g (respaldo)
MS5611           baro(BARO_I2C_ADDR);
GPS_NEO6M        gps(GPS_SERIAL);
StateEstimator   estimator(&imuCal, &adxl); // Estimador con cal. + ADXL375
PIDController    pid;
FlightFSM        fsm;
ServoController  servos;
LoRaTelemetry    lora;
Buzzer           buzzer;
RunCam           camera(runcamSerial);
TestMode         testMode(imu, baro, gps, estimator, pid, servos, lora, buzzer);
#if ENABLE_SD_LOGGING
  DataLogger     logger;
#endif

// --- Estructuras de datos compartidas ---
IMU_Data    imu_data;
Baro_Data   baro_data;
GPS_Data    gps_data;
PID_Output  pid_out;
ServoAngles servo_angles;

// --- Variables de tiempo del loop ---
uint32_t last_loop_us    = 0;
uint32_t last_debug_ms   = 0;
uint32_t last_baro_ms    = 0;

// --- Estado de inicialización ---
bool imu_ok    = false;
bool baro_ok   = false;
bool adxl_ok   = false;   // ADXL375 disponible
bool lora_ok   = false;
#if ENABLE_SD_LOGGING
bool logger_ok = false;
#endif


// =============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // Esperar terminal (máx 3s)

    Serial.println("==============================================");
    Serial.println("  CARONTE V1 — Orbital Dynamics");
    Serial.println("  Inicializando sensores...");
    Serial.println("==============================================");

    // --- Configurar pines pirotécnicos a LOW primero (seguridad) ---
    pinMode(PIN_PYRO1, OUTPUT); digitalWrite(PIN_PYRO1, LOW);
    pinMode(PIN_PYRO2, OUTPUT); digitalWrite(PIN_PYRO2, LOW);
    pinMode(PIN_PYRO3, OUTPUT); digitalWrite(PIN_PYRO3, LOW);
    pinMode(PIN_PYRO4, OUTPUT); digitalWrite(PIN_PYRO4, LOW);

    // --- Configurar pines LED ---
    // LED1/2/3: Normalmente controlados junto con sus MOSFETs pirotécnicos.
    //           Aquí solo se usan brevemente para confirmar sensores y luego
    //           se apagan obligatoriamente antes de salir del setup().
    // LED4:     Único LED disponible para comunicar estado de la FC.
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_LED4, OUTPUT);
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, LOW);
    digitalWrite(PIN_LED3, LOW);
    digitalWrite(PIN_LED4, LOW);

    // LED4 parpadea lento durante la inicialización
    led4Blink(2, 300);

    // --- Buzzer ---
    buzzer.begin();
    buzzer.startup();

    // --- Bus I2C ---
    Wire.begin();
    Wire.setClock(400000);  // Fast mode: 400kHz

    // --- Inicializar IMU ---
    imu_ok = imu.begin();
    imuCal.begin();  // Cargar calibración desde Flash si existe
    if (imu_ok) {
        digitalWrite(PIN_LED1, HIGH);   // Confirmación visual breve — se apaga al final
        imu.calibrate(200);             // ~1 segundo
    } else {
        Serial.print("[ERROR] IMU: ");
        Serial.println(imu.getLastError());
    }

    // --- Inicializar Barómetro ---
    baro_ok = baro.begin();
    if (baro_ok) {
        digitalWrite(PIN_LED2, HIGH);   // Confirmación visual breve — se apaga al final
        baro.setGroundLevel();
    } else {
        Serial.print("[ERROR] Barometro: ");
        Serial.println(baro.getLastError());
    }

    // --- Inicializar GPS ---
    gps.begin(GPS_BAUD);
    // Intentar establecer altitud MSL terrestre desde GPS (no bloquea el boot
    // si no hay fix — se reintentará en el loop cuando la FC esté en PAD)
    if (gps.setGroundAltMSL(10)) {
        estimator.setGroundAltMSL(gps.groundAltMSL());
        digitalWrite(PIN_LED3, HIGH);
    }

    // Pequeña pausa para que el operador vea los LEDs de confirmación
    delay(800);

    // *** APAGAR LED1, LED2, LED3 OBLIGATORIAMENTE ***
    // A partir de aquí solo el hardware pirotécnico los controla
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, LOW);
    digitalWrite(PIN_LED3, LOW);

    // --- Comunicar resultado por LED4 y Buzzer ---
    // Inicializar ADXL375 (alto rango ±200g — respaldo de saturación ICM)
    adxl_ok = adxl.begin();
    if (!adxl_ok) {
        Serial.println("[WARN] ADXL375 no disponible — sin respaldo alta-g");
    }

    if (imu_ok && baro_ok) {
        led4Blink(3, 100);
        digitalWrite(PIN_LED4, HIGH);
        buzzer.sensorsOK();
        Serial.println("\n[OK] Sistema listo. Iniciando loop de sensores.");
    } else {
        led4SOS();
        buzzer.sensorsError();
        Serial.println("\n[WARN] Iniciando con sensores degradados.");
    }

    // Disparar primera conversión asíncrona del barómetro
    baro.triggerPressure();
    last_baro_ms  = millis();
    last_loop_us  = micros();
    last_debug_ms = millis();

    // Inicializar estimador de estado
    estimator.begin();
    estimator.resetAttitude();

    // Inicializar controlador PID
    pid.begin();

    // Inicializar máquina de estados
    fsm.begin();

    // Inicializar servos (posición neutra)
    servos.begin();

    // Inicializar LoRa E32
    lora_ok = lora.begin();
    if (!lora_ok) {
        Serial.println("[WARN] LoRa no disponible — vuelo sin telemetria");
    }

#if ENABLE_SD_LOGGING
    logger_ok = logger.begin();
    if (!logger_ok) {
        Serial.println("[WARN] SD no disponible — vuelo sin logging");
    }
#endif

    // Inicializar RunCam — grabación controlada por la estación terrena
    // No se inicia automáticamente; esperar CMD_RUNCAM_START vía LoRa
    runcamSerial.begin(RUNCAM_BAUD);
    camera.begin();

    // =========================================================================
    // WATCHDOG DE HARDWARE — inicializar AL FINAL de setup()
    // =========================================================================
    // El IWDG del STM32F405 usa el oscilador LSI (~32kHz) y es completamente
    // independiente del núcleo — sigue corriendo aunque el CPU quede colgado.
    //
    // Una vez iniciado NO puede detenerse, ni por software ni por el depurador.
    // Si el loop principal no llama a IWatchdog.reload() en menos de
    // WATCHDOG_TIMEOUT_US, el IWDG fuerza un reset del sistema.
    //
    // Se inicia aquí (y no al principio de setup) porque varias rutinas de
    // inicialización son bloqueantes y más lentas que el timeout:
    //   - imu.calibrate(200):     ~1.5s
    //   - gps.setGroundAltMSL():  hasta ~4.5s si no hay fix inmediato
    //   - lora.begin():           hasta 2s esperando AUX
    //
    // Causa más probable de reset en vuelo: deadlock I2C, excepción FPU
    // sin handler, o corrupción de puntero en el loop de 100Hz.
    // =========================================================================
    if (IWatchdog.isReset(true)) {
        // Si arrancamos por reset del watchdog, loguearlo antes de continuar
        Serial.println("[IWDG] WARN: sistema reiniciado por WATCHDOG — "
                       "loop bloqueado en vuelo anterior");
        buzzer.beep(440, 200);
        delay(100);
        buzzer.beep(440, 200);
    }
    IWatchdog.begin(WATCHDOG_TIMEOUT_US);
    Serial.print("[IWDG] Watchdog activo — timeout: ");
    Serial.print(WATCHDOG_TIMEOUT_US / 1000);
    Serial.println(" ms");
}

// =============================================================================
void loop() {
    uint32_t now_us = micros();

    // =========================================================================
    // LOOP PRINCIPAL A 100Hz (cada 10,000 µs)
    // =========================================================================
    if ((now_us - last_loop_us) >= CONTROL_LOOP_US) {
        last_loop_us = now_us;

        // ------------------------------------------------------------------
        // PASO 1: Leer IMU principal ICM-45686 y ADXL375 de alto rango
        // ICM-45686: alta precisión, rango ±20g
        // ADXL375:   menor precisión, rango ±200g (respaldo en alta-g)
        // ------------------------------------------------------------------
        if (imu_ok) {
            imu.read(imu_data);
        }

        // Leer ADXL375 y rellenar campos de alto rango en IMU_Data
        imu_data.adxl_valid = false;
        if (adxl_ok) {
            ADXL375_Data adxl_data;
            if (adxl.read(adxl_data)) {
                imu_data.accel_hg_x = adxl_data.accel_x;
                imu_data.accel_hg_y = adxl_data.accel_y;
                imu_data.accel_hg_z = adxl_data.accel_z;
                imu_data.adxl_valid = true;
            }
        }

        // Detección de saturación del ICM-45686 con histéresis:
        //   Activa a 18g (ICM satura a 20g — margen de 2g)
        //   Desactiva a 15g para evitar oscilaciones en la frontera
        {
            float icm_mag = sqrtf(imu_data.accel_x * imu_data.accel_x +
                                  imu_data.accel_y * imu_data.accel_y +
                                  imu_data.accel_z * imu_data.accel_z);
            static bool icm_sat = false;
            if (!icm_sat && icm_mag > ICM_SATURATION_MS2) {
                icm_sat = true;
                Serial.println("[WARN] ICM saturado — usando ADXL375");
            } else if (icm_sat && icm_mag < ICM_RESUME_MS2) {
                icm_sat = false;
                Serial.println("[INFO] ICM recuperado");
            }
            // Solo activa el handoff si el ADXL tiene datos válidos
            imu_data.imu_saturated = icm_sat && imu_data.adxl_valid;
        }

        // ------------------------------------------------------------------
        // PASO 2: Leer barómetro (asíncrono — no bloquea el loop)
        // El barómetro tarda ~20ms por lectura completa, lo corremos
        // desacoplado del loop de control.
        // ------------------------------------------------------------------
        if (baro_ok) {
            baro.readResult(baro_data);

            // Si terminó la conversión, disparar la siguiente
            if (baro_data.valid) {
                baro.triggerPressure();
            }
        }

        // ------------------------------------------------------------------
        // PASO 3: Actualizar GPS (parsea bytes UART disponibles, no bloquea)
        // ------------------------------------------------------------------
        gps.update();

        if (gps.newDataAvailable()) {
            gps.read(gps_data);
            // Fase 3: actualizar GPS_INS y altitud GPS en el EKF
            estimator.updateGPS(gps_data);
            // Si aún no tenemos referencia MSL y ahora hay fix, intentar establecerla
            if (estimator.ekf().groundAltMSL() == 0.0f &&
                gps_data.fix && gps_data.valid) {
                if (gps.setGroundAltMSL(5)) {
                    estimator.setGroundAltMSL(gps.groundAltMSL());
                }
            }
            // LED3 NO se toca aquí — es exclusivo del canal pirotécnico 3
        }

        // ------------------------------------------------------------------
        // PASO 4: Actualizar estimador de estado
        // ------------------------------------------------------------------
        estimator.update(imu_data, baro_data, CONTROL_LOOP_DT);
        const State& state = estimator.getState();

        // ------------------------------------------------------------------
        // PASO 5: Controlador PID → torques y fuerzas requeridas
        // ------------------------------------------------------------------
        if (state.attitude_valid) {
            pid.update(state, CONTROL_LOOP_DT, pid_out);
        }

        // ------------------------------------------------------------------
        // PASO 6: Máquina de estados del vuelo
        // ------------------------------------------------------------------
        FlightPhase prev_phase = fsm.getPhase();
        fsm.update(state);
        FlightPhase curr_phase = fsm.getPhase();

        // ------------------------------------------------------------------
        // PASO 7: Control de servos
        // ------------------------------------------------------------------
        servos.update(pid_out, state, fsm.isControlActive(), servo_angles);

#if ENABLE_SD_LOGGING
        // ------------------------------------------------------------------
        // PASO 8: Logging de datos a SD
        // ------------------------------------------------------------------
        // Todo el ciclo de vida del archivo está aquí — única ubicación
        // para openFlightFile(), flush(), close() y log().
        // Antes existía código duplicado en PASO 6 que causaba una segunda
        // llamada a openFlightFile() en la misma transición PAD→POWERED_1,
        // lo que podía abrir un segundo archivo o corromper el primero.

        if (logger_ok) {
            // Abrir archivo en el momento exacto del despegue
            if (prev_phase == PHASE_PAD && curr_phase == PHASE_POWERED_1) {
                logger.openFlightFile();
            }

            // Flush forzado en eventos críticos — minimiza pérdida de datos
            // si la alimentación falla justo después de apogeo o aterrizaje
            if (curr_phase != prev_phase) {
                if (curr_phase == PHASE_APOGEE || curr_phase == PHASE_LANDED) {
                    logger.flush();
                }
            }

            // Escribir fila de datos en cada ciclo de 100Hz
            if (logger.isLogging()) {
                logger.log(millis(), state, fsm.getFlightState(),
                           pid_out, servo_angles, gps_data);
            }

            // Cerrar archivo limpiamente al aterrizar
            if (prev_phase != PHASE_LANDED && curr_phase == PHASE_LANDED) {
                logger.close();
            }
        }
#endif

        // Detener grabación de cámara al aterrizar (independiente del logging)
        if (prev_phase != PHASE_LANDED && curr_phase == PHASE_LANDED) {
            camera.stopRecording();
        }
    }

    // =========================================================================
    // TELEMETRÍA LoRa — Fuera del loop de 100Hz
    // =========================================================================
    if (lora_ok) {
        // Si el modo de prueba está activo, no enviamos telemetría de vuelo
        if (!testMode.isActive()) {
            lora.update(estimator.getState(),
                        fsm.getFlightState(),
                        pid_out, servo_angles, gps_data,
                        imu_ok, baro_ok);
        }

        if (lora.hasCommand()) {
            UplinkCmd cmd = lora.getCommand();

            // Activar modo de prueba (solo en PAD)
            if (cmd.type == CMD_ENTER_TEST) {
                if (fsm.getPhase() == PHASE_PAD) {
                    testMode.enter();
                } else {
                    lora.sendTestResponse(0, TEST_EVT_ERROR,
                        "Solo activable en PAD");
                }
            }
            // Si el modo de prueba está activo, darle prioridad en los comandos
            else if (testMode.isActive()) {
                if (!testMode.handleCommand(cmd)) {
                    if (cmd.type == CMD_FORCE_PHASE)
                        fsm.forcePhase((FlightPhase)cmd.data[0]);
                    else if (cmd.type == CMD_RUNCAM_START)
                        camera.startRecording();
                    else if (cmd.type == CMD_RUNCAM_STOP)
                        camera.stopRecording();
                }
            }
            // Modo vuelo normal
            else {
                if (cmd.type == CMD_FORCE_PHASE) {
                    fsm.forcePhase((FlightPhase)cmd.data[0]);
                } else if (cmd.type == CMD_SET_GAINS) {
                    float kp = cmd.data[1] / 100.0f;
                    float ki = cmd.data[2] / 100.0f;
                    float kd = cmd.data[3] / 100.0f;
                    if      (cmd.data[0] == 0) pid.setGainsPitch(kp, ki, kd);
                    else if (cmd.data[0] == 1) pid.setGainsYaw(kp, ki, kd);
                    else if (cmd.data[0] == 2) pid.setGainsRoll(kp, ki, kd);
                } else if (cmd.type == CMD_RUNCAM_START) {
                    camera.startRecording();
                } else if (cmd.type == CMD_RUNCAM_STOP) {
                    camera.stopRecording();
                } else if (cmd.type == CMD_CAL_START) {
                    // Solo permitir calibración en PAD y desarmado
                    if (fsm.getPhase() == PHASE_PAD && !fsm.isArmed()) {
                        imuCal.startCalibration();
                        Serial.println("[CMD] Calibracion iniciada desde GS");
                    } else {
                        Serial.println("[CMD] Calibracion rechazada — solo en PAD+DESARMADO");
                    }
                } else if (cmd.type == CMD_CAL_NEXT) {
                    if (imuCal.isActive()) {
                        imuCal.nextPosition();
                    }
                } else if (cmd.type == CMD_CAL_ABORT) {
                    imuCal.abort();

                } else if (cmd.type == CMD_ZUPT_COMMIT) {
                    // Aplicar bias ZUPT a calibración — solo en PAD y desarmado.
                    // data[0]: 0 = solo RAM (por defecto), 1 = RAM + Flash
                    if (fsm.getPhase() == PHASE_PAD && !fsm.isArmed()) {
                        bool to_flash = (cmd.data[0] == 1);
                        bool ok = to_flash
                            ? estimator.saveZUPTBias()
                            : estimator.commitZUPTBias();
                        if (ok) {
                            lora.sendTestResponse(0, TEST_EVT_DATA,
                                to_flash ? "ZUPT commiteado a Flash" : "ZUPT commiteado a RAM");
                            Serial.print("[CMD] ZUPT commit ");
                            Serial.println(to_flash ? "RAM+Flash" : "RAM");
                        } else {
                            lora.sendTestResponse(0, TEST_EVT_ERROR,
                                "ZUPT commit rechazado (no convergio)");
                            Serial.println("[CMD] ZUPT commit rechazado");
                        }
                    } else {
                        lora.sendTestResponse(0, TEST_EVT_ERROR,
                            "ZUPT commit: solo en PAD+DESARMADO");
                        Serial.println("[CMD] ZUPT commit rechazado — PAD+DESARMADO requerido");
                    }
                }
            }
        }
    }

    // Notificar al estimador si estamos en PAD (activa ZUPT)
    estimator.setPadPhase(fsm.getPhase() == PHASE_PAD);

    // =========================================================================
    // TEST MODE UPDATE — A 100Hz cuando está activo
    // =========================================================================
    if (testMode.isActive()) {
        testMode.update();
    }

    // =========================================================================
    // CALIBRACIÓN IMU — Actualizar máquina de estados si está activa
    // =========================================================================
    if (imuCal.isActive()) {
        // update() retorna true cuando termina (OK o error)
        // Se llama en cada ciclo del loop — la recolección de muestras
        // ocurre a la cadencia del loop principal (100Hz)
        imuCal.update();
    }

    // =========================================================================
    // BUZZER PERIÓDICO — Pitidos de localización según fase
    // =========================================================================
    FlightPhase current_phase = fsm.getPhase();
    if (current_phase == PHASE_DESCENT) {
        buzzer.updateRecovery();   // Pitido cada 2s durante descenso
    } else if (current_phase == PHASE_LANDED) {
        buzzer.updateLanded();     // Pitido cada 5s en tierra
    }

    // =========================================================================
    // DEBUG SERIAL — Imprime datos cada 500ms (no afecta el loop de control)
    // =========================================================================
    if ((millis() - last_debug_ms) >= 500) {
        last_debug_ms = millis();
        printDebug();
    }

    // =========================================================================
    // WATCHDOG FEED — Al final del loop, fuera del bloque de 100Hz
    // =========================================================================
    // Colocado aquí (y no dentro del if de 100Hz) para que el IWDG se
    // alimente en cada iteración del loop(), independientemente del timing.
    // Si el CPU queda bloqueado en cualquier punto por encima de esta línea
    // durante más de WATCHDOG_TIMEOUT_US, el IWDG fuerza el reset.
    //
    // Frecuencia mínima de feed en condiciones normales: cada ~10µs (spin del
    // loop esperando el siguiente ciclo de 100Hz). Mucho más que suficiente.
    IWatchdog.reload();
}

// =============================================================================
// FUNCIONES AUXILIARES
// =============================================================================

void printDebug() {
    const State& state = estimator.getState();

    Serial.println("--- SENSORES ---");

    // IMU
    if (imu_data.valid) {
        Serial.print("  Accel [m/s2]  X:");
        Serial.print(imu_data.accel_x, 3);
        Serial.print("  Y:"); Serial.print(imu_data.accel_y, 3);
        Serial.print("  Z:"); Serial.println(imu_data.accel_z, 3);

        Serial.print("  Gyro [rad/s]  X:");
        Serial.print(imu_data.gyro_x, 4);
        Serial.print("  Y:"); Serial.print(imu_data.gyro_y, 4);
        Serial.print("  Z:"); Serial.println(imu_data.gyro_z, 4);
    } else {
        Serial.println("  IMU: sin datos");
    }

    // Barómetro
    if (baro_data.valid) {
        Serial.print("  Presion: "); Serial.print(baro_data.pressure, 1); Serial.print(" Pa");
        Serial.print("  Temp: ");    Serial.print(baro_data.temperature, 1); Serial.print(" C");
        Serial.print("  Alt AGL: "); Serial.print(baro_data.altitude_agl, 2); Serial.println(" m");
    } else {
        Serial.println("  Barometro: sin datos");
    }

    // GPS
    Serial.print("  GPS Fix: "); Serial.print(gps_data.fix ? "SI" : "NO");
    Serial.print("  Sats: ");    Serial.print(gps_data.satellites);
    if (gps_data.fix) {
        Serial.print("  Vel: "); Serial.print(gps_data.speed_ms, 1); Serial.print(" m/s");
    }
    Serial.println();

    Serial.println("--- ESTADO ESTIMADO ---");
    Serial.print("  Actitud [deg]  Roll:");
    Serial.print(state.roll  * RAD_TO_DEG, 2);
    Serial.print("  Pitch:");
    Serial.print(state.pitch * RAD_TO_DEG, 2);
    Serial.print("  Yaw:");
    Serial.println(state.yaw * RAD_TO_DEG, 2);

    Serial.print("  Alt AGL: ");   Serial.print(state.altitude_agl, 2);   Serial.print(" m");
    Serial.print("  Vel Vert: ");  Serial.print(state.vertical_speed, 2);  Serial.print(" m/s");
    Serial.print("  Densidad: ");  Serial.print(state.air_density, 4);     Serial.println(" kg/m3");

    Serial.print("  Propulsion activa: "); Serial.println(state.high_accel ? "SI" : "NO");

    Serial.println("--- PID ---");
    Serial.print("  Torque [N·m]  Roll:");
    Serial.print(pid_out.torque_roll, 4);
    Serial.print("  Pitch:"); Serial.print(pid_out.torque_pitch, 4);
    Serial.print("  Yaw:");   Serial.println(pid_out.torque_yaw, 4);

    Serial.print("  Lift   [N]    Roll:");
    Serial.print(pid_out.lift_roll, 4);
    Serial.print("  Pitch:"); Serial.print(pid_out.lift_pitch, 4);
    Serial.print("  Yaw:");   Serial.println(pid_out.lift_yaw, 4);

    if (pid_out.saturated_pitch || pid_out.saturated_yaw || pid_out.saturated_roll) {
        Serial.print("  [WARN] Integral saturado —");
        if (pid_out.saturated_roll)  Serial.print(" ROLL");
        if (pid_out.saturated_pitch) Serial.print(" PITCH");
        if (pid_out.saturated_yaw)   Serial.print(" YAW");
        Serial.println();
    }

    Serial.println("--- VUELO ---");
    Serial.print("  Fase: "); Serial.print(PHASE_NAMES[fsm.getPhase()]);
    Serial.print("  Control: "); Serial.print(fsm.isControlActive() ? "ACTIVO" : "INACTIVO");
    Serial.print("  T+"); Serial.print(fsm.getFlightState().flight_time_ms / 1000.0f, 2); Serial.println("s");
    Serial.print("  Alt max: "); Serial.print(fsm.getFlightState().max_altitude_agl, 1); Serial.println(" m");

    Serial.println("--- ALETAS [deg] ---");
    Serial.print("  Aleta1:"); Serial.print(servo_angles.delta1 * RAD_TO_DEG, 2);
    Serial.print("  Aleta2:"); Serial.print(servo_angles.delta2 * RAD_TO_DEG, 2);
    Serial.print("  Aleta3:"); Serial.print(servo_angles.delta3 * RAD_TO_DEG, 2);
    Serial.print("  Aleta4:"); Serial.print(servo_angles.delta4 * RAD_TO_DEG, 2);
    if (servo_angles.saturated) Serial.print("  [SAT]");
    Serial.println();
    Serial.println("------------------------");
}

// =============================================================================
// FIN DEL SKETCH
// =============================================================================
