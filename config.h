#pragma once

// =============================================================================
// CARONTE V1 - ARCHIVO DE CONFIGURACIÓN CENTRAL
// Orbital Dynamics | Rev 9.0
// =============================================================================

// -----------------------------------------------------------------------------
// PINES DE HARDWARE (según esquemático Caronte V1 Rev 9.0)
// -----------------------------------------------------------------------------

// --- I2C (Bus compartido ICM-45686 + MS5611) ---
#define PIN_I2C_SCL         PB6
#define PIN_I2C_SDA         PB7

// --- IMU ICM-45686 (U2) ---
// SAO = GND → dirección I2C: 0x68
#define IMU_I2C_ADDR        0x68
#define PIN_IMU_INT1        PC13
#define PIN_IMU_INT2        PB2

// --- IMU de alto rango ADXL375 (U5) ---
// SDO = GND → dirección I2C: 0x53  (comparte bus con ICM y MS5611)
// CS  = 3.3V → modo I2C
// Rango: ±200g | Resolución: 49 mg/LSB | ODR: 200Hz
// Rol: respaldo de aceleración cuando ICM-45686 se satura (>20g)
//      + fusión en EKF durante fases de alta-g (POWERED_1/2)
#define ADXL375_I2C_ADDR    0x53

// Umbral de saturación del ICM-45686 [m/s²]
// El ICM tiene rango ±20g. Activamos handoff al ADXL375 a 18g para
// evitar leer valores corruptos durante el acercamiento a la saturación.
// 18g × 9.80665 = 176.52 m/s²
#define ICM_SATURATION_MS2  176.52f   // 18g en m/s²

// Histéresis para volver al ICM después de un episodio de alta-g [m/s²]
// Volvemos al ICM cuando |a| cae por debajo de 15g (147.1 m/s²)
#define ICM_RESUME_MS2      147.10f   // 15g en m/s²

// --- Barómetro MS5611 (U3) ---
// CSB = 3.3V → dirección I2C: 0x77
#define BARO_I2C_ADDR       0x77

// --- GPS (GPS1 — 8821WV-04P) ---
// USART1: PA9=TX_STM32→RX_GPS | PA10=RX_STM32←TX_GPS
// HardwareSerial Serial1(PA10, PA9) declarado en Caronte_V1.ino
#define GPS_SERIAL          Serial1
#define GPS_BAUD            9600

// --- LoRa Ebyte E32-900T20D (LORA) ---
// USART6: PC6=TX_STM32→RXD_E32 | PC7=RX_STM32←TXD_E32
// HardwareSerial Serial3(PC7, PC6) declarado en Caronte_V1.ino
#define LORA_SERIAL         Serial3
#define LORA_BAUD           9600
#define PIN_LORA_M0         PA4
#define PIN_LORA_M1         PA6
#define PIN_LORA_AUX        PA5

// --- Servos ---
// Servo1/2: PA0/PA1 — TIM5_CH1/CH2
// Servo3/4: PB0/PB1 — TIM3_CH3/CH4 (reasignados de PA2/PA3 por conector Comp)
#define PIN_SERVO1          PA0
#define PIN_SERVO2          PA1
#define PIN_SERVO3          PB0
#define PIN_SERVO4          PB1

// --- Conector Comp (MCU secundario / Raspberry Pi) ---
// USART2: PA2=TX_STM32→Pi-RX | PA3=RX_STM32←Pi-TX
// HardwareSerial SerialComp(PA3, PA2) — declarar si se implementa comunicación
// Por ahora los pines están reservados; no inicializar para evitar conflictos
#define PIN_COMP_TX         PA2
#define PIN_COMP_RX         PA3

// --- Canales pirotécnicos ---
#define PIN_PYRO1           PB3
#define PIN_PYRO2           PB4
#define PIN_PYRO3           PB5

// --- Conector SEP (XH2.54-5P) ---
// Pines: GND | 3.3V | Pyro4 | Ret1 | Ret2
// NOTA: Sep1/Sep2 eliminados en REV 9.0 — la alimentación de los pines Ret
//       viene ahora directamente del rail de 3.3V del conector, no de GPIOs.
//       Esto libera PC3 y PC4. Ret1/Ret2 siguen siendo entradas con pulldown.
#define PIN_PYRO4           PC5
#define PIN_RET1            PB12
#define PIN_RET2            PB8

// --- LEDs ---
// LED1/2/3: Activan simultáneamente con sus MOSFETs pirotécnicos (Q1/Q2/Q5)
//           Parpadeo breve en setup para confirmar sensores — apagar antes de salir.
// LED4:     Canal exclusivo de estado de la FC.
#define PIN_LED1            PB13
#define PIN_LED2            PB14
#define PIN_LED3            PC0
#define PIN_LED4            PA15

// --- Buzzer (BZ1) ---
// Buzzer pasivo controlado por Q3 (2N7000) — soporta tone() para frecuencias
// PA7 = TIM14_CH1 → compatible con PWM para generación de tonos
#define PIN_BUZZER          PA7

// --- RunCam Split 4 V2 (U5) ---
// SoftwareSerial: PC1=TX_STM32→RX_Cam | PC2=RX_STM32←TX_Cam
// SoftwareSerial runcamSerial(PC2, PC1) declarado en Caronte_V1.ino
// USART6 liberado para el E32. La RunCam solo recibe comandos de 4 bytes
// ocasionalmente — SoftwareSerial a 115200 es suficiente para TX.
#define PIN_RUNCAM_TX       PC1
#define PIN_RUNCAM_RX       PC2
#define RUNCAM_BAUD         115200

// --- MicroSD (SDIO — pines fijos del periférico SDIO del STM32F405) ---
// PC8=DAT0, PC9=DAT1, PC10=DAT2, PC11=DAT3, PC12=CLK, PD2=CMD
// USART6 (PC6/PC7) liberado para E32 — SDIO completamente disponible
#define ENABLE_SD_LOGGING   1


// -----------------------------------------------------------------------------
// PARÁMETROS FÍSICOS DEL COHETE
// *** PLACEHOLDER - Reemplazar cuando el diseño esté finalizado ***
// -----------------------------------------------------------------------------

#define INERTIA_IXX         0.005441f
#define INERTIA_IXY        -0.011128f
#define INERTIA_IXZ        -0.085051f
#define INERTIA_IYX        -0.011128f
#define INERTIA_IYY         0.005457f
#define INERTIA_IYZ         0.041753f
#define INERTIA_IZX        -0.085051f
#define INERTIA_IZY         0.041753f
#define INERTIA_IZZ         0.048966f

#define LEVER_ARM_D         0.80f
#define DELTA_MAX_DEG       15.0f
#define DELTA_MAX_RAD       (DELTA_MAX_DEG * DEG_TO_RAD)


// -----------------------------------------------------------------------------
// PARÁMETROS DEL CONTROLADOR PID
// *** PLACEHOLDER - Requieren sintonización experimental ***
// -----------------------------------------------------------------------------

#define PID_PITCH_KP        1.0f
#define PID_PITCH_KI        0.0f
#define PID_PITCH_KD        0.1f
#define PID_YAW_KP          1.0f
#define PID_YAW_KI          0.0f
#define PID_YAW_KD          0.1f
#define PID_ROLL_KP         0.5f
#define PID_ROLL_KI         0.0f
#define PID_ROLL_KD         0.05f

#define TARGET_PITCH        0.0f
#define TARGET_YAW          0.0f
#define TARGET_ROLL         0.0f


// -----------------------------------------------------------------------------
// PARÁMETROS DEL LOOP DE CONTROL
// -----------------------------------------------------------------------------

#define CONTROL_LOOP_HZ     100
#define CONTROL_LOOP_DT     (1.0f / CONTROL_LOOP_HZ)
#define CONTROL_LOOP_US     (1000000 / CONTROL_LOOP_HZ)
#define PID_INTEGRAL_LIMIT  10.0f


// -----------------------------------------------------------------------------
// PARÁMETROS DE SERVOS
// -----------------------------------------------------------------------------

#define SERVO_PWM_MIN_US    1000
#define SERVO_PWM_MID_US    1500
#define SERVO_PWM_MAX_US    2000


// -----------------------------------------------------------------------------
// ATMÓSFERA ESTÁNDAR ISA
// -----------------------------------------------------------------------------

#define ISA_P0              101325.0f
#define ISA_T0              288.15f
#define ISA_L               0.0065f
#define ISA_R               287.05f
#define ISA_G               9.80665f


// -----------------------------------------------------------------------------
// WATCHDOG DE HARDWARE (IWDG)
// -----------------------------------------------------------------------------
// El IWDG del STM32F405 usa el oscilador LSI interno (~32kHz).
// Una vez iniciado NO puede detenerse — ni siquiera por el depurador.
//
// Timeout elegido: 500ms
//   - Loop nominal: 10ms (100Hz)  → 50 ciclos de margen antes del reset
//   - waitAUX() LoRa (max):  100ms máximo en loop
//   - logger.flush() apogeo: ~20ms
//   - Causa de reset: loop congelado por FPU exception, corrupción de
//     puntero, deadlock I2C, o cualquier condición no manejada.
//
// NOTA: No inicializar antes de que setup() termine — las rutinas de
// calibración IMU y setup GPS pueden tardar varios segundos.
// -----------------------------------------------------------------------------
#define WATCHDOG_TIMEOUT_US     500000UL   // 500ms en microsegundos
