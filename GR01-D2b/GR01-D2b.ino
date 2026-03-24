#include <ax12.h>

#include "poses.h"
#include "robot.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>


// ROBOT CONFIG

static double q0[DOFs]  = {0.0,    2.89,   -2.89,   0.0,     0.0};
static double qtg[DOFs] = {0.3530, 1.7217, -2.6405, 0.9188,  1.5708};
static double qtr[DOFs] = {1.5458, 1.1833, -0.9807, -1.7734, 3.1166};
static double qtv[DOFs] = {0.7484, 2.0075, -1.3783, -0.6292, 1.5708};

static const uint16_t T_HOME_MS = 1500;
static const uint16_t T_1A_MS   = 2000;
static const uint16_t T_1B_1_MS = 1500;
static const uint16_t T_1B_2_MS = 1500;
static const unsigned long PLOT_DURATION_MS = 10000; // 10 s

// EMG CONFIG

#define EMG_PIN A0

// Muestreo periódico
static const unsigned long EMG_TS_US = 1000; // 1 kHz

// Filtro simple en tiempo real:
// 1) baseline lento (quita DC / deriva)
// 2) rectificación
// 3) low-pass de envolvente
static const float BASELINE_ALPHA = 0.995f;
static const float ENV_ALPHA      = 0.75f;

// Histéresis y anti-rebote
static float emgThresholdHigh = 45.0f;
static float emgThresholdLow  = 30.0f;
static const unsigned long REFRACTORY_MS = 300;

// Confirmación de relajación para evitar apertura accidental
static const unsigned long RELEASE_CONFIRM_MS = 300;
static const unsigned long RELEASE_PLOT_DT_MS = 20;

// EMG STATE

volatile int   emgRaw = 0;
volatile float emgBaseline = 512.0f;
volatile float emgHighPass = 0.0f;
volatile float emgRectified = 0.0f;
volatile float emgEnvelope = 0.0f;

bool emgStateHigh = false;
unsigned long lastEmgSampleUs = 0;
unsigned long lastTriggerMs = 0;

// Para visualización serie
unsigned long lastPrintMs = 0;

// DECLARATIONS

void MenuOptions();
void Run_D2_EMG_Sequence();
void UpdateEMG();
bool EMGContractionDetected();
bool IsEMGCurrentlyContracted();
void CalibrateEMGThreshold();
void WaitForEMGTrigger(const char* message);
void WaitForEMGRelease(const char* message);
void PlotEMG();

// SETUP

void setup()
{
  ROBOT_Init();

  Serial.begin(115200);
  delay(300);

  pinMode(EMG_PIN, INPUT);

  Serial.println("############################################");
  Serial.println("GR01-D2b READY - D2 Part 3 (EMG + Gripper)");
  Serial.println("############################################");
  Serial.println("Recommended order:");
  Serial.println("  c -> calibrate EMG");
  Serial.println("  p -> plot EMG in Serial Plotter");
  Serial.println("  8 -> run full D2 sequence with EMG control");
  Serial.println("  m -> monitor EMG");
  Serial.println();

  MenuOptions();
}

// LOOP

void loop()
{
  UpdateEMG();

  if (Serial.available() <= 0) return;

  int inByte = Serial.read();

  switch (inByte)
  {
    case '0':
      SERVOS_ServosOff();
      Serial.println("Servos OFF (relax).");
      break;

    case '1':
      SERVOS_ServosOn();
      Serial.println("Servos ON (hold).");
      break;

    case '3':
      ROBOT_GripperClose();
      Serial.println("Gripper CLOSE.");
      break;

    case '4':
      ROBOT_GripperOpen();
      Serial.println("Gripper OPEN.");
      break;

    case 'c':
    case 'C':
      CalibrateEMGThreshold();
      break;

    case 'm':
    case 'M':
      Serial.println("EMG monitor for 10 s: raw, envelope, thrHigh");
      {
        unsigned long t0 = millis();
        while (millis() - t0 < 10000UL)
        {
          UpdateEMG();
          if (millis() - lastPrintMs >= 10)
          {
            lastPrintMs = millis();
            Serial.print(emgRaw);
            Serial.print(",");
            Serial.print(emgEnvelope, 2);
            Serial.print(",");
            Serial.println(emgThresholdHigh, 2);
          }
        }
      }
      Serial.println("End monitor.");
      break;

    case 'p':
    case 'P':
      PlotEMG();
      break;

    case '8':
      Run_D2_EMG_Sequence();
      break;

    default:
      Serial.println("Unknown option.");
      break;
  }

  MenuOptions();
}

// MENU

void MenuOptions()
{
  Serial.println("\n###########################");
  Serial.println("0) Relax Servos");
  Serial.println("1) Hold Servos");
  Serial.println("3) Gripper Close");
  Serial.println("4) Gripper Open");
  Serial.println("c) Calibrate EMG threshold");
  Serial.println("m) Monitor EMG for 10 s");
  Serial.println("p) Plot EMG signal (Serial Plotter) for 10s");
  Serial.println("8) Run D2 sequence with EMG gripper control");
  Serial.println("###########################\n");
}

// EMG UPDATE

void UpdateEMG()
{
  unsigned long nowUs = micros();
  if (nowUs - lastEmgSampleUs < EMG_TS_US) return;
  lastEmgSampleUs = nowUs;

  int sample = analogRead(EMG_PIN);
  emgRaw = sample;

  // Baseline lento -> elimina componente DC y deriva
  emgBaseline = BASELINE_ALPHA * emgBaseline + (1.0f - BASELINE_ALPHA) * (float)sample;

  // High-pass aproximado
  emgHighPass = (float)sample - emgBaseline;

  // Rectificación
  emgRectified = fabs(emgHighPass);

  // Envolvente
  emgEnvelope = ENV_ALPHA * emgEnvelope + (1.0f - ENV_ALPHA) * emgRectified;
}

// EMG DETECTOR

bool EMGContractionDetected()
{
  unsigned long nowMs = millis();

  if (!emgStateHigh && emgEnvelope >= emgThresholdHigh)
  {
    if (nowMs - lastTriggerMs > REFRACTORY_MS)
    {
      emgStateHigh = true;
      lastTriggerMs = nowMs;
      return true;
    }
  }

  if (emgStateHigh && emgEnvelope <= emgThresholdLow)
  {
    emgStateHigh = false;
  }

  return false;
}

bool IsEMGCurrentlyContracted()
{
  // Estado estable con histéresis:
  // - si supera thrHigh, consideramos "contraído"
  // - si baja de thrLow, consideramos "relajado"
  // - entre ambos, mantenemos el último estado

  if (emgEnvelope >= emgThresholdHigh)
  {
    emgStateHigh = true;
  }
  else if (emgEnvelope <= emgThresholdLow)
  {
    emgStateHigh = false;
  }

  return emgStateHigh;
}

// CALIBRATION

void CalibrateEMGThreshold()
{
  Serial.println("\n=== EMG CALIBRATION START ===");
  Serial.println("Phase 1: RELAX for 5 seconds...");

  float restMean = 0.0f;
  float restMax  = 0.0f;
  unsigned long count = 0;

  unsigned long t0 = millis();
  while (millis() - t0 < 5000UL)
  {
    UpdateEMG();
    restMean += emgEnvelope;
    if (emgEnvelope > restMax) restMax = emgEnvelope;
    count++;
  }
  restMean /= (float)count;

  Serial.println("Phase 2: make STRONG contractions for 5 seconds...");

  float actMean = 0.0f;
  float actMax  = 0.0f;
  count = 0;
  t0 = millis();
  while (millis() - t0 < 5000UL)
  {
    UpdateEMG();
    actMean += emgEnvelope;
    if (emgEnvelope > actMax) actMax = emgEnvelope;
    count++;
  }
  actMean /= (float)count;

  emgThresholdHigh = restMax + 0.20f * (actMax - restMax);
  emgThresholdLow  = restMax + 0.10f * (actMax - restMax);

  if (emgThresholdLow >= emgThresholdHigh)
  {
    emgThresholdLow = 0.7f * emgThresholdHigh;
  }

  Serial.print("restMean = "); Serial.println(restMean, 2);
  Serial.print("restMax  = "); Serial.println(restMax, 2);
  Serial.print("actMean  = "); Serial.println(actMean, 2);
  Serial.print("actMax   = "); Serial.println(actMax, 2);
  Serial.print("thrHigh  = "); Serial.println(emgThresholdHigh, 2);
  Serial.print("thrLow   = "); Serial.println(emgThresholdLow, 2);
  Serial.println("=== EMG CALIBRATION END ===\n");
}

// WAIT TRIGGER

void WaitForEMGTrigger(const char* message)
{
  Serial.println(message);
  Serial.println("Perform one clear muscle contraction...");

  while (true)
  {
    UpdateEMG();

    if (millis() - lastPrintMs >= 20)
    {
      lastPrintMs = millis();
      Serial.print(emgRaw);
      Serial.print(",");
      Serial.print(emgEnvelope, 2);
      Serial.print(",");
      Serial.println(emgThresholdHigh, 2);
    }

    if (EMGContractionDetected())
    {
      Serial.println("EMG trigger detected.");
      delay(200);
      return;
    }
  }
}

void WaitForEMGRelease(const char* message)
{
  Serial.println(message);
  Serial.println("Relax the forearm to open the gripper...");

  unsigned long belowSinceMs = 0;

  while (true)
  {
    UpdateEMG();

    if (millis() - lastPrintMs >= RELEASE_PLOT_DT_MS)
    {
      lastPrintMs = millis();
      Serial.print(emgRaw);
      Serial.print(",");
      Serial.print(emgEnvelope, 2);
      Serial.print(",");
      Serial.println(emgThresholdLow, 2);
    }

    // Solo consideramos "release" si la señal permanece
    // por debajo del umbral bajo durante un tiempo continuo.
    if (emgEnvelope <= emgThresholdLow)
    {
      if (belowSinceMs == 0)
      {
        belowSinceMs = millis();
      }

      if (millis() - belowSinceMs >= RELEASE_CONFIRM_MS)
      {
        emgStateHigh = false;
        Serial.println("EMG release detected.");
        delay(200);
        return;
      }
    }
    else
    {
      // Si vuelve a subir, cancelamos la posible relajación
      belowSinceMs = 0;
      emgStateHigh = true;
    }
  }
}

// EMG PLOT MODE

void PlotEMG()
{
  unsigned long t0 = millis();

  // En este modo mandamos solo datos para el Serial Plotter
  while (millis() - t0 < PLOT_DURATION_MS)
  {
    UpdateEMG();

    bool contracted = IsEMGCurrentlyContracted();
    int trigger = (emgEnvelope >= emgThresholdHigh) ? 1 : 0;

    Serial.print("raw:");
    Serial.print(emgRaw);
    Serial.print(",");

    Serial.print("envelope:");
    Serial.print(emgEnvelope, 2);
    Serial.print(",");

    Serial.print("thrHigh:");
    Serial.print(emgThresholdHigh, 2);
    Serial.print(",");

    Serial.print("thrLow:");
    Serial.print(emgThresholdLow, 2);
    Serial.print(",");

    Serial.print("contracted:");
    Serial.print(contracted ? 1 : 0);
    Serial.print(",");

    Serial.print("trigger:");
    Serial.println(trigger);

    // Permitir salida manual si entra un carácter por serie
    if (Serial.available() > 0)
    {
      char c = Serial.read();
      if (c == 'q' || c == 'Q')
      {
        break;
      }
    }

    delay(10); // ~100 Hz
  }

  Serial.println("End plot mode.");
}

// FULL SEQUENCE

void Run_D2_EMG_Sequence()
{
  Serial.println("\n=== D2 + EMG START ===");

  SERVOS_ServosOn();
  delay(200);

  Serial.println("Going to q0...");
  ROBOT_SetSingleTrajectory(q0, T_HOME_MS, CUBIC1);
  delay(T_HOME_MS + 300);

  Serial.println("Opening gripper...");
  ROBOT_GripperOpen();
  delay(600);

  Serial.println("Trajectory 1a: q0 -> q(tg)...");
  ROBOT_SetSingleTrajectory(qtg, T_1A_MS, CUBIC1);
  delay(T_1A_MS + 300);

  // Esperar a que empiece la contracción en q(tg)
  WaitForEMGTrigger("At grasp pose q(tg). Contract forearm to grasp.");
  Serial.println("Closing gripper (EMG control)...");
  ROBOT_GripperClose();
  emgStateHigh = true;
  delay(800);

  // Hacer la trayectoria completa SIN abrir durante el movimiento
  Serial.println("Trajectory 1b: q(tg) -> q(tv) -> q(tr)...");
  ROBOT_SetDoubleTrajectory(qtv, qtr, T_1B_1_MS, T_1B_2_MS, CUBIC2);

  unsigned long trajStart = millis();
  unsigned long trajDuration = (unsigned long)T_1B_1_MS + (unsigned long)T_1B_2_MS + 400UL;

  while (millis() - trajStart < trajDuration)
  {
    UpdateEMG();

    // Mientras se mueve, no abrimos la pinza.
    // Solo monitorizamos.
    if (millis() - lastPrintMs >= 20)
    {
      lastPrintMs = millis();
      Serial.print(emgRaw);
      Serial.print(",");
      Serial.print(emgEnvelope, 2);
      Serial.print(",");
      Serial.println(emgThresholdHigh, 2);
    }
  }

  // Ya en q(tr), mantener cerrada mientras siga contraído
  WaitForEMGRelease("At release pose q(tr). Keep holding if contracted.");
  Serial.println("Opening gripper (EMG relaxation control)...");
  ROBOT_GripperOpen();
  delay(800);

  Serial.println("Returning to q0...");
  ROBOT_SetSingleTrajectory(q0, T_HOME_MS, CUBIC1);
  delay(T_HOME_MS + 300);

  Serial.println("=== D2 + EMG END ===\n");
}