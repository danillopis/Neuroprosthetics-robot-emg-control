#include <ax12.h>

#include "poses.h"
#include "robot.h"
#include <avr/interrupt.h>
#include <avr/io.h>

static double q0[DOFs] = {0.0, 2.89, -2.89, 0.0, 0.0};

// Resultados del D1 (tg y tr)
static double qtg[DOFs] = {0.3530, 1.7217, -2.6405, 0.9188, 1.5708};
static double qtr[DOFs] = {1.5458, 1.1833, -0.9807, -1.7734, 3.1166};

// Punto intermedio para evitar obstáculo
static double qtv[DOFs] = {0.7484, 2.0075, -1.3783, -0.6292, 1.5708};

// Tiempos (ms). 
static const uint16_t T_HOME_MS = 1500;
static const uint16_t T_1A_MS   = 2000;
static const uint16_t T_1B_1_MS = 1500;
static const uint16_t T_1B_2_MS = 1500;

void MenuOptions();
void Run_D2_FullSequence();

void setup()
{
  ROBOT_Init();

  Serial.begin(115200);
  delay(200);

  Serial.println("###########################");
  Serial.println("Robot READY - GRXX D2");
  Serial.println("###########################");
  delay(500);

  MenuOptions();
}

void loop()
{
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

    case '7':
      Run_D2_FullSequence();
      break;

    default:
      Serial.println("Unknown option.");
      break;
  }

  MenuOptions();
}

void MenuOptions()
{
  Serial.println("\n###########################");
  Serial.println("0) Relax Servos");
  Serial.println("1) Hold Servos");
  Serial.println("3) Gripper Close");
  Serial.println("4) Gripper Open");
  Serial.println("7) Run D2 full sequence (1a+grasp, 1b+release)");
  Serial.println("###########################\n");
}

void Run_D2_FullSequence()
{
  Serial.println("\n=== D2 START ===");

  // IMPORTANTE: hold
  SERVOS_ServosOn();
  delay(200);

  // Ir a una pose conocida (q0) y abrir pinza
  Serial.println("Going to q0 (home-like)...");
  ROBOT_SetSingleTrajectory(q0, T_HOME_MS, CUBIC1);
  delay(T_HOME_MS + 300);

  Serial.println("Opening gripper...");
  ROBOT_GripperOpen();
  delay(600);

  // 1a + 2a) Ir a tg y agarrar
  Serial.println("Trajectory 1a: q0 -> q(tg) (CUBIC1)...");
  ROBOT_SetSingleTrajectory(qtg, T_1A_MS, CUBIC1);
  delay(T_1A_MS + 300);

  Serial.println("Closing gripper (grasp)...");
  ROBOT_GripperClose();
  delay(800);

  // 1b + 2b) tg -> tv -> tr y soltar
  Serial.println("Trajectory 1b: q(tg) -> q(tv) -> q(tr) (CUBIC2)...");
  ROBOT_SetDoubleTrajectory(qtv, qtr, T_1B_1_MS, T_1B_2_MS, CUBIC2);
  delay(T_1B_1_MS + T_1B_2_MS + 400);

  Serial.println("Opening gripper (release)...");
  ROBOT_GripperOpen();
  delay(800);

  // Volver a q0
  Serial.println("Returning to q0...");
  ROBOT_SetSingleTrajectory(q0, T_HOME_MS, CUBIC1);
  delay(T_HOME_MS + 300);

  Serial.println("=== D2 END ===\n");
}