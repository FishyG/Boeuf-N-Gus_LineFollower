/*
Projet: main.cpp robo swag
Equipe: P23 - Bozo Squad
Auteurs: Jessy, RAJOUTER VOS NOMS ICI
Description: Code epreuve du parcour
Date: Aujourd'hui
*/

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */

#include <LibRobus.h> // Essentielle pour utiliser RobUS

#define PULSES_LEFT 1730
#define PULSES_RIGHT 1696

// Les IDs des timers je les mets ici
enum TimerIDs {
  TIMER_ID_PID,
  TIMER_ID_DEBUG,
};

typedef struct {
  float kp;
  float ki;
  float integral;
  float speed;
  int32_t total_pulses;
  int32_t target_pulses;
} MotorParams;

// Si vous avez a modifier les valeur de PID, elles vont ici
MotorParams motor_left = {
  .kp = 0.0001,
  .ki = 0.00004,

  .integral = 0,
  .speed = 0,
  .total_pulses = 0,
  .target_pulses = 0
};

MotorParams motor_right = {
  .kp = 0.0001,
  .ki = 0.00004,

  .integral = 0,
  .speed = 0,
  .total_pulses = 0,
  .target_pulses = 0
};

// PI(pas de D) pour les moteurs
void updatePI() {
  int32_t left_pulses = ENCODER_ReadReset(LEFT);
  int32_t right_pulses = ENCODER_ReadReset(RIGHT);

  float left_error = motor_left.target_pulses - left_pulses;
  float right_error = motor_right.target_pulses - right_pulses;

  motor_left.total_pulses += left_pulses;
  motor_right.total_pulses += right_pulses;

  motor_left.integral += left_error;
  motor_right.integral += right_error;

  motor_left.speed += motor_left.kp * left_error + motor_left.ki * motor_left.integral;
  motor_right.speed += motor_right.kp * right_error + motor_right.ki * motor_right.integral;

  if (motor_left.speed > 1.0)
    motor_left.speed = 1.0;
  if (motor_left.speed < -1.0)
    motor_left.speed = -1.0;

  if (motor_right.speed > 1.0)
    motor_right.speed = 1.0;
  if (motor_right.speed < -1.0)
    motor_right.speed = -1.0;

  MOTOR_SetSpeed(LEFT, motor_left.speed);
  MOTOR_SetSpeed(RIGHT, motor_right.speed);
}

void printDebug() {
  Serial.print("L total: ");
  Serial.print(motor_left.total_pulses);

  Serial.print(" | L integral: ");
  Serial.print(motor_left.integral);

  Serial.print(" | L speed: ");
  Serial.print(motor_left.speed);

  Serial.print(" | R total: ");
  Serial.print(motor_right.total_pulses);

  Serial.print(" | R integral: ");
  Serial.print(motor_right.integral);

  Serial.print(" | R speed: ");
  Serial.println(motor_right.speed);
}

void turnLeft90() {
  int32_t target_turn_pulses = PULSES_LEFT;

  motor_left.total_pulses = 0;
  motor_right.total_pulses = 0;

  motor_left.integral = 0;
  motor_right.integral = 0;

  while (
    -motor_left.total_pulses < target_turn_pulses ||
     motor_right.total_pulses < target_turn_pulses) {
    int32_t avg_pulses = (-motor_left.total_pulses + motor_right.total_pulses) / 2;
    int32_t remaining = target_turn_pulses - avg_pulses;

    if (remaining > 500) {
      motor_left.target_pulses = -200;
      motor_right.target_pulses = 200;
    }
    else if (remaining > 200) {
      motor_left.target_pulses = -120;
      motor_right.target_pulses = 120;
    }
    else {
      motor_left.target_pulses = -60;
      motor_right.target_pulses = 60;
    }

    SOFT_TIMER_Update();
  }

  motor_left.target_pulses = 0;
  motor_right.target_pulses = 0;
}

void moveDistance(float distance_m) {
  int32_t distance_target = distance_m * 13367;

  motor_left.total_pulses = 0;
  motor_right.total_pulses = 0;

  motor_left.integral = 0;
  motor_right.integral = 0;

  while (motor_left.total_pulses < distance_target || motor_right.total_pulses < distance_target) {
    int32_t remaining =
      distance_target -
      (motor_left.total_pulses + motor_right.total_pulses) / 2;

    if (remaining > 3000) {
      motor_left.target_pulses = 400;
      motor_right.target_pulses = 400;
    }
    else {
      motor_left.target_pulses = 150;
      motor_right.target_pulses = 150;
    }

    SOFT_TIMER_Update();
  }

  motor_left.target_pulses = 0;
  motor_right.target_pulses = 0;

  motor_left.speed = 0;
  motor_right.speed = 0;
}

/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
// -> Se fait appeler au debut du programme
// -> Se fait appeler seulement un fois
// -> Generalement on y initilise les varibbles globales
void setup() {
  BoardInit();

  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);

  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);

  SOFT_TIMER_SetCallback(TIMER_ID_PID, updatePI);
  SOFT_TIMER_SetDelay(TIMER_ID_PID, 100);
  SOFT_TIMER_SetRepetition(TIMER_ID_PID, -1);
  SOFT_TIMER_Enable(TIMER_ID_PID);

  SOFT_TIMER_SetCallback(TIMER_ID_DEBUG, printDebug);
  SOFT_TIMER_SetDelay(TIMER_ID_DEBUG, 500);
  SOFT_TIMER_SetRepetition(TIMER_ID_DEBUG, -1);
  SOFT_TIMER_Enable(TIMER_ID_DEBUG);

}


/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop() {
  SOFT_TIMER_Update();
  // Code de bozo, si un des bumpers est pressed, essaie de tourner de 90deg
  if (ROBUS_IsBumper(FRONT) || ROBUS_IsBumper(REAR) || ROBUS_IsBumper(LEFT) || ROBUS_IsBumper(RIGHT)) {
    turnLeft90();
  }
}
