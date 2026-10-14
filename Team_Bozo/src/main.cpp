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

#define PULSES_LEFT 1200
#define PULSES_RIGHT 1100

#define pin_rouge 49
#define pin_vert 48

#define distance 0.5

// put function declarations here:
char Va_Nord(char orientation);
char Va_Ouest(char orientation);
char Va_Sud(char orientation);
char Va_Est(char orientation);
void fonction_futur_pos();
int deja_alle();
void tableau_post_tape();
void tableau_post_mur();
void tableau_post_historique();
void ajouter_position_actuelle();
void detection_mur();
void enlever_derniere();
void updatePI();
void printDebug();
void moveDistance(float distance_m);
void turnLeft90();
void turnRight90();

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
  .kp = 0.0006,
  .ki = 0.00004,

  .integral = 0,
  .speed = 0,
  .total_pulses = 0,
  .target_pulses = 0
};

MotorParams motor_right = {
  .kp = 0.0006,
  .ki = 0.00004,

  .integral = 0,
  .speed = 0,
  .total_pulses = 0,
  .target_pulses = 0
};

char ori = 'N';
int pos[4] = {2,1,0,0};
int direction_possible[4];
int futur_pos[4];
int map_deja_alle[30][4];
int nb_deja_alle = 0;
int mur = 0;
int map_tape[][2][4] = {{{1,1,0,0},{1,1,0,0}}, //les positions ont 4 coordonées mais les 2 dernières sont toujours 0
				{{2,1,0,0},{1,1,1,0}},
				{{3,1,0,0},{1,0,1,0}},
				{{1,2,0,0},{1,0,0,1}},
				{{2,2,0,0},{1,0,0,1}},
				{{3,2,0,0},{1,0,0,1}},
				{{1,4,0,0},{1,0,0,1}},
				{{2,4,0,0},{1,0,0,1}},
				{{3,4,0,0},{1,0,0,1}},
				{{1,6,0,0},{1,0,0,1}},
			  {{2,6,0,0},{1,0,0,1}},
				{{3,6,0,0},{1,0,0,1}},
			  {{1,8,0,0},{1,0,0,1}},
   			{{2,8,0,0},{1,0,0,1}},
			  {{3,8,0,0},{1,0,0,1}},
		    {{1,3,0,0},{1,1,0,1}},
		    {{3,3,0,0},{1,0,1,1}},
			  {{1,5,0,0},{1,1,0,1}},
		    {{3,5,0,0},{1,0,1,1}},
			  {{1,7,0,0},{1,1,0,1}},
				{{3,7,0,0},{1,0,1,1}},
	      {{1,9,0,0},{1,1,0,1}}, 
		    {{3,9,0,0},{1,0,1,1}},
		    {{1,10,0,0},{0,0,0,1}},
		    {{2,10,0,0},{0,0,0,1}},
	      {{3,10,0,0},{0,0,0,1}}};

void setup() {
  BoardInit();

  pinMode(pin_rouge, INPUT);
  pinMode(pin_vert, INPUT);

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

  delay(10);

  Serial.println("Waiting...");
  while (!ROBUS_IsBumper(REAR)) {
    delay(80);
    SOFT_TIMER_Update();
  }
  Serial.println("Going!");
}

void loop() {
  if (pos[1] < 10) {
    Serial.print("Pos: ");
		Serial.println(pos[1]);
		int deplacement_fait = 0;
		direction_possible[0] = 1;
    direction_possible[1] = 1;
    direction_possible[2] = 1;
    direction_possible[3] = 1;
        
    ajouter_position_actuelle();
    tableau_post_tape();
        
    while (deplacement_fait == 0) {
			
			if (direction_possible[0] == 1) {
        Serial.println("Doing 1");
				ori = Va_Nord(ori);
				detection_mur();
        Serial.print("Mur: ");
        Serial.println(mur);
				tableau_post_mur();
				fonction_futur_pos();
				tableau_post_historique();
        for (int i = 0; i < 4; i++){
        Serial.print(direction_possible[i]);}
        Serial.println("");
			
				if (direction_possible[0] == 1){
          Serial.println("Doing move 1");
					moveDistance(distance);
          Serial.println("Done");
					pos[1]++;
					deplacement_fait++;
					break;
				}
			}
		  if (direction_possible[1] == 1) {
				Serial.println("Doing 2");
        ori = Va_Est(ori);
				detection_mur();
        Serial.print("Mur: ");
        Serial.println(mur);
				tableau_post_mur();
				fonction_futur_pos();
				tableau_post_historique();
        for (int i = 0; i < 4; i++){
        Serial.print(direction_possible[i]);}
        Serial.println("");  

				if (direction_possible[1] == 1){
          Serial.println("Doing move 2");
					moveDistance(distance);
          Serial.println("Done");
					pos[0]++;
					deplacement_fait++;
          break;
        }
			}
			if (direction_possible[2] == 1) {
				Serial.println("Doing 3");
        ori = Va_Ouest(ori);
				detection_mur();
        Serial.print("Mur: ");
        Serial.println(mur);
				tableau_post_mur();
				fonction_futur_pos();
				tableau_post_historique();
        for (int i = 0; i < 4; i++){
        Serial.print(direction_possible[i]);}
			  Serial.println("");
				if (direction_possible[2] == 1){
					Serial.println("Doing move 3");
          moveDistance(distance);
          Serial.println("Done");
					pos[0]--;
					deplacement_fait++;
          break;
				}
			}
			if (direction_possible[3] == 1) {
				Serial.println("Doing 4");
        ori = Va_Sud(ori);
				detection_mur();
        Serial.println(mur);
				tableau_post_mur();
				fonction_futur_pos();
				tableau_post_historique();
        for (int i = 0; i < 4; i++){
        Serial.print(direction_possible[i]);}
			  Serial.println("");
				if (direction_possible[3] == 1){
					Serial.println("Doing move 4");
          moveDistance(distance);
					pos[1]--;
					deplacement_fait++;
          break;
				}
			}
			//else {
      //  Serial.println("Enlever last");
			//	enlever_derniere();
			//}
      SOFT_TIMER_Update();
		}
	}
	else {
  }
}


// put function definitions here:
char Va_Nord(char orientation) {
	switch (orientation) {
		case 'N':
			
			break;
		case 'W':
			turnRight90();
			
			break;
		case 'S':
			turnRight90();
			turnRight90();

			break;
		case 'E':
			turnLeft90();
			break;
		}
	orientation = 'N';
	return orientation;
}

char Va_Ouest(char orientation) {
	switch (orientation) {
		case 'N':
			turnLeft90();
			break;
		case 'W':
			break;
		case 'S':
			turnRight90();
			break;
		case 'E':
			turnLeft90();
			turnLeft90();
			break;
		}
	orientation = 'W';
	return orientation;
}

char Va_Sud(char orientation) {
	switch (orientation) {
		case 'N':
			turnLeft90();
			turnLeft90();
			break;
		case 'W':
			turnLeft90();
			break;
		case 'S':
			break;
		case 'E':
			turnRight90();
			break;
		}
	orientation = 'S';
	return orientation;
}

char Va_Est(char orientation) {
	switch (orientation) {
		case 'N':
			turnRight90();
			break;
		case 'W':
			turnLeft90();
			turnLeft90();
			break;
		case 'S':
			turnLeft90();
			break;
		case 'E':
			break;
		}
	orientation = 'E';
	return orientation;
}

void fonction_futur_pos() { //fonction qui nous donne la futur position envisagée

	futur_pos[0] = pos[0];
	futur_pos[1] = pos[1];
	futur_pos[2] = pos[2];
	futur_pos[3] = pos[3];
	if (ori == 'N')
	{
		futur_pos[1] = futur_pos[1] + 1;
	}
	if (ori == 'E')
	{
		futur_pos[0] = futur_pos[0] + 1;
	}
	if (ori == 'W')
	{
		futur_pos[0] = futur_pos[0] - 1;
	}
	if (ori == 'S')
	{
		futur_pos[1] = futur_pos[1] - 1;
	}
}

int deja_alle() { //fonction qui nous dit si la prochaine case où il veut aller a déjà été exploré

	int size = sizeof(map_deja_alle) / sizeof(map_deja_alle[0]);
	for (int i = 0; i < size; i++)
	{
		if (futur_pos[0] == map_deja_alle[i][0] && futur_pos[1] == map_deja_alle[i][1])
		{
			return 0;  //si deja_alle == 0 alors on enleve cette possibilité de direction_possible
		}
	}
	return 1;  //si deja_alle == 1 on fait rien
}

void tableau_post_tape() { //fonction qui modifie le tableau direction_possible si il est sur une case avec un tape

	int size = sizeof(map_tape) / sizeof(map_tape[0]);
	for (int i = 0; i < size; i++)
	{
		if (pos[0] == map_tape[i][0][0] && pos[1] == map_tape[i][0][1])
		{
			direction_possible[0] = map_tape[i][1][0];
			direction_possible[1] = map_tape[i][1][1];
			direction_possible[2] = map_tape[i][1][2];
			direction_possible[3] = map_tape[i][1][3];
			break;
		}
	}
}

void tableau_post_mur() { //fonction qui modifie le tableau direction_possible si il y a un mur EN FACE de lui

	if (mur == 1)  //j'ai dit si il y a un mur on aura la variable mur = 1
	{
		if (ori == 'N')
		{
			direction_possible[0] = 0;
		}	
		if (ori == 'E')
		{
			direction_possible[1] = 0;
		}
		if (ori == 'W')
		{
			direction_possible[2] = 0;
		}
		if (ori == 'S')
		{
			direction_possible[3] = 0;
		}
	}
}

void tableau_post_historique() { //fonction qui modifie le tableau direction_possible si il était déjà dans cette case

	if (deja_alle() == 0)
	{
		if (ori == 'N')
		{
			direction_possible[0] = 0;
		}	
		if (ori == 'E')
		{
			direction_possible[1] = 0;
		}
		if (ori == 'W')
		{
			direction_possible[2] = 0;
		}
		if (ori == 'S')
		{
			direction_possible[3] = 0;
		}
	}
}

void ajouter_position_actuelle() {
    
    if (nb_deja_alle < 30) 
    {
        map_deja_alle[nb_deja_alle][0] = pos[0];
        map_deja_alle[nb_deja_alle][1] = pos[1];
        map_deja_alle[nb_deja_alle][2] = 0;
        map_deja_alle[nb_deja_alle][3] = 0;
        nb_deja_alle++;
    }
}

void detection_mur(){
	int led_rouge = digitalRead(pin_rouge);
	int led_verte = digitalRead(pin_vert);
  
  for (int i = 0; i < 500; i++)
  {
    delay(2);
    SOFT_TIMER_Update();
  }

  Serial.print("Mur: ");
	if (led_rouge == LOW && led_verte == LOW){
		mur = 1;
    Serial.println("yes");
	} else {
    mur = 0;
    Serial.println("yes");
  }
  
}

void enlever_derniere() {
	for(int i = 0; i<4; i++) {
		map_deja_alle[nb_deja_alle][i] = 0;
	}
}

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
  Serial.println("Running turn left");
  int32_t target_turn_pulses = PULSES_LEFT;

  motor_left.total_pulses = 0;
  motor_right.total_pulses = 0;

  motor_left.integral = 0;
  motor_right.integral = 0;

  while (-motor_left.total_pulses < target_turn_pulses || motor_right.total_pulses < target_turn_pulses) {
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
  Serial.println("done left");
}

void turnRight90() {
  Serial.println("Running turn right");
  int32_t target_turn_pulses = PULSES_RIGHT;

  motor_left.total_pulses = 0;
  motor_right.total_pulses = 0;

  motor_left.integral = 0;
  motor_right.integral = 0;

  while ( motor_left.total_pulses < target_turn_pulses || -motor_right.total_pulses < target_turn_pulses) {
    int32_t avg_pulses = (motor_left.total_pulses - motor_right.total_pulses) / 2;
    int32_t remaining = target_turn_pulses - avg_pulses;

    if (remaining > 500) {
      motor_left.target_pulses = 200;
      motor_right.target_pulses = -200;
    }
    else if (remaining > 200) {
      motor_left.target_pulses = 120;
      motor_right.target_pulses = -120;
    }
    else {
      motor_left.target_pulses = 60;
      motor_right.target_pulses = -60;
    }

    SOFT_TIMER_Update();
  }
  
  motor_left.total_pulses = 0;
  motor_right.total_pulses = 0;
  Serial.println("done right");
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
  SOFT_TIMER_Update();
}