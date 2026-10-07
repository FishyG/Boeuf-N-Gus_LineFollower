/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

/*
Inclure les librairies de functions que vous voulez utiliser
*/
#include <LibRobus.h>

/*
Variables globales et defines
 -> defines...
 -> L'ensemble des fonctions y ont acces
*/

#define PPT 3200.0
#define PULSES_PAR_MM (3200.0 / (PI * 76.2))
#define LARGEUR_ROBOT 184.15
#define PERIODE 50
#define KP 0.1
#define KI 0.2

#define V_MAX_MOTEUR 1
#define V_MIN_MOTEUR 0.15
#define V_PPS_MAX 10272

bool bumperArr, bumperSide;
int vertpin = 48;
int rougepin = 49;
bool vert = false;
bool rouge = false;
int etat = 0; // = 0 arrêt 1 = avance 2 = recule 3 = TourneDroit 4 = TourneGauche
int etatPast = 0;
float vitesse_base = 0.35;

int32_t deplacement_total_droit = 0;
int32_t deplacement_total_gauche = 0;
int32_t deplacement_total_moyen = 0;
int32_t deplacement_a_faire = 0;
int32_t integrale_gauche = 0;
int32_t integrale_droite = 0;
int32_t v_moteur_gauche_pps = 0;
int32_t v_moteur_droit_pps = 0;
int32_t compteur = 0;
int sens_rotation = 0;


double calculer_vitesse_moteur (int32_t vitesse_pps, int32_t v_pps_max)
{
  double vitesse_moteur = (double) vitesse_pps / (double) v_pps_max;
  return vitesse_moteur;
}

int32_t PID(int32_t pps_cible, int32_t nb_pulses, int32_t &integrale)
{
  int32_t erreur = pps_cible - nb_pulses * 1000L / PERIODE;
  integrale += erreur;
  return (pps_cible + KP * erreur + KI * integrale);
}

void blabla()
{
  int32_t pulse_moteur_gauche = ENCODER_ReadReset(LEFT);
  int32_t pulse_moteur_droit = ENCODER_ReadReset(RIGHT);

  int32_t v_pps_cible = V_PPS_MAX * vitesse_base;

  double v_moteur_gauche;
  double v_moteur_droit;

  if (compteur > 4)
  {
    v_moteur_gauche_pps = PID(v_pps_cible, pulse_moteur_gauche, integrale_gauche);
    v_moteur_droit_pps = PID(v_pps_cible, pulse_moteur_droit, integrale_droite);
    v_moteur_gauche = calculer_vitesse_moteur(v_moteur_gauche_pps, V_PPS_MAX);  
    v_moteur_droit = calculer_vitesse_moteur(v_moteur_droit_pps, V_PPS_MAX);  
  }
  else
  {
    Serial.println("Acceleration");
    v_moteur_gauche = 0.2 + compteur * 0.03;
    v_moteur_droit = 0.2 + compteur * 0.03;
  }

  deplacement_total_gauche += pulse_moteur_gauche;
  deplacement_total_droit += pulse_moteur_droit;

  ++compteur;

  MOTOR_SetSpeed(LEFT, v_moteur_gauche);
  MOTOR_SetSpeed(RIGHT, v_moteur_droit);
}

void blablaRond()
{
  Serial.println("BlablaRond");
  int32_t pulse_moteur_gauche = ENCODER_ReadReset(LEFT);
  int32_t pulse_moteur_droit = ENCODER_ReadReset(RIGHT);

  pulse_moteur_gauche = abs(pulse_moteur_gauche);
  pulse_moteur_droit = abs(pulse_moteur_droit);

  int32_t v_pps_cible = V_PPS_MAX * vitesse_base;

  double v_moteur_gauche;
  double v_moteur_droit;

  if (compteur > 4)
  {
    v_moteur_gauche_pps = PID(v_pps_cible, pulse_moteur_gauche, integrale_gauche);
    v_moteur_droit_pps = PID(v_pps_cible, pulse_moteur_droit, integrale_droite);
    v_moteur_gauche = calculer_vitesse_moteur(v_moteur_gauche_pps, V_PPS_MAX);  
    v_moteur_droit = calculer_vitesse_moteur(v_moteur_droit_pps, V_PPS_MAX);  
  }
  else
  {
    Serial.println("Tourner acceleration");
    v_moteur_gauche = 0.2 + compteur * 0.03;
    v_moteur_droit = 0.2 + compteur * 0.03;
  }

  deplacement_total_gauche += pulse_moteur_gauche;
  deplacement_total_droit += pulse_moteur_droit;

  ++compteur;

  Serial.print(deplacement_total_gauche);
  Serial.print("\t");
  Serial.print(deplacement_total_droit);
  Serial.print("\t");

  MOTOR_SetSpeed(LEFT, sens_rotation * v_moteur_gauche);
  MOTOR_SetSpeed(RIGHT, -1 * sens_rotation * v_moteur_droit);
}

void arret(){
  v_moteur_gauche_pps = 0;
  v_moteur_droit_pps = 0;
  deplacement_total_droit = 0;
  deplacement_total_gauche = 0;
  deplacement_total_moyen = 0;
  integrale_droite = 0;
  integrale_gauche = 0;
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);
}

void deplacement(int distance)
{
  deplacement_a_faire = PULSES_PAR_MM * distance;
  deplacement_total_droit = 0;
  deplacement_total_gauche = 0;
  integrale_gauche = 0;
  integrale_droite = 0;
  compteur = 0;

  SOFT_TIMER_SetCallback(0, blabla);
  SOFT_TIMER_SetDelay(0, PERIODE);

  SOFT_TIMER_Enable(0);
  while (deplacement_total_gauche < deplacement_a_faire || deplacement_total_droit < deplacement_a_faire)
  {
    SOFT_TIMER_Update();
  };

  SOFT_TIMER_Disable(0);
  arret();
}

void rotation(int angle)
{
  deplacement_a_faire = LARGEUR_ROBOT * PI * angle / 360 * PULSES_PAR_MM;
  deplacement_total_droit = 0;
  deplacement_total_gauche = 0;
  integrale_gauche = 0;
  integrale_droite = 0;
  compteur = 0;

  Serial.println("Distance en pulse");
  Serial.println(deplacement_a_faire);

  SOFT_TIMER_SetCallback(0, blablaRond);
  SOFT_TIMER_SetDelay(0, PERIODE);

  SOFT_TIMER_Enable(0);
  while (deplacement_total_gauche < deplacement_a_faire || deplacement_total_droit < deplacement_a_faire)
  {
    SOFT_TIMER_Update();
  };

  SOFT_TIMER_Disable(0);
  arret();
}

void avancer()
{
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  deplacement(500);
}

void reculer(){
}

void tourneDroit()
{
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  sens_rotation = 1;
  rotation(90);
  Serial.println("ici");
}

void tourneGauche(){
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  sens_rotation = -1;
  rotation(90);
  Serial.println("ici 2");
}


void setup(){
  BoardInit();
  
  //initialisation
  pinMode(vertpin, INPUT);
  pinMode(rougepin, INPUT);
  delay(100);
}

/*
Fonctions de boucle infini
 -> Se fait appeler perpetuellement suite au "setup"
*/
void loop() {
  bumperArr = ROBUS_IsBumper(3);
  if (bumperArr) {
    delay(500);

    avancer();
    delay(100);
    tourneDroit();
    delay(100);
    tourneGauche();

    bumperArr = false;
  }
}