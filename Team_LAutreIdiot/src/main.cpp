/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

/*
Numerotation des cases du parcours
+-----+-----+-----+
|  27    28    29 |
|                 |
|  24    25    26 |
|                 |
|  21    22    23 |
|                 |
|  18    19    20 |
|                 |
|  15    16    17 |
|                 |
|  12    13    14 |
|                 |
|   9    10    11 |
|                 |
|   6     7     8 |
|                 |
|   3     4     5 |
|                 |
|   0     1     2 |
+-----+-----+-----+
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

// 92.72 + 92.68 - 6.58 - 6.62 + 2

#define PPT 3200.0
#define PULSES_PAR_MM (3200.0 / (PI * 76.37))
#define LARGEUR_ROBOT 103 + 72 + 6.5 // (epaisseur entre roues - cell) + cell + epaisseur de roue
#define PERIODE 50
#define KP 0.1
#define KI 0.2

#define V_MAX_MOTEUR 1
#define V_MIN_MOTEUR 0.15
#define V_PPS_MAX 10272

#define NB_CASES  30
#define NORD      3
#define EST       1
#define OUEST     -1
#define SUD       -3

// outils de navigation
int adjacence[NB_CASES][NB_CASES] = {
  { 0, 1, 0, 1, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 0
  { 1, 0, 1, 0, 1, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 1
  { 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 2
  { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 3
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 4
  { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 5
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 1,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 6
  { 0, 0, 0, 0, 0, 0, 1, 0, 1, 0,  1, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 7
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,  0, 1, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 8
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 1, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 9
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 1, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 10
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 1, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 11
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 1, 0, 1, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 12
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 1, 0, 1, 0, 1, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 13
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 1, 0, 0, 0, 1, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 14
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 1, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 15
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 1,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 16
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  1, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 17
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 1,  0, 1, 0, 0, 0, 0, 0, 0, 0, 0 }, // 18
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 1, 0,  1, 0, 1, 0, 0, 0, 0, 0, 0, 0 }, // 19
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 1,  0, 0, 0, 1, 0, 0, 0, 0, 0, 0 }, // 20
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 1, 0, 0, 0, 0, 0 }, // 21
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 1, 0, 0, 0, 0 }, // 22
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 1, 0, 0, 0 }, // 23
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 1, 0, 1, 0, 0 }, // 24
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 1, 0, 1, 0, 1, 0 }, // 25
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 1, 0, 0, 0, 1 }, // 26
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 27
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 28
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }  // 29
};

int historique[30];
int increment_historique = 0;
int orientation = NORD;

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


int somme_rangee(int mat[])
{
  int somme = 0;
  for (int i = 0; i < NB_CASES; ++i)
  {
    somme += mat[i];
  }

  return somme;
}

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

void tourne_droit()
{
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  sens_rotation = 1;
  rotation(90);
}

void demi_tour()
{
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  sens_rotation = 1;
  rotation(90);
}

void tourne_gauche(){
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  sens_rotation = -1;
  rotation(90);
}

void detection_cul_de_sac()
{
  int pos_actuelle = historique[increment_historique];

  for (int i = 0; i < increment_historique; ++i)
  {
    if (pos_actuelle == historique[i])
    {
      // cul-de-sac detecte
      adjacence[historique[i]][historique[i + 1]] = 0;
      increment_historique = i;
      break; // pas obligatoire vu qu'on modifie increment_historique, mais pour la clarte du code
    }
  }
}

void marche_arriere()
{
  for(int i=0;i<4;i++){
    AX_BuzzerON();
    delay(100);
    AX_BuzzerOFF();
    delay(100);  
  }
  delay(400);

}

void lecture_capteur_ir()
{
  vert = digitalRead(vertpin);
  rouge = digitalRead(rougepin);
}

void maj_adjacence_capteur_ir()
{
  int pos_actuelle = historique[increment_historique];

  lecture_capteur_ir();

  // TODO mettre la matrice d'adjacence à jour des 2 bords
  if (vert || rouge)
  {
    switch (orientation)
    {
      case NORD:
        adjacence[pos_actuelle][pos_actuelle + 3] = 0;
        break;
      case OUEST:
        adjacence[pos_actuelle][pos_actuelle - 1] = 0;
        break;
      case EST:
        adjacence[pos_actuelle][pos_actuelle + 1] = 0;
        break;
      default:
        // 
        break;
    }
  }
}

int numero(int direction)
{
  switch (direction)
  {
    case NORD:  return 0;
    case EST:   return 1;
    case SUD:   return 2;
    case OUEST: return 3;
  }
  return 0;
}

int trouver_prochaine_orientation()
{
}

void tourner_prochaine_orientation(int prochaine_orientation)
{
  int diff = (numero(prochaine_orientation) - numero(orientation) + 4) % 4;

  if (diff == 1)
  {
    tourne_droit();
  }
  else if (diff == 3)
  {
    tourne_gauche();
  }
  else if (diff == 2)
  {
    demi_tour();
  }

  orientation = prochaine_orientation;
}

void init_historique()
{
  for (int i = 0; i < 30; ++i)
  {
    historique[i] = -1;
  }
}

void setup(){
  BoardInit();
  
  //initialisation
  pinMode(vertpin, INPUT);
  pinMode(rougepin, INPUT);
  init_historique();
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

    bumperArr = false;

    
    int pos_actuelle = historique[increment_historique];
    if (pos_actuelle >= 27)
    {
      // nous sommes a la fin. On fais le reculons
      marche_arriere();
    }
    else
    {
      // on continue d'explorer le labyrinthe
      maj_adjacence_capteur_ir();
      if (adjacence[pos_actuelle][pos_actuelle + orientation] == 1)
      {
        avancer();
      }
      else
      {
        int prochaine_orientation = trouver_prochaine_orientation();
        if (prochaine_orientation == 0)
        {
          retourner_pos_precedente();
        }
        else
        {
          tourner_prochaine_orientation(prochaine_orientation);
        }
      }
    }
  }
}