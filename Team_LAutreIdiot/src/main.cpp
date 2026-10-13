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
#define LARGEUR_ROBOT (103 + 72 + 6.5) // (epaisseur entre roues - cell) + cell + epaisseur de roue
#define PERIODE 25
#define KP 0.1
#define KI 0.2

#define V_MAX_MOTEUR 1
#define V_MIN_MOTEUR 0.15
#define V_PPS_MAX 10272

#define COMPENSATION_INERTIE 120

#define NB_CASES  30
#define NORD      3
#define EST       1
#define OUEST     -1
#define SUD       -3

#define SEUIL_SIFFLET 550
#define MAX_SIFFLET   714

#define PIN_5KHZ A0

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
bool demarre;
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

bool ouvert(int pos, int dir)
{
  if (dir == EST && pos % 3 == 2) return false;
  if (dir == OUEST && pos % 3 == 0) return false;
  int cible = pos + dir;
  return cible >= 0 && cible < NB_CASES && adjacence[pos][cible];
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
    v_moteur_gauche = 0.2 + compteur * 0.03;
    v_moteur_droit = 0.2 + compteur * 0.03;
  }

  deplacement_total_gauche += pulse_moteur_gauche;
  deplacement_total_droit += pulse_moteur_droit;

  ++compteur;

  MOTOR_SetSpeed(LEFT, sens_rotation * v_moteur_gauche);
  MOTOR_SetSpeed(RIGHT, -1 * sens_rotation * v_moteur_droit);

  Serial.print(compteur); Serial.print("\t");
  Serial.print(pulse_moteur_gauche); Serial.print("\t");
  Serial.print(pulse_moteur_droit); Serial.print("\t");
  Serial.print(deplacement_total_gauche); Serial.print("\t");
  Serial.print(deplacement_total_droit); Serial.print("\t");
  Serial.print(v_moteur_gauche); Serial.print("\t");
  Serial.println(v_moteur_droit);
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
  deplacement_a_faire = LARGEUR_ROBOT * PI * angle / 360 * PULSES_PAR_MM - COMPENSATION_INERTIE;
  deplacement_total_droit = 0;
  deplacement_total_gauche = 0;
  integrale_gauche = 0;
  integrale_droite = 0;
  compteur = 0;

  Serial.print("--- Rotation "); Serial.print(angle);
  Serial.print(" deg | cible = "); Serial.println(deplacement_a_faire);

  SOFT_TIMER_SetCallback(0, blablaRond);
  SOFT_TIMER_SetDelay(0, PERIODE);
  SOFT_TIMER_Enable(0);
  while (deplacement_total_gauche < deplacement_a_faire || deplacement_total_droit < deplacement_a_faire)
  {
    SOFT_TIMER_Update();
  }
  SOFT_TIMER_Disable(0);

  int32_t g = deplacement_total_gauche;
  int32_t d = deplacement_total_droit;
  arret();

  delay(300); // laisse le robot s'immobiliser
  int32_t glisse_g = abs(ENCODER_Read(LEFT));
  int32_t glisse_d = abs(ENCODER_Read(RIGHT));

  Serial.print("Fin   G="); Serial.print(g); Serial.print(" D="); Serial.println(d);
  Serial.print("Inertie G="); Serial.print(glisse_g); Serial.print(" D="); Serial.println(glisse_d);
  Serial.print("Total G="); Serial.print(g + glisse_g); Serial.print(" D="); Serial.println(d + glisse_d);
  Serial.print("Erreur G="); Serial.print(g + glisse_g - deplacement_a_faire);
  Serial.print(" D="); Serial.println(d + glisse_d - deplacement_a_faire);
}

void avancer()
{
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  deplacement(500);
  ++increment_historique;
  historique[increment_historique] = historique[increment_historique - 1] + orientation;
  adjacence[historique[increment_historique]][historique[increment_historique - 1]] = 0;
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
  rotation(182);
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
  vert = !digitalRead(vertpin);
  rouge = !digitalRead(rougepin);
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
      case SUD:
        adjacence[pos_actuelle][pos_actuelle - 3] = 0;
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
  int pos_actuelle = historique[increment_historique];

  int somme_rangee_nb = somme_rangee(adjacence[pos_actuelle]);
  if (somme_rangee_nb == 0)
  {
    return 0;
  }
  else if (adjacence[pos_actuelle][pos_actuelle + NORD])
  {
    return NORD;
  }
  else if (adjacence[pos_actuelle][pos_actuelle + EST])
  {
    return EST;
  }
  else if (adjacence[pos_actuelle][pos_actuelle + OUEST])
  {
    return OUEST;
  }
  else if (adjacence[pos_actuelle][pos_actuelle + SUD])
  {
    return SUD;
  }
  return 0;
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

void retourner_pos_precedente()
{
  if (increment_historique == 0)
  {
    // Aucune case precedente : on ne peut pas reculer
    return;
  }

  int pos_actuelle = historique[increment_historique];
  int pos_precedente = historique[increment_historique - 1];

  adjacence[pos_precedente][pos_actuelle] = 0;

  tourner_prochaine_orientation(pos_precedente - pos_actuelle);

  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  deplacement(500);

  // Retirer la case actuelle de l'historique
  historique[increment_historique] = -1;
  --increment_historique;
}

void init_historique()
{
  for (int i = 0; i < 30; ++i)
  {
    historique[i] = -1;
  }
}

// Fonction de detection de sifflet 5kHz : retourne vrai ou faux
bool sifflet5kHz() {
  // Son qui doit etre detecte au moins 18 fois sur 20
  int son = 0;
  // 20 iterations
  for (int i = 0; i < 20; i++)
  {
    // Lecture du sifflet PIN A0
    int sifflet = analogRead(PIN_5KHZ);
    // s'assurer qu'il s'agit du sifflet
    if (sifflet >= SEUIL_SIFFLET && sifflet <= MAX_SIFFLET)
    {
      son ++; 
    }
    // Si on a 18 sons, il y a un sifflet
    if (son >= 18)
    {
      return true;
    }
    delay(50);
  }
  // Pas de sifflet
  return false;
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
  increment_historique = 0;
  historique[increment_historique] = 1;
  orientation = NORD;
  demarre = false;
  delay(100);
}

/*
Fonctions de boucle infini
 -> Se fait appeler perpetuellement suite au "setup"
*/
void loop() {
  /*
  bumperArr = ROBUS_IsBumper(3);

  if (bumperArr || demarre) {
    delay(500);
    avancer();
    
    delay(5000);
    demi_tour();
    
    //delay(500);
    avancer();

    //delay(500);
    demi_tour();
  }

  */
  //bumperArr = ROBUS_IsBumper(3);

  if (sifflet5kHz() || demarre) {
    delay(500);
    demarre = true;
    Serial.println("Debut");
    
    int pos_actuelle = historique[increment_historique];
    Serial.println(pos_actuelle);
    if (pos_actuelle >= 27)
    {
      // nous sommes a la fin. On fais le reculons
      marche_arriere();
    }
    else
    {
      // on continue d'explorer le labyrinthe
      maj_adjacence_capteur_ir();
      if (ouvert(pos_actuelle, orientation) == 1)
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