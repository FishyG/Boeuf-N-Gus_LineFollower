/*
Projet: Labyrinthe
Equipe: P23
Auteurs: Xavier Dupuis
Description: Breve description du script
Date: 2026-10-01
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

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */

#include <LibRobus.h> // Essentielle pour utiliser RobUS


/* ****************************************************************************
Variables globales et defines
**************************************************************************** */

// constantes
#define NB_CASES  30
#define NORD      0
#define EST       1
#define OUEST     2
#define SUD       3

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

// outils de detection
int vertpin = 48;
int rougepin = 49;
bool vert = false;
bool rouge = false;



/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */
void init_historique()
{
  for (int i = 0; i < 30; ++i)
  {
    historique[i] = -1;
  }
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

void lecture_capteur_ir()
{
  vert = digitalRead(vertpin);
  rouge = digitalRead(rougepin);
  delay(10);
}

void maj_adjacence_capteur_ir()
{
  int pos_actuelle = historique[increment_historique];

  lecture_capteur_ir();

  if (vert || rouge)
  {
    switch (orientation)
    {
      case NORD:
        matrice[pos_actuelle][pos_actuelle + 3] = 0;
        break;
      case OUEST:
        matrice[pos_actuelle][pos_actuelle - 1] = 0;
        break;
      case EST:
        matrice[pos_actuelle][pos_actuelle + 1] = 0;
        break;
      default:
        // 
        break;
    }
  }
}

// utilitaires (a sortir potentiellement)
int somme_rangee (int rangee[])
{
  int longueur = sizeof(rangee) / sizeof(rangee[0]);
  int somme = 0;

  for (int i = 0; i < longueur; ++i)
  {
    somme += rangee[i];
  }

  return somme;
}

// fonction de deplacement
void avancer() {}
void reculer() {}
void tourner_gauche() {}
void tourner_droite() {}

void essai_prochain_deplacement_latteral()
{
  int pos_actuelle = historique[increment_historique];

  if (somme_rangee(adjacence[pos_actuelle] == 2))
  {
    // on a l'embarras du choix d'aller a gauche ou a droite
    tourner_gauche();
    maj_adjacence_capteur_ir();

    // Si la voie est disponible a l'OUEST on y va
    // Si la voie n'est pas libre, on se reoriente vers le NORD et on assume qu'on devra aller vers l'EST, ce qu'on fera au prochain call de loop()
    if (adjacence[pos_actuelle][pos_actuelle - 1] == 1)
    {
      avancer();
    }
    tourner_droite();
  }
  else
  {
    // on doit trouver s'il faut tourner a droite ou a gauche
  }
}

/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
// -> Se fait appeler au debut du programme
// -> Se fait appeler seulement un fois
// -> Generalement on y initialise les varibbles globales

void setup(){
  BoardInit();
  init_historique();
  historique[increment_historique] = 1;

  //initialisation du capteur de couleur
  pinMode(vertpin, INPUT);
  pinMode(rougepin, INPUT);
  delay(100);

}


/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
void loop() {
  int pos_actuelle = historique[increment_historique];
  if (pos_actuelle >= 27)
  {
    // nous sommes a la fin. On fais le reculons
    marche_arriere();
  }
  else
  {
    // on continue d'explorer le labyrinthe
    detection_cul_de_sac();
    maj_adjacence_capteur_ir();
    if (adjacence[pos_actuelle][pos_actuelle + 3] == 1)
    {
      avancer();
    }
    else
    {
      if (somme_rangee(adjacence[pos_actuelle]) == 0)
      {
        // on est entre dans un cul de sac par le sud
        reculer();
      }
      else
      {
        essai_prochain_deplacement_latteral();
      }
    }
  }


  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  delay(10);// Delais pour decharger le CPU
}