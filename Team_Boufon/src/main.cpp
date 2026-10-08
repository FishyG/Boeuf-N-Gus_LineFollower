/*
Projet: Defi du parcours (labyrinthe)
Equipe: Beuf-N-GUS, P-23, Team Boufon
Auteurs: Eva Desbiens, Thomas Blanchard, Karolann Mauger et Christopher Plantevin
Date: 2026-10-08
*/

/* ****************************************************************************
REGLES POUR TRAVAILLER A 4 DANS CE FICHIER
- Chacun travaille SEULEMENT dans sa section.
- On ne change pas le nom ou les parametres d'une fonction sans avertir le groupe.
- Toutes les constantes (#define) vont dans la SECTION 0.
- Pull (fetch+rebase) avant de commencer, commit et pull (fetch+rebase) avant de push.

  SECTION 0 : includes, defines, variables globales
  SECTION 1 : deplacements
  SECTION 2 : capteurs (IR, sifflet)
  SECTION 3 : carte des murs + BFS
  SECTION 4 : navigation
  SECTION 5 : setup() + loop()
**************************************************************************** */

#include <LibRobus.h> 


/* ============================================================================
   SECTION 0 : DEFINES ET VARIABLES GLOBALES                         
   ============================================================================ */

// ---- Broches ----
#define PIN_VERT      48    
#define PIN_ROUGE     49   
#define PIN_5KHZ      A0   
#define PIN_AMBIANT   A1    
#define SEUIL_SIFFLET 50   

// ---- Moteurs et PI  ----
#define VITESSE    0.40
#define KP         0.002
#define KI         0.001
#define MAX_SOMME  2000

// ---- Robot (Modification a faire) ----
#define PULSES_PAR_TOUR 3200   
#define DIAMETRE_ROUE   7.62   
#define ENTRAXE         18.7   
#define TAILLE_CASE     50.0  

// ---- Grille : 10 rangees x 3 colonnes ----
//   rangee 9  [Fin][Fin][Fin]
//     ...
//   rangee 0  [   ][Dep][   ]
//            col 0 col 1 col 2
#define NB_RANGEES   10
#define NB_COLONNES  3
#define RANGEE_FIN   9
#define DEPART_R     0
#define DEPART_C     1
#define INFINI       999

// ---- Cotes (orientation) ----
// Tourner a droite = +1, a gauche = -1 (modulo 4)
#define HAUT    0  
#define DROITE  1
#define BAS     2  
#define GAUCHE  3

// ---- Modes du BFS ----
#define NOUVEAU          0   
#define CONNU_SEULEMENT  1   

// ---- Etats du robot ----
#define ATTENTE  0
#define ALLER    1
#define RETOUR   2
#define TERMINE  3
#define ERREUR   4

#define MAX_PAS  60

// ---- Variables globales ----
int  robotR    = DEPART_R;   
int  robotC    = DEPART_C;   
int  direction = HAUT;       
int  etat      = ATTENTE;

bool mur[NB_RANGEES][NB_COLONNES][4];    
bool connu[NB_RANGEES][NB_COLONNES][4];  
int  dist[NB_RANGEES][NB_COLONNES];   


/* ============================================================================
   PROTOTYPES
   ============================================================================ */

// SECTION 1 : deplacements
void arret();
void avancerCase();
void tourner(int quarts);
void faireFace(int coteVoulu);

// SECTION 2 : capteurs
bool murDevant();
bool sifflet5kHz();
bool bumperArriere();
void beep(int nombre);

// SECTION 3 : carte + BFS
void initCarte();
void ajouterMur(int r, int c, int cote);
void marquerLibre(int r, int c, int cote);
bool dansGrille(int r, int c);
int  voisinR(int r, int cote);
int  voisinC(int c, int cote);
int  coteOppose(int cote);
void calculerDistances(bool versFin, int mode);
int  meilleurCote(int r, int c, int mode);
void afficherCarte();

// SECTION 4 : navigation
void scannerCellule();
bool naviguerVers(bool versFin, int mode);


/* ============================================================================
   SECTION 1 : DEPLACEMENTS
   ============================================================================ */

void arret() {
  // TODO: MOTOR_SetSpeed(LEFT, 0) et MOTOR_SetSpeed(RIGHT, 0)
}

// Avance exactement d'une case (TAILLE_CASE) en ligne droite.
// - ENCODER_Reset(LEFT) / ENCODER_Reset(RIGHT)
// - cible = TAILLE_CASE / (PI * DIAMETRE_ROUE) * PULSES_PAR_TOUR
// - Tant que moyenne(encG, encD) < cible :
//     erreur = encG - encD ; somme += erreur (limiter a +/- MAX_SOMME)
//     correction = KP * erreur + KI * somme
//     gauche = VITESSE - correction ; droite = VITESSE + correction (limiter -1..1)
//     delay(10)
// - arret(), delay(200)
void avancerCase() {
  // TODO
}

// Tourne sur place. quarts : +1 = 90 deg droite, -1 = 90 deg gauche, 2 = demi-tour.
// - cible = (PI * ENTRAXE / 4) * |quarts| convertie en pulses
// - roue gauche +, roue droite - pour tourner a droite (inverse pour la gauche)
// - arret(), delay(200)
// - direction = ((direction + quarts) % 4 + 4) % 4   <- modulo POSITIF
void tourner(int quarts) {
  // TODO
}

// Tourne le robot pour qu'il regarde coteVoulu (HAUT, DROITE, BAS ou GAUCHE).
// diff = ((coteVoulu - direction) % 4 + 4) % 4
//   diff 1 -> tourner(+1), diff 2 -> tourner(2), diff 3 -> tourner(-1)
void faireFace(int coteVoulu) {
  // TODO
}


/* ============================================================================
   SECTION 2 : CAPTEURS
   ============================================================================ */

// VRAI s'il y a un panneau DANS LA CASE ACTUELLE devant le robot.
// Ne doit PAS voir le panneau de la case suivante (calibrer la portee).
// - 7 lectures de PIN_VERT / PIN_ROUGE (digitalRead), 3 ms entre chaque
// - VRAI si au moins 4 lectures disent "obstacle" (combinaison a confirmer)
bool murDevant() {
  // TODO
  return false;
}

// VRAI si le sifflet de 5 kHz est entendu.
// - 20 lectures (analogRead(PIN_5KHZ) - analogRead(PIN_AMBIANT)) > SEUIL_SIFFLET
// - VRAI si au moins 18 sur 20
bool sifflet5kHz() {
  // TODO
  return false;
}

// VRAI si le bumper arriere est appuye (depart de secours / arret d'urgence).
bool bumperArriere() {
  // TODO: return ROBUS_IsBumper(REAR);
  return false;
}

// Fait "nombre" bips courts (AX_BuzzerON / AX_BuzzerOFF, 100 ms chacun).
void beep(int nombre) {
  // TODO
}


/* ============================================================================
   SECTION 3 : CARTE DES MURS + BFS 
   ============================================================================ */

// Remet mur[][][] et connu[][][] a FAUX, puis ajoute :
// - les bordures (gauche de col 0, droite de col 2, bas de rangee 0, haut de rangee 9)
// - les tapes fixes : rangees 1, 3, 5, 7, 9 -> ajouterMur(r, 0, DROITE) et ajouterMur(r, 1, DROITE)
void initCarte() {
  // TODO
}

// Met un mur sur le cote de la case (r, c) ET sur le cote oppose de la case voisine.
// Marque aussi les 2 cotes comme connus.
void ajouterMur(int r, int c, int cote) {
  // TODO
}

// Marque le cote de la case (r, c) ET le cote oppose du voisin comme connus (pas de mur).
void marquerLibre(int r, int c, int cote) {
  // TODO
}

// VRAI si (r, c) est dans la grille.
bool dansGrille(int r, int c) {
  // TODO
  return false;
}

// Rangee de la case voisine du cote donne (HAUT = r + 1, BAS = r - 1, sinon r).
int voisinR(int r, int cote) {
  // TODO
  return r;
}

// Colonne de la case voisine du cote donne (DROITE = c + 1, GAUCHE = c - 1, sinon c).
int voisinC(int c, int cote) {
  // TODO
  return c;
}

// HAUT <-> BAS, DROITE <-> GAUCHE : (cote + 2) % 4
int coteOppose(int cote) {
  // TODO
  return cote;
}

// Remplit dist[][] avec un BFS (flood fill) a partir des cibles.
// - versFin = VRAI : cibles = les 3 cases de RANGEE_FIN
// - versFin = FAUX : cible  = (DEPART_R, DEPART_C)
// - Un passage est ouvert si :
//     mode NOUVEAU       : mur == FAUX
//     mode CONNU_SEULEMENT : mur == FAUX ET connu == VRAI
// - File simple : 2 tableaux fileR[30], fileC[30] + indices debut/fin
void calculerDistances(bool versFin, int mode) {
  // TODO
}

// Retourne le cote ouvert (selon mode) qui mene au voisin avec la plus petite dist.
// En cas d'egalite, preferer le cote "direction" (tout droit = moins de virages).
// Retourne -1 si aucun voisin n'a une distance < INFINI.
int meilleurCote(int r, int c, int mode) {
  // TODO
  return -1;
}

// Affiche la carte et dist[][] dans le moniteur serie (debug).
void afficherCarte() {
  // TODO
}


/* ============================================================================
   SECTION 4 : NAVIGATION
   ============================================================================ */

// Scanne les cotes INCONNUS de la case actuelle (devant en premier, puis gauche,
// droite, derriere). Pour chaque cote : faireFace(cote), puis
// murDevant() ? ajouterMur(...) : marquerLibre(...).
void scannerCellule() {
  // TODO
}

// Deplace le robot jusqu'a la cible. Retourne VRAI si arrive, FAUX si echec.
// Boucle (max MAX_PAS fois) :
//   - si arrive (versFin : robotR == RANGEE_FIN, sinon case Depart) -> VRAI
//   - si bumperArriere() -> arret(), FAUX
//   - si mode == NOUVEAU : scannerCellule()
//   - calculerDistances(versFin, mode)
//   - cote = meilleurCote(robotR, robotC, mode) ; si -1 -> FAUX
//     (retour : si -1 en CONNU_SEULEMENT, passer en NOUVEAU et recommencer)
//   - faireFace(cote)
//   - si murDevant() -> ajouterMur(...), recommencer (securite)
//   - avancerCase(), robotR = voisinR(...), robotC = voisinC(...)
bool naviguerVers(bool versFin, int mode) {
  // TODO
  return false;
}


/* ============================================================================
   SECTION 5 : SETUP ET LOOP
   ============================================================================ */

void setup() {
  BoardInit();
  Serial.begin(9600);

  pinMode(PIN_VERT, INPUT);
  pinMode(PIN_ROUGE, INPUT);

  initCarte();
  robotR    = DEPART_R;
  robotC    = DEPART_C;
  direction = HAUT;
  etat      = ATTENTE;

  beep(3);   // pret
}

void loop() {
  switch (etat) {
    case ATTENTE:
      if (sifflet5kHz() || bumperArriere()) {
        beep(2);
        delay(500);
        etat = ALLER;
      }
      break;

    case ALLER:
      if (naviguerVers(true, NOUVEAU)) {
        beep(1);
        etat = RETOUR;
      } else {
        etat = ERREUR;
      }
      afficherCarte();
      break;

    case RETOUR:
      if (naviguerVers(false, CONNU_SEULEMENT)) {
        faireFace(HAUT);
        beep(3);
        etat = TERMINE;
      } else {
        etat = ERREUR;
      }
      afficherCarte();
      break;

    case ERREUR:
      arret();
      beep(5);
      etat = TERMINE;
      break;

    case TERMINE:
      arret();
      break;
  }

  delay(10); // Delais pour decharger le CPU
}
