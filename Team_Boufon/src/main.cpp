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
// ---- Seuils ---- base sur 3,2V min : 3,2 / 5 * 1024 = 655, max : 3,49 / 5 * 1024 = 714
#define SEUIL_SIFFLET 614
#define MAX_SIFFLET   714

#define PERIODE          10       
#define V_PPS_MAX        10272.0  
#define ACCEL            1.5      
#define KP               0.003    
#define KS               0.003    
#define PAUSE_MOUVEMENT  200     


#define VITESSE          0.30     
#define V_FIN            0.08     
#define V_MAINTIEN       0.15     
#define TEMPS_MAINTIEN   300      
#define RATIO_DROITE     1.0      


#define VITESSE_TOURNE   0.20     
#define V_FIN_TOURNE     0.15     
#define GLISSE_VIRAGE    62       
#define FACTEUR_TOURNE_D 1.08     
#define FACTEUR_TOURNE_G 1.08     


#define DEBUG_DEPLACEMENT      1     
#define MODE_TEST_DEPLACEMENT  1      
#define PAUSE_ENTRE_TESTS      5000


#define PPT             3200.0                    
#define DIAMETRE_ROUE   76.2                      
#define PULSES_PAR_MM   (PPT / (PI * DIAMETRE_ROUE))
#define ENTRAXE         177.8    
#define TAILLE_CASE     500.0

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



long resteG    = 0;   
long resteD    = 0;   
long erreurCap = 0;   


/* ============================================================================
   PROTOTYPES
   ============================================================================ */

// SECTION 1 : deplacements
void arret();
void avancerCase();
void tourner(int quarts);
void faireFace(int coteVoulu);
long mmEnPulses(float mm);
void mouvement(long cible, int sensG, int sensD, float vMax);
void testLigneDroite();
void testVirages();
void testCarres();
void testAllerRetour();
void testFaireFace();
void testsDeplacement();

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

// Coupe les 2 moteurs.
void arret() {
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
}

// Convertit une distance en mm en pulses d'encodeur.
long mmEnPulses(float mm) {
  return (long)(mm * PULSES_PAR_MM);
}

void mouvement(long cible, int sensG, int sensD, float vMax) {
  if (cible <= 0) return;

  bool ligneDroite = (sensG == sensD);
  long cibleNormale = cible;

  if (!ligneDroite) {
    if (sensG > 0) cible -= erreurCap; 
    else           cible += erreurCap;   
    cible -= GLISSE_VIRAGE;
  }

  long cibleG = cible;
  long cibleD = (long)(cible * RATIO_DROITE);
  if (ligneDroite) {
    
    cibleG += sensG * resteG;
    cibleD += sensD * resteD;
  }

  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);

  const float dt    = PERIODE / 1000.0;
  const float vMaxP = vMax * V_PPS_MAX;                                    
  const float vFinP = (ligneDroite ? V_FIN : V_FIN_TOURNE) * V_PPS_MAX;   
  const float accP  = ACCEL * V_PPS_MAX;                                   

  float pRef = 0;  
  float vRef = 0;  
  bool maintien = false;
  unsigned long debutMaintien = 0;

  while (true) {
    unsigned long debutPeriode = millis();
    float vAvance, vMinCmd, vMaxCmd;

    if (pRef < cible) {
      
      float vFrein = sqrt(2.0 * accP * (cible - pRef));
      vRef = min(vRef + accP * dt, vMaxP);
      vRef = min(vRef, max(vFrein, vFinP));
      pRef = min(pRef + vRef * dt, (float)cible);
      vAvance = vRef / V_PPS_MAX;
      vMinCmd = 0.0;  
      vMaxCmd = 1.0;
    } else {
      
      if (!ligneDroite) break;   
      if (!maintien) {
        maintien = true;
        debutMaintien = millis();
      }
      if (millis() - debutMaintien >= TEMPS_MAINTIEN) break;
      vAvance = 0.0; 
      vMinCmd = -V_MAINTIEN;
      vMaxCmd =  V_MAINTIEN;
    }

    float refG = pRef * cibleG / cible;
    float refD = pRef * cibleD / cible;
    long posG = sensG * ENCODER_Read(LEFT);
    long posD = sensD * ENCODER_Read(RIGHT);

    
    if (!maintien && (posG < -200 || posD < -200)) {
      arret();
      Serial.println("ERREUR mouvement : une roue tourne a l'envers");
      break;
    }

    float erreurG = refG - posG;
    float erreurD = refD - posD;
    float sync = KS * (erreurG - erreurD);

    float vG = constrain(vAvance + KP * erreurG + sync, vMinCmd, vMaxCmd);
    float vD = constrain(vAvance + KP * erreurD - sync, vMinCmd, vMaxCmd);
    MOTOR_SetSpeed(LEFT,  sensG * vG);
    MOTOR_SetSpeed(RIGHT, sensD * vD);

    while (millis() - debutPeriode < PERIODE) {}   
  }

  
  arret();
  delay(PAUSE_MOUVEMENT);   
  long finG = ENCODER_Read(LEFT);
  long finD = ENCODER_Read(RIGHT);

  if (ligneDroite) {
    
    resteG = constrain(sensG * cibleG - finG, -300, 300);
    resteD = constrain(sensD * cibleD - finD, -300, 300);
  } else {
    long ecart = (abs(finG) + abs(finD)) / 2 - cibleNormale;
    if (sensG > 0) erreurCap += ecart;
    else           erreurCap -= ecart;
    erreurCap = constrain(erreurCap, -150, 150);
  }

  if (DEBUG_DEPLACEMENT) {
    Serial.print(ligneDroite ? "AVANCE" : "VIRAGE");
    Serial.print("  cible ");     Serial.print(cible);
    Serial.print("  G ");         Serial.print(finG);
    Serial.print("  D ");         Serial.print(finD);
    if (ligneDroite) {
      Serial.print("  resteG ");  Serial.print(resteG);
      Serial.print("  resteD ");  Serial.println(resteD);
    } else {
      Serial.print("  erreurCap "); Serial.println(erreurCap);
    }
  }
}

void avancerCase() {
  mouvement(mmEnPulses(TAILLE_CASE), +1, +1, VITESSE);
}

void tourner(int quarts) {
  if (quarts == 0) return;

  long cible = mmEnPulses(PI * ENTRAXE / 4.0 * abs(quarts));
  if (quarts > 0) mouvement(cible * FACTEUR_TOURNE_D, +1, -1, VITESSE_TOURNE);  
  else            mouvement(cible * FACTEUR_TOURNE_G, -1, +1, VITESSE_TOURNE);   

  direction = ((direction + quarts) % 4 + 4) % 4; 
}

void faireFace(int coteVoulu) {
  int diff = ((coteVoulu - direction) % 4 + 4) % 4;

  if (diff == 1)      tourner(+1);
  else if (diff == 2) tourner(2);
  else if (diff == 3) tourner(-1);
}



void testLigneDroite() {
  for (int i = 0; i < 6; i++) avancerCase();
}


void testVirages() {
  for (int i = 0; i < 4; i++) tourner(+1);
  delay(PAUSE_ENTRE_TESTS);
  for (int i = 0; i < 4; i++) tourner(-1);
}


void testCarres() {
  for (int i = 0; i < 4; i++) {
    avancerCase();
    tourner(+1);
  }
  delay(PAUSE_ENTRE_TESTS);
  for (int i = 0; i < 4; i++) {
    avancerCase();
    tourner(-1);
  }
}


void testAllerRetour() {
  avancerCase();
  avancerCase();
  tourner(2);
  avancerCase();
  avancerCase();
  tourner(2);
}


void testFaireFace() {
  faireFace(DROITE);
  faireFace(BAS);
  faireFace(GAUCHE);
  faireFace(HAUT);
}

void testsDeplacement() {
  Serial.println("--- Test ligne droite 3 m ---");
  testLigneDroite();
  delay(PAUSE_ENTRE_TESTS);

  Serial.println("--- Test virages ---");
  testVirages();
  delay(PAUSE_ENTRE_TESTS);

  Serial.println("--- Test carres ---");
  testCarres();
  delay(PAUSE_ENTRE_TESTS);

  Serial.println("--- Test aller-retour ---");
  testAllerRetour();
  delay(PAUSE_ENTRE_TESTS);

  Serial.println("--- Test faireFace ---");
  testFaireFace();
  Serial.println("--- Fin des tests ---");
}


/* ============================================================================
   SECTION 2 : CAPTEURS
   ============================================================================ */

// Fonction de mur devant() : retourne vrai ou faux
bool murDevant() 
{
  // Obstacle qui doit etre d'au moins 4 sur 7
  int obstacle = 0;
  // On boucle 7 fois
  for (int i = 0; i < 7; i++) 
  {
    // Lecture des pins
    int vert = digitalRead(PIN_VERT);
    int rouge = digitalRead(PIN_ROUGE);
    // Detection de mur
    if (vert == HIGH && rouge == HIGH)
    {
      obstacle ++;
    }
    if(vert == HIGH || rouge == HIGH) 
    {
      obstacle ++;
    }
    // Si on a 4 positif, il y a un mur
    if (obstacle >= 4) 
    {
      return true;
    }
    delay(3);
  }
  // Pas de mur
  return false;
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
    delay(2);
  }
  // Pas de sifflet
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

  if (MODE_TEST_DEPLACEMENT) {
    delay(2000);
    testsDeplacement();
  }
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
