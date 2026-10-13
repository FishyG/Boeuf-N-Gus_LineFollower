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
#include <stdio.h>


/* ============================================================================
   SECTION 0 : DEFINES ET VARIABLES GLOBALES                         
   ============================================================================ */

// ---- Broches ----
#define PIN_VERT      48    
#define PIN_ROUGE     49   
#define PIN_5KHZ      A0   
#define PIN_AMBIANT   A1    
// ---- Seuils ---- base sur 3,2V min : 3,2 / 5 * 1024 = 655, max : 3,49 / 5 * 1024 = 714
#define SEUIL_SIFFLET 550
#define MAX_SIFFLET   714

#define PERIODE          10       
#define V_PPS_MAX        10272.0  
#define ACCEL            2.0     
#define KP               0.003    
#define KS               0.003    
#define PAUSE_MOUVEMENT  100  


#define VITESSE          0.45   
#define V_FIN            0.08     
#define V_MAINTIEN       0.15     
#define TEMPS_MAINTIEN   150      
#define RATIO_DROITE     1.0      


#define VITESSE_TOURNE   0.25     
#define V_FIN_TOURNE     0.15     
#define GLISSE_VIRAGE    62       
#define FACTEUR_TOURNE_D 1.08     
#define FACTEUR_TOURNE_G 1.08     


#define DEBUG_DEPLACEMENT      1     
#define MODE_TEST_DEPLACEMENT  0      
#define PAUSE_ENTRE_TESTS      5000


#define PPT             3200.0                    
#define DIAMETRE_ROUE   76.2                      
#define PULSES_PAR_MM   (PPT / (PI * DIAMETRE_ROUE))
#define ENTRAXE         177.8    
#define TAILLE_CASE     500.0
#define DISTANCE_AVANCE 500.0    // mm demandes aux roues pour 1 case : augmenter si le robot n'avance pas assez

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
#define TOTO     5

#define MAX_PAS  60

// ---- Navigation (SECTION 4) ----
#define SCAN_COMPLET      0   // 1 = scanner tous les cotes inconnus de chaque case
                              // 0 = verifier seulement le cote ou on veut aller (moins de virages)
#define DEBUG_NAVIGATION  1   // 1 = affiche chaque decision et la carte dans le moniteur serie

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
bool passageOuvert(int r, int c, int cote, int mode);
void calculerDistances(bool versFin, int mode);
int  meilleurCote(int r, int c, int mode);
void afficherCarte();

// SECTION 4 : navigation
void scannerCellule();
bool naviguerVers(bool versFin, int mode);
bool estArrive(bool versFin);
bool verifierCote(int cote);


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
  mouvement(mmEnPulses(DISTANCE_AVANCE), +1, +1, VITESSE);
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
    if (vert == LOW && rouge == LOW)
    {
      obstacle ++;
    }
    if(vert == LOW || rouge == LOW) 
    {
      obstacle ++;
    }
    // Si on a 4 positif, il y a un mur
    if (obstacle >= 4) 
    {
      return true;
    }
    delay(15);
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
    delay(50);
  }
  // Pas de sifflet
  //beep(4);
  delay(500);
  return false;
}

// VRAI si le bumper arriere est appuye (depart de secours / arret d'urgence).
bool bumperArriere() 
{
  // La fonction le fait... je ne sais pas si c'est nécessaire de faire une fonction d'une fonction
  return (ROBUS_IsBumper(REAR));
}

// Fait "nombre" bips courts (AX_BuzzerON / AX_BuzzerOFF, 100 ms chacun).
void beep(int nombre)
{
  // On active le buzzer le nombre de fois demandée
  for ( int i = 0; i < nombre; i++ )
  {
    AX_BuzzerON();
    delay(100);
    AX_BuzzerOFF();
    delay(100);
  }
}


/* ============================================================================
   SECTION 3 : CARTE DES MURS + BFS 
   ============================================================================ */

// Remet mur[][][] et connu[][][] a FAUX, puis ajoute :
// - les bordures (gauche de col 0, droite de col 2, bas de rangee 0, haut de rangee 9)
// - les tapes fixes : rangees 1, 3, 5, 7, 9 -> ajouterMur(r, 0, DROITE) et ajouterMur(r, 1, DROITE)
void initCarte(){
  for (int r =0; r < NB_RANGEES; r++) {
    for (int c = 0; c < NB_COLONNES; c++) {
      for (int cote = 0; cote < 4; cote++) {
        mur[r][c][cote]   = false;
        connu[r][c][cote] = false;
      }
    }
  }

  // Bordures gauche et droite
  for (int r = 0; r < NB_RANGEES; r++) {
    ajouterMur(r, 0, GAUCHE);
    ajouterMur(r, NB_COLONNES - 1, DROITE);
  }

  //Bordure bas et haut
  for (int c = 0; c < NB_COLONNES; c++) {
    ajouterMur(0, c, BAS);
    ajouterMur(NB_RANGEES - 1, c, HAUT);
  }


  //Tapes fixe
  for (int r = 1; r < NB_RANGEES; r += 2) {
    ajouterMur(r, 0, DROITE);
    ajouterMur(r, 1, DROITE);
  }
}

void ajouterMur(int r, int c, int cote) {
  mur[r][c][cote]   = true;
  connu[r][c][cote] = true;

  int vr = voisinR(r, cote);
  int vc = voisinC(c, cote);
  if (dansGrille(vr, vc)) {
    mur[vr][vc][coteOppose(cote)]   = true;
    connu[vr][vc][coteOppose(cote)] = true;
  }
}
// Marque le cote de la case (r, c) ET le cote oppose du voisin comme connus (pas de mur).
void marquerLibre(int r, int c, int cote) {
  connu[r][c][cote] = true;

  // Le meme cote vu de la case voisine
  int vr = voisinR(r, cote);
  int vc = voisinC(c, cote);
  if (dansGrille(vr, vc)) {
    connu[vr][vc][coteOppose(cote)] = true;
  }
}
bool dansGrille(int r, int c) {
  return r >= 0 && r < NB_RANGEES && c >= 0 && c < NB_COLONNES;
}
// Rangee de la case voisine du cote donne (HAUT = r + 1, BAS = r - 1, sinon r).
int voisinR(int r, int cote) {
  if (cote == HAUT) return r + 1;
  if (cote == BAS)  return r - 1;
  return r;
}
// Colonne de la case voisine du cote donne (DROITE = c + 1, GAUCHE = c - 1, sinon c).
int voisinC(int c, int cote) {
  if (cote == DROITE) return c + 1;
  if (cote == GAUCHE) return c - 1;
  return c;
}
// HAUT <-> BAS, DROITE <-> GAUCHE : (cote + 2) % 4
int coteOppose(int cote) {
  return (cote + 2) % 4;
}
// VRAI si le robot peut passer de la case (r, c) vers la case voisine du cote donne.
//   mode NOUVEAU         : pas de mur connu (un cote inconnu est considere libre)
//   mode CONNU_SEULEMENT : pas de mur ET le cote a deja ete verifie
bool passageOuvert(int r, int c, int cote, int mode) {
  if (!dansGrille(voisinR(r, cote), voisinC(c, cote))) return false;
  if (mur[r][c][cote]) return false;
  if (mode == CONNU_SEULEMENT && !connu[r][c][cote]) return false;
  return true;
}
// Remplit dist[][] avec un BFS (flood fill) a partir des cibles.
// - versFin = VRAI : cibles = les 3 cases de RANGEE_FIN
// - versFin = FAUX : cible  = (DEPART_R, DEPART_C)
// - Un passage est ouvert si :
//     mode NOUVEAU       : mur == FAUX
//     mode CONNU_SEULEMENT : mur == FAUX ET connu == VRAI
// - File simple : 2 tableaux fileR[30], fileC[30] + indices debut/fin
void calculerDistances(bool versFin, int mode) {
  int fileR[NB_RANGEES * NB_COLONNES];
  int fileC[NB_RANGEES * NB_COLONNES];
  int debut = 0;  
  int fin   = 0;  

  // Toutes les distances a INFINI
  for (int r = 0; r < NB_RANGEES; r++) {
    for (int c = 0; c < NB_COLONNES; c++) {
      dist[r][c] = INFINI;
    }
  }

  // Les cibles sont a distance 0 et partent dans la file
  if (versFin) {
    for (int c = 0; c < NB_COLONNES; c++) {
      dist[RANGEE_FIN][c] = 0;
      fileR[fin] = RANGEE_FIN;
      fileC[fin] = c;
      fin++;
    }
  } else {
    dist[DEPART_R][DEPART_C] = 0;
    fileR[fin] = DEPART_R;
    fileC[fin] = DEPART_C;
    fin++;
  }

  // BFS : chaque case donne sa distance + 1 a ses voisins atteignables
  while (debut < fin) {
    int r = fileR[debut];
    int c = fileC[debut];
    debut++;

    for (int cote = 0; cote < 4; cote++) {
      if (!passageOuvert(r, c, cote, mode)) continue;

      int vr = voisinR(r, cote);
      int vc = voisinC(c, cote);
      if (dist[vr][vc] == INFINI) {
        dist[vr][vc] = dist[r][c] + 1;
        fileR[fin] = vr;
        fileC[fin] = vc;
        fin++;
      }
    }
  }
}

// Retourne le cote ouvert (selon mode) qui mene au voisin avec la plus petite dist.
// En cas d'egalite, preferer le cote "direction" (tout droit = moins de virages).
// Retourne -1 si aucun voisin n'a une distance < INFINI.
int meilleurCote(int r, int c, int mode) {
  // Ordre d'essai : tout droit, gauche, droite, derriere.
  // Avec "<" (et non "<="), le premier cote essaye gagne les egalites :
  // on prefere donc le chemin avec le moins de virages.
  int ordre[4] = {
    direction,
    (direction + 3) % 4,   // gauche
    (direction + 1) % 4,   // droite
    (direction + 2) % 4    // derriere
  };

  int meilleur = -1;
  int meilleureDist = INFINI;

  for (int i = 0; i < 4; i++) {
    int cote = ordre[i];
    if (!passageOuvert(r, c, cote, mode)) continue;

    int d = dist[voisinR(r, cote)][voisinC(c, cote)];
    if (d < meilleureDist) {
      meilleureDist = d;
      meilleur = cote;
    }
  }
  return meilleur;
}

// Affiche la carte et dist[][] dans le moniteur serie (debug).
// Legende :
//   ---- ou |  = mur (panneau, tape ou bordure)
//   . .  ou :  = cote pas encore verifie
//   vide       = passage verifie libre
//   nombre     = distance jusqu'a la cible (- = inatteignable)
//   R^ R> Rv R< = robot et la direction qu'il regarde
void afficherCarte() {
  const char fleche[4] = {'^', '>', 'v', '<'};   // HAUT, DROITE, BAS, GAUCHE

  Serial.println();
  for (int r = NB_RANGEES - 1; r >= 0; r--) {
    // Ligne du haut des cases
    Serial.print("    ");
    for (int c = 0; c < NB_COLONNES; c++) {
      Serial.print("+");
      if (mur[r][c][HAUT])        Serial.print("----");
      else if (!connu[r][c][HAUT]) Serial.print(" . .");
      else                         Serial.print("    ");
    }
    Serial.println("+");

    // Ligne des cases : mur de gauche + contenu
    Serial.print("r");
    Serial.print(r);
    Serial.print("  ");
    for (int c = 0; c < NB_COLONNES; c++) {
      if (mur[r][c][GAUCHE])        Serial.print("|");
      else if (!connu[r][c][GAUCHE]) Serial.print(":");
      else                           Serial.print(" ");

      if (r == robotR && c == robotC) {
        Serial.print(" R");
        Serial.print(fleche[direction]);
        Serial.print(" ");
      } else if (dist[r][c] >= INFINI) {
        Serial.print("  - ");
      } else {
        if (dist[r][c] < 10) Serial.print(" ");
        if (dist[r][c] < 100) Serial.print(" ");
        Serial.print(dist[r][c]);
        Serial.print(" ");
      }
    }
    // Mur de droite de la derniere colonne
    if (mur[r][NB_COLONNES - 1][DROITE])        Serial.println("|");
    else if (!connu[r][NB_COLONNES - 1][DROITE]) Serial.println(":");
    else                                         Serial.println(" ");
  }

  // Ligne du bas de la rangee 0
  Serial.print("    ");
  for (int c = 0; c < NB_COLONNES; c++) {
    Serial.print("+");
    Serial.print(mur[0][c][BAS] ? "----" : "    ");
  }
  Serial.println("+");

  Serial.print("Robot : rangee ");
  Serial.print(robotR);
  Serial.print(", colonne ");
  Serial.print(robotC);
  Serial.print(", regarde ");
  Serial.println(fleche[direction]);
}


/* ============================================================================
   SECTION 4 : NAVIGATION
   ============================================================================ */

// VRAI si le robot est sur une case cible.
//   versFin = VRAI : n'importe quelle case de la rangee Fin
//   versFin = FAUX : la case Depart
bool estArrive(bool versFin) {
  if (versFin) return robotR == RANGEE_FIN;
  return robotR == DEPART_R && robotC == DEPART_C;
}

// Tourne vers "cote" et regarde s'il y a un panneau. Met la carte a jour.
// Retourne VRAI si le passage est libre.
bool verifierCote(int cote) {
  faireFace(cote);
  if (murDevant()) {
    ajouterMur(robotR, robotC, cote);
    return false;
  }
  marquerLibre(robotR, robotC, cote);
  return true;
}

// Scanne tous les cotes INCONNUS de la case actuelle : devant d'abord (pas de
// virage), puis en tournant toujours dans le meme sens (gauche, derriere, droite).
// Les cotes deja connus (tapes, bordures, deja vus) ne sont pas re-scannes.
void scannerCellule() {
  for (int i = 0; i < 4; i++) {
    int cote = (direction + 4 - i) % 4;   // devant, gauche, derriere, droite
    if (!connu[robotR][robotC][cote]) {
      verifierCote(cote);
    }
  }
}

// Deplace le robot case par case jusqu'a la cible.
// Retourne VRAI si arrive, FAUX si echec (bloque, bumper, trop de pas).
//
// A chaque pas :
//   1. calculer les distances jusqu'a la cible (BFS)
//   2. choisir le meilleur cote (tout droit en priorite)
//   3. si ce cote n'a jamais ete verifie : tourner vers lui et regarder.
//      Panneau -> l'ajouter a la carte et recommencer au 1.
//   4. avancer d'une case
//
// mode NOUVEAU (aller) : un cote inconnu est suppose libre, on le verifie en y allant.
// mode CONNU_SEULEMENT (retour) : seulement les passages deja verifies, donc
//   aucune surprise. Si aucun chemin connu, on repasse en NOUVEAU.
bool naviguerVers(bool versFin, int mode) {
  for (int pas = 0; pas < MAX_PAS; pas++) {
    if (estArrive(versFin)) return true;

    if (bumperArriere()) {
      arret();
      return false;
    }

    if (mode == NOUVEAU && SCAN_COMPLET) {
      scannerCellule();
    }

    
    calculerDistances(versFin, mode);
    int cote = meilleurCote(robotR, robotC, mode);

    if (cote == -1) {
      if (mode == CONNU_SEULEMENT) {
        // Pas de chemin connu : explorer comme a l'aller
        if (DEBUG_NAVIGATION) Serial.println("Aucun chemin connu -> mode NOUVEAU");
        mode = NOUVEAU;
        continue;
      }
      if (DEBUG_NAVIGATION) Serial.println("ERREUR : aucun chemin possible");
      return false;
    }

    // Verifier le cote s'il est inconnu
    if (!connu[robotR][robotC][cote]) {
      if (!verifierCote(cote)) {
        if (DEBUG_NAVIGATION) Serial.println("Panneau detecte -> nouveau chemin");
        continue;
      }
    } else {
      faireFace(cote);
    }

    // Avancer
    avancerCase();
    robotR = voisinR(robotR, cote);
    robotC = voisinC(robotC, cote);

    if (DEBUG_NAVIGATION) afficherCarte();
  }

  if (DEBUG_NAVIGATION) Serial.println("ERREUR : trop de pas (MAX_PAS)");
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

  // if (MODE_TEST_DEPLACEMENT) {
  //   delay(2000);
  //   testsDeplacement();
  // }
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
    
    case TOTO:
      testsDeplacement();
      beep(3);
      etat = TERMINE;
      break;
  }

  delay(10); // Delais pour decharger le CPU
}
