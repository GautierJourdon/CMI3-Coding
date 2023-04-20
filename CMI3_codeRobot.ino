//==============================BIBLIOTHEQUES/LIBRARIES===========================//
#include <SoftwareSerial.h>
#include <SimpleTimer.h>
SoftwareSerial bluetooth(7 ,11); //rx = 7 ; tx = 11; Il faut les brancher sur leurs opposés sur arduino : tx_ard -> rx_blu && rx_ard -> tx_blu
SimpleTimer timer;

//================================MESURES=ROBOT=TRACEUR===========================//

int diametre_roue_D = 6.4, diametre_roue_G = 6.4;
int circonferenceRoueD = 3 * diametre_roue_D, cironference_roue_G = 3 * diametre_roue_G;
int ticksParTourRoueDroite = 1921, ticksParTourRoueGauche = 1882;

//================================ATTRIBUTION=DES=PINS============================//

//Moteur Arrière Droit
const byte PWM1 = 10;
const byte IN1AM1 = 9;
const byte IN2AM1 = 8;
const byte encoderPinM1A = 2;

//Moteur Arrière Gauche
const byte PWM2 = 6;
const byte IN1AM2 = 12;
const byte IN2AM2 = 13;
const byte encoderPinM2A = 3;

//=================================ECHANTILLONAGE=================================//

unsigned int time = 0;
const int frequence_echantillonnage_moteur = 20;
//const int frequence_echantillonnage_moteur_gauche = 20;

//==============================PREPARATION=DU=PID===============================//

//---------Moteur1---------//
const float kp_M1 = 1.2; // Coefficient proportionnel (choisis par essais successifs)
const float ki_M1 = 0; // Coefficient intégrateur
const float kd_M1 = 0; // Coefficient dérivateur

//---------Moteur2---------//
const float kp_M2 = 1.3; // Coefficient proportionnel (choisis par essais successifs)
const float ki_M2 = 0; // Coefficient intégrateur
const float kd_M2 = 0; // Coefficient dérivateur

//Initialisation du calculs de l'asservissement PID
int erreurM1 = 0;
int erreurM2 = 0;
float erreurPrecedenteM1 = 0;
float erreurPrecedenteM2 = 0;
float somme_erreurM1 = 0;
float somme_erreurM2 = 0;

//==================================EN=MOUVEMENT=======================================//

bool enMouvement = false;

//================================DONNER=UNE=CONSIGNE==================================//

double target_cm = 10;
double target_deg = 19.2 * target_cm;

//Moteur Droit
int target_ticks_droit = target_cm * ticksParTourRoueDroite / circonferenceRoueD; //plus simple d'asservir en ticks car ce sera toujours un nombre entier //1944
int tick_imp_droit = 1921;

//Moteur Gauche
int target_ticks_gauche = target_cm * ticksParTourRoueGauche / cironference_roue_G; //plus simple d'asservir en ticks car ce sera toujours un nombre  //1787
int tick_imp_gauche = 1882;

//========================PREPARATION=ENCODEUR=MOTEUR=DROIT===========================//

//initialiser le compteur :
int encoder0PosM1 = 0; //position de départ=0

//========================PREPARATION=ENCODEUR=MOTEUR=GAUCHE==========================//

//initialiser le compteur :
int encoder0PosM2 = 0; //position de départ=0

//====================================BLUETOOTH=PART==================================//
char C;
//Reception bluetooth de charactère via l'application
void receptionChar(char &element){
  if(bluetooth.available()){
    element = bluetooth.read();
    Serial.print(element);
  }
}

//================================VITESSE(S)=DES=MOTEURS================================//

int vitesseMoteurDroit = kp_M1 * erreurM1 + kd_M1 * (erreurM1 - erreurPrecedenteM1) + ki_M1 * (somme_erreurM1);//PWM1
int vitesseMoteurGauche = kp_M2 * erreurM2 + kd_M2 * (erreurM2 - erreurPrecedenteM2) + ki_M2 * (somme_erreurM2);//PWM2



//=======================================ASSERVISSEMENT===========================================//

void asservissementMoteur(){
 time += 20;
 erreurM1 = target_ticks_droit - encoder0PosM1;
 erreurM2 = target_ticks_gauche - encoder0PosM2;

  //Partie inutile car ki = 0
 somme_erreurM1 += erreurM1;
 somme_erreurM2 += erreurM2;

 //Partie inutile car kd = 0
 erreurPrecedenteM1 = erreurM1;
 erreurPrecedenteM2 = erreurM2;

 if (vitesseMoteurDroit > 255) vitesseMoteurDroit = 255;
 else if (vitesseMoteurDroit < -255) vitesseMoteurDroit = -255;
 if (vitesseMoteurGauche > 255) vitesseMoteurGauche = 255;
 else if (vitesseMoteurGauche < -255) vitesseMoteurGauche = -255;
 TournerRoueDroite (vitesseMoteurDroit);
 TournerRoueGauche (vitesseMoteurGauche);

 float distanceR1 = encoder0PosM1 / 1921 * 19.2;
 float distanceR2 = encoder0PosM2 / 1882 * 19.2;

 Serial.println(encoder0PosM1);
 vitesseMoteurDroit = kp_M1 * erreurM1 + kd_M1 * (erreurM1 - erreurPrecedenteM1) + ki_M1 * (somme_erreurM1);
 vitesseMoteurGauche = kp_M2 * erreurM2 + kd_M2 * (erreurM2 - erreurPrecedenteM2) + ki_M2 * (somme_erreurM2);
}

//============================FONCTIONS=DoEncoder=(2Moteurs)=====================================//

//---- Interruption appelée à tous les changements d'état de A
void doEncoderM1A(){
 encoder0PosM1 ++; //modifie le compteur selon les deux états des encodeurs
}

void doEncoderM2A(){
 encoder0PosM2 ++;
}
//======================================FONCTIONS=TROURNER=====================================//

//---- Fonction appelée pour contrôler le moteur
void TournerRoueDroite(int rapportCycliqueM1){
 if (rapportCycliqueM1 > 0)
 {
 digitalWrite(IN1AM1, LOW);
 digitalWrite(IN2AM1, HIGH);
 }
 else
 {
 digitalWrite(IN1AM1, HIGH);
 digitalWrite(IN2AM1, LOW);
 rapportCycliqueM1 = -rapportCycliqueM1;
 Serial.println("1");
 }
 analogWrite(PWM1, rapportCycliqueM1);
}

void TournerRoueGauche(int rapportCycliqueM2){
 if (rapportCycliqueM2 > 0)
 {
 digitalWrite(IN1AM2, LOW);
 digitalWrite(IN2AM2, HIGH);
 }
 else
 {
 digitalWrite(IN1AM2, HIGH);
 digitalWrite(IN2AM2, LOW);
 rapportCycliqueM2 = -rapportCycliqueM2;
 }
 analogWrite(PWM2, rapportCycliqueM2);
}

//===========================================INITIALISATION=TARGET=TICKS==========================//

void updateTargetTicks() {
  target_ticks_droit = target_cm * 1921 / 19.2;
  target_ticks_gauche = target_cm * 1882 / 19.2;
}

//=================================DIRECTIONS========================================//

//Arret progressif du robot
void arretProgressif() {
  while (vitesseMoteurDroit > 0 && vitesseMoteurGauche > 0) {
    vitesseMoteurDroit = vitesseMoteurDroit - 5;
    vitesseMoteurGauche = vitesseMoteurGauche - 5;
    analogWrite(PWM1, vitesseMoteurDroit);
    analogWrite(PWM2, vitesseMoteurGauche);
    delay(50);
  }
  arretStop();
}

//Arrêt du robot
void arretStop(){
  pinMode(IN1AM1, OUTPUT);
  pinMode(IN1AM2, OUTPUT);
  pinMode(IN2AM1, OUTPUT);
  pinMode(IN2AM2, OUTPUT);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  digitalWrite(IN1AM1, LOW);
  digitalWrite(IN2AM1, LOW);
  digitalWrite(IN1AM2, LOW);
  digitalWrite(IN2AM2, LOW);
}

//Aller vers la gauche
void tournerGauche(int cm){  
  int targetRoueDroite = cm * 1921 / 19.2;
  int targetRoueGauche = cm * 1882 / 19.2; 
  pinMode(IN1AM1, OUTPUT);
  pinMode(IN1AM2, OUTPUT);
  pinMode(IN2AM1, OUTPUT);
  pinMode(IN2AM2, OUTPUT);
  enMouvement = true;

  while(enMouvement && (encoder0PosM1 < targetRoueDroite) && (encoder0PosM2 < targetRoueGauche)){

    asservissementMoteur();
    analogWrite(PWM1, abs(vitesseMoteurDroit));
    analogWrite(PWM2, abs(vitesseMoteurGauche));
    digitalWrite(IN1AM1, LOW);
    digitalWrite(IN2AM1, HIGH);
    digitalWrite(IN1AM2, HIGH);
    digitalWrite(IN2AM2, LOW);

    /*
    Serial.println("encodeur droit : ");
    Serial.println(encoder0PosM1);
    Serial.println("encodeur gauche : ");
    Serial.println(encoder0PosM2);
    */

    delay(10);
  }
  delay(10);
  enMouvement = false;
  arretProgressif();
}

//Aller vers la droite
void tournerDroite(int cm){  
  int targetRoueDroite = cm * 1921 / 19.2;
  int targetRoueGauche = cm * 1882 / 19.2; 
  pinMode(IN1AM1, OUTPUT);
  pinMode(IN1AM2, OUTPUT);
  pinMode(IN2AM1, OUTPUT);
  pinMode(IN2AM2, OUTPUT);
  enMouvement = true;

  while(enMouvement && (encoder0PosM1 < targetRoueDroite) && (encoder0PosM2 < targetRoueGauche)){

    asservissementMoteur();
    analogWrite(PWM1, abs(vitesseMoteurDroit));
    analogWrite(PWM2, abs(vitesseMoteurGauche));
    digitalWrite(IN1AM1, HIGH);
    digitalWrite(IN2AM1, LOW);
    digitalWrite(IN1AM2, LOW);
    digitalWrite(IN2AM2, HIGH);

    /*
    Serial.println("encodeur droit : ");
    Serial.println(encoder0PosM1);
    Serial.println("encodeur gauche : ");
    Serial.println(encoder0PosM2);
    */

    delay(10);
  }
  delay(10);
  enMouvement = false;
  arretProgressif();
}

//Aller vers l'avant
void Avancer(int cm){
  int targetRoueDroite = cm * 1921 / 19.2;
  int targetRoueGauche = cm * 1882 / 19.2; 
  pinMode(IN1AM1, OUTPUT);
  pinMode(IN1AM2, OUTPUT);
  pinMode(IN2AM1, OUTPUT);
  pinMode(IN2AM2, OUTPUT);
  enMouvement = true;

  while(enMouvement && (encoder0PosM1 < targetRoueDroite) && (encoder0PosM2 < targetRoueGauche)){

    asservissementMoteur();
    analogWrite(PWM1, vitesseMoteurDroit);
    analogWrite(PWM2, vitesseMoteurGauche);
    digitalWrite(IN1AM1, HIGH);
    digitalWrite(IN2AM1, LOW);
    digitalWrite(IN1AM2, HIGH);
    digitalWrite(IN2AM2, LOW);

    /*
    Serial.println("encodeur droit : ");
    Serial.println(encoder0PosM1);
    Serial.println("encodeur gauche : ");
    Serial.println(encoder0PosM2);
    */
    delay(10);
  }
  delay(1);
  enMouvement = false;
  arretProgressif();
}

//Aller vers l'arrière
void Reculer(int cm){
  int targetRoueDroite = cm * 1921 / 19.2;
  int targetRoueGauche = cm * 1882 / 19.2;
  pinMode(IN1AM1, OUTPUT);
  pinMode(IN1AM2, OUTPUT);
  pinMode(IN2AM1, OUTPUT);
  pinMode(IN2AM2, OUTPUT);
  enMouvement = true;

  while(enMouvement && (encoder0PosM1 < targetRoueDroite) && (encoder0PosM2 < targetRoueGauche)){

    asservissementMoteur();
    analogWrite(PWM1, vitesseMoteurDroit);
    analogWrite(PWM2, vitesseMoteurGauche);
    digitalWrite(IN1AM1, LOW);
    digitalWrite(IN2AM1, HIGH);
    digitalWrite(IN1AM2, LOW);
    digitalWrite(IN2AM2, HIGH);

    /*
    Serial.println("encodeur droit : ");
    Serial.println(encoder0PosM1);
    Serial.println("encodeur gauche : ");
    Serial.println(encoder0PosM2);
    */
    delay(10);
  }
  enMouvement = false;
  arretProgressif();
}


//=============================================FORMES=============================================//

//tracer un Cercle
void circle(int cm){
  int targetRoueDroite = cm * 1984 / 19.2;
  int targetRoueGauche = cm * 1747 / 19.2;
  pinMode(IN1AM1, OUTPUT);
  pinMode(IN1AM2, OUTPUT);
  pinMode(IN2AM1, OUTPUT);
  pinMode(IN2AM2, OUTPUT);
  enMouvement = true;

  while(enMouvement && (encoder0PosM1 < targetRoueDroite) && (encoder0PosM2 < targetRoueGauche)){

    asservissementMoteur();
    analogWrite(PWM1, vitesseMoteurDroit);
    analogWrite(PWM2, vitesseMoteurGauche);
    digitalWrite(IN1AM1, HIGH);
    digitalWrite(IN2AM1, LOW);
    digitalWrite(IN1AM2, LOW);
    digitalWrite(IN2AM2, LOW);

    Serial.println("encodeur droit : ");
    Serial.println(encoder0PosM1);
    Serial.println("encodeur gauche : ");
    Serial.println(encoder0PosM2);

    delay(10);
  }
  delay(10);
  enMouvement = false;
  arretStop();
}

//tracer un Rectangle
void rectangle(int cm){
  //ligne droite
  Avancer(cm);
  for(int i = 0 ; i < 3 ; i++)
  {
    //tourner de 90°
    tournerDroite(cm);
    //avancer en ligne droite
    Avancer(cm);
  }
  delay(10);
  arretStop();
}

//tracer un Triangle
void triangle(int cm){
  //Le triangle doit idéalement être équilatéral
  Avancer(cm);  
  //Les 3 angles = 60°
  for(int i = 0 ; i < 2 ; i++){
    asservissementMoteur();
    tournerDroite(cm);
    Avancer(cm);
  }
  delay(10);
  arretStop();
}

//tracer un Complexe
void complexe(int cm){
  asservissementMoteur();
  circle(cm);
  tournerDroite(cm);
  triangle(cm);
  delay(10);
  arretStop();  
}

//=========================================SETUP&LOOP=========================================//

void setup(){
  Serial.begin(9600);//Lancement du Serial (Courbe, mesures, ...), valeur '9600' par défaut.
  bluetooth.begin(9600);//Lancement du module "Bluetooth", la valeur '9600' est par défaut. (précisée par le prof)
  
  updateTargetTicks();
  
  pinMode(PWM1, OUTPUT); 
  pinMode(PWM2, OUTPUT);
   
  pinMode(IN1AM1, OUTPUT);
  pinMode(IN1AM2, OUTPUT);
  pinMode(IN2AM1, OUTPUT);
  pinMode(IN2AM2, OUTPUT);
  
  pinMode(encoderPinM1A, INPUT);
  pinMode(encoderPinM2A, INPUT);
  
  digitalWrite(encoderPinM1A, HIGH);
  digitalWrite(encoderPinM2A, HIGH);
  // Interruption de l'encodeur A en sortie 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(2), doEncoderM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), doEncoderM2A, CHANGE);
  
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  delay(300);

  timer.setInterval(1000 / frequence_echantillonnage_moteur, asservissementMoteur);
}

void loop(){
  timer.run();
  Avancer(target_cm);
  
  receptionChar(C);
  
  if(C == 'a'){
    Avancer(target_cm);
  }

  if(C == 'r'){
    Reculer(target_cm);
  }

  if(C == 'g'){
    tournerDroite(target_cm);
  }

  if(C == 'd'){
    tournerGauche(target_cm);
  }

  if(C == 's'){
    arretStop();
  }

  if(C == 'c'){
    circle(target_cm);
  }
  
  if(C == 'k'){
    rectangle(target_cm);
  }

  if(C == 't'){
    triangle(target_cm);
  }

  if(C == 'x'){
    complexe(target_cm);
  }
}

//===========================================================================================//
//Informations-Robot-Traceur

//Diamètre des roues : 6.4 cm soit 64 mm
//Circonférence : 2*pi*rayon = pi*diamètre ~ 192 mm = 19.2

//1921 tick par tour (donc 360°) Roue Droite et 1882 pour la roue Gauche