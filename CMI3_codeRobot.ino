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
const float kp_M1 = 1.1; // Coefficient proportionnel (choisis par essais successifs)
const float ki_M1 = 0; // Coefficient intégrateur
const float kd_M1 = 0; // Coefficient dérivateur

//---------Moteur2---------//
const float kp_M2 = 1.1; // Coefficient proportionnel (choisis par essais successifs)
const float ki_M2 = 0; // Coefficient intégrateur
const float kd_M2 = 0; // Coefficient dérivateur

//Initialisation du calculs de l'asservissement PID
int erreurM1 = 0;
int erreurM2 = 0;
float erreurPrecedenteM1 = 0;
float erreurPrecedenteM2 = 0;
float somme_erreurM1 = 0;
float somme_erreurM2 = 0;

//================================DONNER=UNE=CONSIGNE==================================//

double target_cm = 20;
double target_deg = 19.2 * target_cm;

//Moteur Droit
int target_ticks_droit; //plus simple d'asservir en ticks car ce sera toujours un nombre entier //1944
int tick_imp_droit = 1921;

//Moteur Gauche
int target_ticks_gauche; //plus simple d'asservir en ticks car ce sera toujours un nombre  //1787
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
 
 vitesseMoteurDroit = kp_M1 * erreurM1 + kd_M1 * (erreurM1 - erreurPrecedenteM1) + ki_M1 * (somme_erreurM1);
 vitesseMoteurGauche = kp_M2 * erreurM2 + kd_M2 * (erreurM2 - erreurPrecedenteM2) + ki_M2 * (somme_erreurM2);

 if (vitesseMoteurDroit > 255) vitesseMoteurDroit = 255;
 else if (vitesseMoteurDroit < -255) vitesseMoteurDroit = -255;
 if (vitesseMoteurGauche > 255) vitesseMoteurGauche = 255;
 else if (vitesseMoteurGauche < -255) vitesseMoteurGauche = -255;
 TournerRoueDroite (vitesseMoteurDroit);
 TournerRoueGauche (vitesseMoteurGauche);

 float distanceR1 = encoder0PosM1 / 1921 * 19.2;
 float distanceR2 = encoder0PosM2 / 1882 * 19.2;

 Serial.println(encoder0PosM1);
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

//Aller vers la gauche
void tournerGauche(){
  analogWrite(PWM1, vitesseMoteurDroit);
  analogWrite(PWM2, vitesseMoteurGauche);
  digitalWrite(IN1AM1, LOW);
  digitalWrite(IN2AM1, HIGH);
  digitalWrite(IN1AM2, HIGH);
  digitalWrite(IN2AM2, LOW);
}

//Aller vers la droite
void tournerDroite(){
  analogWrite(PWM1, vitesseMoteurDroit);
  analogWrite(PWM2, vitesseMoteurGauche);
  digitalWrite(IN1AM1, HIGH);
  digitalWrite(IN2AM1, LOW);
  digitalWrite(IN1AM2, LOW);
  digitalWrite(IN2AM2, HIGH);
}
/*
void Avancer(int cm) {
  // Calculez la position cible en ticks pour chaque roue
  target_ticks_droit = encoder0PosM1 + cm * 1984 / 19.2;
  target_ticks_gauche = encoder0PosM2 + cm * 1747 / 19.2;

  // Faites avancer le robot jusqu'à ce qu'il atteigne la position cible
  while (abs(encoder0PosM1 - target_ticks_droit) > 10 || abs(encoder0PosM2 - target_ticks_gauche) > 10) {
    // Mettez à jour les vitesses des moteurs en utilisant l'asservissement en position
    asservissementMoteur();

    // Envoyez les commandes aux moteurs
    analogWrite(PWM1, abs(vitesseMoteurDroit));
    analogWrite(PWM2, abs(vitesseMoteurGauche));
    digitalWrite(IN1AM1, vitesseMoteurDroit > 0 ? HIGH : LOW);
    digitalWrite(IN2AM1, vitesseMoteurDroit > 0 ? LOW : HIGH);
    digitalWrite(IN1AM2, vitesseMoteurGauche > 0 ? HIGH : LOW);
    digitalWrite(IN2AM2, vitesseMoteurGauche > 0 ? LOW : HIGH);

    // Attendez un peu avant de mettre à jour les vitesses des moteurs à nouveau
    delay(10);
  }

  // Arrêtez les moteurs
  arretStop();
}
*/
//Aller vers l'avant
void Avancer(int cm){
  int targetRoueDroite = cm * 1921 / 19.2;
  int targetRoueGauche = cm * 1882 / 19.2;  
  while((encoder0PosM1 < targetRoueDroite) && (encoder0PosM2 < targetRoueGauche)){

    asservissementMoteur();

    analogWrite(PWM1, abs(vitesseMoteurDroit));
    analogWrite(PWM2, abs(vitesseMoteurGauche));
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
  arretStop();
}


//Aller vers l'arrière
void Reculer(){
  analogWrite(PWM1, vitesseMoteurDroit);
  analogWrite(PWM2, vitesseMoteurGauche);
  digitalWrite(IN1AM1, LOW);
  digitalWrite(IN2AM1, HIGH);
  digitalWrite(IN1AM2, LOW);
  digitalWrite(IN2AM2, HIGH);
}

//Arrêt du robot
void arretStop(){
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  digitalWrite(IN1AM1, LOW);
  digitalWrite(IN2AM1, LOW);
  digitalWrite(IN1AM2, LOW);
  digitalWrite(IN2AM2, LOW);
}

//=============================================FORMES=============================================//

//tracer un Cercle
void circle(int cm){
  int targetRoueDroite = cm * 1984 / 19.2;
  int targetRoueGauche = cm * 1747 / 19.2;  
  while((encoder0PosM1 < targetRoueDroite) && (encoder0PosM2 < targetRoueGauche)){

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
  arretStop();
}

//tracer un Rectangle
void rectangle(int cm){
  //ligne droite
  Avancer(cm);
  for(int i = 0 ; i < 3 ; i++)
  {
    //tourner de 90°
    digitalWrite(IN1AM1, HIGH);
    digitalWrite(IN2AM1, LOW);
    digitalWrite(IN1AM2, LOW);
    digitalWrite(IN2AM2, HIGH);
    delay(750);
    //avancer en ligne droite
    Avancer(cm);
  } 
}

//tracer un Triangle
void triangle(int cm){
  //Le triangle doit idéalement être équilatéral
  Avancer(cm);  
  //Les 3 angles = 60°
  for(int i = 0 ; i < 2 ; i++){
    analogWrite(PWM1, abs(vitesseMoteurDroit));
    analogWrite(PWM2, abs(vitesseMoteurGauche));
    digitalWrite(IN1AM1, HIGH);
    digitalWrite(IN2AM1, LOW);
    digitalWrite(IN1AM2, LOW);
    digitalWrite(IN2AM2, HIGH);
    delay(1100);
    Avancer(cm);
  }
}

//tracer un Complexe
void complexe(int cm){
  circle(cm);
  //tourner de [...]°;
  triangle(cm);
}
 
//=========================================SETUP&LOOP=========================================//

void setup(){
  Serial.begin(9600);//Lancement du Serial (Courbe, mesures, ...), valeur '9600' par défaut.
  bluetooth.begin(9600);//Lancement du module "Bluetooth", la valeur '9600' est par défaut. (précisée par le prof)
  //====================//
  updateTargetTicks();
  //====================//
  pinMode(PWM1, OUTPUT); 
  pinMode(PWM2, OUTPUT);
  //====================//  
  pinMode(IN1AM1, OUTPUT);
  pinMode(IN1AM2, OUTPUT);
  pinMode(IN2AM1, OUTPUT);
  pinMode(IN2AM2, OUTPUT);
  //====================//
  pinMode(encoderPinM1A, INPUT);
  pinMode(encoderPinM2A, INPUT);
  //====================//
  digitalWrite(encoderPinM1A, HIGH); // Resistance interne arduino ON
  digitalWrite(encoderPinM2A, HIGH);

  // Interruption de l'encodeur A en sortie 0 (pin 2)
  attachInterrupt(0, doEncoderM1A, CHANGE);
  attachInterrupt(1, doEncoderM2A, CHANGE);

  analogWrite(PWM1, 0);
  analogWrite(PWM2, 1);
  delay(300);

  // Interruption pour calcul du PID et asservissement appelée toutes les 10ms
  timer.setInterval(1000 / frequence_echantillonnage_moteur, asservissementMoteur);
}

void loop(){
  
  timer.run();
  updateTargetTicks();
  rectangle(target_cm);
  
  /*
  receptionChar(C);
  
  if(C == 'a'){
    timer.run();
    Avancer(target_cm);
  }

  if(C == 'r'){
    timer.run();
    Reculer();
  }

  if(C == 'g'){
    timer.run();
    tournerDroite();
  }

  if(C == 'd'){
    timer.run();
    tournerGauche();
  }

  if(C == 's'){
    timer.run();
    arretStop();
  }

  if(C == 'c'){
    circle();
    arretStop();
  }
  
  if(C == 'k'){
    timer.run();
    rectangle();
    arretStop();
  }

  if(C == 't'){
    timer.run();
    triangle();
    arretStop();
  }

  if(C == 'x'){
    timer.run();
    complexe();
    arretStop();
  }
  */
}

//===========================================================================================//
//Informations-Robot-Traceur

//Diamètre des roues : 6.4 cm soit 64 mm
//Circonférence : 2*pi*rayon = pi*diamètre ~ 192 mm = 19.2 

//1921 tick par tour (donc 360°) Roue Droite et 1882 pour la roue Gauche