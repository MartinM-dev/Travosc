///include

#include <iostream>
#include <thread>       // std::thread
#include <fstream>		// Edition des fichiers de sauvegarde
#include <string>

using namespace std;

#include <sys/select.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <unistd.h>

#include "tinyosc-sfl.h" // Socket OSC

#include "cstdio"
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <time.h>


/// Commade RPI 

/* gpio

 gpio readall : affiche de le tableau des gpio

 gpio -g mode 5 up
 gpio -g mode 6 up
 gpio -g mode 20 up
 gpio -g mode 21 up

*/

//// Stepper ////

/*

Driver actuel : tmc2209, calcul (irmsx2.5)/1,9 (courant max 2,5a) ///https://docarti.fr/reglage-des-vref-des-drivers/

*/

#include "FlexyStepper.h"
FlexyStepper stepper;

float stepperMaxSpeed = 500;		// limite de vitesse normale
const int stepperSpeedLimit = 750;	// limite de vitesse mode PRO attention DANGER
float stepperAcceleration = 250;	// accélération
float positionmax = 0;				// position coté stepper

#include <wiringPi.h> // Pinnage    // ligne de commande dans visualstudio code : gpio export 17 out ; gpio export 18 out ; gpio export 22 out ; gpio export 27 out ; gpio export 5 in ;  gpio export 6 in ;  gpio export 24 in ;  gpio export 25 in

/*
j'utilise les n° de pin BCMD car j'utilise les initialisations suivantes : 
wiringPiSetupGpio (void) This function needs to be called with root privileges.
wiringPiSetupSys (void) This function doesn't needs to be called with root privileges.
Référence : http://wiringpi.com/reference/setup/
*/

const int stepPin = 22;				// Pin de step
const int directionPin = 27;		// Pin de direction
const int capt1 = 5;				// cote stepper 5
const int capt2 = 6;				// cote poulie 6
const int LED = 17;					// Led Reset Stepper 17
const int manualD = 21;				// manual drive 21
const int manualR = 20;				// manual reverse 20

int dir = 0;						//direction de chariot 0=Neutral, 1=Drive, 2=Reverse

float position[100] = { };			// tableau de positions enregistrées

//calcul du temps qui passe pour Reset manuel

time_t a, b;

// osc
int oscPort = 8888 ;				// fonction du changement d'adresse ip et de port non implémentée
char stepperName[] = "/stepper";	// fonction d'identification du stepper dans l'adressage OSC non implémentéee

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////// Récupération des positions dans le fichier ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void getPosition() {

	ifstream positionsStream("positions.txt", ios::in);  // on ouvre en lecture

	if (positionsStream)  // si l'ouverture a fonctionné
	{
		int i = 1;
		string positionS;

		while (getline(positionsStream, positionS)) { // chaque ligne est une position enregistrée

			position[i] = stof(positionS);
			printf("Position %d is %f, ", i, position[i]);

			i = i + 1;
			if (i > 100) {
				break;
			}
		}

		positionsStream.close();
	}
	else
	{
		puts("Loading position failed");
	}

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////// Enregistremenet des positions dans le fichier ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void storePosition() {

	ofstream positionsStream("positions.txt", ios::out | ios::ate);  // on ouvre en ecriture à la fin du fichier

	if (positionsStream)  // si l'ouverture a fonctionné
	{
		int i = 1;
		while (i <= 100) {
			positionsStream << position[i] << endl;
			i++;
		}

		positionsStream.close();
		puts("Stored");
	}
	else
	{
		puts("Unable to open and store position.txt \n");
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////// Reset du stepper ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void resetPosition() {

	puts("Reset Stepper");
	digitalWrite(LED, HIGH);

	const float homingSpeed = 100.0;                        //Homing Speed 
	positionmax = 12000;                                    //en mm
	const int directionTowardHome = -1;                     //Homing Direction vers poullie

	stepper.setAccelerationInMillimetersPerSecondPerSecond(250.0);    //Accel in mm/second

	if (stepper.moveToHomeInMillimeters(directionTowardHome, homingSpeed, positionmax, capt2) != true) {}
	puts("Pulley side-limit reached, Sensor No 2");

	delay(1000);

	stepper.setSpeedInMillimetersPerSecond(homingSpeed);    //Set la vitesse de homing max
	stepper.setTargetPositionInMillimeters(positionmax);    //Tente d'aller à la position max

	while (!stepper.motionComplete() && digitalRead(capt1) == HIGH)
	{
		stepper.processMovement(); // this call moves themotor
		positionmax = stepper.getCurrentPositionInMillimeters();
	}
	puts("Stepper side-limit reached, Sensor No 1");

	stepper.moveToPositionInMillimeters(positionmax);     //reprend la position si dépassement
	delay(250);
	printf("Position limit is %f, homing done \n", positionmax);

	digitalWrite(LED, LOW);
	puts("Ready to go");
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////// décompte du temps pour manual reset //////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void manualReset() {
	puts("Wait for Manual Reset");
	time(&a);
	while (digitalRead(manualD) == LOW && digitalRead(manualR) == LOW) {
		time(&b);
		if (difftime(b, a) > 3) {
			puts("Launch Manual Reset");
			resetPosition();
			break;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// Thread initialisation socket et get osc ////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void getOsc()
{
	//

	char buffer[2048]; // declare a 2Kb buffer to read packet data into

	// open a socket to listen for datagrams (i.e. UDP packets) on port "oscPort"
	const int fd = socket(AF_INET, SOCK_DGRAM, 0);
	fcntl(fd, F_SETFL, O_NONBLOCK); // set the socket to non-blocking
	struct sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(oscPort);
	sin.sin_addr.s_addr = INADDR_ANY;
	bind(fd, (struct sockaddr*)&sin, sizeof(struct sockaddr_in));

	while (true) {

		fd_set readSet;
		FD_ZERO(&readSet);
		FD_SET(fd, &readSet);
		struct timeval timeout = { 1, 0 }; // select times out after 1 second
		if (select(fd + 1, &readSet, NULL, NULL, &timeout) > 0) {
			struct sockaddr sa; // can be safely cast to sockaddr_in
			socklen_t sa_len = sizeof(struct sockaddr_in);
			int len = 0;
			while ((len = (int)recvfrom(fd, buffer, sizeof(buffer), 0, &sa, &sa_len)) > 0) {
				if (tosc_isBundle(buffer)) {
					tosc_bundle bundle;
					tosc_parseBundle(&bundle, buffer, len);
					const uint64_t timetag = tosc_getTimetag(&bundle);
					tosc_message osc;
					while (tosc_getNextMessage(&bundle, &osc)) {
						tosc_printMessage(&osc);
						puts("NO BUNDLE PLEASE");
					}
				}
				else {
					tosc_message osc;
					tosc_parseMessage(&osc, buffer, len);

					///////////////////////// On cherche la valeur int ou float ///////////////////////

					unsigned int val = 0; // initialisation à 0

					if (*tosc_getFormat(&osc) == 'i') { //On ne regarde le message OSC que si un integer est transmit

						int32_t oscV;
						tosc_getValueMessage(&osc, &oscV);
						val = oscV;
					}
					else if (*tosc_getFormat(&osc) == 'f') { //On ne regarde le message OSC que si un integer est transmit

						int32_t oscV;
						tosc_getValueMessage(&osc, &oscV);
						val = oscV;
					}

					///////////////////////// Si il y'a une valeur, on cherche l'adresse ///////////////////////
					if (val > 0) {
						if (!strcmp(tosc_getAddress(&osc), "/Stop")) {
							stepper.setTargetPositionToStop();
							printf("Stop");
							//dir = 0;
						}
						else if (!strcmp(tosc_getAddress(&osc), "/Dir")) {
							if (val == 1) {
								stepper.setTargetPositionInMillimeters(positionmax);
								stepper.setSpeedInMillimetersPerSecond(50);
								puts("osc Drive");
							}
							else if (val == 2) {
								stepper.setTargetPositionInMillimeters(0);
								stepper.setSpeedInMillimetersPerSecond(50);
								puts("osc Reverse");
							}
						}
						else if (!strcmp(tosc_getAddress(&osc), "/Speed")) {
							if (stepperMaxSpeed != val) {
								stepperMaxSpeed = val;
								if (val > stepperSpeedLimit) {
									printf("Speed %d is not allowed", val);
									val = 250;
								}
								stepper.setSpeedInMillimetersPerSecond(stepperMaxSpeed);
								printf("Speed : %d", val); //stepper.setSpeedInMillimetersPerSecond(stepperMaxSpeed);
								printf("speed %f \n", stepper.getCurrentVelocityInMillimetersPerSecond());
							}
						}
						else if (!strcmp(tosc_getAddress(&osc), "/Get")) { //
							if (position[val] && val <= 100) {
								int newTarget = position[val];
								if (newTarget > positionmax) {
									printf("New target %d is over limit : %f \n", newTarget, positionmax);
									newTarget = positionmax;
								}
								stepper.setTargetPositionInMillimeters(newTarget);
								printf("Get memory : %d, set new target : %d \n", val, newTarget);
							}
							else {
								printf("memory no %d, is unvalid \n", val);
							}
						}
						else if (!strcmp(tosc_getAddress(&osc), "/Store")) {
							if (val <= 100) {
								printf("Try to store %f on memory %d \n", stepper.getCurrentPositionInMillimeters(), val);
								position[val] = stepper.getCurrentPositionInMillimeters();
								storePosition();
							}
						}
						else if (!strcmp(tosc_getAddress(&osc), "/Accel")) {
							stepperAcceleration = val;
							stepper.setAccelerationInMillimetersPerSecondPerSecond(stepperAcceleration);
							printf("Acceleration : %d", val);
						}
						else if (!strcmp(tosc_getAddress(&osc), "/LightON")) {
							digitalWrite(LED, HIGH);  // Activé
							printf("LED on");
						}
						else if (!strcmp(tosc_getAddress(&osc), "/LightOFF")) { //resetPosition()
							digitalWrite(LED, LOW);	  // Désactivé
							printf("LED off");
						}
						else if (!strcmp(tosc_getAddress(&osc), "/Reset")) { //
							if (val == 1) {
								resetPosition();
							}
						}
						else { printf("Command %s not found", tosc_getAddress(&osc)); }
					}
					else
					{
						puts("Val > 0");
					}
					printf("\n");
				}
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////// Main ////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
	//initialisation des pin http://wiringpi.com/reference/setup/

	//wiringPiSetupGpio();	//implique root privileges
	wiringPiSetupSys();	//n'implique pas root privileges

	//initialisation de la led et test On Off pour indiquer à l'utilisateur que tout va bien

	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);
	delay(500);
	digitalWrite(LED, LOW);
	delay(500);

	//Stepper

	pinMode(stepPin, OUTPUT);
	pinMode(directionPin, OUTPUT);

	//Capteurs fin de course

	pinMode(capt1, INPUT);// METTRE EN PULLUP
	pullUpDnControl(capt1, PUD_UP);
	pinMode(capt2, INPUT);// METTRE EN PULLUP
	pullUpDnControl(capt2, PUD_UP);

	// Fonctions manuelles

	pinMode(manualD, INPUT);// METTRE EN PULLUP
	pullUpDnControl(manualD, PUD_UP);
	pinMode(manualR, INPUT);// METTRE EN PULLUP
	pullUpDnControl(manualR, PUD_UP);

	/* Petit test pas vraiment utile
	char szHisstname[23];
	if (0 == gethostname(szHisstname, sizeof(szHisstname))) {
		printf("hello from %s! \n", szHisstname);
	}
	else {
		printf("who is it ? ");
	}
	*/

	// Récupération des positions storées

	getPosition();

	//intialisation de stepper

	stepper.connectToPins(stepPin, directionPin);                     //Adressage des pin du stepper
	stepper.setStepsPerMillimeter(27 * 1);                            //convert Step To mm
	stepper.setSpeedInMillimetersPerSecond(250.0);                    //Speed in mm/second
	stepper.setAccelerationInMillimetersPerSecondPerSecond(250.0);    //Accel in mm/second

	resetPosition();                                                   //Reset du Stepper

	stepper.setSpeedInMillimetersPerSecond(100.0);                    //Speed in mm/second
	stepper.setAccelerationInMillimetersPerSecondPerSecond(100.0);    //Accel in mm/second

	// initialisation du thread qui écoute le port OSC

	std::thread first(getOsc);
	first.detach();
	puts("Thread");

	// c'est parti pour la boucle du programme

	while (true) {

		if (digitalRead(manualD) == LOW) {  // Direct drive 

			stepper.setTargetPositionInMillimeters(positionmax);
			stepper.setSpeedInMillimetersPerSecond(50);

			while (digitalRead(manualD) == LOW) {
				if (digitalRead(manualR) == LOW) { // Manual reset si les deux boutons sont présent
					manualReset();
				}
				puts("Manual Drive");
				stepper.processMovement();
			}
			stepper.setTargetPositionToStop();
		}
		else if (digitalRead(manualR) == LOW) { // Direct reverse 

			stepper.setTargetPositionInMillimeters(0);
			stepper.setSpeedInMillimetersPerSecond(50);

			while (digitalRead(manualR) == LOW) {
				if (digitalRead(manualD) == LOW) { // Manual reset si les deux boutons sont présent
					manualReset();
				}
				puts("Manual Reverse");
				stepper.processMovement();
			}
			stepper.setTargetPositionToStop();
		}
		printf("speed %f \n", stepper.getCurrentVelocityInMillimetersPerSecond());
		stepper.processMovement(); // execute un step a chaque fois que la boucle s'execute 
	}
}