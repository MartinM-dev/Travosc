///include

#include <string>

#include <sys/select.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <unistd.h>

#include "tinyosc-sfl.h" // Socket OSC

#include "cstdio"
#include<stdio.h>
#include<sys/socket.h>
#include<arpa/inet.h>

//// Stepper ////

/*

Driver actuel : tmc2209, calcul (irmsx2.5)/1,9 (courant max 2,5a) ///https://docarti.fr/reglage-des-vref-des-drivers/

*/

#include "FlexyStepper.h"
FlexyStepper stepper;

int stepperMaxSpeed = 250;
const int stepperSpeedLimit = 500;
float stepperAcceleration = 250;
float positionmax = 0;

#include <wiringPi.h> // Pinnage     //gpio export 17 out ; gpio export 18 out ; gpio export 22 out ; gpio export 27 out ; gpio export 5 in ;  gpio export 6 in
const int stepPin = 22;
const int directionPin = 27;
const int capt1 = 5; // cote stepper
const int capt2 = 6; //cote poulie
const int LED = 17;
//#define	LED 20

int dir = 0; //0=Neutral, 1=Drive, 2=Reverse

//////////////////////////////////////////////////////////////////////////////// Reset Position Stepper //////////////////////////////////////////////////////////

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
	printf("Position limit is %f, homing done /n", positionmax);

	digitalWrite(LED, LOW);
	puts("Ready to go");
}

//////////////////////////////////////////////////////////////////////////////// Main //////////////////////////////////////////////////////////

int main(void)
{

	char szHisstname[23];
	if (0 == gethostname(szHisstname, sizeof(szHisstname))) {
		printf("hello from %s! \n", szHisstname);
	}
	else {
		printf("who is it ? ");
	}

	//initialisation des pin

	wiringPiSetupSys();

	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);
	delay(1000);
	digitalWrite(LED, LOW);

	pinMode(stepPin, OUTPUT);
	pinMode(directionPin, OUTPUT);

	pinMode(capt1, INPUT);// METTRE EN PULLUP
	pullUpDnControl(capt1, PUD_UP);
	pinMode(capt2, INPUT);// METTRE EN PULLUP
	pullUpDnControl(capt2, PUD_UP);

	//intialisation de stepper

	stepper.connectToPins(stepPin, directionPin);                     //Adressage des pin du stepper
	stepper.setStepsPerMillimeter(27 * 1);                            //convert Step To mm
	stepper.setSpeedInMillimetersPerSecond(250.0);                    //Speed in mm/second
	stepper.setAccelerationInMillimetersPerSecondPerSecond(250.0);    //Accel in mm/second

	resetPosition();                                                   //Reset du Stepper

	stepper.setSpeedInMillimetersPerSecond(100.0);                    //Speed in mm/second
	stepper.setAccelerationInMillimetersPerSecondPerSecond(100.0);    //Accel in mm/second

	//initialisation socket et osc

	char buffer[2048]; // declare a 2Kb buffer to read packet data into

	// open a socket to listen for datagrams (i.e. UDP packets) on port 8888
	const int fd = socket(AF_INET, SOCK_DGRAM, 0);
	fcntl(fd, F_SETFL, O_NONBLOCK); // set the socket to non-blocking
	struct sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(8888);
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
						else if (!strcmp(tosc_getAddress(&osc), "/Pos")) {
							if (val > positionmax) {
								val = positionmax;
							}
							stepper.setTargetPositionInMillimeters(val);
							printf("New Target : %d", val);
							//	dir = 0;
						}
						else if (!strcmp(tosc_getAddress(&osc), "/Speed")) {
							if (stepperMaxSpeed != val) {
								stepperMaxSpeed = val;
								if (val > stepperSpeedLimit) {
									printf("Speed %d is not allowed", val);
									val = 250;
								}
								stepper.setSpeedInMillimetersPerSecond(stepperMaxSpeed);
								printf("Speed : %d", val);
							}
						}
						else if (!strcmp(tosc_getAddress(&osc), "/Accel")) {
							stepperAcceleration = val;
							stepper.setAccelerationInMillimetersPerSecondPerSecond(stepperAcceleration);
							printf("Acceleration : %d", val);
						}
						else if (!strcmp(tosc_getAddress(&osc), "/Save")) {
							puts("Save la position dans la mémoire sélectionnée");
						}
						else if (!strcmp(tosc_getAddress(&osc), "/lightON")) {
							digitalWrite(LED, HIGH);  // Activé
							printf("LED on");
						}
						else if (!strcmp(tosc_getAddress(&osc), "/lightOFF")) { //resetPosition()
							digitalWrite(LED, LOW);	  // Désactivé
							printf("LED off");
						}
						else if (!strcmp(tosc_getAddress(&osc), "/reset")) { //
							if (val == 1) {
								resetPosition();
							}
						}
						else { printf("Command %s not found", tosc_getAddress(&osc)); }

					}
					printf("\n");

				}
			}
		}

		stepper.processMovement(); // execute un step a chaque fois que la boucle s'execute 

	}
}