/////////////////////////////////////              Stepper Part 1

const int stepPin = 9;
const int directionPin = 8;

FlexyStepper stepper;

int stepperMaxSpeed = 10;
float stepperAcceleration = 250;
float positionmax = 0;
const int capt1 = 11;
const int capt2 = 12;

int dir = 0; //0=Neutral, 1=Drive, 2=Reverse




//////////////////////////////////////////////////////////////////////////////// SETUP //////////////////////////////////////////////////////////

void setup()
{

    Serial.begin(115200);
    ShieldSetup(); //Setup the Ethernet shield
    server.begin(); //starting the server

    Udp.begin(portInt);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(capt1, INPUT_PULLUP);
    pinMode(capt2, INPUT_PULLUP);


    stepper.connectToPins(stepPin, directionPin);                     //Adressage des pin du stepper
    stepper.setStepsPerMillimeter(27 * 1);                            //convert Step To mm
    stepper.setSpeedInMillimetersPerSecond(250.0);                    //Speed in mm/second
    stepper.setAccelerationInMillimetersPerSecondPerSecond(250.0);    //Accel in mm/second

    resetPosition();                                                   //Reset du Stepper

    stepper.setSpeedInMillimetersPerSecond(100.0);                    //Speed in mm/second
    stepper.setAccelerationInMillimetersPerSecondPerSecond(100.0);    //Accel in mm/second
}






//////////////////////////////////////////////////////////////////////////////// Get OSC data //////////////////////////////////////////////////////////

void getdata(OSCMessage& msg, int addrOffset) {

    unsigned int val = 0;

    // Récupération de la valeur
    if (msg.isInt(0)) {
        val = msg.getInt(0);
    } //otherwise it's a floating
    else if (msg.isFloat(0)) {
        val = msg.getFloat(0);
    }

    // Traitement des données reçues      
    if (val >= 0) {

        if (msg.fullMatch("/Stop", addrOffset)) {
            stepper.setTargetPositionToStop();
            dir = 0;
        }

        if (msg.fullMatch("/Dir", addrOffset)) {
            dir = addrOffset;
            switch (dir) {
            case 1:
                stepper.setTargetPositionInMillimeters(positionmax);
                break;
            case 2:
                stepper.setTargetPositionInMillimeters(0);
                break;
            }
        }

        if (msg.fullMatch("/Pos", addrOffset)) {
            if (val > positionmax) {
                val = positionmax;
            }
            stepper.setTargetPositionInMillimeters(val);
            dir = 0;
        }

        if (msg.fullMatch("/Speed", addrOffset)) {
            if (stepperMaxSpeed != val) {
                stepperMaxSpeed = val;
                stepper.setSpeedInMillimetersPerSecond(stepperMaxSpeed);
            }
        }

        if (msg.fullMatch("/Accel", addrOffset)) {
            stepperAcceleration = val;
            stepper.setAccelerationInMillimetersPerSecondPerSecond(stepperAcceleration);
        }

        if (msg.fullMatch("/Reset", addrOffset)) {
            if (val == 1) {
                resetPosition();
                dir = 0;
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////// LOOP //////////////////////////////////////////////////////////

void loop()
{

    EthernetClient client = server.available();
    if (client) {
        affichageWeb(client);
    }

    OSCBundle bundleIN;
    int size;

    if ((size = Udp.parsePacket()) > 0) {
        while (size--)
            bundleIN.fill(Udp.read());


        bundleIN.route("/Slider", getdata);

    }

    stepper.processMovement(); // execute un step a chaque fois que la boucle s'execute ... cherche pas ça reste dans le loop

}


