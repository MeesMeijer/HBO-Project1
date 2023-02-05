// defines pins numbers
// #include <List.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>


enum Dir
{
    vooruit = HIGH,
    achteruit = LOW
};

enum States{
    WAITING = 2,
    VALS = 3,
    EIND = 4,
    STARTING = 5,
    LOADING = 6, 
    RESET = 7,
    ROUND_WINNER = 8,
    GAME_WINNER = 9,
    NO_INPUT = 10
};

struct ResetBtn{
    int pin;
    bool pressed = false;
    bool lastState; 
    unsigned long lastPress = 0;

    ResetBtn() {}
        
    ResetBtn(int pin){
        this->pin = pin;
    }
    
    void init(){
        Serial.println("Init ResetBtn.. ");
    }

    void reset(bool hard)
    {
        this->lastPress = 0;
        this->pressed = false;
    }
};

struct PushBtn: public ResetBtn{
    int ledPin; 
    bool ledOn = false;
    
    PushBtn(int pin, int ledPin)
    {
        this->pin = pin;
        this->ledPin = ledPin;
    }

    void init(int id)
    {
        Serial.println("Init push btn.. id:"+(String)id);
        pinMode(this->ledPin, OUTPUT);
    }

    void turnon(){ 
        Serial.println("Push turn on");
        this->ledOn = true; 
        digitalWrite(this->ledPin, HIGH); 
    }
    
    void turnoff(){ 
        Serial.println("Push turn off");
        this->ledOn = false; 
        digitalWrite(this->ledPin, LOW); 
    }
};

struct RGBLed
{
    int rPin;
    int gPin;
    int bPin;

    RGBLed(int rPin, int gPin, int bPin)
    {
        this->rPin = rPin;
        this->gPin = gPin;
        this->bPin = bPin;
    }

    void init()
    {
        Serial.println("Init Rgb...");

        pinMode(this->rPin, OUTPUT);
        pinMode(this->bPin, OUTPUT);
        pinMode(this->gPin, OUTPUT);

        this->turnoff();
    }

    void setGreen()
    {
        Serial.println("Setting Rgb to green");
        analogWrite(this->rPin, 0);
        analogWrite(this->gPin, 255);
        analogWrite(this->bPin, 0);
    };

    void setRed()
    {
        Serial.println("Setting Rgb to Red");
        analogWrite(this->rPin, 255);
        analogWrite(this->gPin, 0);
        analogWrite(this->bPin, 0);
    };

    void setBlue()
    {
        Serial.println("Setting Rgb to Blue");
        analogWrite(this->rPin, 0);
        analogWrite(this->gPin, 0);
        analogWrite(this->bPin, 255);
    };

    void turnoff()
    {
        Serial.println("Turning rgb off");
        analogWrite(this->rPin, 0);
        analogWrite(this->gPin, 0);
        analogWrite(this->bPin, 0);
    }

    void reset() { turnoff(); }
};

struct LDRSensor
{
    int pin;
    bool blocked;
    unsigned long lastBlock; 
    int treshold = 900;

    LDRSensor(int pin)
    {
        this->pin = pin;
    }

    void init(int id)
    {   
        Serial.println("Init LDR Sensor.. id:"+(String)id);
        pinMode(this->pin, INPUT);
    }

    bool isBlocked()
    {
        int value = analogRead(this->pin);

        if (value > this->treshold){            
            this->lastBlock = 0;
            this->blocked = false;
            return false; 
        }
        
        this->lastBlock = millis();
        this->blocked = true;
        return true;
    }

    void reset(bool hard)
    {
        this->blocked = false;
        this->lastBlock = 0;
    }
};

struct Motor
{
    int dirPin;
    int stepPin;

    int endLimitSwitchPin;
    int startLimitSwitchPin;

    bool isAtEnd;

    Motor(int dirPin, int stepPin, int startPin, int stopPin)
    {
        this->dirPin = dirPin;
        this->stepPin = stepPin;

        this->startLimitSwitchPin = startPin;
        this->endLimitSwitchPin = stopPin;
    }

    void init(int id)
    {
        Serial.println("Init motor.. id:"+(String)id);
        pinMode(this->dirPin, OUTPUT);
        pinMode(this->stepPin, OUTPUT);

        pinMode(this->endLimitSwitchPin, INPUT);
        pinMode(this->startLimitSwitchPin, INPUT);
    }

    bool reset(bool hard)
    {
        Serial.print("Reset: Going to start");

        this->setDirection(Dir::achteruit);

        while (!this->startTouched()) this->moveStep();

        Serial.println(". done");

        this->isAtEnd = false; 
        return true;
    }

    bool move()
    {
        Serial.println("Moving motor.. ");

        this->setDirection(Dir::vooruit);
        for (int x = 0; x < 200; x++)
        {
            if (!endTouched())
                this->moveStep();
            else
            {
                Serial.println("Dino touches end of track");
                this->isAtEnd = true;
                return false;
            }
        }

        return true;
    }

    bool endTouched() { return digitalRead(this->endLimitSwitchPin); }

    bool startTouched() { return digitalRead(this->startLimitSwitchPin); };

    void moveStep()
    {
        digitalWrite(this->stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(this->stepPin, LOW);
        delayMicroseconds(500);
    };

    void setDirection(Dir dir) { digitalWrite(this->dirPin, dir); }
};

struct Player
{
    int id;
    int score = 0;

    Motor *motor;
    LDRSensor *Ldr;
    PushBtn *Btn;

    Player(int id, Motor* Motor, LDRSensor* Ldr, PushBtn* Btn)
    {
        this->motor = Motor;
        this->Ldr = Ldr;
        this->Btn = Btn;
        this->id = id;
    }

    void init()
    {
        Serial.println("Init player.. id:" + (String)this->id);
        this->Btn->init(id);
        this->Ldr->init(id);
        this->motor->init(id);
    }

    void reset(bool hard)
    {
        Serial.println("Resetting player.. id:"+ (String)this->id);
        this->Btn->reset(hard);
        this->Ldr->reset(hard);
        if (hard) this->motor->reset(hard);
    }
};

struct LCD{
    LiquidCrystal_I2C* driver;

    int state;

    LCD(LiquidCrystal_I2C* driver){
        this->driver = driver;
    }   

    void clear(States newState){
        if (this->state == newState) return; 

        this->driver->setCursor(0,0);
        this->driver->print("                ");
        this->driver->setCursor(0,1);
        this->driver->print("                ");
        
        this->state = newState;
    }

    void init(){
        Serial.println("Init Lcd.. ");
        this->driver->init();
        this->driver->backlight();
        this->driver->begin(2, 16);

        this->clear(States::LOADING);

        this->driver->setCursor(0,0);
        this->driver->print("Loading game.. ");
        this->driver->setCursor(0,1);
        this->driver->print("# - # - # - # - #");
    }

    void startGame(){
        this->clear(States::STARTING);

        this->driver->setCursor(0,0);
        this->driver->print("The Game Begins");
        this->driver->setCursor(0,1);
        this->driver->print("Click on blue");
    }

    void resetGame(){
        this->clear(States::RESET);

        this->driver->setCursor(0,0);
        this->driver->print("Resetting.. ");
        this->driver->setCursor(0,1);
        this->driver->print("");
    }

    void roundWinner(Player* winner, float winnerTime, float loserTime){
        this->clear(States::ROUND_WINNER);

        this->driver->setCursor(0,0);
        this->driver->print("Round won by "+ (String)winner->id);
        this->driver->setCursor(0,1);
        this->driver->print((String)winnerTime + " vs " + (String)loserTime + " sec");
    }

    void gameWinner(Player* winner, Player* loser){
        this->clear(States::GAME_WINNER);

        this->driver->setCursor(0,0);
        this->driver->print("We have a winner!");
        this->driver->setCursor(0,1);
        this->driver->print("Player "+ (String)winner->id +" has won!");
    }

    void valsSpel(Player* cheater){
        this->clear(States::VALS);

        this->driver->setCursor(0,0);
        this->driver->print("Player "+ (String)cheater->id + " deed ");
        this->driver->setCursor(0,1);
        this->driver->print("aan vals spel.. ");
    }

    // waiting for player there hand.. 
    void waiting(){
        this->clear(States::WAITING);

        this->driver->setCursor(0,0);
        this->driver->print("Waiting for ");
        this->driver->setCursor(0,1);
        this->driver->print("Players..");
    }

    void noInput(){
        this->clear(States::NO_INPUT);

        this->driver->setCursor(0,0);
        this->driver->print("Timeout.. ");
        this->driver->setCursor(0,1);
        this->driver->print("# - # - # - #");
    }
};

struct GAME{
    Player *player0; 
    Player *player1; 
    LCD* Lcd;
    ResetBtn* Reset; 
    RGBLed* Rgb; 

    bool hasWinner; 
    unsigned long startTime = 0; 

    GAME(Player* player0, Player* player1, LCD* Lcd, ResetBtn* Reset, RGBLed* Rgb){
        this->player0 = player0;
        this->player1 = player1;
        this->Lcd = Lcd;
        this->Reset = Reset;
        this->Rgb = Rgb; 
    }

    void reset(bool hard){
        this->player0->reset(hard);
        this->player1->reset(hard);

        hasWinner = false;
    }

    long generateRandomWaitingTime(){
        return millis() + random(3, 10) * 1e3; 
    }

    void loop(){
        
        if (this->Reset->pressed){
            this->reset(true);
            this->Reset->reset(true);
            return; 
        }

        bool play1 = this->player1->Ldr->isBlocked();
        bool play0 = this->player0->Ldr->isBlocked();

        if (!play0 || !play1){
            this->Lcd->waiting();
            return; 
        }

        this->Lcd->startGame();
        this->hasWinner = false;

        
        unsigned long waitTime = this->generateRandomWaitingTime();

       
        while (millis() < waitTime){
            this->Rgb->setRed();
            delay(300);
            this->Rgb->turnoff();
            delay(300);
            this->Rgb->setRed();
        }

        this->Rgb->setBlue();
        this->startTime = millis();

        while (!hasWinner){
            
            if (this->Reset->pressed) break;

            if ((startTime + 10e3) > millis()) {
                
                this->Lcd->noInput();
                this->Rgb->setRed();

                this->hasWinner = true; 
                delay(5e3);
                break; 
            }
            
            bool value0 = this->player0->Ldr->isBlocked();
            bool value1 = this->player1->Ldr->isBlocked();

            unsigned long pressTime0 = this->player0->Btn->lastPress;
            unsigned long pressTime1 = this->player1->Btn->lastPress;


            if ( (!value0 && !value1) && (pressTime0 != 0 && pressTime1 != 0)){
                
                if ((this->player0->Ldr->lastBlock < (this->startTime + 200) )){
                    this->player0->score -= 1;
                    return this->Lcd->valsSpel(this->player0);
                }else if ((this->player1->Ldr->lastBlock < (this->startTime + 200) ) ){
                    this->player1->score -= 1;
                    return this->Lcd->valsSpel(this->player1);
                }

                if (pressTime0 < pressTime1){

                    this->hasWinner = true;
                    this->player0->score += 1; 

                    bool end = this->player0->motor->move();
                    if (end){ 
                       return this->Lcd->gameWinner(this->player0, this->player1); 
                    }
                    this->Lcd->roundWinner(this->player0, (millis() - pressTime0), (millis() - pressTime1)); 
                    break;
                }

                this->hasWinner = true;
                this->player0->score += 1;
                bool end = this->player1->motor->move();
                if (end){ 
                    return this->Lcd->gameWinner(this->player1, this->player0); 
                }

                this->Lcd->roundWinner(this->player1, (millis() - pressTime1), (millis() - pressTime0));
                break; 
            }
        }

        if (this->Reset->pressed) return this->loop();
        delay(4e3);
        this->hasWinner = false;
    }

};

RGBLed RGB(7, 5, 6);
ResetBtn Reset(2);

LiquidCrystal_I2C lcd(0x27, 20, 21);
LCD Lcd(&lcd);

// De motoren
Motor Motor1(11, 34, 26, 27);
Motor Motor0(12, 35, 28, 29);

// de dome push buttons
PushBtn push1(18, 40);
PushBtn push0(3, 41);

// De LDr (light dependent resistor)
LDRSensor LDR1(A1);
LDRSensor LDR0(A0);

// beide players boven en onder
Player Player1(1, &Motor1, &LDR1, &push1);
Player Player0(0, &Motor0, &LDR0, &push0);

// de game
GAME Game(&Player0, &Player1, &Lcd, &Reset, &RGB);

// dit is een interrupt voor de reset switch
void BtnresetPin()
{
    if ((Reset.lastPress + 2e3) < millis())
    {
        Serial.println("resetten");
        Reset.lastState = Reset.pressed;
        Reset.pressed = true;
        Reset.lastPress = millis();
    };
}

void Btn0()
{
    if (((Player0.Btn->lastPress + 1e3) < millis() )&& !Player0.Btn->pressed)
    {
        Player0.Btn->lastState = Player0.Btn->pressed; 
        Serial.println("Player 0 pressed");
        Player0.Btn->lastPress = millis();
        Player0.Btn->pressed = true; 
    };
}

void Btn1()
{
    if (((Player1.Btn->lastPress + 1e3) < millis()) && !Player1.Btn->pressed)
    {
        Player1.Btn->lastState = Player1.Btn->pressed; 
        Serial.println("Player 1 pressed");
        Player1.Btn->lastPress = millis();
        Player1.Btn->pressed = true; 
    };
}

void checkMotors(Motor* motor0, Motor* motor1){
    motor0->reset(true);
    motor1->reset(true);

    motor0->setDirection(Dir::vooruit);
    motor1->setDirection(Dir::vooruit);

    while (!motor0->endTouched() || !motor1->endTouched()){
        digitalWrite(motor0->stepPin, HIGH);
        digitalWrite(motor1->stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(motor0->stepPin, LOW);
        digitalWrite(motor1->stepPin, LOW);
        delayMicroseconds(500);
    }

    motor0->setDirection(Dir::achteruit);
    motor1->setDirection(Dir::achteruit);
    
    while (!motor0->startTouched() || !motor1->startTouched()){
        digitalWrite(motor0->stepPin, HIGH);
        digitalWrite(motor1->stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(motor0->stepPin, LOW);
        digitalWrite(motor1->stepPin, LOW);
        delayMicroseconds(500);
    }
}

void resetMotors(Motor* motor0, Motor* motor1){
    motor0->setDirection(Dir::achteruit);
    motor1->setDirection(Dir::achteruit);


    while (!motor0->startTouched() || !motor0->startTouched()){

        if (!motor0->startTouched()) digitalWrite(motor0->stepPin, HIGH);
        if (!motor1->startTouched()) digitalWrite(motor1->stepPin, HIGH);
        
        delayMicroseconds(500);

        if (!motor0->startTouched()) digitalWrite(motor0->stepPin, LOW);
        if (!motor1->startTouched()) digitalWrite(motor1->stepPin, LOW);
        
        delayMicroseconds(500);
    }

    Serial.println("Motors reset done.");
}


void setup()
{
    Serial.begin(115200);

    RGB.init();
    Lcd.init();

    Player0.init();
    Player1.init();

    attachInterrupt(digitalPinToInterrupt(push0.pin), Btn0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(push1.pin), Btn1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Reset.pin), BtnresetPin, CHANGE);

    resetMotors(&Motor0, &Motor1);
}


void loop()
{   

    Game.loop();
    // while (!Motor0.isAtEnd || !Motor1.isAtEnd){
    //     Motor0.move();
    //     Motor0.move();
    //     Motor1.move();
    //     Motor1.move();
    // }

    // Motor0.reset();
    // Motor1.reset();
    // Serial.println("----------------------------------------------------------");
    // Serial.println("Push0: " + (String)push0.pressed  + " value: "+ (String)digitalRead(push0.pin));
    // Serial.println("Push1: " + (String)push1.pressed  + " value: "+ (String)digitalRead(push1.pin));
    // // Serial.println("----------------------------------------------------------");
    // Serial.println("");
    // Serial.println("Ldr0: isBlocked: " + (String)LDR0.isBlocked() + " value: "+ (String)analogRead(LDR0.pin));
    // Serial.println("Ldr1: isBlocked: " + (String)LDR1.isBlocked() + " value: "+ (String)analogRead(LDR1.pin));
    // Serial.println("----------------------------------------------------------");

}
