#include "API-Utils.h"
#include "Peripheral.h"
#include "XRanging.h"

LaserSensor laserLEFT;
LaserSensor laserRIGHT;
LaserSensor laserFRONT;
LaserSensor laserBACK;
LaserSensor laserEXTERNAL;

void XRanging_P::init(void)
{

    isXLaserInit[LEFT] = true;
    isXLaserInit[RIGHT] = true;
    isXLaserInit[FRONT] = true;
    isXLaserInit[BACK] = true;
    isXLaserInit[EXTERNAL] = true;

}

void XRanging_P::init(laser_e laser)
{

    isXLaserInit[laser] = true;

}


int16_t XRanging_P::getRange(laser_e laser)
{

    switch (laser) {

    case LEFT:
        if (isXLaserInit[laser])

            return laserLEFT.startRanging();

        else
            return -1;

        break;

    case RIGHT:
        if (isXLaserInit[laser])

            return laserRIGHT.startRanging();

        else
            return -1;

        break;

    case FRONT:
        if (isXLaserInit[laser])

            return laserFRONT.startRanging();

        else
            return -1;

        break;

    case BACK:
        if (isXLaserInit[laser])

            return laserBACK.startRanging();

        else
            return -1;

        break;


    case EXTERNAL:
        if (isXLaserInit[laser])

            return laserEXTERNAL.startRanging();
//            return NewSensorRange;

        else
            return -1;

        break;

    }

}

void xRangingInit(void)
{

    uint8_t address = 42;

    if (isXLaserInit[LEFT]) {
        delay(10);

        GPIO.init(Pin15, OUTPUT); //LEFT
        GPIO.write(Pin15, STATE_LOW);

    }

    if (isXLaserInit[RIGHT]) {
        delay(10);

        GPIO.init(Pin13, OUTPUT); //RIGHT
        GPIO.write(Pin13, STATE_LOW);

    }

    if (isXLaserInit[FRONT]) {
        delay(10);
        GPIO.init(Pin10, OUTPUT);  // FRONT
        GPIO.write(Pin10, STATE_LOW);

    }

    if (isXLaserInit[BACK]) {
        delay(10);
        GPIO.init(Pin9, OUTPUT);  //BACK
        GPIO.write(Pin9, STATE_LOW);

    }


    if (isXLaserInit[EXTERNAL]){
//        delay(10);
//        GPIO.init(Pin8, OUTPUT);  //BACK
//        GPIO.write(Pin8, STATE_LOW);

    }


    if (isXLaserInit[LEFT]) {

        delay(30);

        GPIO.write(Pin15, STATE_HIGH);
        delay(30);

        laserLEFT.init();
        delay(30);

        laserLEFT.setAddress(address);

        address++;

    }

    if (isXLaserInit[RIGHT]) {

        delay(30);

        GPIO.write(Pin13, STATE_HIGH);
        delay(30);

        laserRIGHT.init();
        delay(30);
        laserRIGHT.setAddress(address);

        address++;
    }

    if (isXLaserInit[FRONT]) {

        delay(30);

        GPIO.write(Pin10, STATE_HIGH);
        delay(30);

        laserFRONT.init();
        delay(30);
        laserFRONT.setAddress(address);

        address++;

    }

    if (isXLaserInit[BACK]) {
        delay(30);

        GPIO.write(Pin9, STATE_HIGH);
        delay(30);

        laserBACK.init();
        delay(30);
        laserBACK.setAddress(address);

        address++;

    }


    if (isXLaserInit[EXTERNAL]) {

//        delay(30);
//
//        GPIO.write(Pin8, STATE_HIGH);
//        delay(30);

        laserEXTERNAL.init();
//        useRangingSensor=true;
//        delay(30);
//
//        laserEXTERNAL.setAddress(address);
//
//        address++;

    }



    delay(30);

}

XRanging_P XRanging;

