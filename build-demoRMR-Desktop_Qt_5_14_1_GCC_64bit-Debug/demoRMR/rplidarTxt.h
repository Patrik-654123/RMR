////*************************************************************************************
////*************************************************************************************
//// autor Martin Dekan  mail: dekdekan@gmail.com
////-------------------------------------------------------------------------------------
//// co to je:
//// trieda ktora je schopna pracovat s laserovym dialkomerom RPlidar (firma robopeak)
//// implementovana funkcnost je znacne obmedzena a obsahuje len fukncie ktore som bol
//// schopny vytvorit za jeden den, ked som nemal nic lepsie na praci
//// treba ratat s tym,ze niesu osetrene chybove stavy zariadenia
////*************************************************************************************
////*************************************************************************************
#ifndef RPLIDAR_H
#define RPLIDAR_H
#include <stdio.h>
#include <stdlib.h>
//#include "unistd.h"
//#include "pthread.h"
#include "iostream"
#include "fcntl.h"
#include "string.h"
#include <errno.h>
//#include <termios.h>
//#include <unistd.h>
#include <stdio.h>
//#include <sys/time.h>
#include "rplidar.h"

typedef struct
{
    int timestamp;
    int numberOfScans;
    LaserData Data[1000];
}LaserMeasurementTxt;

class rplidarTxt
{
public:

    rplidarTxt(){}
    virtual  ~ rplidarTxt(){}

    FILE *fp;
    //--pripoji sa na subor s ulozenym meranim...
    int connectToFile(char *filename);
    //ziska meranie z filu
    LaserMeasurementTxt getMeasurementFromFile();

};

#endif
