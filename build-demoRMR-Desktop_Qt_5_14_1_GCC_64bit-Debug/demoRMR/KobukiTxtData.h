#pragma once
#include <vector>
#include <string>
#include <iostream>
#include "stdio.h"

typedef struct
{
    int timestamp;
    int encoderright;
    int encoderleft;
    int gyroangle;
}KobukiData;

class CKobukiTxtData
{
public:
     CKobukiTxtData();
    ~CKobukiTxtData();

    //bud mozete pouzit toto - budete si citat postupne po riadku
    FILE *textfile;
    void openFile(char *filename);
    KobukiData getNewData();

    //alebo toto - nacita vsetko naraz, a ulozi do pola
    std::vector<KobukiData> getAllData(const char *filename);

};
