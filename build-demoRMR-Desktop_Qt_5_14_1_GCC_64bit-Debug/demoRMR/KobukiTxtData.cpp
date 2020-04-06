//#include "stdafx.h"
  #include<cstring>
//
#include "KobukiTxtData.h"
#include <string>
#include <vector>
#include <iostream>
#include <istream>
#include <ostream>
#include <iterator>
#include <sstream>
#include <algorithm>


CKobukiTxtData::CKobukiTxtData()
{
}


CKobukiTxtData::~CKobukiTxtData()
{
}

void CKobukiTxtData::openFile(char * filename)
{
    textfile = fopen(filename, "r+");
}

KobukiData CKobukiTxtData::getNewData()
{
    char buff[500];
    memset(buff, 0, 500 * sizeof(char));
    if (fgets((char*)&buff, 500, textfile) != NULL)
    {
        KobukiData temp;
        std::stringstream test(buff);
        std::string out;
        std::getline(test, out, ' ');


        std::string::size_type sz;   // alias of size_t
        temp.timestamp = std::stoi(out, &sz);
        std::getline(test, out, ' ');
        temp.encoderleft = std::stoi(out, &sz);
        std::getline(test, out, ' ');
        temp.encoderright = std::stoi(out, &sz);
        std::getline(test, out, ' ');
        temp.gyroangle = std::stoi(out, &sz);

        return temp;
    }
    return KobukiData();
}

std::vector<KobukiData> CKobukiTxtData::getAllData(const char * filename)
{
    std::vector<KobukiData> outvektor;
    FILE *fp = fopen(filename, "r+");
    char buff[500];
    memset(buff, 0, 500 * sizeof(char));
    int readsize = 0;
    while (fgets((char*)&buff, 500, fp) != NULL)
    {
        KobukiData temp;
        std::stringstream test(buff);
        std::string out;
        std::getline(test, out, ' ');


        std::string::size_type sz;   // alias of size_t
        temp.timestamp= std::stoi(out, &sz);
        std::getline(test, out, ' ');
        temp.encoderleft = std::stoi(out, &sz);
        std::getline(test, out, ' ');
        temp.encoderright = std::stoi(out, &sz);
        std::getline(test, out, ' ');
        temp.gyroangle = std::stoi(out, &sz);
        outvektor.push_back(temp);
    }

    return outvektor;
}
