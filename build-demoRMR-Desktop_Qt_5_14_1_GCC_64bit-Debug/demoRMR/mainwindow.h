#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include<QMainWindow>
#include<QMutex>
#include<iostream>
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
#include "CKobuki.h"
#include "rplidar.h"

#include "rplidarTxt.h"
#include "KobukiTxtData.h"
#include "map_loader.h"


#define speedStep  0.02

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void robotprocess();
    void laserprocess();
    void processThisLidar(LaserMeasurement &laserData);
    void processThisRobot();

    //PATO
    //lokalizacia
    void processLocalization();
    void setSpeed();
    void setRampSpeed();
    void setAngle(bool clockwise);
    void setAngle();
    void getPossition();

    //PATO
    //uloha2
    void findWay(LaserMeasurement &laserData);

    //PATO uloha3 aj uloha1
    void readLidarSynch();
    void readRobotSynch();
    void writeMap(LaserMeasurement &laserData);
    void saveMap();

    //VITO uloha4
    void getWayPoints(int** map, int xSize, int ySize, double rx, double ry, double finX, double finY);
    void fileToIdealArrayMap(char *filename);
    void evaluateNeighbors(int** map, int xSize, int ySize, queue<Point>* pointsToEvaluate, int x, int y);
    int setDirection(int** map, int xSize, int ySize, int x, int y, int oldDirection);
    void expandObstacles(int** map,int xSize, int ySize);
    bool foundFinish(int** map, Point p, int xSize, int ySize);

    void printToFile(char* file,int** map, int xSize, int ySize, bool points);


    pthread_t robotthreadHandle; // handle na vlakno
    int robotthreadID;  // id vlakna
    static void *robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }

    pthread_t laserthreadHandle; // handle na vlakno
    int laserthreadID;  // id vlakna
    static void *laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();

        return 0;
    }
    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int las_s,  las_recv_len;
    unsigned int las_slen;
    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

    int rob_s,  rob_recv_len;
    unsigned int rob_slen;

    QMutex mutex;
private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_7_clicked();

private:
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     CKobuki robot;
     TKobukiData robotdata;

     //PATO uloha 1. 3.
     CKobukiTxtData robotTxt;
     KobukiData robotTxtData;

     rplidarTxt lidarTxtData;
     LaserMeasurementTxt copyOfLaserTxtData;
     clock_t t_offset;
     double rx,rrx,ry,rry,rfi;
     int** glob_map;
     int** map;
     int map_x,map_y;


     //uloha 4
     map_loader mapLoader;
     TMapArea idealMap;
     queue<Point> wayPoints;
     int** idealArrayMap;
     int widthOfCell = 5; //cm
     //

     int datacounter;
public slots:
     void setUiValues(double robotX,double robotY,double robotFi);
signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo
};

#endif // MAINWINDOW_H
