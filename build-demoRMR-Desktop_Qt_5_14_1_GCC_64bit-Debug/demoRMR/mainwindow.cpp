///RIADENIE MOBILNYCH ROBOTOV
///Viktor Luckanic  xluckanic@is.stuba.sk
///Patrik Hercut    xhercut@is.stuba.sk

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "map_loader.h"

#include <QPainter>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <cmath>
#include <bits/stdc++.h>
#include <iostream>
#include <fstream>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ipaddress="192.168.1.14";    //robot IP
    // ipaddress = "127.0.0.1";  //simulator IP

    ui->setupUi(this);
    datacounter=0;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);

    //pero-lidar data
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);

    //pero-poloha robota a ciela
    QPen pero1;
    pero1.setWidth(4);
    pero1.setColor(Qt::red);

    //pero-smer obchadzania prekazky
    QPen pero2;
    pero2.setWidth(3);
    pero2.setColor(Qt::blue);

    //parametre okien do ktorych sa vykresluje
    QRect rect;
    QRect rect2;
    rect= ui->frame->geometry();
    rect2=ui->frame_2->geometry();

    //vykreslenie okien
    painter.drawRect(rect);
    painter.drawRect(rect2);

    //vykreslenie dat do okien
    if(updateLaserPicture==1)
    {
        mutex.lock();
        updateLaserPicture=0;

        //aktualny laser scan
        painter.setPen(pero);
        for(int k=0;k<copyOfLaserData.numberOfScans;k++)
        {
            int dist=copyOfLaserData.Data[k].scanDistance/15;
            int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x();
            int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();
            if(rect.contains(xp,yp))
                painter.drawEllipse(QPoint(xp, yp),2,2);
        }

        //globalna mapa
        int xMap,yMap;
        for(int x = 0;x<240;x++)
        {
            for(int y = 0; y<240;y++)
            {
                if(glob_map[x][y]==1)
                {
                    xMap=rect2.topLeft().x()+x;
                    yMap=rect2.topLeft().y()+y;
                    painter.drawEllipse(QPoint(xMap, yMap),1,1);
                }
            }
        }
        painter.setPen(pero1);

        //poloha ciela
        xMap=rect2.topLeft().x()+((int)(rrx*1000)/50);
        yMap=rect2.topLeft().y()+((int)(rry*1000)/50);
        painter.drawEllipse(QPoint(xMap, yMap),1,1);

        //poloha robota
        xMap=rect2.topLeft().x()+((int)(rx*1000)/50);
        yMap=rect2.topLeft().y()+((int)(ry*1000)/50);
        painter.drawEllipse(QPoint(xMap, yMap),1,1);

        //mozne prechody, ako obist prekazku
        painter.setPen(pero2);
        while(!transitions.empty())
        {
            xMap=rect2.topLeft().x()+((int)(transitions.front().first/50));
            yMap=rect2.topLeft().y()+((int)(transitions.front().second/50));
            painter.drawEllipse(QPoint(xMap, yMap),1,1);
            transitions.pop();
        }
        mutex.unlock();
    }
}

void MainWindow::processThisRobot()
{
    //po starte nastav parametre robota
    if (!robotdata.robotOn)
    {
        robotdata.offsetR=robotdata.EncoderRight;
        robotdata.offsetL=robotdata.EncoderLeft;
        robotdata.robotOn=true;

        robotdata.robotX=6;
        robotdata.robotReqX=8.5;
        robotdata.robotY=6;
        robotdata.robotReqY=7.5;

        robotdata.robotReqSpeed=0;
        robotdata.robotSpeed=0;
        robotdata.robotFi=(90*PI)/180; //pri citani dat zo suboru
        robotdata.robotFiDeg=robotdata.robotFi*(180/PI);
        robotdata.robotReqAngle=0;
        robotdata.robotRadius=0;
    }

    //fukncia lokalizacie, odometrie
    processLocalization();

    //vypis hodnot do okien
    if(datacounter%5==0)
    {
     ui->lineEdit_2->setText(QString::number(robotdata.robotX));
     ui->lineEdit_3->setText(QString::number(robotdata.robotY));
     ui->lineEdit_4->setText(QString::number(robotdata.robotFiDeg));
     ui->lineEdit_5->setText(QString::number(robotdata.EncoderLeft));
     ui->lineEdit_6->setText(QString::number(robotdata.EncoderRight));
     ui->lineEdit_7->setText(QString::number(robotdata.robotSpeed));
     ui->lineEdit_10->setText(QString::number(robotdata.robotReqSpeed));
     ui->lineEdit_11->setText(QString::number(robotdata.robotReqAngle));
    }
    datacounter++;
}

void MainWindow::processThisLidar(LaserMeasurement &laserData)
{
    mutex.lock();
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));

    //zapis scan do mapy
    writeMap(laserData);
    //najdi prekazky
    checkWay(laserData);

    updateLaserPicture=1;
    mutex.unlock();
    update();
}

void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}

//call back funkcie tlacidiel
void MainWindow::on_pushButton_9_clicked() //start
{
    //synchornizacia casu citania z textu
    mutex.lock();
    t_offset=clock();
    mutex.unlock();

    //vytvorenie vlakien robota a lidaru
    laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
    robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);

    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    //alokuj globalnu mapu
    glob_map = new int*[240];

    for(int i=0; i<240; i++)
    {
        glob_map[i]=new int[240];

        for(int j=0; j<240; j++)
        {
            glob_map[i][j] = 0;
        }
    }   

//    const QString kkk = mapa[0][0];
//    ui->label_19->setText(QString::pointer(mapa));
//    std::cout<<mapa;
}

void MainWindow::on_pushButton_2_clicked() //dopredu
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(300);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1){}
}

void MainWindow::on_pushButton_3_clicked() //spat
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(-250);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1){}
}

void MainWindow::on_pushButton_6_clicked() //vlavo
{
    std::vector<unsigned char> mess=robot.setRotationSpeed(M_PI/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1){}
}

void MainWindow::on_pushButton_5_clicked() //vpravo
{
   std::vector<unsigned char> mess=robot.setArcSpeed(100,-100);
   if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1){}
}

void MainWindow::on_pushButton_4_clicked() //stop
{
   std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
   if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1){}
}

void MainWindow::on_pushButton_clicked()   //nastav req. hodnoty
{
    robotdata.robotReqX=ui->lineEdit_8->text().toDouble();
    robotdata.robotReqY=ui->lineEdit_9->text().toDouble();
    robotdata.robotReqSpeed=ui->lineEdit_10->text().toShort();
    robotdata.robotReqAngle=ui->lineEdit_11->text().toDouble();
}

void MainWindow::on_pushButton_10_clicked() //pridaj XY do queue
{
    pair<double,double> posXY;
    posXY=make_pair(ui->lineEdit_8->text().toDouble(),ui->lineEdit_9->text().toDouble());

    robotdata.positionQ.push(posXY);
}

void MainWindow::on_pushButton_7_clicked() //LIDAR scan
{
    mutex.lock();
    robotdata.lidarScan=true;
    mutex.unlock();
}

void MainWindow::on_pushButton_11_clicked() //planovanie z robot mapy
{
    // planovanie v nami spravenej mape
    if(map_x != 0) ///////////////////////////////////////////////////////////////////////
    {
        mutex.lock();

        double curX=ui->lineEdit->text().toDouble()*100;
        double curY=ui->lineEdit_12->text().toDouble()*100;
        double reqX=ui->lineEdit_13->text().toDouble()*100;
        double reqY=ui->lineEdit_14->text().toDouble()*100;


        //vytvor pomocne pole
        int** tmpMap = new int*[map_x];

        for(int i=0; i<map_x; i++)
        {
            tmpMap[i]=new int[map_y];

            for(int j=0; j<map_y; j++)
            {
                tmpMap[i][j] = mapa[i][j];
            }
        }

        expandObstacles(tmpMap, map_x, map_y);
        getWayPoints(tmpMap, map_x, map_y, curX, curY, reqX, reqY);

        printToFile("mapa_robot.txt", tmpMap, map_x, map_y, true);




        // dealokuj pomocne pole
        for( int i = 0 ; i < map_x ; i++ )
        {
            delete[] tmpMap[i];
        }
        delete[] tmpMap;




        mutex.unlock();
    }

}

void MainWindow::on_pushButton_8_clicked() //planovanie z idealnej mapy
{
    // planovanie z mapy idealnej z .txt
    int xSize = 120;
    int ySize = 120;

    double curX=ui->lineEdit->text().toDouble()*100;
    double curY=ui->lineEdit_12->text().toDouble()*100;
    double reqX=ui->lineEdit_13->text().toDouble()*100;
    double reqY=ui->lineEdit_14->text().toDouble()*100;

    fileToIdealArrayMap("priestor.txt");


    //vytvor pomocne pole
    int** tmpMap = new int*[xSize];

    for(int i=0; i<xSize; i++)
    {
        tmpMap[i]=new int[ySize];

        for(int j=0; j<ySize; j++)
        {
            tmpMap[i][j] = idealArrayMap[i][j];
        }
    }

    expandObstacles(tmpMap, xSize, ySize);
    getWayPoints(tmpMap, xSize, ySize, curX, curY, reqX, reqY);
    printToFile("mapa_ideal.txt", tmpMap, xSize, ySize, true);

//    printToFile("IDEAL.txt", idealArrayMap, xSize, ySize, false);

    // dealokuj pomocne pole
    for( int i = 0 ; i < xSize; i++ )
    {
        delete[] tmpMap[i];
    }
    delete[] tmpMap;

}

//UDP lidar
void MainWindow::laserprocess()
{
    //precitaj data lidaru z textu
    readLidarSynch();
    //uloz precitanu mapu
    saveMap();

    // Initialize Winsock
    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){}

    int las_broadcastene=1;
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,(char*)&las_broadcastene,sizeof(las_broadcastene));

    //zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);//toto je port z ktoreho pocuvame
    las_si_me.sin_addr.s_addr =htonl(INADDR_ANY);//moze dojst od hocikial..

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);//toto je port na ktory posielame
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());//htonl(INADDR_BROADCAST);
    bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;

    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, las_slen) == -1){}

    LaserMeasurement measure;
    while(1)
    {
        if ((las_recv_len = recvfrom(las_s, (char*)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other,&las_slen)) == -1)
        {
            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);

        //funkcia na spracovanie dat robota
        processThisLidar(measure);
    }
}

//UDP robot
void MainWindow::robotprocess()
{
    //precitaj data lidaru z textu
    readRobotSynch();

    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){}

    char rob_broadcastene=1;
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {}

    usleep(100*1000);

    mess=robot.setSound(440,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {}

    unsigned char buff[50000];

    while(1)
    {
        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other, &rob_slen)) == -1)
        {
            continue;
        }

        int returnval=robot.fillData(robotdata,(unsigned char*)buff);

        if(returnval==0)
        {
            processThisRobot();
        }
    }
}

//funkcia na prechodu na ziadanu poziciu
void MainWindow::getPossition()
{
    double deltaX=robotdata.robotReqX-robotdata.robotX;
    double deltaY=robotdata.robotReqY-robotdata.robotY;
    std::cout<<"deltaX="<<deltaX<<endl;
    std::cout<<"deltaY="<<deltaY<<endl;

    if(abs(deltaX) > 0.1 || abs(deltaY) > 0.1)
    {
        //Patametre linearneho regulatora
        double kRo=300;//100;
        double kAlfa=800;//500;
        double kBeta=-150;//-100;

        double Ro=sqrt(pow(deltaX,2.0)+pow(deltaY,2.0));
        double Alfa=-robotdata.robotFi+atan2(deltaX,deltaY); // pozriet poradie delta
        double Beta=-robotdata.robotFi-Alfa;

        std::cout<<"Ro:"<<Ro*(180/PI)<<endl;
        std::cout<<"Alfa:"<<Alfa*(180/PI)<<endl;
        std::cout<<"Beta:"<<Beta*(180/PI)<<endl;

        //akcny zasah rychlost/polomer
        double v=kRo*Ro;
        double r=(v/(kAlfa*Alfa+kBeta*Beta));

        robotdata.robotRadius=r;

        //rozbeh po rampe/potom reguluj
        //if(v >= 50 && robotdata.robotSpeed < 50)
        if(robotdata.robotSpeed < v)
            robotdata.robotSpeed += 5;
        else
            robotdata.robotSpeed = v;

        //obmedzenia
        if(robotdata.robotSpeed >= 300)
            robotdata.robotSpeed = 300;

        if(abs(robotdata.robotRadius) > 30000)
            robotdata.robotRadius=(robotdata.robotRadius > 0) ? 30000 : -30000;
    }else
    {
        robotdata.robotSpeed=0;
        robotdata.robotRadius=0;

        //nova ziadana hodnota polohy,
        if(robotdata.positionQ.empty())
        {
        }
        else{
            robotdata.robotReqX=robotdata.positionQ.front().first;
            robotdata.robotReqY=robotdata.positionQ.front().second;

            robotdata.positionQ.pop();

            //vypocitaj  nove natocenie robota
            robotdata.robotReqAngle=atan2(robotdata.robotReqY-robotdata.robotY,robotdata.robotReqX-robotdata.robotX)*(180/PI);
            if(robotdata.robotReqAngle < 0)
                robotdata.robotReqAngle += 360;
            robotdata.robotRotated=false;
        }
    }

    std::vector<unsigned char> mess=robot.setArcSpeed(robotdata.robotSpeed,robotdata.robotRadius);
    if(sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1){}
}

//funkcia nastavenie natocenia robota
void MainWindow::setAngle()
{
    int deltaFi=((int)robotdata.robotReqAngle % 360)-((int)robotdata.robotFiDeg % 360);

    if(deltaFi > 180)
        deltaFi=deltaFi-360;
    if(deltaFi <-180)
        deltaFi=deltaFi+360;

    if(abs(deltaFi)>3)
    {
        double maxRotSpeed= (deltaFi >= 0) ? PI/2 : -PI/2;
        double step = (deltaFi >= 0) ? 0.04 : -0.04;
        double K = maxRotSpeed/100.0;
        double reqRotSpeed = K*abs(deltaFi);

        if (abs(reqRotSpeed) > abs(robotdata.robotReqRotSpeed))
            robotdata.robotReqRotSpeed += step;
        else
            robotdata.robotReqRotSpeed=reqRotSpeed;

        if(abs(robotdata.robotReqRotSpeed) > abs(maxRotSpeed))
            robotdata.robotReqRotSpeed=maxRotSpeed;

    }else
    {
        robotdata.robotReqRotSpeed=0;
        robotdata.robotRotated=true;
    }

    std::vector<unsigned char> mess=robot.setRotationSpeed(robotdata.robotReqRotSpeed);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1){}
}

//funkcia lokalizacie odometira
void MainWindow::processLocalization()
{
    //pomocne premene
    double lr,ll,l,dAlfa,Fi_k1;

    //Osetrenie pretecenia encoderov
    //lavy
    if(abs(robotdata.EncoderLeft-robotdata.offsetL)>(numeric_limits<unsigned short>::max()/2))
    {
        if(robotdata.EncoderLeft<robotdata.offsetL)
        {
            ll=(robot.getTickConst())*(robotdata.EncoderLeft+(numeric_limits<unsigned short>::max()-robotdata.offsetL));
        }
        else
        {
            ll=-1*((robot.getTickConst())*((numeric_limits<unsigned short>::max()-robotdata.EncoderLeft)+robotdata.offsetL));
        }
    }
    else
    {
        ll=(robot.getTickConst())*(robotdata.EncoderLeft-robotdata.offsetL);
    }
    //pravy
    if(abs(robotdata.EncoderRight-robotdata.offsetR)>(numeric_limits<unsigned short>::max()/2))
    {
        if((robotdata.EncoderRight<robotdata.offsetR))
        {
            lr=(robot.getTickConst())*(robotdata.EncoderRight+(numeric_limits<unsigned short>::max()-robotdata.offsetR));
        }else
        {
            lr=-1*(robot.getTickConst())*((numeric_limits<unsigned short>::max()-robotdata.EncoderRight)+robotdata.offsetR);
        }
    }else
    {
        lr=(robot.getTickConst())*(robotdata.EncoderRight-robotdata.offsetR);
    }

    //ulozenie hodnot encoderov
    robotdata.offsetR=robotdata.EncoderRight;
    robotdata.offsetL=robotdata.EncoderLeft;

    l=(ll+lr)/2;
    dAlfa=(lr-ll)/robot.getBconst();
    Fi_k1=robotdata.robotFi+dAlfa;

    if(ll==lr)
    {
        //seria priamociarych pohybov
        robotdata.robotX=robotdata.robotX+l*cos(robotdata.robotFi);
        robotdata.robotY=robotdata.robotY+l*sin(robotdata.robotFi);
    }else
    {
        //pohyb po kruznici
        robotdata.robotX=robotdata.robotX+((robot.getBconst()*(lr+ll))/(2*(lr-ll)))*(sin(Fi_k1)-sin(robotdata.robotFi));
        robotdata.robotY=robotdata.robotY-((robot.getBconst()*(lr+ll))/(2*(lr-ll)))*(cos(Fi_k1)-cos(robotdata.robotFi));
    }

    //ulozenie hodnot uhlov
    robotdata.robotFi=Fi_k1;
    robotdata.robotFiDeg=robotdata.robotFi*(180/PI);

    //ulozenie do pomocnych premenych mapy a detegovania prekazok
    mutex.lock();
    rx=robotdata.robotX;
    rrx=robotdata.robotReqX;
    ry=robotdata.robotY;
    rry=robotdata.robotReqY;
    rfi=robotdata.robotFi;
    mutex.unlock();
}

//ciel> ulozi do wayPoints body prechodu
void MainWindow::getWayPoints(int** map, int xSize, int ySize, double rx, double ry, double finX, double finY)
{
    queue<Point> pointsToEvaluate;
    Point pointFin, p;
    int newDirection;
    int oldDirection = 0;

    pointFin.x = (finX)/widthOfCell;
    pointFin.y = (finY)/widthOfCell;

    int x = ((int)rx)/widthOfCell;
    int y = ((int)ry)/widthOfCell;

    //vyprazdni wayPoints
    while(!wayPoints.empty()){
        wayPoints.pop();
    }

    if (x > 0 && y > 0 && x < xSize && y < ySize){
        if (map[x][y] == 0 && map[(int)pointFin.x][(int)pointFin.y] == 0){
            map[x][y] = -1;
            map[(int)pointFin.x][(int)pointFin.y] = 2;
            ui->label_19->setText("OK");
        }
        else{
            //std::cout<<"Moja pozicia alebo ciel v prekazke"<<endl;
            ui->label_19->setText("unreachable positions");
            return;
        }
    }
    else{
        //std::cout<<"Moja pozicia alebo ciel v prekazke"<<endl;
        ui->label_19->setText("invalid or out of map position");
        return;
    }

    pointsToEvaluate.push(pointFin);

    //ohodnocuje postupne bunky kym nepride k -1
    while (!pointsToEvaluate.empty()){

        p = pointsToEvaluate.front();
        pointsToEvaluate.pop();

        evaluateNeighbors(map, xSize, ySize, &pointsToEvaluate,(int)p.x,(int)p.y);

        if (foundFinish(map, p, xSize, ySize))
            break;
    }

    //naplnuje body kadial treba ist
    while (1){

        newDirection = setDirection(map, xSize, ySize, x, y, oldDirection);

        // nie je cesta (ciel je mimo bludiska)
        if (newDirection == 20){
            ui->label_19->setText("impossible path");
            break;
        }

        //narazil na ciel
        if (newDirection == 10){
            p.x = finX/100;
            p.y = finY/100;
            wayPoints.push(p);
            wayPoints.pop();
            break;
        }

        if (newDirection != oldDirection){
            p.x = (x*widthOfCell - widthOfCell/2.0)/100;
            p.y = (y*widthOfCell - widthOfCell/2.0)/100;
            wayPoints.push(p);
        }

        oldDirection = newDirection;

        if (newDirection == 1)
            x += 1;
        else if (newDirection == 2)
            y += 1;
        else if (newDirection == 3)
            x -= 1;
        else
            y -= 1;
    }
}

//urcite smer dalsieho pohybu // 1-vpravo 2-hore 3-vlavo 4-dole
int MainWindow::setDirection(int** map, int xSize, int ySize, int x, int y, int oldDirection){

    int min = 1000000;
    int direction = 20; //zaciatocna


    if (x+1 < xSize && map[x+1][y] > 1 && map[x+1][y] <= min) {

        min = map[x+1][y];
        direction = 1;
    }
    if (x-1 > 0 && map[x-1][y] > 1 &&  map[x-1][y] <= min) {

        if (map[x-1][y] == min && oldDirection == 3){
            direction = 3;
        }
        else if (map[x-1][y] < min){
            min = map[x-1][y];
            direction = 3;
        }
    }
    if (y+1 < ySize && map[x][y+1] > 1 && map[x][y+1] <= min) {

        if (map[x][y+1] == min && oldDirection == 2){
            direction = 2;
        }
        else if (map[x][y+1] < min){
            min = map[x][y+1];
            direction = 2;
        }
    }
    if (y-1 > 0 && map[x][y-1] > 1 && map[x][y-1] <= min) {

        if (map[x][y-1] == min && oldDirection == 4){
            direction = 4;
        }
        else if (map[x][y-1] < min){
            min = map[x][y-1];
            direction = 4;
        }
    }

    //ked je v cieli
    if (min == 2)
        return 10;

    return direction;
}

//4-susedne ohodnotenie
void MainWindow::evaluateNeighbors(int** map,int xSize, int ySize, queue<Point>* pointsToEvaluate , int x, int y){

    Point p;

    if (x+1 < xSize && map[x+1][y] == 0 ) {
        map[x+1][y] = map[x][y]+1;
        p.x = x+1;
        p.y = y;
        (*pointsToEvaluate).push(p);
    }
    if (x-1 >= 0 && map[x-1][y] == 0 ) {
        map[x-1][y] = map[x][y]+1;
        p.x = x-1;
        p.y = y;
        (*pointsToEvaluate).push(p);
    }
    if (y+1 < ySize && map[x][y+1] == 0 ) {
        map[x][y+1] = map[x][y]+1;
        p.x = x;
        p.y = y+1;
        (*pointsToEvaluate).push(p);
    }
    if (y-1 >= 0 && map[x][y-1] == 0 ) {
        map[x][y-1] = map[x][y]+1;
        p.x = x;
        p.y = y-1;
        (*pointsToEvaluate).push(p);
    }
}

//prerobi mapu z TMapArea do podobz 2D pola
void MainWindow::fileToIdealArrayMap(char* filename){

    mapLoader.load_map(filename,idealMap);

    int x1,x2,y1,y2;
    int pred, po;

    //alokuje idealnu mapu
    idealArrayMap = new int*[120];

    for(int i=0; i<120; i++)
    {
        idealArrayMap[i]=new int[120];

        for(int j=0; j<120; j++)
        {
            idealArrayMap[i][j] = 0;
        }
    }

    //prepocita stenu dookola
    for (int i = 0; i < idealMap.wall.numofpoints; ++i) {

        if (idealMap.wall.numofpoints - i > 1){
            pred = i;
            po = i+1;
        }
        else{
            pred = i;
            po = 0;
        }

        // ciara v x smere
        if (idealMap.wall.points[po].point.x != idealMap.wall.points[pred].point.x){

            x1 = ((int) idealMap.wall.points[pred].point.x) / widthOfCell;
            x2 = ((int) idealMap.wall.points[po].point.x) / widthOfCell;

            if (x2 < x1){
                int temp = x2;
                x2 = x1;
                x1 = temp;
            }

            for (int j = x1; j <= x2; j++){
                idealArrayMap[j][((int) idealMap.wall.points[i].point.y) / widthOfCell] = 1;
            }

            // ciara v smere y
        }else{

            y1 = ((int) idealMap.wall.points[pred].point.y) / widthOfCell;
            y2 = ((int) idealMap.wall.points[po].point.y) / widthOfCell;

            if (y2 < y1){
                int temp = y2;
                y2 = y1;
                y1 = temp;
            }

            for (int j = y1; j <= y2; j++){
                idealArrayMap[((int) idealMap.wall.points[i].point.x) / widthOfCell][j] = 1;
            }
        }
    }

    vector<TMapObject>::iterator it = idealMap.obstacle.begin();
    vector<TMapObject>::iterator end = idealMap.obstacle.end();

    //prepocita prekazky vo vnutri
    for (; it != end; it++)
    {
        for (int i = 0; i < it->numofpoints; ++i)
        {
            if (it->numofpoints - i > 1){
                pred = i;
                po = i+1;
            }
            else{
                pred = i;
                po = 0;
            }

            // ciara v x smere
            if (it->points[po].point.x != it->points[pred].point.x){

                x1 = ((int) it->points[pred].point.x) / widthOfCell;
                x2 = ((int) it->points[po].point.x) / widthOfCell;

                if (x2 < x1){
                    int temp = x2;
                    x2 = x1;
                    x1 = temp;
                }

                for (int j = x1; j <= x2; j++){
                    idealArrayMap[j][((int) it->points[i].point.y) / widthOfCell] = 1;
                }

                // ciara v smere y
            }else{

                y1 = ((int) it->points[pred].point.y) / widthOfCell;
                y2 = ((int) it->points[po].point.y) / widthOfCell;

                if (y2 < y1){
                    int temp = y2;
                    y2 = y1;
                    y1 = temp;
                }

                for (int r = y1; r <= y2; r++){
                    idealArrayMap[((int) it->points[i].point.x) / widthOfCell][r] = 1;
                }
            }
        }
    }
}

//rozsirenie prekazky v mape
void MainWindow::expandObstacles(int** map, int xSize, int ySize)
{
    double robotRadius = 20.0; //cm
    int numOfExpandedCells = (int) (robotRadius/widthOfCell);

    //da 2 tam kde je okraj (rezerva)
    for(int x = 0; x < xSize; x++){

        for(int y = 0; y < ySize; y++){

            if (map[x][y] == 1){

                for (int i = -numOfExpandedCells; i < numOfExpandedCells; i++){
                    for (int j = -numOfExpandedCells; j < numOfExpandedCells; j++){

                        if (x+i < xSize && x+i > 0 && y+j < ySize && y+j > 0 &&  map[x+i][y+j] == 0)
                            map[x+i][y+j] = 2;
                    }
                }
            }
        }
    }

    //spravi z 2 (okraje) 1, aby to bolo brane ako prekazka
    for(int x = 0; x < xSize; x++){

        for(int y = 0; y < ySize; y++){

            if (map[x][y] == 2)
                map[x][y] = 1;
        }
    }
}

//citanie data z robota v casovej vzorke
void MainWindow::readRobotSynch()
{
    //casova synchronizacia
    mutex.lock();
    clock_t t_ofset_robot= t_offset;
    mutex.unlock();

    //citanie dat robota z textoveho dokumentu
    robotTxt.openFile("robotdata.txt");
    if (robotTxt.textfile != NULL)
    {
        robotTxtData=robotTxt.getNewData();
        while(1)
        {
            if(robotTxtData.timestamp <= ((clock()-t_ofset_robot)/100))
            {
                robotdata.EncoderLeft=robotTxtData.encoderleft;
                robotdata.EncoderRight=robotTxtData.encoderright;
                robotdata.GyroAngle=robotTxtData.gyroangle;

                processThisRobot();

                robotTxtData=robotTxt.getNewData();
                if(robotTxtData.timestamp==0)
                {
                    std::cout<<"read robot_data finished"<<endl;
                    break;
                }
            }
        }
    }
    //textovy dokument robota precitany
}

//citanie data z lidaru v casovej vzorke
void MainWindow::readLidarSynch()
{
    //casova synchronizacia
    mutex.lock();
    clock_t t_ofset_laser= t_offset;
    mutex.unlock();

    //citanie dat lidaru, z textoveho dokumentu
    lidarTxtData.connectToFile("lidardata.txt");

    if(lidarTxtData.fp!= NULL)
    {
        LaserMeasurementTxt measureTxt;
        LaserMeasurement measure_tmp;
        measureTxt=lidarTxtData.getMeasurementFromFile();
        while(1)
        {
            if(measureTxt.timestamp <= ((clock()-t_ofset_laser)/100))
            {
                for (int i=0;i<sizeof(measureTxt.Data)/sizeof(measureTxt.Data[1]);i++)
                {
                    measure_tmp.Data[i]=measureTxt.Data[i];
                }

                measure_tmp.numberOfScans=measureTxt.numberOfScans;
                processThisLidar(measure_tmp);

                measureTxt=lidarTxtData.getMeasurementFromFile();

                if(measureTxt.numberOfScans==-1)
                {
                    std::cout<<"read lidar_data finished"<<endl;
                    break;
                }
            }
            //textovy dokument precitany
        }
    }

}

//zapis snimky lidaru do globalnej mapy
void MainWindow::writeMap(LaserMeasurement &laserData)
{
    //if(robotdata.lidarScan){
    double x_gi,y_gi,angle;

    for (int i=0;i<laserData.numberOfScans;i++)
    {
        angle=rfi-(laserData.Data[i].scanAngle*(PI/180)); //Pri sumulatore +

        x_gi=(rx*1000)+laserData.Data[i].scanDistance*cos(angle);
        y_gi=(ry*1000)+laserData.Data[i].scanDistance*sin(angle);

        x_gi=x_gi/50;
        y_gi=y_gi/50;

        if(x_gi>239)
            x_gi=239;
        if(y_gi>239)
            y_gi=239;

        glob_map[(int)(x_gi)][(int)(y_gi)]=1;
    }
    glob_map[(int)(rx*20)][(int)(ry*20)]=0;
    //robotdata.lidarScan=false;}
}

//zmensi mapu a ulozi
void MainWindow::saveMap()
{
    int x_first,x_last,y_first,y_last;
    x_first=x_last=y_first=y_last=0;

    //prejdi celu mapu najdi platne data
    for(int i = 0;i <240; i++)
    {
        for(int j = 0; j<240;j++)
        {
            if(glob_map[i][j] == 1 && x_first == 0)
            {   x_first=i;
                x_last=i;
            }else if(glob_map[i][j] == 1 && i > x_last)
                x_last=i;

            if(glob_map[i][j] == 1 && y_first == 0)
                y_first=j;
            else if(glob_map[i][j] == 1 && y_first > j)
                y_first=j;

            if(glob_map[i][j] == 1 && y_last < j)
                y_last=j;
        }
    }

    map_x = x_last-x_first;
    map_y = y_last-y_first;

    //alokuj mapu a napln orezanu mapu
    mapa = new int*[map_x];

    for(int i=0;i<map_x;i++)
    {
        mapa[i]=new int[map_y];
    }

    for(int i=0;i<map_x;i++)
    {
        for(int j=0;j<map_y;j++)
        {
            mapa[i][j]=glob_map[x_first+i][y_first+j];
        }
    }

    printToFile("mapa.txt", mapa, map_x, map_y, false);
    printToFile("mapa_cela.txt", glob_map, 240, 240 , false);

    // dealokuj globalnu mapu
    for( int i = 0 ; i < 240 ; i++ )
    {
        delete[] glob_map[i];
    }
    delete[] glob_map;
}

//deteguj prekazky
void MainWindow::checkWay(LaserMeasurement &laserData)
{
    double angle,crit_dist,scan_dist;
    double dist=sqrt(pow(rx-rrx,2.0)+pow(ry-rry,2.0));
    double alfa=atan2(rry-ry,rrx-rx);
    double alfa_z=atan(b/dist);

    double start_zone=rfi*(180/PI)-(alfa+alfa_z)*(180/PI);
    if (start_zone<0)
        start_zone=360+start_zone;
    if (start_zone>=360)
        start_zone=start_zone-360;

    double stop_zone=start_zone+(2*alfa_z*(180/PI));
    if(stop_zone > 360)
        stop_zone=stop_zone-360;

    //over ci prichadza ku kolizii smerom k cielu
    bool colision=false;
    for (int i=0;i<laserData.numberOfScans;i++)
    {
        angle=laserData.Data[i].scanAngle;
        crit_dist= b/cos(angle*(PI/180));
        scan_dist= laserData.Data[i].scanDistance/1000;
        if((start_zone < stop_zone && angle >= start_zone && angle <= stop_zone)||
                (start_zone > stop_zone && (angle >= start_zone || angle <= stop_zone)))
        {
            if((abs(crit_dist) >= scan_dist && abs(crit_dist) <= dist ) || scan_dist < dist)
            {
                colision=true;
                break;
            }
        }
    }

    if(colision)
    {
        //Najdi prechody
        findTransitions(laserData);
        //Over prechody
        checkTransitions(laserData);
        //Vyber prechod
        choseTransition();
    }
}

//najdi prechody
void MainWindow::findTransitions(LaserMeasurement &laserData)
{
    double dist,dist_b,scan_dist,angle,alfa;
    double scan_dist_1=laserData.Data[0].scanDistance;
    double x_pr,y_pr;

    for (int i=0;i<laserData.numberOfScans;i++)
    {
        if(abs(scan_dist_1-laserData.Data[i].scanDistance) > 2*b*1000)
        {
            angle=rfi-(laserData.Data[i].scanAngle*(PI/180));
            scan_dist= laserData.Data[i].scanDistance;
            if( scan_dist_1 < scan_dist)
            {
                dist=(scan_dist+(scan_dist_1-scan_dist)/2);
                x_pr=(rx*1000)+dist*cos(angle);
                y_pr=(ry*1000)+dist*sin(angle);

                alfa=atan(b*1000/dist);
                dist_b=(b*1000)/sin(alfa);

                x_pr=(rx*1000)+dist_b*cos(angle-alfa);
                y_pr=(ry*1000)+dist_b*sin(angle-alfa);
            }else if (scan_dist_1 >= scan_dist)
            {
                dist=(scan_dist+(scan_dist_1-scan_dist)/2);
                x_pr=(rx*1000)+dist*cos(angle);
                y_pr=(ry*1000)+dist*sin(angle);

                alfa=atan(b*1000/dist);
                dist_b=(b*1000)/sin(alfa);

                x_pr=(rx*1000)+dist_b*cos(angle+alfa);
                y_pr=(ry*1000)+dist_b*sin(angle+alfa);
            }
            transitions.push(make_pair(x_pr,y_pr));
        }
        scan_dist_1=laserData.Data[i].scanDistance;
    }
}

//over prechody
void MainWindow::checkTransitions(LaserMeasurement &laserData)
{
    queue<pair<double,double>> transition_tmp;
    double angle,crit_dist,scan_dist,dist,alfa,alfa_z;

    while(!transitions.empty())
    {
        dist=sqrt(pow(rx-(transitions.front().first/1000),2.0)+pow(ry-(transitions.front().second)/1000,2.0));
        alfa=atan2((transitions.front().second/1000)-ry,(transitions.front().first/1000)-rx);
        alfa_z=atan(b/dist);

        double start_zone=rfi*(180/PI)-(alfa+alfa_z)*(180/PI);
        if (start_zone<0)
            start_zone=360+start_zone;
        if (start_zone>=360)
            start_zone=start_zone-360;

        double stop_zone=start_zone+(2*alfa_z*(180/PI));
        if(stop_zone > 360)
            stop_zone=stop_zone-360;

        //Over ci prichadza ku kolizii
        bool colision=false;
        for (int i=0;i<laserData.numberOfScans;i++)
        {
            angle=laserData.Data[i].scanAngle;
            crit_dist= b/cos(angle*(PI/180));
            scan_dist= laserData.Data[i].scanDistance/1000;
            if((start_zone < stop_zone && angle >= start_zone && angle <= stop_zone)||
               (start_zone > stop_zone && (angle >= start_zone || angle <= stop_zone)))
            {
                if((abs(crit_dist) >= scan_dist && abs(crit_dist) <= dist ) || scan_dist < dist)
                {
                    colision=true;
                    break;
                }
            }
        }
        if(!colision)
        {
            transition_tmp.push((transitions.front()));
        }
        transitions.pop();
    }
    transitions=transition_tmp;
}

//vyber prechod v smere do ciela
void MainWindow::choseTransition()
{
  pair<double,double> transition_tmp;
  double dist,dist_best,pass_x,pass_y;
  pass_x=(transitions.front().first)/1000;
  pass_y=(transitions.front().second)/1000;
  dist_best=sqrt(pow(rx-pass_x,2.0)+pow(ry-pass_y,2.0))+sqrt(pow(pass_x-rrx,2.0)+pow(pass_y-rry,2.0));
  transition_tmp=transitions.front();
  transitions.pop();
  while(!transitions.empty())
  {
      pass_x=(transitions.front().first)/1000;
      pass_y=(transitions.front().second)/1000;
      dist=sqrt(pow(rx-pass_x,2.0)+pow(ry-pass_y,2.0))+sqrt(pow(pass_x-rrx,2.0)+pow(pass_y-rry,2.0));
      if(dist<dist_best)
      {
          transition_tmp=transitions.front();
          dist_best=dist;
      }
      transitions.pop();
  }
  transitions.push(transition_tmp);
}

//zapis do textoveho suboru
void MainWindow::printToFile(char* file,int** map, int xSize, int ySize, bool points){

    ofstream myfile (file);
    if (myfile.is_open())
    {
        for(int i = 0;i<xSize; i++)
        {
            for(int j = 0; j<ySize;j++)
            {
                myfile << map[i][j]<<' ';
            }
            myfile<<";"<<"\n";
        }

        if (points)
        {
            myfile<<"\n\nBODY:\n"<<' ';
            queue<Point> wayPointsCopy = wayPoints;
            QString tmpp = "";

            while(!wayPointsCopy.empty()) {

                myfile<< wayPointsCopy.front().x<<' ';
                myfile<< wayPointsCopy.front().y<<"\n";

                tmpp.append("[ ");
                tmpp.append(QString::number(wayPointsCopy.front().x));
                tmpp.append(" ; ");
                tmpp.append(QString::number(wayPointsCopy.front().y));
                tmpp.append(" ]\n");
                wayPointsCopy.pop();

            }

            if (wayPoints.empty())
                ui->textEdit->setText(" ");
            else{
                const QString tmp = tmpp;
                ui->textEdit->setText(tmp);
            }
        }
        myfile.close();
    }
}

//zplav. algoritmus ukoncovacia podmienka
bool MainWindow::foundFinish(int** map, Point p, int xSize, int ySize){

    if ((int)p.x+1 < xSize){
        if (map[(int)p.x+1][(int)p.y] == -1)
            return true;
    }
    if ((int)p.x-1 > 0){
        if (map[(int)p.x-1][(int)p.y] == -1)
            return true;
    }
    if ((int)p.y+1 < ySize){
        if (map[(int)p.x][(int)p.y+1] == -1)
            return true;
    }
    if ((int)p.y-1 > 0){
        if (map[(int)p.x][(int)p.y-1] == -1)
            return true;
    }


    if ((int)p.x+1 < xSize && (int)p.y+1 < ySize){
        if (map[(int)p.x+1][(int)p.y+1] == -1)
            return true;
    }
    if ((int)p.x+1 < xSize && (int)p.y-1 > 0){
        if (map[(int)p.x+1][(int)p.y-1] == -1)
            return true;
    }
    if ((int)p.x-1 > 0 && (int)p.y+1 < ySize){
        if (map[(int)p.x-1][(int)p.y+1] == -1)
            return true;
    }
    if ((int)p.x-1 > 0 && (int)p.y-1 > 0){
        if (map[(int)p.x-1][(int)p.y-1] == -1)
            return true;
    }
    return false;

}
