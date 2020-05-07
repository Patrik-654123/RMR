///RIADENIE MOBILNYCH ROBOTOV
///Viktor Luckanic  xluckanic@is.stuba.sk
///Patrik Hercut    xhercut@is.stuba.sk

#include "mainwindow.h"
#include <QApplication>
#include <math.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
