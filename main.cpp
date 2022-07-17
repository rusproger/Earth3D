#include <QApplication>
#include <QTextCodec>
#include <QTimer>
#include <time.h>


#include "map3d.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QTextCodec *codec = QTextCodec::codecForName("CP1251");
    QTextCodec::setCodecForTr(codec);

    QTimer Timer;
    Timer.setInterval(1000);
    Timer.start();

    CMap3d w3d;
    QObject::connect(&Timer, SIGNAL(timeout()), &w3d, SLOT(update()));
    w3d.show();

    return a.exec();
}
