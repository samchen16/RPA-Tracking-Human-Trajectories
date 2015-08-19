#include "trajectoryvisualizer.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    TrajectoryVisualizer w;
    w.show();

    return a.exec();
}
