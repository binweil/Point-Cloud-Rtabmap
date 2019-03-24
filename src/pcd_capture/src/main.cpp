#include <QApplication>
#include "viewer.h"

int main(int argc, char** argv)
{
    QApplication app(argc,argv);
    Viewer pcd_viewer;
    pcd_viewer.show();
    return app.exec();
}
