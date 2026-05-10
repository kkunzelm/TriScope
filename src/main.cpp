#include <QApplication>
#include "ui/MainWindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName("Video Measuring Microscope");
    app.setApplicationVersion("1.0");
    app.setOrganizationName("KHK");

    MainWindow w;
    w.show();

    return app.exec();
}
