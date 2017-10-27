#include <QApplication>

#include "UI/imageprocessingwindow.hpp"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ImageProcessingWindow w;
    w.show();

    return a.exec();
}
