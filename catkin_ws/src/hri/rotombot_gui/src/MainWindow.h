#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <QGraphicsPixmapItem>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem/path.hpp>
#include "QtRosNode.h"

//BUTTER FILTER A Ó B EN X O Y
//cutoff frequency X: 0.7
//                 T: 0.2
//lowpass filter

#define BFA0X 1.0
#define BFA1X 1.161917483671732
#define BFA2X 0.695942755789651
#define BFA3X 0.137761301259893
#define BFB0X 0.374452692590159
#define BFB1X 1.123358077770478
#define BFB2X 1.123358077770478
#define BFB3X 0.374452692590159
#define BFA0T 1.0
#define BFA1T -2.937170728449891
#define BFA2T 2.876299723479332
#define BFA3T -0.939098940325283
#define BFB0T 0.0000037568380197861
#define BFB1T 0.0000112705140593583
#define BFB2T 0.0000112705140593583
#define BFB3T 0.0000037568380197861


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QtRosNode* qtRosNode;

    void setRosNode(QtRosNode* qtRosNode);
    void closeEvent(QCloseEvent *event);

    //Elements for drawing the minirobot
    QGraphicsScene* scRobot;
    QGraphicsScene* scCamera;
    QGraphicsScene* scImu;
    QGraphicsEllipseItem* giBaseEllipse;
    std::vector<QGraphicsLineItem*> giDistSensors;
    std::vector<float> distSensorAngles;
    QGraphicsRectItem* giLeftTire;
    QGraphicsRectItem* giRightTire;
    QGraphicsEllipseItem* giRearTire;
    //Elements for drawing accelerometer arrows
    QGraphicsScene* scAccel;
    QGraphicsLineItem* giAccelX;
    QGraphicsLineItem* giAccelY;
    QGraphicsLineItem* giAccelZ;
    QGraphicsPixmapItem* giCamera;
    QPixmap pmCamera;

    std::vector<double> temp;
    std::vector<double> filteredTemp;

public slots:
    //Slots for signals emitted in the QtRosNode (e.g. a topic is received)
    void updateGraphicsReceived();
    void btnFwdPressed();
    void btnFwdReleased();
    void btnBwdPressed();    
    void btnBwdReleased();
    void btnLeftPressed();
    void btnLeftReleased();
    void btnRightPressed();
    void btnRightReleased();
    void rbSonarsClicked();
    void rbCameraClicked();
    void rbMagnetClicked();
    void rbGyroClicked  ();
    void rbAccelClicked ();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
