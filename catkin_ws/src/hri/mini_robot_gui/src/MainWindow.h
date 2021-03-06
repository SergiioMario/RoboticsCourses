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
    QGraphicsEllipseItem* giBaseEllipse;
    std::vector<QGraphicsEllipseItem*> giDistSensors;
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
    void chkShowCameraChanged(int value);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
