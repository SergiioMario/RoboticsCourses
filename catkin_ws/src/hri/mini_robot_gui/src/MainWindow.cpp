#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    scRobot = new QGraphicsScene(0,0,320,240,ui->graphicsViewRobot);
    giBaseEllipse = scRobot->addEllipse(45, 5, 230, 230, QPen(Qt::gray), QBrush(Qt::gray));
    giLeftTire    = scRobot->addRect(60, 85, 40, 75, QPen(Qt::black), QBrush(Qt::black));
    giRightTire   = scRobot->addRect(220, 85, 40, 75, QPen(Qt::black), QBrush(Qt::black));
    giRearTire    = scRobot->addEllipse(145, 200, 30, 30, QPen(Qt::black), QBrush(Qt::black));
    giDistSensors.push_back(scRobot->addEllipse(45,  120, 10,10, QPen(Qt::darkRed), QBrush(Qt::darkRed)));
    giDistSensors.push_back(scRobot->addEllipse(57,   70, 10,10, QPen(Qt::darkRed), QBrush(Qt::darkRed)));
    giDistSensors.push_back(scRobot->addEllipse(88,   30, 10,10, QPen(Qt::darkRed), QBrush(Qt::darkRed)));
    giDistSensors.push_back(scRobot->addEllipse(133,   8, 10,10, QPen(Qt::darkRed), QBrush(Qt::darkRed)));
    giDistSensors.push_back(scRobot->addEllipse(177,    8, 10,10, QPen(Qt::darkRed), QBrush(Qt::darkRed)));
    giDistSensors.push_back(scRobot->addEllipse(222,   30, 10,10, QPen(Qt::darkRed), QBrush(Qt::darkRed)));
    giDistSensors.push_back(scRobot->addEllipse(253,   70, 10,10, QPen(Qt::darkRed), QBrush(Qt::darkRed)));
    giDistSensors.push_back(scRobot->addEllipse(265, 120, 10,10, QPen(Qt::darkRed), QBrush(Qt::darkRed)));
    ui->graphicsViewRobot->setScene(scRobot);

    scAccel  = new QGraphicsScene(0,0,135,125, ui->graphicsViewAccel);
    giAccelX = scAccel->addLine(67, 62, 22, 101, QPen(Qt::red, 5));
    giAccelY = scAccel->addLine(67, 62, 127, 62, QPen(Qt::green, 5));
    giAccelZ = scAccel->addLine(67, 62, 67, 122, QPen(Qt::blue, 5));
    ui->graphicsViewAccel->setScene(scAccel);

    QIcon icoFwd(":/images/btnUp");
    QIcon icoBwd(":/images/btnDown");
    QIcon icoLeft(":/images/btnLeft");
    QIcon icoRight(":/images/btnRight");
    ui->btnFwd->setIcon(icoFwd);
    ui->btnBwd->setIcon(icoBwd);
    ui->btnLeft->setIcon(icoLeft);
    ui->btnRight->setIcon(icoRight);

    QObject::connect(ui->btnFwd, SIGNAL(pressed()), this, SLOT(btnFwdPressed()));
    QObject::connect(ui->btnFwd, SIGNAL(released()), this, SLOT(btnFwdReleased()));
    QObject::connect(ui->btnBwd, SIGNAL(pressed()), this, SLOT(btnBwdPressed()));
    QObject::connect(ui->btnBwd, SIGNAL(released()), this, SLOT(btnBwdReleased()));
    QObject::connect(ui->btnLeft, SIGNAL(pressed()), this, SLOT(btnLeftPressed()));
    QObject::connect(ui->btnLeft, SIGNAL(released()), this, SLOT(btnLeftReleased()));
    QObject::connect(ui->btnRight, SIGNAL(pressed()), this, SLOT(btnRightPressed()));
    QObject::connect(ui->btnRight, SIGNAL(released()), this, SLOT(btnRightReleased()));
}

MainWindow::~MainWindow()
{
    
    delete ui;
}

void MainWindow::setRosNode(QtRosNode* qtRosNode)
{
    this->qtRosNode = qtRosNode;

    //Connect signals from QtRosNode to MainWindow
    //For example, when ros finishes or when a rostopic is received
    QObject::connect(qtRosNode, SIGNAL(onRosNodeFinished()), this, SLOT(close()));
    QObject::connect(qtRosNode, SIGNAL(updateGraphics()), this, SLOT(updateGraphicsReceived()));
}

//
//SLOTS FOR SIGNALS EMITTED IN THE MAINWINDOW
//
void MainWindow::closeEvent(QCloseEvent *event)
{
    this->qtRosNode->gui_closed = true;
    this->qtRosNode->wait();
    //event->accept();
}

//
//SLOTS FOR SIGNALS EMITTED IN THE QTROSNODE
//

void MainWindow::updateGraphicsReceived()
{
    for(int i=0; i < qtRosNode->sensorDistances.size(); i++)
        if(qtRosNode->sensorDistances[i] == 0)
            giDistSensors[i]->setBrush(QBrush(Qt::red));
        else
            giDistSensors[i]->setBrush(QBrush(Qt::darkRed));

    int x0 = (int)(45.9627*qtRosNode->sensorAccelerometer[0]); //45.96 = 60[pixels]* cos(40°)
    int x1 = (int)(38.5673*qtRosNode->sensorAccelerometer[0]); //38.57 = 60[pixels]* sin(40°)
    int y1 = (int)(60*qtRosNode->sensorAccelerometer[1]);
    int z1 = (int)(60*qtRosNode->sensorAccelerometer[2]);
    
    giAccelX->setLine(67,62, 67-x0, 62 + x1);
    giAccelY->setLine(67,62, 67+y1, 62);
    giAccelZ->setLine(67,62, 67, 62+z1);

    ui->lblLightSensorL->setText("L: " + QString::number((int)(qtRosNode->sensorLightL)));
    ui->lblLightSensorR->setText("R: " + QString::number((int)(qtRosNode->sensorLightR)));
    ui->lblAccelMvnAvg->setText("Accel Mvn Avg: " + QString::number(qtRosNode->accelMvnAvg, 'f', 5));
}

void MainWindow::btnFwdPressed()
{
    float speed = (ui->vsSpeed->value() / 100.0);
    qtRosNode->leftSpeed = speed;
    qtRosNode->rightSpeed = speed;
}

void MainWindow::btnFwdReleased()
{
    qtRosNode->leftSpeed = 0;
    qtRosNode->rightSpeed = 0;
}

void MainWindow::btnBwdPressed()
{
    float speed = (ui->vsSpeed->value() / 100.0);
    qtRosNode->leftSpeed = -speed;
    qtRosNode->rightSpeed = -speed;
}

void MainWindow::btnBwdReleased()
{
    qtRosNode->leftSpeed = 0;
    qtRosNode->rightSpeed = 0;
}

void MainWindow::btnLeftPressed()
{
    float speed = (ui->vsSpeed->value() / 100.0);
    qtRosNode->leftSpeed = -speed;
    qtRosNode->rightSpeed = speed;
}

void MainWindow::btnLeftReleased()
{
    qtRosNode->leftSpeed = 0;
    qtRosNode->rightSpeed = 0;
}

void MainWindow::btnRightPressed()
{
    float speed = (ui->vsSpeed->value() / 100.0);
    qtRosNode->leftSpeed = speed;
    qtRosNode->rightSpeed = -speed;
}

void MainWindow::btnRightReleased()
{
    qtRosNode->leftSpeed = 0;
    qtRosNode->rightSpeed = 0;
}
