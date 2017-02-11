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

    QObject::connect(ui->btnFwd, SIGNAL(pressed()), this, SLOT(btnFwdPressed()));
    QObject::connect(ui->btnFwd, SIGNAL(released()), this, SLOT(btnFwdReleased()));
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
}

void MainWindow::btnFwdPressed()
{
    qtRosNode->leftSpeed = 128;
    qtRosNode->rightSpeed = 128;
}

void MainWindow::btnFwdReleased()
{
    qtRosNode->leftSpeed = 0;
    qtRosNode->rightSpeed = 0;
}
