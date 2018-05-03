#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    scRobot  = new QGraphicsScene(0,0,320,240,ui->graphicsViewRobot);
    giBaseEllipse = scRobot->addEllipse(155, 112, 10, 16, QPen(Qt::gray), QBrush(Qt::gray));
    //giLeftTire    = scRobot->addRect(60, 85, 40, 75, QPen(Qt::black), QBrush(Qt::black));
    //giRightTire   = scRobot->addRect(220, 85, 40, 75, QPen(Qt::black), QBrush(Qt::black));
    //giRearTire    = scRobot->addEllipse(145, 200, 30, 30, QPen(Qt::black), QBrush(Qt::black));
    giDistSensors.push_back(scRobot->addLine(160, 120, 160, 240, QPen(Qt::blue, 5)));
    giDistSensors.push_back(scRobot->addLine(160, 120, 320, 240, QPen(Qt::blue, 5)));
    giDistSensors.push_back(scRobot->addLine(160, 120, 320, 120, QPen(Qt::blue, 5)));
    giDistSensors.push_back(scRobot->addLine(160, 120, 320,   0, QPen(Qt::blue, 5)));
    giDistSensors.push_back(scRobot->addLine(160, 120, 160,   0, QPen(Qt::blue, 5)));
    giDistSensors.push_back(scRobot->addLine(160, 120,   0,   0, QPen(Qt::blue, 5)));
    giDistSensors.push_back(scRobot->addLine(160, 120,   0, 120, QPen(Qt::blue, 5)));
    giDistSensors.push_back(scRobot->addLine(160, 120,   0, 240, QPen(Qt::blue, 5)));
    ui->graphicsViewRobot->setScene(scRobot);
    distSensorAngles.push_back(-M_PI);
    distSensorAngles.push_back(-M_PI + 0.7);
    distSensorAngles.push_back(-M_PI/2);
    distSensorAngles.push_back(-0.7);
    distSensorAngles.push_back(0);
    distSensorAngles.push_back(0.7);
    distSensorAngles.push_back(M_PI/2);
    distSensorAngles.push_back(M_PI - 0.7);
    ui->rbSonars->setChecked(true);

    scImu  = new QGraphicsScene(0,0,320,240, ui->graphicsViewRobot);
    giAccelX = scImu->addLine(52, 47, 22, 90, QPen(Qt::red, 5));
    giAccelY = scImu->addLine(52, 47, 105, 47, QPen(Qt::green, 5));
    giAccelZ = scImu->addLine(52, 47, 52, 5, QPen(Qt::blue, 5));

    scCamera = new QGraphicsScene(0,0,320,240,ui->graphicsViewRobot);
    giCamera = scCamera->addPixmap(pmCamera);

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
    QObject::connect(ui->rbSonars, SIGNAL(clicked()), this, SLOT(rbSonarsClicked()));
    QObject::connect(ui->rbCamera, SIGNAL(clicked()), this, SLOT(rbCameraClicked()));
    QObject::connect(ui->rbMagnet, SIGNAL(clicked()), this, SLOT(rbMagnetClicked()));
    QObject::connect(ui->rbGyro, SIGNAL(clicked()), this, SLOT(rbGyroClicked  ()));
    QObject::connect(ui->rbAccel, SIGNAL(clicked()), this, SLOT(rbAccelClicked ()));

    for(int i=0; i < 4; i++)
    {
	temp.push_back(0);
	filteredTemp.push_back(0);
    }
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
    //Update sonars graphics is the corresponding radio button is checked.
    if(ui->rbSonars->isChecked())
       for(int i=0; i < qtRosNode->sensorDistances.size(); i++)
       {
	   float x = -qtRosNode->sensorDistances[i]*120*sin(distSensorAngles[i]) + 160;
	   float y = -qtRosNode->sensorDistances[i]*120*cos(distSensorAngles[i]) + 120;
	   giDistSensors[i]->setLine(160, 120, x, y);
	   if(qtRosNode->sensorDistances[i] < 0.15)
	       giDistSensors[i]->setPen(QPen(Qt::red, 5));
	   else
	       giDistSensors[i]->setPen(QPen(Qt::blue, 5));
       }

    
    if(ui->rbCamera->isChecked())
    {
	pmCamera.loadFromData(qtRosNode->imgCompressed.data(), qtRosNode->imgCompressed.size(), "JPG");
	giCamera->setPixmap(pmCamera);
    }

    //Acceloremeter data come in the order: AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ.
    int x0;
    int x1;
    int y1;
    int z1;

    if(ui->rbAccel->isChecked())
    {
	x0 = (int)(84.2649*qtRosNode->sensorAccelerometer[0]); //84.2649 = 110[pixels]* cos(40°)
	x1 = (int)(70.7066*qtRosNode->sensorAccelerometer[0]); //70.7066 = 110[pixels]* sin(40°)
	y1 = (int)(110*qtRosNode->sensorAccelerometer[1]);                                     
	z1 = (int)(110*qtRosNode->sensorAccelerometer[2]);                                     
    }
    if(ui->rbGyro->isChecked())
    {//Angular speed come in degrees per second.
	x0 = (int)(84.2649*qtRosNode->sensorAccelerometer[3]/60.0); //84.2649 = 110[pixels]* cos(40°)
	x1 = (int)(70.7066*qtRosNode->sensorAccelerometer[3]/60.0); //70.7066 = 110[pixels]* sin(40°)
	y1 = (int)(110*qtRosNode->sensorAccelerometer[4]/60.0);                                     
	z1 = (int)(110*qtRosNode->sensorAccelerometer[5]/60.0);                                     
    }
    if(ui->rbMagnet->isChecked())
    {
	x0 = (int)(84.2649*qtRosNode->sensorAccelerometer[6]/0.3); //84.2649 = 110[pixels]* cos(40°)
	x1 = (int)(70.7066*qtRosNode->sensorAccelerometer[6]/0.3); //70.7066 = 110[pixels]* sin(40°)
	y1 = (int)(110*qtRosNode->sensorAccelerometer[7]/0.3);                                     
	z1 = (int)(110*qtRosNode->sensorAccelerometer[8]/0.3);                                     
    }
    
    giAccelX->setLine(160, 120, 160-x0, 120 + x1);
    giAccelY->setLine(160, 120, 160+y1, 120);
    giAccelZ->setLine(160, 120, 160,    120 +z1);

    temp.push_back(qtRosNode->sensorTemp);
    temp.erase(temp.begin());
    filteredTemp.erase(filteredTemp.begin());
    filteredTemp.push_back(0);
    filteredTemp[3]  = temp[3]*BFB0T + temp[2]*BFB1T * temp[1]*BFB2T + temp[0]*BFB3T;
    filteredTemp[3] -= filteredTemp[2]*BFA1T + filteredTemp[1]*BFA2T + filteredTemp[0]*BFA3T;

    ui->lblBatt->setText("Batt:  " + QString::number(qtRosNode->sensorBatt, 'f', 2));
    ui->pbBatt->setValue((int)((qtRosNode->sensorBatt - 7.15)/1.25*100));
    ui->txtTemp->setText(QString::number(qtRosNode->sensorTemp, 'f', 1));
    ui->lblLightSensorL->setText("L: " + QString::number(qtRosNode->sensorLightL, 'f', 3));
    ui->lblLightSensorR->setText("R: " + QString::number(qtRosNode->sensorLightR, 'f', 3));
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

void MainWindow::rbSonarsClicked()
{
    ui->graphicsViewRobot->setScene(scRobot);
}

void MainWindow::rbCameraClicked()
{
    ui->graphicsViewRobot->setScene(scCamera);
}

void MainWindow::rbMagnetClicked()
{
    ui->graphicsViewRobot->setScene(scImu);
}

void MainWindow::rbGyroClicked  ()
{
    ui->graphicsViewRobot->setScene(scImu);
}

void MainWindow::rbAccelClicked ()
{
    ui->graphicsViewRobot->setScene(scImu);
}
