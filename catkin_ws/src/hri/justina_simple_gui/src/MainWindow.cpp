#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QtWidgets/QFileDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->ui->laRbArticular->setChecked(true);
    this->ui->raRbArticular->setChecked(true);
    this->laLastRadioButton = 0;
    this->raLastRadioButton = 0;
    this->scCamera = new QGraphicsScene(0,0,320,240, ui->graphicsViewCamera);
    this->giCamera = scCamera->addPixmap(pmCamera);
    ui->graphicsViewCamera->setScene(this->scCamera);

    //Navigation
    QObject::connect(ui->navTxtStartPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navTxtGoalPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnCalcPath, SIGNAL(clicked()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnExecPath, SIGNAL(clicked()), this, SLOT(navBtnExecPath_pressed()));
    QObject::connect(ui->navTxtMove, SIGNAL(returnPressed()), this, SLOT(navMoveChanged()));
    //Hardware
    QObject::connect(ui->hdTxtPan, SIGNAL(valueChanged(double)), this, SLOT(hdPanTiltChanged(double)));
    QObject::connect(ui->hdTxtTilt, SIGNAL(valueChanged(double)), this, SLOT(hdPanTiltChanged(double)));
    QObject::connect(ui->laTxtOpenGripper, SIGNAL(valueChanged(double)), this, SLOT(laOpenGripperChanged(double)));
    QObject::connect(ui->laTxtCloseGripper, SIGNAL(valueChanged(double)), this, SLOT(laCloseGripperChanged(double)));
    QObject::connect(ui->raTxtOpenGripper, SIGNAL(valueChanged(double)), this, SLOT(raOpenGripperChanged(double)));
    QObject::connect(ui->raTxtCloseGripper, SIGNAL(valueChanged(double)), this, SLOT(raCloseGripperChanged(double)));
    QObject::connect(ui->laTxtXYZ, SIGNAL(returnPressed()), this, SLOT(laValuesChanged()));
    QObject::connect(ui->laTxtRPY, SIGNAL(returnPressed()), this, SLOT(laValuesChanged()));
    QObject::connect(ui->laTxtElbow, SIGNAL(returnPressed()), this, SLOT(laValuesChanged()));
    QObject::connect(ui->raTxtXYZ, SIGNAL(returnPressed()), this, SLOT(raValuesChanged()));
    QObject::connect(ui->raTxtRPY, SIGNAL(returnPressed()), this, SLOT(raValuesChanged()));
    QObject::connect(ui->raTxtElbow, SIGNAL(returnPressed()), this, SLOT(raValuesChanged()));
    QObject::connect(ui->laRbCartesian, SIGNAL(clicked()), this, SLOT(laRadioButtonClicked()));
    QObject::connect(ui->laRbCartesianRobot, SIGNAL(clicked()), this, SLOT(laRadioButtonClicked()));
    QObject::connect(ui->laRbArticular, SIGNAL(clicked()), this, SLOT(laRadioButtonClicked()));
    QObject::connect(ui->raRbCartesian, SIGNAL(clicked()), this, SLOT(raRadioButtonClicked()));
    QObject::connect(ui->raRbCartesianRobot, SIGNAL(clicked()), this, SLOT(raRadioButtonClicked()));
    QObject::connect(ui->raRbArticular, SIGNAL(clicked()), this, SLOT(raRadioButtonClicked()));
    //Torso
    QObject::connect(ui->trsTxtSpine, SIGNAL(valueChanged(double)), this, SLOT(torsoPoseChanged(double)));
    QObject::connect(ui->trsTxtWaist, SIGNAL(valueChanged(double)), this, SLOT(torsoPoseChanged(double)));
    QObject::connect(ui->trsTxtShoulders, SIGNAL(valueChanged(double)), this, SLOT(torsoPoseChanged(double)));
    QObject::connect(ui->trsTxtLoc, SIGNAL(returnPressed()), this, SLOT(torsoLocChanged()));
    //Speech synthesis and recog
    QObject::connect(ui->spgTxtSay, SIGNAL(returnPressed()), this, SLOT(spgSayChanged()));
    QObject::connect(ui->sprTxtFakeRecog, SIGNAL(returnPressed()), this, SLOT(sprFakeRecognizedChanged()));
    //Vision

    //HRI

    this->robotX = 0;
    this->robotY = 0;
    this->robotTheta = 0;
    this->laIgnoreValueChanged = false;
    this->raIgnoreValueChanged = false;
    
    /*QScrollArea *scrollArea = new QScrollArea;
    scrollArea->setBackgroundRole(QPalette::Dark);
    scrollArea->setWidget(this->ui->labelAnswerResp);*/
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

bool MainWindow::strToFloatArray(std::string str, std::vector<float>& result)
{
    result.clear();
    std::vector<std::string> parts;
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    for(size_t i=0; i < parts.size(); i++)
    {
        std::stringstream ssValue(parts[i]);
        float value;
        if(!(ssValue >> value))
            return false;
        result.push_back(value);
    }
    return true;
}


void MainWindow::navBtnCalcPath_pressed()
{
    float startX, startY, startTheta;
    float goalX = 0;
    float goalY = 0;
    float goalTheta;
    std::string start_location = "";
    std::string goal_location = "";
    std::vector<std::string> parts;

    std::string str = this->ui->navTxtStartPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(str.compare("") == 0 || str.compare("robot") == 0) //take robot pose as start position
    {
        this->ui->navTxtStartPose->setText("Robot");
        JustinaNavigation::getRobotPose(this->robotX, this->robotY, this->robotTheta);
        startX = this->robotX;
        startY = this->robotY;
        startTheta = this->robotTheta;
    }
    else if(parts.size() >= 2) //Given data correspond to numbers
    {
        std::stringstream ssStartX(parts[0]);
        std::stringstream ssStartY(parts[1]);
        if(!(ssStartX >> startX) || !(ssStartY >> startY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
    else //Given data correspond to location
        start_location = parts[0];

    str = this->ui->navTxtGoalPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() >= 2)
    {
        std::stringstream ssGoalX(parts[0]);
        std::stringstream ssGoalY(parts[1]);
        if(!(ssGoalX >> goalX) || !(ssGoalY >> goalY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
    else
        goal_location = parts[0];

    
    if(start_location.compare("") == 0 && goal_location.compare("") == 0)
        JustinaNavigation::calcPathAStar(startX, startY, goalX, goalY, this->calculatedPath);
    /*
    else if(start_location.compare("") == 0 && goal_location.compare("") != 0)
        JustinaNavigation::planPath(startX, startY, goal_location, this->calculatedPath);
    else if(start_location.compare("") != 0 && goal_location.compare("") == 0)
        JustinaNavigation::planPath(start_location, goalX, goalY, this->calculatedPath);
    else
        JustinaNavigation::planPath(start_location, goal_location, this->calculatedPath);
    */
}

void MainWindow::navBtnExecPath_pressed()
{/*
    float goalX = 0;
    float goalY = 0;
    float goalTheta;
    std::string goal_location = "";
    std::vector<std::string> parts;

    std::string str = this->ui->navTxtGoalPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() >= 2)
    {
        std::stringstream ssGoalX(parts[0]);
        std::stringstream ssGoalY(parts[1]);
        if(!(ssGoalX >> goalX) || !(ssGoalY >> goalY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
        if(parts.size() > 2)
        {
            std::stringstream ssGoalAngle(parts[2]);
            if(!(ssGoalAngle >> goalTheta))
            {
                this->ui->navTxtStartPose->setText("Invalid format");
                return;
            }
            ////this->ui->navLblStatus->setText("Base Status: Moving to goal point...");
            //JustinaNavigation::startGetClose(goalX, goalY, goalTheta);
        }
        else
        {
            //// this->ui->navLblStatus->setText("Base Status: Moving to goal point...");
            //JustinaNavigation::startGetClose(goalX, goalY);
        }
        return;
    }
    else
    {
        goal_location = parts[0];
        // //this->ui->navLblStatus->setText("Base Status: Moving to goal point...");
        //JustinaNavigation::startGetClose(goal_location);
    }*/
    JustinaNavigation::executePath();
}

void MainWindow::navMoveChanged()
{
    std::vector<std::string> parts;
    std::string str = this->ui->navTxtMove->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() < 1)
        return;

    if(parts[0].compare("l") == 0)
    {
        if(parts.size() < 2)
            return;
        std::stringstream ssLateral(parts[1]);
        float lateral;
        if(!(ssLateral >> lateral))
            return;
        //JustinaNavigation::startMoveLateral(lateral);
        return;
    }

    float dist = 0;
    float angle = 0;
    std::stringstream ssDist(parts[0]);
    if(!(ssDist >> dist))
        return;
    if(parts.size() > 1)
    {
        std::stringstream ssAngle(parts[1]);
        if(!(ssAngle >> angle))
            return;
    }
    //JustinaNavigation::startMoveDistAngle(dist, angle);
}

void MainWindow::hdPanTiltChanged(double)
{
    float goalPan = this->ui->hdTxtPan->value();
    float goalTilt = this->ui->hdTxtTilt->value();
    std::cout << "QMainWindow.->Setting new head goal pose: " << goalPan << "  " << goalTilt  << std::endl;
    JustinaHardware::setHeadGoalPose(goalPan, goalTilt);
    //JustinaManip::startHdGoTo(goalPan, goalTilt);
}

void MainWindow::laAnglesChanged(double d)
{
    if(this->laIgnoreValueChanged)
        return;
    
    std::vector<float> goalAngles;
    /*goalAngles.push_back(this->ui->laTxtAngles0->value());
    goalAngles.push_back(this->ui->laTxtAngles1->value());
    goalAngles.push_back(this->ui->laTxtAngles2->value());
    goalAngles.push_back(this->ui->laTxtAngles3->value());
    goalAngles.push_back(this->ui->laTxtAngles4->value());
    goalAngles.push_back(this->ui->laTxtAngles5->value());
    goalAngles.push_back(this->ui->laTxtAngles6->value());*/

    //JustinaManip::startLaGoToArticular(goalAngles);
}

void MainWindow::raAnglesChanged(double d)
{
    if(this->raIgnoreValueChanged)
        return;
    
    std::vector<float> goalAngles;
    /*goalAngles.push_back(this->ui->raTxtAngles0->value());
    goalAngles.push_back(this->ui->raTxtAngles1->value());
    goalAngles.push_back(this->ui->raTxtAngles2->value());
    goalAngles.push_back(this->ui->raTxtAngles3->value());
    goalAngles.push_back(this->ui->raTxtAngles4->value());
    goalAngles.push_back(this->ui->raTxtAngles5->value());
    goalAngles.push_back(this->ui->raTxtAngles6->value());*/

    //JustinaManip::startRaGoToArticular(goalAngles);
}

void MainWindow::laValuesChanged()
{
    std::vector<float> xyz;
    std::vector<float> rpy;
    std::vector<float> elbow;
    std::vector<float> values;
    this->strToFloatArray(this->ui->laTxtXYZ->text().toStdString(), xyz);
    this->strToFloatArray(this->ui->laTxtRPY->text().toStdString(), rpy);
    this->strToFloatArray(this->ui->laTxtElbow->text().toStdString(), elbow);
    values.insert(values.end(), xyz.begin(), xyz.end());
    values.insert(values.end(), rpy.begin(), rpy.end());
    values.insert(values.end(), elbow.begin(), elbow.end());
    bool success = values.size() == 7 || values.size() == 6 || values.size() == 3;
    if(!success) //If cannot get floats, then it is assumed that a predefined position is given
    {
        std::string goalLoc = this->ui->laTxtXYZ->text().toStdString();
        if(goalLoc.compare("") != 0)
        {
            //JustinaManip::startLaGoTo(goalLoc);
            ////JustinaManip::laGoTo(goalLoc, 5000);
        }
    }
    else
    {
        if(values.size() == 7)
            JustinaHardware::setLeftArmGoalPose(values);
	/*
        if(this->ui->laRbCartesianRobot->isChecked())
            JustinaManip::startLaGoToCartesianWrtRobot(values);
        else if(this->ui->laRbCartesian->isChecked())
            JustinaManip::startLaGoToCartesian(values);
        else if(this->ui->laRbArticular->isChecked())
            JustinaManip::startLaGoToArticular(values);*/
    }
}

void MainWindow::raValuesChanged()
{
    std::vector<float> xyz;
    std::vector<float> rpy;
    std::vector<float> elbow;
    std::vector<float> values;
    this->strToFloatArray(this->ui->raTxtXYZ->text().toStdString(), xyz);
    this->strToFloatArray(this->ui->raTxtRPY->text().toStdString(), rpy);
    this->strToFloatArray(this->ui->raTxtElbow->text().toStdString(), elbow);
    values.insert(values.end(), xyz.begin(), xyz.end());
    values.insert(values.end(), rpy.begin(), rpy.end());
    values.insert(values.end(), elbow.begin(), elbow.end());
    bool success = values.size() == 7 || values.size() == 6 || values.size() == 3;
    if(!success) //If cannot get floats, then it is assumed that a predefined position is given
    {
        std::string goalLoc = this->ui->raTxtXYZ->text().toStdString();
        if(goalLoc.compare("") != 0)
        {
            //JustinaManip::startRaGoTo(goalLoc);
            ////JustinaManip::raGoTo(goalLoc, 5000);
        }
    }
    else
    {
        if(values.size() == 7)
            JustinaHardware::setRightArmGoalPose(values);
        /*if(this->ui->raRbCartesianRobot->isChecked())
            JustinaManip::startRaGoToCartesianWrtRobot(values);
        else if(this->ui->raRbCartesian->isChecked())
            JustinaManip::startRaGoToCartesian(values);
        else if(this->ui->raRbArticular->isChecked())
            JustinaManip::startRaGoToArticular(values);*/
    }
}

void MainWindow::laOpenGripperChanged(double d)
{
    //JustinaManip::startLaOpenGripper((float)d);
}

void MainWindow::raOpenGripperChanged(double d)
{
    //JustinaManip::startRaOpenGripper((float)d);
}

void MainWindow::laCloseGripperChanged(double d)
{
    //JustinaManip::startLaCloseGripper((float)d);
}

void MainWindow::raCloseGripperChanged(double d)
{
    //JustinaManip::startRaCloseGripper((float)d);
}

void MainWindow::laRadioButtonClicked()
{
    int currentRb = -1;
    if(this->ui->laRbArticular->isChecked()) currentRb = 0;
    else if(this->ui->laRbCartesian->isChecked()) currentRb = 1;
    else if(this->ui->laRbCartesianRobot->isChecked()) currentRb = 2;
    else return;

    if(currentRb == this->laLastRadioButton)
        return;
    
    this->laIgnoreValueChanged = true;

    std::vector<float> xyz;
    std::vector<float> rpy;
    std::vector<float> elbow;
    std::vector<float> oldValues;
    std::vector<float> newValues;
    this->strToFloatArray(this->ui->laTxtXYZ->text().toStdString(), xyz);
    this->strToFloatArray(this->ui->laTxtRPY->text().toStdString(), rpy);
    this->strToFloatArray(this->ui->laTxtElbow->text().toStdString(), elbow);
    oldValues.insert(oldValues.end(), xyz.begin(), xyz.end());
    oldValues.insert(oldValues.end(), rpy.begin(), rpy.end());
    oldValues.insert(oldValues.end(), elbow.begin(), elbow.end());
    bool success = oldValues.size() == 7;
    
    if(!success)
    {
        this->laIgnoreValueChanged = false;
        if(this->laLastRadioButton == 0) this->ui->laRbArticular->setChecked(true);
        if(this->laLastRadioButton == 1) this->ui->laRbCartesian->setChecked(true);
        if(this->laLastRadioButton == 2) this->ui->laRbCartesianRobot->setChecked(true);
        return;
    }
    
    if(this->ui->laRbArticular->isChecked())
    {
        this->ui->laLblGoalValues->setText("Angles:");
        //if(this->laLastRadioButton == 2)
        //    JustinaTools::transformPose("base_link", oldValues, "left_arm_link0", oldValues);  
        //success = JustinaManip::inverseKinematics(oldValues, newValues);
    }
    else if(this->ui->laRbCartesian->isChecked())
    {
        this->ui->laLblGoalValues->setText("XYZ  RPY  Elbow:");
        //if(this->laLastRadioButton == 0)
        //    success = JustinaManip::directKinematics(newValues, oldValues);
        //else
        //    success = JustinaTools::transformPose("base_link", oldValues, "left_arm_link1", newValues);
    }
    else
    {
        this->ui->laLblGoalValues->setText("XYZ  RPY  Elbow:");
        //if(this->laLastRadioButton == 0)
        //    success = JustinaManip::directKinematics(oldValues, oldValues);
        
        //success = JustinaTools::transformPose("left_arm_link1", oldValues, "base_link", newValues);
    }
    if(!success)
    {
        this->laIgnoreValueChanged = false;
        if(this->laLastRadioButton == 0) this->ui->laRbArticular->setChecked(true);
        if(this->laLastRadioButton == 1) this->ui->laRbCartesian->setChecked(true);
        if(this->laLastRadioButton == 2) this->ui->laRbCartesianRobot->setChecked(true);
        return;
    }

    QString str = QString::number(newValues[0],'f',3)+"  "+QString::number(newValues[1],'f',3)+"  "+QString::number(newValues[2],'f',3);
    this->ui->laTxtXYZ->setText(str);
    str = QString::number(newValues[3],'f',3)+"  "+QString::number(newValues[4],'f',3)+"  "+QString::number(newValues[5],'f',3);
    this->ui->laTxtRPY->setText(str);
    str = QString::number(newValues[6],'f',3);
    this->ui->laTxtElbow->setText(str);
    
    this->laLastRadioButton = currentRb;
    this->laIgnoreValueChanged = false;
}

void MainWindow::raRadioButtonClicked()
{
    int currentRb = -1;
    if(this->ui->raRbArticular->isChecked()) currentRb = 0;
    else if(this->ui->raRbCartesian->isChecked()) currentRb = 1;
    else if(this->ui->raRbCartesianRobot->isChecked()) currentRb = 2;
    else return;

    if(currentRb == this->raLastRadioButton)
        return;
    
    this->raIgnoreValueChanged = true;

    std::vector<float> xyz;
    std::vector<float> rpy;
    std::vector<float> elbow;
    std::vector<float> oldValues;
    std::vector<float> newValues;
    this->strToFloatArray(this->ui->raTxtXYZ->text().toStdString(), xyz);
    this->strToFloatArray(this->ui->raTxtRPY->text().toStdString(), rpy);
    this->strToFloatArray(this->ui->raTxtElbow->text().toStdString(), elbow);
    oldValues.insert(oldValues.end(), xyz.begin(), xyz.end());
    oldValues.insert(oldValues.end(), rpy.begin(), rpy.end());
    oldValues.insert(oldValues.end(), elbow.begin(), elbow.end());
    bool success = oldValues.size() == 7;

    if(!success)
    {
        this->raIgnoreValueChanged = false;
        if(this->raLastRadioButton == 0) this->ui->raRbArticular->setChecked(true);
        if(this->raLastRadioButton == 1) this->ui->raRbCartesian->setChecked(true);
        if(this->raLastRadioButton == 2) this->ui->raRbCartesianRobot->setChecked(true);
        return;
    }
    
    if(this->ui->raRbArticular->isChecked())
    {
        this->ui->raLblGoalValues->setText("Angles:");
        //if(this->raLastRadioButton == 2)
        //    JustinaTools::transformPose("base_link", oldValues, "right_arm_link1", oldValues);  
        //success = JustinaManip::inverseKinematics(oldValues, newValues);
    }
    else if(this->ui->raRbCartesian->isChecked())
    {
        this->ui->raLblGoalValues->setText("XYZ  RPY  Elbow:");
        //if(this->raLastRadioButton == 0)
        //    success = JustinaManip::directKinematics(newValues, oldValues);
        //else
        //    success = JustinaTools::transformPose("base_link", oldValues, "right_arm_link0", newValues);
    }
    else
    {
        this->ui->raLblGoalValues->setText("XYZ  RPY  Elbow:");
        //if(this->raLastRadioButton == 0)
        //    success = JustinaManip::directKinematics(oldValues, oldValues);
        
	// success = JustinaTools::transformPose("right_arm_link1", oldValues, "base_link", newValues);
    }
    if(!success)
    {
        this->raIgnoreValueChanged = false;
        if(this->raLastRadioButton == 0) this->ui->raRbArticular->setChecked(true);
        if(this->raLastRadioButton == 1) this->ui->raRbCartesian->setChecked(true);
        if(this->raLastRadioButton == 2) this->ui->raRbCartesianRobot->setChecked(true);
        return;
    }

    QString str = QString::number(newValues[0],'f',3)+"  "+QString::number(newValues[1],'f',3)+"  "+QString::number(newValues[2],'f',3);
    this->ui->raTxtXYZ->setText(str);
    str = QString::number(newValues[3],'f',3)+"  "+QString::number(newValues[4],'f',3)+"  "+QString::number(newValues[5],'f',3);
    this->ui->raTxtRPY->setText(str);
    str = QString::number(newValues[6],'f',3);
    this->ui->raTxtElbow->setText(str);
    
    this->raLastRadioButton = currentRb;
    this->raIgnoreValueChanged = false;
}

void MainWindow::torsoPoseChanged(double d)
{
    float goalSpine = this->ui->trsTxtSpine->value();
    float goalWaist = this->ui->trsTxtWaist->value();
    float goalShoulders = this->ui->trsTxtShoulders->value();
    std::cout << "QMainWindow.->Setting new torso pose: " << goalSpine << "  " << goalWaist << "  " << goalShoulders << std::endl;
    //JustinaManip::startTorsoGoTo(goalSpine, goalWaist, goalShoulders);
    this->ui->trsLblStatus->setText("Status: Moving to ...");
}

void MainWindow::torsoLocChanged()
{
}

void MainWindow::spgSayChanged()
{
    std::string strToSay = this->ui->spgTxtSay->text().toStdString();
    std::cout << "QMainWindow.->Saying: " << strToSay << std::endl;
    //JustinaHRI::say(strToSay);
}

void MainWindow::sprFakeRecognizedChanged()
{
    std::string strToFake = this->ui->sprTxtFakeRecog->text().toStdString();
    std::cout << "QMainWindow.->Faking recog speech: " << strToFake << std::endl;
    JustinaHRI::fakeRecognizedSpeech(strToFake);
}

//
//SLOTS FOR SIGNALS EMITTED IN THE QTROSNODE
//

void MainWindow::updateGraphicsReceived()
{
    float rX;
    float rY;
    float rT;
    JustinaNavigation::getRobotPose(rX, rY, rT);
    //std::cout << "MainWindow.->Current pose: " << currentX << "  " << currentY << "  " << currentTheta << std::endl;
    QString robotTxt = "Robot Pose: "+ QString::number(rX,'f',3) + "  " + QString::number(rY,'f',3) + "  " + QString::number(rT,'f',4);
    this->ui->navLblRobotPose->setText(robotTxt);
    this->robotX = rX;
    this->robotY = rY;
    this->robotTheta = rT;

    float pan;
    float tilt;
    JustinaHardware::getHeadCurrentPose(pan, tilt);
    QString headTxt = QString::number(pan, 'f', 4) + "  " + QString::number(tilt, 'f', 4);
    this->ui->hdLblHeadPose->setText(headTxt);
    this->headPan = pan;
    this->headTilt = tilt;
/*
    if(JustinaNavigation::isGoalReached())
        this->ui->navLblStatus->setText("Base Status: Goal Reached (Y)");
    else
        this->ui->navLblStatus->setText("Base Status: Moving to goal pose...");

    if(JustinaManip::isLaGoalReached())
        this->ui->laLblStatus->setText("LA: Goal Reached (Y)");
    else
        this->ui->laLblStatus->setText("LA: Moving to goal...");

    if(JustinaManip::isRaGoalReached())
        this->ui->raLblStatus->setText("RA: Goal Reached (Y)");
    else
        this->ui->raLblStatus->setText("RA: Moving to goal...");
    
    if(JustinaManip::isHdGoalReached())
        this->ui->hdLblStatus->setText("Status: Goal Pose reached (Y)");
    else
        this->ui->hdLblStatus->setText("Status: Moving to goal pose...");

    if(JustinaManip::isTorsoGoalReached())
        this->ui->trsLblStatus->setText("Status: Goal Reached!");*/

    pmCamera.loadFromData(qtRosNode->imgCompressed.data(), qtRosNode->imgCompressed.size(), "JPG");
    giCamera->setPixmap(pmCamera);
}
