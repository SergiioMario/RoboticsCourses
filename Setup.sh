#!/bin/bash
#FORMAT
FRM='\033['
BLACK='0;30'
RED='1;31'
GREEN='1;32'
YELLOW='1;33'
BLUE='1;34'
PURPLE='1;35'
CYAN='1;36'
WHITE='1;37'
BGBLACK=';40m'
BGRED=';41m'
BGGREEN=';42m'
BGYELLOW=';43m'
BGBLUE=';44m'
BGWHITE=';47m'
NC='\033[0m'

if [ $# -eq 0 ] ; then
	echo -e "${FRM}${GREEN}${BGRED} No option supplied, use one of the following...${NC}"
	echo -e "\t-i, --install"
	echo -e "\t\t To install RoboticsCourses software for the first time"
	echo -e "\t\t (${FRM}${RED}${BGBLACK}must be executed as sudo${NC})"
	echo -e "\t-u, --update"
	echo -e "\t\t To update an already existent RoboticsCourses installation,"
	echo -e "\t\t this includes udev rules, folder creations and user groups"
else
	SOURCE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
	echo -e "${FRM}${WHITE}${BGBLUE}The source directory is $SOURCE_DIR ${NC}"
	if [ "$1" == "-i" ] || [ "$1" == "--install" ]; then
		if [ ! "$EUID" -ne 0 ]; then
			echo -e "This script ${FRM}${RED}${BGBLACK}must be executed as normal user${NC}"
			exit;
		fi
		INSTALL_DIR=""
		if [ $# -eq 2 ] ; then
			INSTALL_DIR=$2
			if [ ! -d "$INSTALL_DIR" ]; then
				echo -e "${FRM}${RED}${BGBLACK}Not exist installation directory${NC}"
				exit -1
			else
				echo -e "${FRM}${WHITE}${BGBLUE}The installation directory is $INSTALL_DIR ${NC}"
			fi
		else
			INSTALL_DIR=$HOME
			echo -e "${FRM}${WHITE}${BGBLUE}The installation directory is $INSTALL_DIR ${NC}"
		fi
		#SCRIPT START
		#THE REAL STUFF
		echo -e "${FRM}${WHITE}${BGBLUE} Installing Justina's dependencies ${NC}"
		sudo apt-get -y update
		sudo apt-get install -y freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev doxygen graphviz mono-complete
		sudo apt-get install -y build-essential libgtk2.0-dev libjpeg-dev libtiff5-dev libjasper-dev libopenexr-dev cmake python-dev python-numpy python-tk libtbb-dev libeigen3-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev sphinx-common texlive-latex-extra libv4l-dev libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev ant default-jdk libvtk6.2
		echo -e "${FRM}${GREEN}${BGBLUE} Jusina's dependencies have been installed ${NC}"

		#TODO Validate that as necesary, because this is for test kinect one installation
		sudo apt-get install -y libturbojpeg libjpeg-turbo8-dev
		sudo apt-get install -y libglfw3-dev
		#TODO END
		
		cd $INSTALL_DIR
		dlib_file="v19.6.zip"
		dlib_file_desc="dlib-19.6"
		dlib_file_path="$INSTALL_DIR/$dlib_file"
		if [ ! -f "$dlib_file_path" ]; then
			echo -e "${FRM}${WHITE}${BGBLUE} Downloading dlib library ${NC}"
			wget https://github.com/davisking/dlib/archive/v19.6.zip
			unzip $dlib_file
			echo -e "${FRM}${GREEN}${BGBLUE} dlib library have been downloading ${NC}"
		fi
		echo -e "${FRM}${WHITE}${BGBLUE} Installing dlib library ${NC}"
		cd $dlib_file_desc
		mkdir build
		cd build
		cmake ..
		make -j4
		sudo make install
		echo -e "${FRM}${GREEN}${BGBLUE} dlib library have been installing ${NC}"

		echo -e "${FRM}${WHITE}${BGBLUE} Preparing to build Prime sense drivers ${NC}"
		cd $INSTALL_DIR
		mkdir -p prime_sense
		cd prime_sense
		sensorKinect_file="$(pwd)/SensorKinect"
		if [ ! -d "$sensorKinect_file" ]; then
			git clone https://github.com/ph4m/SensorKinect.git
		fi
		cd SensorKinect
		git checkout unstable
		echo -e "${FRM}${GREEN}${BGBLUE} Prime sense drivers have been prepared ${NC}"
		cd ../SensorKinect/Platform/Linux/CreateRedist
		echo -e "${FRM}${WHITE}${BGBLUE} Installing Prime sense drivers ${NC}"
		./RedistMaker
		cd ../Redist/Sensor-Bin-Linux-x64-v5.1.2.1/
		sudo ./install.sh
		echo -e "${FRM}${GREEN}${BGBLUE}Prime sense drivers have been installed${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE}Installing NITE for skeleton traking${NC}"
		cd $INSTALL_DIR
		nite_file="$(pwd)/NITE-Bin-Linux-x64-v1.5.2.23.tar.zip"
		if [ ! -f "$nite_file" ]; then
			wget http://www.openni.ru/wp-content/uploads/2013/10/NITE-Bin-Linux-x64-v1.5.2.23.tar.zip
			unzip NITE-Bin-Linux-x64-v1.5.2.23.tar.zip
			tar -xvf NITE-Bin-Linux-x64-v1.5.2.23.tar.bz2
		fi
		cd NITE-Bin-Dev-Linux-x64-v1.5.2.23
		sudo ./install.sh
		echo -e "${FRM}${GREEN}${BGBLUE}NITE correctly installed ${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE}Installing OpenNI to update default libraries${NC}"
		cd $INSTALL_DIR
		openni_file_dir="$(pwd)/OpenNI"
		if [ ! -f "$openni_file_dir" ]; then
			git clone https://github.com/OpenNI/OpenNI
		fi
		cd OpenNI/
		git checkout unstable
		cd Platform/Linux/CreateRedist
		./RedistMaker	
		cd ../Redist/OpenNI-Bin-Dev-Linux-x64-v1.5.8.5/
		sudo ./install.sh
		echo -e "${FRM}${GREEN}${BGBLUE}OpenNI have been installed to update default libraries${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE}Installing ros package's dependencies${NC}"
		sudo apt-get -y install ros-kinetic-urg-node
		sudo apt-get -y install ros-kinetic-joy
		sudo apt-get -y install ros-kinetic-openni-camera
		sudo apt-get -y install ros-kinetic-openni-launch
		sudo apt-get -y install ros-kinetic-openni2-camera
		sudo apt-get -y install ros-kinetic-openni2-launch
		sudo apt-get -y install ros-kinetic-amcl
		sudo apt-get -y install ros-kinetic-tf2-bullet
		sudo apt-get -y install ros-kinetic-fake-localization
		sudo apt-get -y install ros-kinetic-map-server
		sudo apt-get -y install ros-kinetic-sound-play
		sudo apt-get -y install ros-kinetic-gmapping
		sudo apt-get -y install ros-kinetic-dynamixel-sdk
		echo -e "${FRM}${GREEN}${BGBLUE}Ros package's dependencies have been installed${NC}"

		echo -e "${FRM}${WHITE}${BGBLUE}Installing pyRobotics and clips dependencies${NC}"
		cd $SOURCE_DIR/ToInstall/pyRobotics-1.8.0
		sudo python setup.py config
		sudo python setup.py install
		cd $SOURCE_DIR/ToInstall/CLIPS/instPy
		sudo python setup.py config
		sudo python setup.py install
		cd $SOURCE_DIR/ToInstall/CLIPS/pyclips
		sudo python setup.py config
		sudo python setup.py install
		echo -e "${FRM}${GREEN}${BGBLUE}pyRobotics and clips dependencies has been installed${NC}"

		#TODO Validate that as necesary, because this is for test the kinect one installation
		cd $INSTALL_DIR
		git clone https://github.com/OpenKinect/libfreenect2.git
		cd libfreenect2
		mkdir build
		sudo rm build/*
		cd build
		cmake ..
		make -j4
		sudo make install
		sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
		#TODO END

		
		echo -e "${FRM}${WHITE}${BGBLUE} Preparing the serial library, that use jrk controller${NC}"
		cd $INSTALL_DIR
		git clone https://github.com/wjwwood/serial
		cd serial
		mkdir build
		echo -e "${FRM}${GREEN}${BGBLUE} The serial lib have been prepared ${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE} Installing the serial library, that use jrk controller${NC}"
		cmake ..
		sudo make install
		echo -e "${FRM}${GREEN}${BGBLUE} The serial lib have been installed ${NC}"

		echo -e "${FRM}${WHITE}${BGBLUE}Installing basic audio libraries${NC}"
		sudo apt-get -y install libzbar-dev
		echo -e "${FRM}${WHITE}${BGBLUE}Audio support will be installed, choose <yes> when asked for real time permissions${NC}"
		read -p "(Waiting for key press in order to continue)"
		sudo apt-get -y install jackd2 libjack-jackd2-dev pulseaudio-module-jack qjackctl
		echo -e "${FRM}${WHITE}${BGBLUE}Installing kinect audio driver${NC}"
		sudo apt-get -y install kinect-audio-setup
		echo -e "${FRM}${WHITE}${BGBLUE}Installing pyaudio lib for directional audio node${NC}"
		sudo apt-get -y install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
		sudo apt-get -y install ffmpeg libav-tools
		sudo pip install pyaudio==0.2.9 --upgrade
		echo -e "${FRM}${GREEN}${BGBLUE}Audio libraries have been installed${NC}"

		if [ ! -d "/media/$USER/usbPDF/" ]; then
			sudo mkdir /media/$USER/USBPDF/
			mkdir /home/$USER/objs/
			#Add user to dialout, in order to use Arduino and Texas instrument board----
			sudo adduser $USER dialout
		fi

		echo "source $SOURCE_DIR/catkin_ws/devel/setup.bash" >> /home/$USER/.bashrc
		source /home/$USER/.bashrc
		source $SOURCE_DIR/catkin_ws/devel/setup.bash
		echo -e "${FRM}${WHITE}${BGBLUE}Copying the rules of Justina hardware to system${NC}"
		cd $SOURCE_DIR
		sudo cp ToInstall/USB/80-justinaRobot.rules /etc/udev/rules.d/
		sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
		echo -e "${FRM}${WHITE}${BGBLUE}Sourcing to get git branch and alias launchers${NC}"
		echo "green=\"\[\033[01;32m\]\"" >> /home/$USER/.bashrc
		echo "blue=\"\[\033[01;34m\]\"" >> /home/$USER/.bashrc
		echo "purple=\"\[\033[01;35m\]\"" >> /home/$USER/.bashrc
		echo "red=\"\[\033[01;31m\]\"" >> /home/$USER/.bashrc
		echo "yellow=\"\[\033[01;33m\]\"" >> /home/$USER/.bashrc
		echo "reset=\"\[\033[0m\]\"" >> /home/$USER/.bashrc
		echo "export GIT_PS1_SHOWDIRTYSTATE=1" >> /home/$USER/.bashrc
		echo "export PS1=\"\$red\u@\$green\h\$yellow:\$red\\\$(__git_ps1)\$blue\\\\W\$green->\$reset \"" >> /home/$USER/.bashrc
		echo "alias em='emacs24 -nw'" >> /home/$USER/.bashrc
		#echo "alias jsea='roslaunch surge_et_ambula justina.launch'" >> /home/$USER/.bashrc
		#echo "alias jseas='roslaunch surge_et_ambula justina_simul.launch'" >> /home/$USER/.bashrc
		echo -e "${FRM}${RED}${BGWHITE}You can now ${NC}${FRM}${BLACK}${BGWHITE}behold${NC}${FRM}${RED}${BGWHITE} the power of Justina software${NC}"
	elif [ "$1" == "-u" ] || [ "$1" == "--update" ]; then
		if [ ! -d "/media/$USER/usbPDF/" ]; then
			sudo mkdir /media/$USER/USBPDF/
			mkdir /home/$USER/objs/
			#Add user to dialout, in order to use Arduino and Texas instrument board----
			sudo adduser $USER dialout
		fi
		echo "source /opt/ros/kinetic/setup.bash" >> /home/$USER/.bashrc
		echo "source $SOURCE_DIR/catkin_ws/devel/setup.bash" >> /home/$USER/.bashrc
		source /home/$USER/.bashrc
		source $SOURCE_DIR/catkin_ws/devel/setup.bash
		echo -e "${FRM}${WHITE}${BGBLUE}Sourcing to get git branche and alias launchers${NC}"
		echo "green=\"\[\033[01;32m\]\"" >> /home/$USER/.bashrc
		echo "blue=\"\[\033[01;34m\]\"" >> /home/$USER/.bashrc
		echo "purple=\"\[\033[01;35m\]\"" >> /home/$USER/.bashrc
		echo "red=\"\[\033[01;31m\]\"" >> /home/$USER/.bashrc
		echo "yellow=\"\[\033[01;33m\]\"" >> /home/$USER/.bashrc
		echo "reset=\"\[\033[0m\]\"" >> /home/$USER/.bashrc
		echo "export GIT_PS1_SHOWDIRTYSTATE=1" >> /home/$USER/.bashrc
		echo "export PS1=\"\$red\u@\$green\h\$yellow:\$red\\\$(__git_ps1)\$blue\\\\W\$green->\$reset \"" >> /home/$USER/.bashrc
		echo "alias em='emacs24 -nw'" >> /home/$USER/.bashrc
		#echo "alias jsea='roslaunch surge_et_ambula justina.launch'" >> /home/$USER/.bashrc
		#echo "alias jseas='roslaunch surge_et_ambula justina_simul.launch'" >> /home/$USER/.bashrc
		echo -e "${FRM}${WHITE}${BGBLUE}Copying the rules of Justina to system${NC}"
		cd $SOURCE_DIR
		sudo cp ToInstall/USB/80-justinaRobot.rules /etc/udev/rules.d/
		sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
		echo -e "${FRM}${RED}${BGWHITE}You can now ${NC}${FRM}${BLACK}${BGWHITE}behold${NC}${FRM}${RED}${BGWHITE} the power of Justina software${NC}"
	else
		echo -e "${FRM}${CYAN}${BGRED} Invalid option supplied, use one of the following...${NC}"
		echo -e "\t-i, --install"
		echo -e "\t\t To install Justina software for first time"
		echo -e "\t\t (${FRM}${RED}${BGBLACK}must be executed as sudo${NC})"
		echo -e "\t-u, --update"
		echo -e "\t\t To update an already existent Justina installation,"
		echo -e "\t\t this includes udev rules, folder creations and user groups"
	fi
fi
