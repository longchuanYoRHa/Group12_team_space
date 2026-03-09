------------------------------------------------------------
INTEL REALSENSE – FULL CLEAN CLEANUP + SOURCE BUILD GUIDE
(All actions inside ~/vision_testing/calibration unless noted)
------------------------------------------------------------

1. ACTIVATE PYTHON ENVIRONMENT & INSTALL CMAKE
source robots/bin/activate
pip install cmake


2. ADD INTEL’S OFFICIAL REALSENSE APT KEY
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
| sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null


3. UPDATE SYSTEM PACKAGE LISTS
sudo apt update


4. ADD INTEL APT REPOSITORY
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/librealsense.list


5. CREATE AND ENTER WORKSPACE FOLDER
mkdir -p ~/vision_testing/calibration
cd ~/vision_testing/calibration


------------------------------------------------------------
6. FULL CLEANUP OF OLD INSTALLS
------------------------------------------------------------
sudo apt purge librealsense2* librscalibrationtool* librscalibrationapi*
sudo dkms remove -m librealsense2-dkms -v 1.3.28 --all || true
sudo rm -rf /var/lib/dkms/librealsense2-dkms
sudo rm -f /etc/apt/sources.list.d/librealsense.list
sudo rm -f /etc/apt/keyrings/librealsense.pgp
sudo rm -f /var/crash/librealsense2*
sudo rm -f /var/crash/*
sudo apt autoremove -y
sudo apt autoclean
sudo apt clean


------------------------------------------------------------
7. INSTALL BUILD DEPENDENCIES
------------------------------------------------------------
sudo apt update
sudo apt install -y \
    git cmake build-essential pkg-config \
    libusb-1.0-0-dev libudev-dev \
    libgtk-3-dev \
    libglfw3-dev libglfw3 \
    libssl-dev


------------------------------------------------------------
8. CLONE LIBREALSENSE INTO YOUR FOLDER
------------------------------------------------------------
cd ~/vision_testing/calibration
git clone https://github.com/realsenseai/librealsense
cd librealsense


------------------------------------------------------------
9. INSTALL UDEV RULES
------------------------------------------------------------
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger


------------------------------------------------------------
10. BUILD LIBREALSENSE INSIDE YOUR FOLDER
------------------------------------------------------------
cd ~/vision_testing/calibration/librealsense
mkdir build && cd build

cmake .. -DBUILD_EXAMPLES=true
make -j$(nproc)
sudo make install


------------------------------------------------------------
11. VERIFY INSTALLATION
------------------------------------------------------------
realsense-viewer


------------------------------------------------------------
12. FIX DUPLICATE UDEV RULES (IF WARNING APPEARS)
------------------------------------------------------------
Remove APT-installed conflicting rule:
sudo rm -f /lib/udev/rules.d/60-librealsense2-udev-rules.rules

Reload rules:
sudo udevadm control --reload-rules
sudo udevadm trigger


------------------------------------------------------------
DONE — You now have a clean, source-built RealSense installation.
------------------------------------------------------------
