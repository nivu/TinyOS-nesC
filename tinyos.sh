sudo apt install openjdk-8-jdk gcc-msp430 nescc make automake autoconf build-essential python-pip git

git clone https://github.com/navneetnivu07/tinyos-main.git

cd tinyos-main
cd tools
sudo ./Bootstrap
sudo ./configure
sudo make
sudo make install

sudo pip install PySerial


