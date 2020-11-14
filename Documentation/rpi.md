# Sleepy Pi 2 Configuration


## Raspberry Pi 4 Headless Setup

### Step 1

Download the Raspberry Pi OS Imager from: https://www.raspberrypi.org/software/

Choose "Raspberry Pi OS" and write it to the SD card.

Once complete, remove and reinsert the SD card and add the following two files to the /boot drive.

1. ssh
* Simply create an empty file called `ssh`.
2. wpa_supplicant.conf
* This file will include your WiFi credentials:

```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=CA

network={
 ssid="SSID"
 psk="PASSKEY"
}
```

Run:
```
sudo apt update
sudo apt full-upgrade
```

Run:
```
sudo raspi-config
```
Perform following actions: 
1. Rename hostname
2. Enable camera 
3. Enable I2C
4. Enable SPI


### Make Python 3 default

Run:
```
python --version
nano ~/.bashrc
```

Add the following lines to the end of the file:
```
alias python='/usr/bin/python3'
alias pip=pip3
```

Run:
```
source ~/.bashrc
```
Then confirm that Python 3 is your version
```
python --version
```


## Sleep Pi 2 Configuration

### Step 1
The first step is to download the `Sleepy-Pi-Setup.sh` script to the `/home` directory on the Raspberry Pi.

This can be accomplished by running the following command on the RPi:
`wget https://raw.githubusercontent.com/SpellFoundry/Sleepy-Pi-Setup/master/Sleepy-Pi-Setup.sh`

To run the script, you first need to make it into an executable by:
```
chmod +x Sleepy-Pi-Setup.sh
```

Then run the script with administrator privileges using:
```
sudo ./Sleepy-Pi-Setup.sh
```




### Programming the Arduino
```
sudo gpio mode 15 in
sudo gpio mode 16 in
sudo gpio readall
```
