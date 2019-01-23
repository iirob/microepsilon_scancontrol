#!/bin/bash

LIB_DIR='/usr/local/lib/'
INCLUDE_DIR='/usr/local/include/'
OPT_DIR='/opt/scanCONTROL/'

DEVICE='device_properties.dat'
DEVICE_DIR='/misc/'
BIN_DIR='/lib/x86_64/'
INCLUDE='/include/'
MESCAN='/libmescan/'
LLT='/libllt/'
LLT_H='llt.h'
LLT_SO='libllt.so.0.2.0'
MESCAN_H='mescan.h'
MESCAN_H2='LLTDataTypes.h'
MESCAN_H3='mescan_adv.h'
MESCAN_H4='mescan_basic.h'
MESCAN_SO='libmescan.so.0.2.0'
DIR_ERROR="You have to run ./install.sh \"/path/to/scanCONTROL Linux SDK 0.2.0/\" (lib is located there)"

if ! [ -d "$1$BIN_DIR" ]
then
    echo "Could not find $BIN_DIR folder."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$INCLUDE$MESCAN$MESCAN_H" ]
then
    echo "Could not find $MESCAN_H in $1$INCLUDE$MESCAN."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$INCLUDE$MESCAN$MESCAN_H2" ]
then
    echo "Could not find $MESCAN_H2 in $1$INCLUDE$MESCAN."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$INCLUDE$MESCAN$MESCAN_H2" ]
then
    echo "Could not find $MESCAN_H2 in $1$INCLUDE$MESCAN."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$INCLUDE$MESCAN$MESCAN_H2" ]
then
    echo "Could not find $MESCAN_H2 in $1$INCLUDE$MESCAN."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$BIN_DIR$MESCAN_SO" ]
then
    echo "Could not find $MESCAN_SO in $1$BIN_DIR."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$INCLUDE$LLT$LLT_H" ]
then
    echo "Could not find $LLT_H in $1$INCLUDE$LLT."
    echo $DIR_ERROR
    exit 1
fi


if ! [ -s "$1$BIN_DIR$LLT_SO" ]
then
    echo "Could not find $LLT_SO in $1$BIN_DIR."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$DEVICE_DIR$DEVICE" ]
then
    echo "Could not find $DEVICE in $1$DEVICE_DIR."
    echo $DIR_ERROR
    exit 1
fi

echo "Installing prerequisites for aravis-0.6, need sudo rights."
sudo apt-get install -y autoconf intltool automake gtk-doc-tools


echo "Downloading and extracting aravis-0.6 to your home."
cd ~
wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.1.tar.xz -O aravis-0.6.1.tar.xz
tar xfJ aravis-0.6.1.tar.xz
rm aravis-0.6.1.tar.xz

echo "Configuring, Compiling and Installing aravis-0.6."
cd aravis-0.6.1
./configure
make
sudo make install

echo "Creating directories need sudo rights."
sudo rm -r -- "$INCLUDE_DIR$MESCAN"
sudo rm -r -- "$INCLUDE_DIR$LLT"
sudo mkdir -p -- "$INCLUDE_DIR$MESCAN"
sudo mkdir -p -- "$INCLUDE_DIR$LLT"
sudo mkdir -p -- "$OPT_DIR"


echo "Copying libs to $LIB_DIR, need sudo rights."
sudo cp -- "$1$BIN_DIR$MESCAN_SO" "$LIB_DIR"
sudo cp -- "$1$BIN_DIR$LLT_SO" "$LIB_DIR"
sudo chmod 755 "$LIB_DIR$LLT_SO"
sudo chmod 755 "$LIB_DIR$MESCAN_SO"

echo "running ldconfig to create symlinks and cache."
sudo ldconfig
sudo rm -- "$LIB_DIR"libllt.so
sudo rm -- "$LIB_DIR"libmescan.so
sudo ln -s -- "$LIB_DIR$LLT_SO" "$LIB_DIR"libllt.so
sudo ln -s -- "$LIB_DIR$MESCAN_SO" "$LIB_DIR"libmescan.so

echo "Copying headers to $INCLUDE_DIR, need sudo rights."
sudo cp -- "$1$INCLUDE$MESCAN$MESCAN_H" "$INCLUDE_DIR$MESCAN"
sudo cp -- "$1$INCLUDE$MESCAN$MESCAN_H2" "$INCLUDE_DIR$MESCAN"
sudo cp -- "$1$INCLUDE$MESCAN$MESCAN_H3" "$INCLUDE_DIR$MESCAN"
sudo cp -- "$1$INCLUDE$MESCAN$MESCAN_H4" "$INCLUDE_DIR$MESCAN"
sudo cp -- "$1$INCLUDE$LLT$LLT_H" "$INCLUDE_DIR$LLT"
sudo chmod -R u+rwX,g+rX,o+rX "$INCLUDE_DIR$MESCAN"
sudo chmod -R u+rwX,g+rX,o+rX "$INCLUDE_DIR$LLT"

echo "Copying $DEVICE to $OPT_DIR, need sudo rights."
sudo cp -- "$1$DEVICE_DIR$DEVICE" "$OPT_DIR"
sudo chmod -R u+rwX,g+rX,o+rX "$OPT_DIR"
