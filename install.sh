#!/bin/bash

LIB_FOLDER='/usr/local/lib'
INCLUDE_FOLDER='/usr/local/include'
OPT_FOLDER='/opt/scanCONTROL/'

DEVICE='device_properties.dat'
DEVICE_FOLDER='/examples/mescanread/'
BIN_FOLDER='/binaries(x86_64)/'
MESCAN='/libmescan/'
LLT='/libllt/'
LLT_H='libllt.h'
LLT_SO='libllt.so.1.0'
LLT_H2='LLTDataTypes.h'
MESCAN_H='libmescan.h'
MESCAN_SO='libmescan.so.1.0'
FOLDER_ERROR="You have to run ./install.sh \"/path/to/scanCONTROL/\""

if ! [ -d $1$BIN_FOLDER$MESCAN ]
then
    echo "Could not find $MESCAN folder in $1$BIN_FOLDER."
    echo $FOLDER_ERROR
    exit 1
fi

if ! [ -d $1$BIN_FOLDER$LLT ]
then
    echo "Could not find $LLT folder in $1$BIN_FOLDER."
    echo $FOLDER_ERROR
    exit 1
fi

if ! [ -s $1$BIN_FOLDER$MESCAN$MESCAN_H ]
then
    echo "Could not find $MESCAN_H in $1$BIN_FOLDER$MESCAN."
    echo $FOLDER_ERROR
    exit 1
fi

if ! [ -s $1$BIN_FOLDER$MESCAN$MESCAN_SO ]
then
    echo "Could not find $MESCAN_SO in $1$BIN_FOLDER$MESCAN."
    echo $FOLDER_ERROR
    exit 1
fi

if ! [ -s $1$BIN_FOLDER$LLT$LLT_H ]
then
    echo "Could not find $LLT_H in $1$BIN_FOLDER$LLT."
    echo $FOLDER_ERROR
    exit 1
fi

if ! [ -s $1$BIN_FOLDER$LLT$LLT_H2 ]
then
    echo "Could not find $LLT_H2 in $1$BIN_FOLDER$LLT."
    echo $FOLDER_ERROR
    exit 1
fi

if ! [ -s $1$BIN_FOLDER$LLT$LLT_SO ]
then
    echo "Could not find $LLT_SO in $1$BIN_FOLDER$LLT."
    echo $FOLDER_ERROR
    exit 1
fi

if ! [ -s $1$DEVICE_FOLDER$DEVICE ]
then
    echo "Could not find $DEVICE in $1$DEVICE_FOLDER."
    echo $FOLDER_ERROR
    exit 1
fi

set -e

echo "Installing prerequisites for aravis-0.4, need sudo rights."
sudo apt-get install -y autoconf intltool automake gtk-doc-tools


echo "Downloading and extracting aravis-0.4 to your home."
cd ~
wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.4/aravis-0.4.1.tar.xz -O aravis-0.4.1.tar.xz
tar xfJ aravis-0.4.1.tar.xz
rm aravis-0.4.1.tar.xz

echo "Configuring, Compiling and Installing aravis-0.4."
cd aravis-0.4.1
./configure
make
sudo make install

echo "Creating directories need sudo rights."
sudo mkdir -p $INCLUDE_FOLDER$MESCAN
sudo mkdir -p $INCLUDE_FOLDER$LLT
sudo mkdir -p $OPT_FOLDER


echo "Copying libs to $LIB_FOLDER, need sudo rights."
sudo cp $1$BIN_FOLDER$MESCAN$MESCAN_SO $LIB_FOLDER
sudo cp $1$BIN_FOLDER$LLT$LLT_SO $LIB_FOLDER

echo "running ldconfig to create symlinks and cache."
sudo ldconfig

echo "Copying headers to $INCLUDE_FOLDER, need sudo rights."
sudo cp $1$BIN_FOLDER$MESCAN$MESCAN_H  $INCLUDE_FOLDER$MESCAN
sudo cp $1$BIN_FOLDER$LLT$LLT_H  $INCLUDE_FOLDER$LLT
sudo cp $1$BIN_FOLDER$LLT$LLT_H2 $INCLUDE_FOLDER$LLT

echo "Copying $DEVICE to $OPT_FOLDER, need sudo rights."
sudo cp $1$DEVICE_FOLDER$DEVICE $OPT_FOLDER

