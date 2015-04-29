#!/bin/bash

cp ./v4l/tbs62x0fe_driver_dvbc.o.x86 ./v4l/tbs62x0fe_driver.o

cp ./v4l/tbs5880fe_driver_dvbc.o.x86 ./v4l/tbs5880fe_driver.o

cp ./v4l/tbs5280fe_driver_dvbc.o.x86 ./v4l/tbs5280fe_driver.o

cp ./v4l/tbs62x1fe_driver_dvbc.o.x86 ./v4l/tbs62x1fe_driver.o
cp ./v4l/tbs6290fe_driver_dvbc.o.x86 ./v4l/tbs6290fe_driver.o
cp ./v4l/tbs5881fe_driver_dvbc.o.x86 ./v4l/tbs5881fe_driver.o
cp ./v4l/tbs5220fe_driver_dvbc.o.x86 ./v4l/tbs5220fe_driver.o

cp ./v4l/tbs5281fe_driver_dvbc.o.x86 ./v4l/tbs5281fe_driver.o

cp ./v4l/tbs6205fe_driver_dvbc.o.x86 ./v4l/tbs6205fe_driver.o

echo "TBS 62xy, 5880 and 528x drivers configured for DVBC (x86 platform)."
