# DubinsPilot


Unfortunately, the QWT-library used in this project introduced some breaking changes in their interface from version 6.0 to version 6.1. This is the reason, that currently, this application needs Qt4 and qwt.6.0.2 to be built. If anybody has ideas, how to overcome this issue, please tell me.

## Installation

To build and run the DubinsPilot software, some prerequisites need to be met:

### Preparation
First of all some libraries need to be installed:  
Qt4 is probably in the package manager:  
`sudo apt-get install qt4-default`  
Now install the GeographicLib package also from the package manager:  
`sudo apt-get install libgeographic-dev`

Due to the named breaking changes, an older version of qwt needs to be used. To build and install the qwt-6.0.2 version follow these steps:  
Downlaod and inflate the sources for qwt.6.0.2:  
`wget -qO- -O tmp.zip https://sourceforge.net/projects/qwt/files/qwt/6.0.2/qwt-6.0.2.zip && unzip tmp.zip && rm tmp.zip`

Descend in the qwt-6.0.2 directory:  
`cd qwt-6.0.2`  
Generate the Makefile:  
`qmake qwt.pro`  
Build and install the qwt-6.0.2 library  
`make`  
`sudo make install`  
Add the path to the `libqwt.so.6` to the run-time linker?  
`sudo echo "/usr/local/qwt-6.0.2/lib" > /etc/ld.so.conf.d/qwt.conf`  
Reload the library search configuration:  
`sudo ldconfig`  
return from qwt-6.0.2 directory  
`cd ..`

### Downloading and building the DubinsPilot app and the DubinsViewer web-app:

Download the files to build the DubinsPilot and the DubinsViewer applications:

`git clone https://github.com/opt12/DubinsPilot.git`  
`git clone https://github.com/opt12/DubinsViewer.git`  

Build the DubinsPilot Application  
`cd DubinsPilot`  
`qmake DubinsPilot.pro`  
`make`

Copy the config files to the appropriate position in your .config files  
`mkdir ~/.config/EEE/`  
`copy ./config_EEE/PID_Parameters.conf ~/.config/EEE/`

Check the path to the logfiles given in the config-file (key: `initialLogFileDir`)  
`nano ~/.config/EEE/PID_Parameters.conf`  
Get out of the DubinsPilot directory.  
`cd ..`

Prepare the DubinsViewerClient Application:  
(The DubinsViewerClient currently queries the server at address  
const baseURL = 'http://localhost:3001/api/queries'.  
If you want the viewer app to be available from the outside, replace this by an address reachable within your network. E. g.:  
`const baseURL = 'http://192.168.XXX.XXX:3001/api/queries';` in file `DubinsViewerClient/app/containers/MapContainer/sagas.js`)  

`cd DubinsViewer/`  
Install all npm modules:  
`npm i`

Build the bundle for the browser:  
`npm run build`

Change to the DubinsViewerServer directory:  
`cd ../DubinsViewerServer`  
Install all needed modules:  
`npm i`

## Running the applicattion

Now everything should be prepared and ready. Use two terminals and start the DubinsPilot application:  
`./DubinsPilot/target/DubinsPilot`  
and start the server of the Viewer  
`npm start --prefix ./DubinsViewer/DubinsViewerServer`

Now start X-Plane on a machine whose broadcast messages go to the machine, 
DubinsPilot is running on. When X-Plane is also up and running, the "XPlane Connection" LEDs in the user-interface will change to green in the app. 
If this doesn't happen, the broadcast messages won't come through.

To view the position and state of the simulated aircraft, open up a recent web-browser and go to either [http://localhost:3001](http://localhost:3001) or, 
if you have set up a different address before to that address [http://192.168.XXX.XXX:3001](http://192.168.XXX.XXX:3001).

**Have fun**

I'll be glad to receive pull requests to integrate more features.




