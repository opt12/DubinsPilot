# DubinsPilot

**_The project now works with Qt5 and the latest (6.1.x) version of QWT. this should ease the installation on a recent system quite a lot_**

The DubinsPilot software is part of my Bachelor thesis **Implementierung eines Notlandeassistenten auf Basis von Dubins-Kurven unter Berücksichtigung des Windes**

In this bachelor thesis an emergency landing assistant (ELA) is developed that is able to fly an aircraft in a loss of thrust emergency to a reachable emergency landing field (ELF). The path planning to reach this ELF is based on Dubins curves. The novelty concept in this thesis is the path-tracking of the autopilot component in the so called air-frame which is a reference system that moves with the wind. By using this air-frame reference, the path planning algorithm can operate with simple geometries such as circle segments and straight lines. The developed software is connected to the commercial flight simulation software X-Plane and extensive experiments are conducted to evaluate the suitability and accuracy of the air-frame based path planning and tracking in no-wind as well as in constant wind situations.


The main outcome of the experiments was, that the developed autopilot in the air-frame works very well regarding the target achievement precision. During the experiments it was also ascertained, that further research is needed to develop a model to predict the  duration of descent to incorporate the wind into the path planning in advance. In the end of the thesis some proposals are made how to further improve the path planning.

The PDF for this thesis can befound in the `thesis` subdirectory.

## Installation

The following installation instructions work on my machine, which is Ubuntu 16.04. Besides the explicitly named libraries, a recent version of `Node.js` and `npm` is needed. This can both be installed from the package manager.

Unfortunately, the QWT-library used in this project introduced some breaking changes in their interface from version 6.0 to version 6.1. This older version is not compatible with Qt 5. This is the reason, that currently, this application needs Qt4 and qwt-6.0.2 to be built from sources. If anybody has ideas, how to overcome this issue, please tell me.

To build and run the DubinsPilot software, some prerequisites need to be met:

### Preparation
First of all some libraries need to be installed:  
Qt5 is probably in the package manager:  
`sudo apt-get install qt5-default`  
Now install the GeographicLib package also from the package manager:  
`sudo apt-get install libgeographic-dev`  
Additionally, you need QWT (currently Version 6.1 is supported). This also ought to be in the package manager:  
`sudo apt-get install qwt`  


### Downloading and building the DubinsPilot app and the DubinsViewer web-app:

Download the files to build the DubinsPilot and the DubinsViewer applications:

`git clone https://github.com/opt12/DubinsPilot.git`  
`git clone https://github.com/opt12/DubinsViewer.git`  

Build the DubinsPilot Application  
`cd DubinsPilot`  
`qmake DubinsPilot.pro`  
`make`

Copy the config files to the appropriate position in your .config files  
`mkdir ~/.config/EEE/ && cp ./config_EEE/PID_Parameters.conf ~/.config/EEE/`  
This confgiuration contains parameters for the stock X-Plane Cessna 172. It has to be adapted when using another aircraft model.

Check the path to the logfiles given in the config-file (key: `initialLogFileDir`)  
`nano ~/.config/EEE/PID_Parameters.conf`  
Get out of the DubinsPilot directory.  
`cd ..`

Prepare the DubinsViewerClient Application:  
(The DubinsViewerClient currently queries the server at address  
const baseURL = 'http://localhost:3001/api/queries'.  
If you want the viewer app to be available from the outside, replace this by an address reachable within your network. E. g.  
`const baseURL = 'http://192.168.XXX.XXX:3001/api/queries';`  
This lives in the file `DubinsViewer/DubinsViewerClient/app/containers/MapContainer/sagas.js`)  

`cd DubinsViewer/DubinsViewerClient/`  
Install all npm modules:  
`npm i`

Build the bundle for the browser:  
`npm run build`

Change to the DubinsViewerServer directory:  
`cd ../DubinsViewerServer`  
Install all needed modules:  
`npm i`  
Leave the entire DubinsViewer directory:  
`cd ../..`

## Running the applicattion

Now everything should be prepared and ready. Use two terminals and start the DubinsPilot application:  
`./DubinsPilot/target/DubinsPilot`  
and start the server of the Viewer  
`npm start --prefix ./DubinsViewer/DubinsViewerServer`

Now start X-Plane on a machine whose broadcast messages go to the machine, 
DubinsPilot is running on. When X-Plane is also up and running, the "XPlane Connection" LEDs in the user-interface will change to green in the app. 
If this doesn't happen, the broadcast messages won't come through.

To view the position and state of the simulated aircraft, open up a recent web-browser and go to either [http://localhost:3001](http://localhost:3001) or, 
if you have set up a different address before, go to that address [http://192.168.XXX.XXX:3001](http://192.168.XXX.XXX:3001).

**Have fun**

I'll be glad to receive pull requests to integrate more features.
