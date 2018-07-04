//*********************************************************************************
// Arduino PID Library Version 1.0.1 Modified Version for C++
// Platform Independent
// 
// Revision: 1.1
// 
// Description: The PID Controller module originally meant for Arduino made
// platform independent. Some small bugs present in the original Arduino source
// have been rectified as well.
// 
// For a detailed explanation of the theory behind this library, go to:
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// 
// Revisions can be found here:
// https://github.com/tcleg
// 
// Modified by: Trent Cleghorn , <trentoncleghorn@gmail.com>
// 
// Copyright (C) Brett Beauregard , <br3ttb@gmail.com>
// 
//                                 GPLv3 License
// 
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or (at your option) any later 
// version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with 
// this program.  If not, see <http://www.gnu.org/licenses/>.
//*********************************************************************************

// 
// Header Guard
// 
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

//*********************************************************************************
// Headers
//*********************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <functional>


//*********************************************************************************
// Macros and Globals
//*********************************************************************************

typedef enum
{
    MANUAL,
    AUTOMATIC
}
PIDMode;

typedef enum
{
    DIRECT,
    REVERSE
}
PIDDirection;

//*********************************************************************************
// Class
//*********************************************************************************

class
PIDControl
{
    public:
        // 
        // Constructor
        // Description:
        //      Initializes the PIDControl instantiation. This should be called at 
        //      least once before any other PID functions are called on the 
        //      instantiation.
        // Parameters:
        //      kp - Positive P gain constant value.
        //      ki - Positive I gain constant value.
        //      kd - Positive D gain constant value.
        //      sampleTimeSeconds - Interval in seconds on which PIDCompute will be 
        //          called.
        //      minOutput - Constrain PID output to this minimum value.
        //      maxOutput - Constrain PID output to this maximum value.
        //      mode - Tells how the controller should respond if the user has 
        //          taken over manual control or not.
        //          MANUAL:    PID controller is off. User can manually control the 
        //                     output.
        //          AUTOMATIC: PID controller is on. PID controller controls the 
        //                     output.
        //      controllerDirection - The sense of direction of the controller
        //          DIRECT:  A positive setpoint gives a positive output.
        //          REVERSE: A positive setpoint gives a negative output.
        // Returns:
        //      Nothing.
        // 
        PIDControl(double kp, double ki, double kd, double sampleTimeSeconds,
                   double minOutput, double maxOutput, PIDMode mode,
                   PIDDirection controllerDirection);     	
        
        // 
        // PID Compute
        // Description:
        //      Should be called on a regular interval defined by sampleTimeSeconds.
        //      Typically, PIDSetpointSet and PIDInputSet should be called 
        //      immediately before PIDCompute.
        // Parameters:
        //      None.
        // Returns:
        //      True if in AUTOMATIC. False if in MANUAL.
        //                     
        bool PIDCompute(); 
        
        // 
        // PID Mode Set
        // Description:
        //      Sets the PID controller to a new mode. Tells how the controller 
        //      should respond if the user has taken over manual control or not.
        // Parameters:
        //      mode - 
        //          MANUAL:    PID controller is off. User can manually control the 
        //                     output.
        //          AUTOMATIC: PID controller is on. PID controller controls the 
        //                     output.
        // Returns:
        //      Nothing.
        //              
        void PIDModeSet(PIDMode mode);                                                                                                                                       
        
        // 
        // PID Output Limits Set
        // Description:
        //      Sets the new output limits. The new limits are applied to the PID
        //      immediately.
        // Parameters:
        //      min - Constrain PID output to this minimum value.
        //      max - Constrain PID output to this maximum value.
        // Returns:
        //      Nothing.
        // 
        void PIDOutputLimitsSet(double min, double max);
        
        // 
        // PID Tunings Set
        // Description:
        //      Sets the new gain constant values.
        // Parameters:
        //      kp - Positive P gain constant value.
        //      ki - Positive I gain constant value.
        //      kd - Positive D gain constant value.
        // Returns:
        //      Nothing.
        // 
        void PIDTuningsSet(double kp, double ki, double kd);
        
        // 
        // PID Tuning Gain Constant P Set
        // Description:
        //      Sets the proportional gain constant value.
        // Parameters:
        //      kp - Positive P gain constant value.
        // Returns:
        //      Nothing.
        // 
        void PIDTuningKpSet(double kp);
        
        // 
        // PID Tuning Gain Constant I Set
        // Description:
        //      Sets the proportional gain constant value.
        // Parameters:
        //      ki - Positive I gain constant value.
        // Returns:
        //      Nothing.
        // 
        void PIDTuningKiSet(double ki);
        
        // 
        // PID Tuning Gain Constant D Set
        // Description:
        //      Sets the proportional gain constant value.
        // Parameters:
        //      kd - Positive D gain constant value.
        // Returns:
        //      Nothing.
        // 
        void PIDTuningKdSet(double kd);
        
        // 
        // PID Controller Direction Set
        // Description:
        //      Sets the new controller direction.
        // Parameters:
        //      controllerDirection - The sense of direction of the controller
        //          DIRECT:  A positive setpoint gives a positive output
        //          REVERSE: A positive setpoint gives a negative output
        // Returns:
        //      Nothing.
        // 
        void PIDControllerDirectionSet(PIDDirection controllerDirection);	  									  									  									  
        
        // 
        // PID Sample Time Set
        // Description:
        //      Sets the new sampling time (in seconds).
        // Parameters:
        //      sampleTimeSeconds - Interval in seconds on which PIDCompute will be 
        //          called.
        // Returns:
        //      Nothing.
        // 
        void PIDSampleTimeSet(double sampleTimeSeconds);
        
        // 
        // PID Setpoint Set
        // Description:
        //      Alters the setpoint the PID controller will try to achieve.
        // Parameters:
        //      setpoint - The desired setpoint the PID controller will try to 
        //          obtain.
        // Returns:
        //      Nothing.
        // 
        inline void PIDSetpointSet(double setpoint) { this->setpoint = setpoint; }
        
        // 
        // PID Input Set
        // Description:
        //      Should be called before calling PIDCompute so the PID controller 
        //      will have an updated input value to work with.
        // Parameters:
        //      input - The value the controller will work with.
        // Returns:
        //      Nothing.
        // 
        inline void PIDInputSet(double input) { this->input = input; }
        
        // 
        // PID Output Get
        // Description:
        //      Typically, this function is called after PIDCompute in order to
        //      retrieve the output of the controller.
        // Parameters:
        //      None.
        // Returns:
        //      The output of the specific PID controller.
        // 
        inline double PIDOutputGet() { return this->output; }
        
        // 
        // PID Proportional Gain Constant Get
        // Description:
        //      Returns the proportional gain constant value the particular
        //      controller is set to.
        // Parameters:
        //      None.
        // Returns:
        //      The proportional gain constant.
        // 
        inline double PIDKpGet() { return this->dispKp; }
        
        // 
        // PID Integral Gain Constant Get
        // Description:
        //      Returns the integral gain constant value the particular
        //      controller is set to.
        // Parameters:
        //      None.
        // Returns:
        //      The integral gain constant.
        // 
        inline double PIDKiGet() { return this->dispKi; }
        
        // 
        // PID Derivative Gain Constant Get
        // Description:
        //      Returns the derivative gain constant value the particular
        //      controller is set to.
        // Parameters:
        //      None.
        // Returns:
        //      The derivative gain constant.
        // 
        inline double PIDKdGet() { return this->dispKd; }
        
        // 
        // PID Mode Get
        // Description:
        //      Returns the mode the particular controller is set to.
        // Parameters:
        //      None.
        // Returns:
        //      MANUAL or AUTOMATIC depending on what the user set the 
        //      controller to.
        // 
        inline PIDMode PIDModeGet() { return this->mode; }
        
        // 
        // PID Direction Get
        // Description:
        //      Returns the direction the particular controller is set to.
        // Parameters:
        //      None.
        // Returns:
        //      DIRECT or REVERSE depending on what the user set the
        //      controller to.
        // 
        inline PIDDirection PIDDirectionGet() { return this->controllerDirection; }
        
        inline double lastErrorGet() { return this->error; }

        inline void PIDResetInternals() {
        	iTerm = 0.0f;
        	input = 0.0f;
        	lastInput = 0.0f;
        	output = 0.0f;
        }

        //
        // Calculate the error to counteract upon. Overriding the default may
        // be used to have more complex forms of error calculation available
        // Description:
        //      returns the error between setpoint and input value
        // Parameters:
        //      setpoint and input
        // Returns:
        //      The calculated and somehow weighted error
        //TODO check how to get this function inline
        std::function<double(double, double)> calculateError =
			[this](double _setpoint, double _input)
			{
        		return _setpoint - _input;
			};


    private:
        // 
        // Input to the PID Controller
        // 
        double input;
        
        // 
        // Previous input to the PID Controller
        // 
        double lastInput;
        
        // 
        // Last error of the PID controller (for logging purposes)
        //
        double error;

        //
        // Output of the PID Controller
        // 
        double output;
        
        // 
        // Gain constant values that were passed by the user
        // These are for display purposes
        // 
        double dispKp;
        double dispKi;
        double dispKd;
        
        // 
        // Gain constant values that the controller alters for
        // its own use
        // 
        double alteredKp;
        double alteredKi;
        double alteredKd;
        
        // 
        // The Integral Term
        // 
        double iTerm;
        
        // 
        // The interval (in seconds) on which the PID controller
        // will be called
        // 
        double sampleTime;
        
        // 
        // The values that the output will be constrained to
        // 
        double outMin;
        double outMax;
        
        // 
        // The user chosen operating point
        // 
        double setpoint;
        
        // 
        // The sense of direction of the controller
        // DIRECT:  A positive setpoint gives a positive output
        // REVERSE: A positive setpoint gives a negative output
        // 
        PIDDirection controllerDirection;
        
        // 
        // Tells how the controller should respond if the user has
        // taken over manual control or not
        // MANUAL:    PID controller is off.
        // AUTOMATIC: PID controller is on.
        // 
        PIDMode mode;
};

#endif  // PID_CONTROLLER_H
