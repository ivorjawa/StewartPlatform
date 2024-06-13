# This is Brett Beauregard's Arduino PID library, ported to Python by Jan Kujawa
# Also contains a function to generate P, I, D constants from timing via the
# Ziegler-Nichols method
# module requires a millis() function, in PyBricks this is 
# pybricks.tools.StopWatch().time()
# set PID.millis before constructing any pid objects

### PID controller 
# http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
# https://github.com/br3ttb/Arduino-PID-Library/

# on PyBricks, use millis
# on Desktop, PID.setmillis(lambda: time.time()*1000) # must be set before creating any PID objects
millis = None # must be set
def setmillis(millisfn):
    global millis
    millis = millisfn
    
class PID(object):
    AUTOMATIC = 1
    MANUAL = 0
    DIRECT = 0
    REVERSE = 1
    P_ON_M = 0
    P_ON_E = 1
    
    def __init__(self, Setpoint, Kp,  Ki,  Kd,  POn,  ControllerDirection):
        #self.millis = millis # needs to be a function that returns milliseconds
        self.myOutput = 0;
        #self.myInput = Input;
        self.mySetpoint = Setpoint;
        self.inAuto = False;
        self.lastInput = 0

        self.SetOutputLimits(0, 255);   #default output limit corresponds to
                                        #the arduino pwm limits

        self.SampleTime = 100;            #default Controller Sample Time is 0.1 seconds

        self.SetControllerDirection(ControllerDirection);
        self.SetTunings(Kp, Ki, Kd, POn);

        self.lastTime = millis()-self.SampleTime;
        #print("New PID setpoint:%f p:%f i:%f d:%f" % (self.mySetpoint, self.kp, self.ki, self.kd))
        
    def Compute(self, myInput):
       if(not self.inAuto):
           return False;
       now = millis();
       timeChange = (now - self.lastTime);
       
       if(timeChange>=self.SampleTime):
          #/*Compute all the working error variables*/
          inpu = myInput;
          error = self.mySetpoint - inpu;
          dInput = (inpu - self.lastInput);
          self.outputSum+= (self.ki * error);
          #print("in, er, din, sp", inpu, error, dInput, self.mySetpoint)

          #/*Add Proportional on Measurement, if P_ON_M is specified*/
          if(not self.pOnE):
              self.outputSum-= self.kp * dInput;

          if(self.outputSum > self.outMax): 
              self.outputSum= self.outMax;
          elif(self.outputSum < self.outMin):
              self.outputSum= self.outMin;

          #/*Add Proportional on Error, if P_ON_E is specified*/
          output = 0
          if(self.pOnE):
              output = self.kp * error;
          else:
              output = 0;

          #/*Compute Rest of PID Output*/
          output += self.outputSum - self.kd * dInput;

          if(output > self.outMax):
              output = self.outMax;
          elif(output < self.outMin): 
              output = self.outMin;
          self.myOutput = output;

          #/*Remember some variables for next time*/
          self.lastInput = inpu;
          self.lastTime = now;
          return True;

       else:
           return False;

    def SetTunings(self, Kp, Ki, Kd, POn):
        if (Kp<0 or Ki<0 or Kd<0):
            return;

        self.pOn = POn;
        self.pOnE = (POn == self.P_ON_E);

        self.dispKp = Kp; self.dispKi = Ki; self.dispKd = Kd;

        SampleTimeInSec = (self.SampleTime)/1000;
        self.kp = Kp;
        self.ki = Ki * SampleTimeInSec;
        self.kd = Kd / SampleTimeInSec;

        if(self.controllerDirection == self.REVERSE):
          self.kp = (0 - self.kp);
          self.ki = (0 - self.ki);
          self.kd = (0 - self.kd);
          
    def SetSampleTime(self, NewSampleTime):
        if (NewSampleTime > 0):
            ratio  = NewSampleTime / self.SampleTime;
            self.ki *= ratio;
            self.kd /= ratio;
            self.SampleTime = NewSampleTime;
    
    def SetOutputLimits(self, Min, Max):
        if(Min >= Max): 
            return;
        self.outMin = Min;
        self.outMax = Max;

        if(self.inAuto):
           if(self.myOutput > self.outMax):
               self.myOutput = self.outMax;
           elif(self.myOutput < self.outMin):
               self.myOutput = self.outMin;

           if(self.outputSum > self.outMax):
               self.outputSum= outMax;
           elif(self.outputSum < self.outMin):
               self.outputSum= self.outMin;
   
    def SetMode(self, Mode):
        newAuto = (Mode == self.AUTOMATIC);
        if(newAuto and not self.inAuto):
            #/*we just went from manual to auto*/
            self.Initialize(self.lastInput);
        self.inAuto = newAuto;
    
    def Initialize(self, myInput):
        self.outputSum = self.myOutput;
        self.lastInput = myInput;
        if(self.outputSum > self.outMax):
            self.outputSum = self.outMax;
        elif(self.outputSum < self.outMin):
            self.outputSum = self.outMin;
    
    def SetControllerDirection(self, Direction):
        if(self.inAuto and (Direction !=self.controllerDirection)):
            self.kp = (0 - self.kp);
            self.ki = (0 - self.ki);
            self.kd = (0 - self.kd);
        self.controllerDirection = Direction;

# https://en.wikipedia.org/wiki/Ziegler–Nichols_method
# Applies the selected controller scheme from the table in the above page
# Ku is the P value selected to create stable oscillations
# Tu is the period of those oscillations
# calculated unused constants useful for making transfer functions
# See above for more info

def ziegler_nichols(kind, Ku, Tu):
    Kp = 0
    Ki = 0
    Kd = 0

    ct = kind.lower()
    if ct == "p": # proportional
        Kp = 0.5 * Ku
    elif ct == "pi": # proportional integral
        Kp = 0.45 * Ku
        Ti = 0.83 * Tu
        Ki = (0.54 * Ku) / Tu
    elif ct == "pd": # proportional differential
        Kp = 0.8 * Ku
        Td = 0.125 * Tu
        Kd = 0.10 * Ku * Tu
    elif ct == "pid": # classic PID
        Kp = 0.6 * Ku
        Ti = 0.5 * Tu
        Td = 0.125 * Tu
        Ki = (1.2 * Ku) / Tu
        Kd = 0.075 * Ku * Tu
    elif ct == "pir": # Pessen Integrate Rule
        Kp = 0.7 * Ku
        Ti = 0.4 * Tu
        Td = 0.15 * Tu
        Ki = (1.75 * Ku) / Tu
        Kd = 0.105 * Ku * Tu
    elif ct == "so": # some overshoot
        Kp = 0.33 * Ku
        Ti = 0.5 * Tu
        Td = 0.33 * Tu
        Ki = (0.66 * Ku) / Tu
        Kd = 0.11 * Ku * Tu
    elif ct == "no": # no overshoot
        Kp = 0.2 * Ku
        Ti = 0.5 * Tu
        Td = 0.33 * Tu
        Ki = (0.4 * Ku) / Tu
        Kd = 0.066 * Ku * Tu
    else:
        raise ValueError(f"Uknown control type {kind}")
    return (Kp, Ki, Kd)



