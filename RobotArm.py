#!/usr/bin/env python
'''
Created on Feb 20, 2013

@author: matth
'''

from Phidgets.Devices.Stepper import Stepper
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, \
    ErrorEventArgs, InputChangeEventArgs, CurrentChangeEventArgs, \
    StepperPositionChangeEventArgs, VelocityChangeEventArgs
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Devices.AdvancedServo import AdvancedServo
from Phidgets.Devices.Servo import ServoTypes
from Phidgets.Devices.InterfaceKit import InterfaceKit
from Phidgets.Phidget import *

#Basic imports
from ctypes import *
from time import *

import threading
import sys
import types
import traceback
import cmd
import os
import termios
import fcntl
import math


#Project imports
from PhidgetMux import *

def getch():
    fd = sys.stdin.fileno()
    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)

    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

    try:
        while 1:            
            try:
                c = sys.stdin.read(1)
                break
            except IOError: 
                pass
    finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
            fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
            return c


"""
LSpecError - Exception class for link errors
"""
class LSpecError(Exception):
    def __init__(self, value, message = None):
        self._code = value
        self._message = message
    def __str__(self):
        return repr(self.value)
    
    NOT_CALIBRATED_AND_NO_LIMITSWITCH = 1
    LIMIT_NOT_REACHED = 2
    NOT_CALIBRATED = 3
    MOVE_EXCEEDS_LIMIT = 4
    COLLISION_POSSIBLE = 5

'''
LinkSpec - Class to carry link specs and implement link management code
'''
class LinkSpec:
    '''
    Initializer:
        name   - The name of the link 
        jType  - The kind of link: 'r' for rotational, 'p' for linear Pythagorean
        motor  - Reference to a motor device object
        longLimitSwitch - Reference to a switch device object that is tripped at the low limit of
                    motion. Set to None if no switch is present.
        shortLimitSwitch - Reference to a switch device object that is tripped at the short limit of
                    motion. Set to None if no switch is present.
        resolution - The step resolution after all gearing and microstepping. For 'r' type
                    links this would be expressed in degrees.
        stepsBetweenLimitsHint - The not-to-exceed number of raw steps between limit switches. 
                    This is used as a hint to tell us how many steps it should take to find one
                    limit or the other.
        stepsBetweenLimits - The absolute number of raw steps between limit switches. This is used as
                    to tell us the distance to the other limit, assuming we use the opposite limit as
                    a calibration point.
        switchHysteresisSteps - The number of steps required to overcome the hysteresis in the
                    limit switch. This is a function of the switch itself and the mechanical
                    arrangement of the arm.
        acceleration - The base acceleration for this arm, in steps/second^2
        velocityLimit - The top speed for the stepper motor

    '''
    def __init__(self, name, motor, longLimitSwitch, shortLimitSwitch, 
                 resolution, stepsBetweenLimitsHint, stepsBetweenLimits, 
                 switchHysteresisSteps, acceleration, velocityLimit):
        self._name = name
        self._motor = motor
        self._longLimitSwitch = longLimitSwitch
        self._shortLimitSwitch = shortLimitSwitch
        self._resolution = resolution
        self._stepsBetweenLimitsHint = stepsBetweenLimitsHint
        self._stepsBetweenLimits = stepsBetweenLimits
        self._switchHysteresisSteps = switchHysteresisSteps
        self._acceleration = acceleration
        self._velocityLimit = velocityLimit    
        self._calibrated = False

    
    def eStop(self):
        """eStop - Emergency Stop
        This function stops all movement in the particular arm, and marks the arm as uncalibrated.
        """
        pass

    
    def setStandbyMode(self, state):
        self._motor.setStandbyMode(state)

    
    def setEngaged(self, state):
        self._motor.setEngaged(state)


    def setLimitSwitchFunctions(self, longLimitSwitchFunction = None, shortLimitSwitchFunction = None):
        """Function to set and clear limit switch handlers.
        
        Parameters:
            longLimitSwitchFunction - Function to call when low limit switch is hit
            shortLimitSwitchFunction - Function to call when short limit switch is hit
        
        Both parameters default to "None".
        
        Note: cancelLimitSwitchFunctions is an alias of this function, and is intended to be called
        with no arguments.
        """
        if(self._longLimitSwitch):
            self._longLimitSwitch.setOnInputChangeHandler(None)
        
        if(self._shortLimitSwitch):
            self._shortLimitSwitch.setOnInputChangeHandler(None)

    # This function is just an alias of setLimitSwitches, but with no parameters
    cancelLimitSwitchFunctions = setLimitSwitchFunctions

        
    def calibrate(self):
        """Calibrate the link
        This function may cause the link to run between its limits. Be sure that it can do
        that without crashing.
        
        Determining how to calibrate: basically we need to run to a limit. We try for a short
        limit, but of there's no short limit switch we try for a long limit. Obviously we're 
        screwed if we can't find one of those either.
        
        Exceptions:
            NOT_CALIBRATED_AND_NO_LIMITSWITCH if no limit switch can be identified
        """
        if(self._shortLimitSwitch):
            self.runToShortLimit()
        elif(self._longLimitSwitch):
            self.runToLongLimit()
        else:
            raise LSpecError(LSpecError.NOT_CALIBRATED_AND_NO_LIMITSWITCH)
        
    def jog(self, direction, steps):
        """Jog the arm in one direction or another. This function causes unchecked movement
        in one direction or another, so be sure the movement won't cause a crash.
        The motor's position is updated correctly, so calibration is not changed.
        """
        # Need a little finesse because we specify the direction and number of
        # steps instead of an absolute position.
        if(direction):
            steps *= -1

        self._targetPosition = self._motor.getCurrentPosition() + steps

        self._motor.setEngaged(True)
        self._motor.setAcceleration(self._acceleration)
        self._motor.setVelocityLimit(self._velocityLimit)            
        self._motor.setTargetPosition(self._targetPosition)

        # Sit and wait for the motor to stop.
        while(not self._motor.getStopped()):
            sleep(.1)

        
    def runToLimit(self, direction, limitSwitch):
        """Generic run to limit function. Provide the direction as a boolean and the
        switch object. The link must be in its calibrated state if moving to a limit
        that does not have a switch. The link is recalibrated each time it moves to
        a limit that has a switch.
        
        Parameters:
            direction - Boolean indicating short (True) or long limit (False)
            limitSwitch - Reference to the limit switch governing this move

        Exceptions:
            LSpecError.NOT_CALIBRATED_AND_NO_LIMITSWITCH - Raised if the limit requested
                does not have a switch and the limit is not known.
            LSpecError.LIMIT_NOT_REACHED - Raised if the motor traveled an amount that should
                have triggered the limit switch without triggering it.

        """
        self._limitSwitch = limitSwitch        
        self._dirString = "long"
        if(direction):
            self._dirString = "short"
        
        if(limitSwitch == None):
            if(self._calibrated == False):
                raise LSpecError(LSpecError.NOT_CALIBRATED_AND_NO_LIMITSWITCH)

            # No limit switch, but we're calibrated and so know where we are.
            print "Running {} to {} limit (dead reckoning).".format(self._name, self._dirString)
            self._targetPosition = self._stepsBetweenLimits
            if(direction):
                self._targetPosition = 0
        else:
            # Limit switch is present, so we're taking the opportunity to re-calibrate.
            print "Running {} to {} limit (limit switch).".format(self._name, self._dirString)
            if(limitSwitch.getInputState()):
                # At limit, need to jog
                self.jog(not direction, self._switchHysteresisSteps)

            # We don't use the setLimitSwitchFunction handler because there's only one limit switch and
            # it's difficult to determine where to put the parameter in the function :-(            
            self._limitSwitch.setOnInputChangeHandler(self.runToLimitSwitchHandler)

            self._motor.setCurrentPosition(0)
            self._targetPosition = self._stepsBetweenLimitsHint
            if(direction):
                self._motor.setCurrentPosition(self._stepsBetweenLimitsHint)
                self._targetPosition = 0

        # Now go
        self._motor.setEngaged(True)
        self.setEngaged(True)
        self._motor.setAcceleration(self._acceleration)
        self._motor.setVelocityLimit(self._velocityLimit)            
        self._motor.setTargetPosition(self._targetPosition)

        # Sit and wait for the motor to stop.
        while(not self._motor.getStopped()):
            sleep(.1)
                    
        self.cancelLimitSwitchFunctions()
        
        if(self._calibrated):
            if(direction):
                self._motor.setCurrentPosition(0)
            else:
                self._motor.setCurrentPosition(self._stepsBetweenLimits)
        else:
            # If we get here it's because the limit switch didn't trip. The one
            # non-error state is if we were in a calibrated state.
            raise LSpecError(LSpecError.LIMIT_NOT_REACHED)

        print "{} is at its {} limit.".format(self._name, self._dirString)


    def runToLimitSwitchHandler(self,e):
        '''
        Handle the low limit switch event by stopping the motor and resetting the inputChangeHandler
        to None. Also mark the link as calibrated and set up the current stepper position.
        '''
        self._motor.setCurrentPosition(0)
        self._calibrated = True
        self._limitSwitch.setOnInputChangeHandler(None)
        print "{} {} limit sensed.".format(self._name, self._dirString)


    def runToLongLimit(self):
        '''
        runToLongLimit - Run the motor until link reaches the long limit or the
        the long limit switch is hit.
        
        Exceptions:
            See runToLimit()
        '''
        self.runToLimit(False, self._longLimitSwitch)

    def runToShortLimit(self):
        '''
        runToShortLimit - Run the motor until it reaches the short limit or the
        the short limit switch is hit.

        Exceptions:
            See runToLimit()
        '''
        self.runToLimit(True, self._shortLimitSwitch)

class LeadscrewTriangle():
    """Solve the leadscrew triangle to determine 
    the lead screw length needed for the arm to reach a particular angle
    """    
    def __init__(self, aLength, bLength, baseAngle):
        """Initializer takes the dimensions in millimeters 
        of two of the known sides of the lead screw triangle
        and the base angle used to calculate the supplement of
        the incoming angle. Usually either 90 or 180.
        """
        self._aLength = aLength
        self._bLength = bLength
        self._baseAngle = baseAngle
    
    def length(self, theta):
        """Return the required length in millimeters of the leadscrew given
        the supplied theta value for the arm
        """
        # The theta that's passed in is actually the supplement of the desired
        # angle (angle C), so compute the supplement first. Turn it into radians
        # while we're at it.
        self._theta = math.radians(self._baseAngle - theta)

        # Now we have aLength, bLength and angle C. The task now is to solve
        # for side c.
        # Fortunately this kind of triangle lends itself to the classic solution:
        #    c**2 = a**2 + b**2 - 2ab * cos(C)
        return math.sqrt((self._aLength**2) + 
                         (self._bLength**2) - 
                         (2 * self._aLength * self._bLength * math.cos(self._theta)))
    
class StepperLeadScrewLinkSpec(LinkSpec):
    """Handle a link with a prismatic joint powered by a stepper motor connected 
    to a lead screw
    This subclass of LinkSpec is designed specifically for prismatic joints
    actuated by a stepper/leadscrew combination and is capable of moving the
    link to a particular angle.
    """
    def __init__(self, name, motor, longLimitSwitch, shortLimitSwitch, 
                 resolution, stepsBetweenLimitsHint, stepsBetweenLimits, 
                 switchHysteresisSteps, acceleration, velocityLimit, lowAngleLimit,
                 highAngleLimit, aLength, bLength, baseAngle):
        """Class to implement links with prismatic joints powered by stepper motor/lead screw combination
        
        Inherits from LinkSpec.
        
        Parameters:
               resolution - millimeters per step of the stepper motor 
               aLength - length in millimeters between the arm pivot and the pusher bar
               bLength - length in millimeters between the arm pivot and the stepper motor pivot
               lowAngleLimit - low joint limits, in degrees, of the joint. 
               highAngleLimit - high joint limits, in degrees, of the joint.
               other values as specified in the LinkSpec class.
        """
        LinkSpec.__init__(self, name, motor, longLimitSwitch, shortLimitSwitch, 
                          resolution, stepsBetweenLimitsHint, stepsBetweenLimits, 
                          switchHysteresisSteps, acceleration, velocityLimit)
        self._lowAngleLimit = lowAngleLimit
        self._highAngleLimit = highAngleLimit
        self._lst = LeadscrewTriangle(aLength, bLength, baseAngle)
        self._deadSteps = int(self._lst.length(highAngleLimit) / resolution )   # This will give us the number if steps in the 'dead' area of the lead screw
        
    def goToAngle(self, destinationAngle, waitForStop):
        """Set the joint to the designated angle
        
        Parameters:
            destinationAngle - The angle to which to move the joint.
            waitForStop -    A flag, which if true, will cause the function to wait until the
                            joint has stopped before returning.

        Exceptions:
            NOT_CALIBRATED - Raised if link has not been calibrated.
            MOVE_EXCEEDS_LIMIT - Raised if the proposed move is out of range of
              the link 
            COLLISION_POSSIBLE - Raised if the proposed move might cause a collision. 
              Note this is only a hint, and absence of this exception does not necessarily 
              give the move a clean bill of health.
        """

        if(not self._calibrated):
            raise LSpecError(LSpecError.NOT_CALIBRATED)

        if( (destinationAngle < self._lowAngleLimit) or (destinationAngle > self._highAngleLimit)):
            raise LSpecError(LSpecError.MOVE_EXCEEDS_LIMIT)

        # Target position is determined by calling the LeadscrewTriangle.length
        # method to tell us how long side c (the lead screw) of the
        # lead screw/arm/pusher triangle needs to be. We then compute stepper
        # position needed to achieve that length.
        self._targetPosition = int(self._lst.length(destinationAngle) / self._resolution) - self._deadSteps

        print "Moving link {} to {} degrees (stepper position {})".format(self._name, destinationAngle, self._targetPosition)

        # Now go. We engage here in case the motors weren't engaged.
        self._motor.setEngaged(True)
        self.setStandbyMode(False)
        self._motor.setAcceleration(self._acceleration)
        self._motor.setVelocityLimit(self._velocityLimit)
        self.setLimitSwitchFunctions(self.gotoAngleLimitSwitchHandler, self.gotoAngleLimitSwitchHandler)
        self._motor.setTargetPosition(self._targetPosition)

        # Sit and wait for the motor to stop if we've been asked to do so.
        while(waitForStop and (not self._motor.getStopped())):
            sleep(.1)
        
    def gotoAngleLimitSwitchHandler(self, e):
        '''
        Handle a limit switch event by stopping the motor and resetting the inputChangeHandler
        to None. Also mark the link as uncalibrated.
        '''
        self._motor.setCurrentPosition(0)
        self._calibrated = False
        self.cancelLimitSwitchHandlers()
        self.setStandbyMode(True)
        print "{}: Unexpected limit switch activation sensed. Motor stopped".format(self._name)

class StepperRotaryLinkSpec(LinkSpec):
    """Handle a link with a rotary joint powered by a stepper motor
    This subclass of LinkSpec is designed specifically for rotary joints and is capable of moving the link to a particular angle.
    """
    def __init__(self, name, motor, longLimitSwitch, shortLimitSwitch, 
                 resolution, stepsBetweenLimitsHint, stepsBetweenLimits, 
                 switchHysteresisSteps, acceleration, velocityLimit, lowAngleLimit,
                 highAngleLimit):
        """Class to implement links with rotary joints powered by stepper motor
        
        Parameters:
                   lowLimit, highLimit - low and high joint limits, in degrees, of the joint.
                    other values as specified in superior class LinkSpec.
        """
        LinkSpec.__init__(self, name, motor, longLimitSwitch, shortLimitSwitch, 
                          resolution, stepsBetweenLimitsHint, stepsBetweenLimits, 
                          switchHysteresisSteps, acceleration, velocityLimit)
        self._lowAngleLimit = lowAngleLimit
        self._highAngleLimit = highAngleLimit

    def goToAngle(self, destinationAngle, waitForStop):
        """Set the joint to the designated angle
        
        Parameters:
            destinationAngle - The angle to which to move the joint.
            waitForStop -    A flag, which if true, will cause the function to wait until the
                            joint has stopped before returning.

        Exceptions:
            NOT_CALIBRATED - Raised if link has not been calibrated.
            MOVE_EXCEEDS_LIMIT - Raised if the proposed move is out of range of the link
            COLLISION_POSSIBLE - Raised if the proposed move might cause a collision. Note this is only
                                a hint, and absence of this exception does not necessarily give the move
                                a clean bill of health.
        """

        if(not self._calibrated):
            raise LSpecError(LSpecError.NOT_CALIBRATED)

        if( (destinationAngle < self._lowAngleLimit) or (destinationAngle > self._highAngleLimit)):
            raise LSpecError(LSpecError.MOVE_EXCEEDS_LIMIT)

        # Target position is determined by subtracting the destination angle from the lower angle limit to 
        # arrive at the absolute number of degrees form the point of view of the joint. From there we divide
        # by the motor resolution to arrive at the position in steps.
        self._targetPosition = int((destinationAngle - self._lowAngleLimit) / self._resolution)
        
        print "Moving link {} to {} degrees (stepper position {})".format(self._name, destinationAngle, self._targetPosition)
        # Now go. We engage here in case the motors weren't engaged.
        self._motor.setEngaged(True)
        self.setStandbyMode(False)
        self._motor.setAcceleration(self._acceleration)
        self._motor.setVelocityLimit(self._velocityLimit)
        self.setLimitSwitchFunctions(self.gotoAngleLimitSwitchHandler, self.gotoAngleLimitSwitchHandler)
        self._motor.setTargetPosition(self._targetPosition)

        # Sit and wait for the motor to stop if we've been asked to do so.
        while(waitForStop and (not self._motor.getStopped())):
            sleep(.1)
        
    def gotoAngleLimitSwitchHandler(self, e):
        '''
        Handle a limit switch event by stopping the motor and resetting the inputChangeHandler
        to None. Also mark the link as uncalibrated.
        '''
        self._motor.setCurrentPosition(0)
        self._calibrated = False
        self.cancelLimitSwitchHandlers()
        self.setStandbyMode(True)
        print "{}: Unexpected limit switch activation sensed. Motor stopped".format(self._name)
        
'''
Our device inventory:
'''
# Base stepper motor
m0 = StepperDevice(serialNum = 322147,
                   deviceIndex = 0,
                   cwIsPositive = True,
                   standbyCurrent = .5, 
                   operatingCurrent = 1.7)

# Base limit switch
ls1 = IKInputDevice(serialNum = 179249,
                    deviceIndex = 0)

# Link 1 stepper motor
m1 = StepperDevice(serialNum = 322156, 
                   deviceIndex = 0, 
                   cwIsPositive = True,
                   standbyCurrent = 0, 
                   operatingCurrent = 2.3)

# Link 1 limit switch
ls2 = IKInputDevice(serialNum = 179249,
                    deviceIndex = 1)

# Link 2 stepper motor
m2 = StepperDevice(serialNum = 322159, 
                   deviceIndex = 0, 
                   cwIsPositive = True,
                   standbyCurrent = 0, 
                   operatingCurrent = 2.3)

# Link 2 limit switch
ls4 = IKInputDevice(serialNum = 179249,
                    deviceIndex = 2)

# Gripper Rotate
m3 = AdvancedServoDevice(serialNum = 177147,
                         deviceIndex = 0)

# Gripper Open/Close
m4 = AdvancedServoDevice(serialNum = 177147,
                         deviceIndex = 1)

# Wrist Rotate
m5 = AdvancedServoDevice(serialNum = 177147,
                         deviceIndex = 2)

# Gripper Swing
m6 = AdvancedServoDevice(serialNum = 177147,
                         deviceIndex = 3)
   
'''
The first link. Its joint is operated by 
a stepper motor and it rotates.
The basic step resolution is 1.8 deg.
The controller micro-steps 16:1.
The motor has a 5.18:1 gear reduction
There is an additional belt-reduction with a 3 to 1 ratio.
So the final step resolution is: 0.00723938223938 degree, or 49,728 steps per revolution of the base
Limit switch 1 is the short limit

'''
l0 = StepperRotaryLinkSpec(name = "Base", 
           motor = m0,
           longLimitSwitch = ls1,
           shortLimitSwitch = None,
           resolution = 0.00723938223938,
           stepsBetweenLimitsHint = 31000,
           stepsBetweenLimits = 30864,
           switchHysteresisSteps = 3729,
           acceleration = 12000,
           velocityLimit = 4000,
           lowAngleLimit = 70,
           highAngleLimit = 290)

'''
The second link. Its joint is operated by
a stepper motor and it's linear.
The step resolution is 1.8 deg.
The controller micro-steps 16:1.
The stepper drives a lead screw that travels 25.4 mm every 20.8 revolutions.
The stepper drives a lead screw that travels 1.22115384615385 mm per revolution.
The lead screw triangle variable side is 95.25 mm

So the final step resolution is 0.00038161057692 mm/step (1.22115384615385 / 360 / 1.8 * 16)

In calculating the position, we have to subtract "dead space" from the lead screw length.

velocityLimit 12000

'''
l1 = StepperLeadScrewLinkSpec(name = "Link 1", 
           motor = m1, 
           longLimitSwitch = None, 
           shortLimitSwitch = ls2, 
           resolution = 0.00038161057692,
           stepsBetweenLimitsHint = 283200,
           stepsBetweenLimits = 249600, 
           switchHysteresisSteps = 17280,
           acceleration = 24000, 
           velocityLimit = 4000,
           lowAngleLimit = 10,
           highAngleLimit = 80,
           aLength = 88.62,
           bLength = 192,
           baseAngle = 90)

'''
The third link. Its joint is operated by
a stepper motor and it's linear.
The step resolution is 1.8 deg.
The controller micro-steps 16:1.
The stepper drives a lead screw that travels 1.22115384615385 mm per revolution.
The lead screw triangle variable side is 108 mm

So the final step resolution is 0.00038161057692 mm/step (1.22115384615385 / 360 / 1.8 * 16)
velocityLimit 40000
'''
l2 = StepperLeadScrewLinkSpec(name = "Link 2",
           motor = m2, 
           shortLimitSwitch = ls4, 
           longLimitSwitch = None, 
           resolution = 0.00038161057692, 
           stepsBetweenLimitsHint = 423040, 
           stepsBetweenLimits = 398080, 
           switchHysteresisSteps = 17280,
           acceleration = 80000, 
           velocityLimit = 4000,
           lowAngleLimit = 20,
           highAngleLimit = 155,
           aLength = 85,
           bLength = 205,
           baseAngle = 180)

'''
The third link. Its joint is operated by
a servomotor and it's linear.
'''
l3 = LinkSpec("Link 3", m3, None, None, 1, 180, 0, 0, 0, 0)

#
exitFlag = threading.Event()

class deviceTracker():
    """Device tracker class
    This class tracks the various sensing devices in the system and
    maintains a flag indicating whether they are all online or not.
    """
    def __init__(self, linkList):
        self.allDevicesOnline = threading.Event()
        self._linkList = linkList
        
    def start(self):
        """Start the device tracker - this function registers the
        attach/detach event handlers.
        """
        self.devCount = 0
        for i in self._linkList:
            if( i._motor != None): 
                i._motor.setOnAttachHandler(self.onAttach)
                i._motor.setOnDetachHandler(self.onDetach)
                self.devCount += 1
            if( i._longLimitSwitch != None):
                i._longLimitSwitch.setOnAttachHandler(self.onAttach)
                i._longLimitSwitch.setOnDetachHandler(self.onDetach)
                self.devCount += 1
            if( i._shortLimitSwitch != None):
                i._shortLimitSwitch.setOnAttachHandler(self.onAttach)
                i._shortLimitSwitch.setOnDetachHandler(self.onDetach)
                self.devCount += 1
        self.devMax = self.devCount

    def stop(self):
        for i in self._linkList:
            if( i._motor != None): 
                i._motor.setOnAttachHandler(None)
            if( i._longLimitSwitch != None):
                i._longLimitSwitch.setOnAttachHandler(None)
            if( i._shortLimitSwitch != None):
                i._shortLimitSwitch.setOnAttachHandler(None)
                self.devCount = 0
    
    def onAttach(self, e):
        self.devCount -= 1
#        print "Attach event: device {} just attached. {} device(s) of {} are currently online".format(e.getSerialNum(), self.devMax - self.devCount, self.devMax)
        if(self.devCount == 0):
            print "All devices are now online"
            self.allDevicesOnline.set()
            
    def onDetach(self, e):
        self.devCount += 1
        print "Detach event: {} device(s) currently offline".format(self.devCount)
        self.allDevicesOnline.clear()
        
    def wait(self, timeout = None):
        try:
            self.allDevicesOnline.wait(timeout)
        except:
            raise
        
class iKine():
    """Perform inverse kinematic calculations for the arm
    Given a tuple of three values that define the cartesian coordinates of a
    point in space, this class will tell you the angles required for the base,
    link 1, , and link 2, and will tell you the offset from 90 degrees for
    link 2 so that the manipulator can be positioned.
    All values are in millimeters or degrees.
    """
    
    def __init__(self, xOffset, yOffset, zOffset, link1Length, link2Length):
        self._xOffset = xOffset
        self._yOffset = yOffset
        self._zOffset = zOffset
        self._link1Length = link1Length
        self._link2Length = link2Length
    
    def convert(self, p):
        """convert - perform the conversion from point to angles
        p is a tuple of three values representing a point in space.
        """
        self.x, self.y, self.z = p
        
        ''' Calculate the angle needed for the base. We do this by drawing a right triangle,
        whose base length is represented by the value of x and whose altitude is
        represented by the value of y. The Pythagorean theorem then tells us the
        length of the hypotenuse. Given that information we can determine the
        value of the base-hypotenuse angle and from there the angle at the base
        of the triangle.
        '''
        self.c = math.hypot(self.x, self.y)    # Break out the hypotenuse length so we can use it later.
        self._theta0 = 90 - math.degrees(math.acos(self.y/self.c))

        '''Calculate the values for theta1, theta2, and theta3.
        '''
        
        if(self.c > self._link1Length):
            self._theta2 = math.acos(((self._link2Length**2) + (self._link1Length**2) - (self.c**2)) / (2 * self._link2Length * self._link1Length))
            self._anglecd = math.asin((self.z * (math.sin(self._theta2))) / self.c)
            self._anglebc = math.asin((self._link2Length * math.sin(self._theta2)) / self.c)
            self._theta1 = self._anglebc + self._anglecd
        else:
            self._angleac = math.acos(((self._link2Length**2) + (self.c**2) - (self._link1Length**2)) / (2 * self._link2Length * self.c))
            self._theta2 = math.asin((self.c * math.sin(self._angleac)) / self._link1Length)
            self._anglecd = math.asin((self.z * (math.sin(self._theta2))) / self.c)
            self._anglebc = math.radians(180) - self._theta2 - self._angleac
            self._theta1 = self._anglebc + self._anglecd

        self._theta3 = math.radians(270) - self._anglecd - self._anglebc - self._theta2
        
        return self._theta0, math.degrees(self._theta1), math.degrees(self._theta2), math.degrees(self._theta3)
    
    
    def theta0(self):
        """Return theta 0 value (base angle) - theta0 is already converted to degrees.
        """
        return self._theta0
    
    def theta1(self):
        """Return theta 1 value (link 1 angle)
        """
        return math.degrees(self._theta1)
    
    def theta2(self):
        """Return theta 2 value (link 2 angle)
        """
        return math.degrees(self._theta2)
    
    def theta3(self):
        """Return theta 3 value (offset from 90 degrees for link 2)
        """
        return math.degrees(self._theta3)
            

class arm():
    def __init__(self, links):
        self._links = links
        self._kine = iKine(xOffset = 0, 
                           yOffset = 0, 
                           zOffset = 0, 
                           link1Length = 465, 
                           link2Length = 330)

    def start(self):        
        # Start the device tracker and wait for devices to come online
        self._dt = deviceTracker(self._links)
        self._dt.start()

        # Open each device
        for i in self._links:
            if( i._motor != None): 
                i._motor.openDevice()
            if(i._longLimitSwitch != None):
                i._longLimitSwitch.openDevice()
            if(i._shortLimitSwitch != None):
                i._shortLimitSwitch.openDevice()
    
        print "Waiting for devices to connect..."
        try:
            self._dt.wait(10) 
        except:
            print "Timed out"

    def goToCoordinates(self, x, y, z):
        print "X: {}  Y: {}  Z: {}".format(x, y, z)
        self._kine.convert((x, y, z))
        self.goToAngles(self._kine.theta0(), self._kine.theta1(), self._kine.theta2())

    def goToAngles(self, theta0, theta1, theta2):
        print "Theta 0: {}  Theta 1: {}  Theta 2: {}".format(theta0, theta1, theta2)
        self._links[0].goToAngle(theta0, False)
        self._links[1].goToAngle(theta1, False)
        self._links[2].goToAngle(theta2, False)
        
        
        
    def setStandbyMode(self, state):
        """Set standby current or operational current
        """
        for i in self._links:
            i.setStandbyMode(state)

    def setEngaged(self, state):
        """Engage or disengage all of the motors
        """
        for i in self._links:
            i.setEngaged(state)
        
    def stop(self):
        '''Stop the arm and its processes
        '''
        self._dt.stop()
        self.setEngaged(False)

    def getLink(self, whichLink):
        return(self._links[whichLink])
    
    def calibrate(self):
        """Calibrate the arms. This involves running them to their
        limits. The algorithm is as follows:
        1. Run link 1 to its short limit
        2. Run link 2 to its short limit
        3. Run link 0 to its short limit
        """
        self.setEngaged(True)   # We don't want any motors to drift...
        self._links[1].calibrate()
        self._links[2].calibrate()
        self._links[0].calibrate()        
        self.setStandbyMode(True)


    def park(self):
        """Park the arms
        The arms are parked in their compact form to fit in the
        30 x 30 cm space required for the contest.
        """
        self.setEngaged(True)   # We don't want any motors to drift...
        self.calibrate()
        self._links[0].goToAngle(270, True)
        self._links[2].runToLongLimit()
        # TODO: Fold gripper
        self.setStandbyMode(True)


    def ready(self):
        """Place the arm in the "Ready" pose- basically with link 0
        pointed at 180, link 1 at its short limit, link 2 at 90, and
        the gripper at 180.
        """
        self.setEngaged(True)   # We don't want any motors to drift...
        self.calibrate()
        self._links[0].goToAngle(180, True)
        self._links[2].runToLongLimit()
        # TODO: set gripper to 180
        self.setStandbyMode(True)
        
                
class robotCmd(cmd.Cmd):
    def __init__(self, arm):
        cmd.Cmd.__init__(self)
        self.prompt = 'Ready: '
        self._arm = arm
        self._link = self._arm.getLink(0)

    def do_calibrate(self, arg):
        self._arm.calibrate()

    def do_goto(self, arg):
        x = arg.split()
        if(len(x) != 3):
            print "must have three arguments!"
            return

        myArm.goToCoordinates(int(x[0]), int(x[1]), int(x[2]))


    def do_park(self, arg):
        myArm.park()


    def do_ready(self, arg):
        myArm.ready()

        
    def do_link(self, arg):
        """ select the link to work with
        """
        if(arg):
            self._link = self._arm.getLink(int(arg))

        print "Now working with link '{}'.".format(self._link._name)

        
    def do_runtolongLimit(self, arg):
        '''Run the selected link to its low limit
        '''
        try:
            self._link.runToLongLimit()
        except:
            print "Exception"
        finally:
            pass
    
    def do_runtoshortlimit(self, arg):
        '''Run to the link short limit
        '''
        try:
            self._link.runToShortLimit()
        except:
            print "Exception"
        finally:
            pass

    def do_angle(self, arg):
        x = arg.split()
        if(len(x) != 1):
            print "must have one argument (the angle!)"
            return

        try:
            self._link.goToAngle(float(x[0]), True)
        except :
            print "Exception"
        finally:
            self._link.setStandbyMode(True)
        
    def do_quit(self, arg):
        '''Quit the Robot Arm command processor
        '''
        exitFlag.set()
        myArm.stop()
        print "Bye bye..."
        sys.exit(0)
        
    def do_devices(self, arg):
        '''
        Display the known devices and selected information about them.
        '''
        print("|------------|------------------------------------------------|--------------|------------|")
        print("|- Attached -|-                  Name/Type                   -|- Serial No. -|-  Version -|")
        print("|------------|------------------------------------------------|--------------|------------|")
        for i in arm._links:
            for j in [i._motor, i._longLimitSwitch, i._shortLimitSwitch]:
                if(j != None):
                    print("|- %8s -|- %44s -|- %10d -|- %8d -|" % 
                      (j.isAttached(), j.getDeviceName(), j.getSerialNum(), j.getDeviceVersion()))    
        
        print("|------------|------------------------------------------------|--------------|------------|")

    # Shortcuts
    do_q = do_quit
    do_rtll = do_runtolongLimit
    do_rtsl = do_runtoshortlimit
    do_cal = do_calibrate

#
# Main Program Code   
if __name__ == '__main__':
    myArm = arm([l0, l1, l2])
    myArm.start()
    rc = robotCmd(myArm)
    rc.cmdloop()

    myArm.stop()        
    exitFlag.set()
