"""Sample Webots controller for the wall following benchmark."""

from controller import Robot

def getDistance(sensor):
    """
    Return the distance of an obstacle for a sensor.

    The value returned by the getValue() method of the distance sensors
    corresponds to a physical value (here we have a sonar, so it is the
    strength of the sonar ray). This function makes a conversion to a
    distance value in meters.
    """
    distance = ((1000 - sensor.getValue()) / 1000) * 5
    return distance

class MyRobot:
    # Maximum speed for the velocity value of the wheels.
    # Don't change this value.
    MAX_SPEED = 5.24
    
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.front = [self.sensor(0),
                      self.sensor(1),
                      self.sensor(2),
                      self.sensor(3),
                      self.sensor(4),
                      self.sensor(5),
                      self.sensor(6),
                      self.sensor(7)]
        self.back  = [self.sensor(8),
                      self.sensor(9),
                      self.sensor(10),
                      self.sensor(11),
                      self.sensor(12),
                      self.sensor(13),
                      self.sensor(14),
                      self.sensor(15)]
        self.wheels = [self.wheel('left'), self.wheel('right')]
        self.stage = 0
        
    def sensor(self, number):
        name = 'so%d' % number
        sensor = self.robot.getDistanceSensor(name)
        sensor.enable(self.timestep)
        return sensor
        
    def wheel(self, name):
        name = '%s wheel' % name
        wheel = self.robot.getMotor(name)
        # We will use the velocity parameter of the wheels, so we need to
        # set the target position to infinity.
        wheel.setPosition(float('inf'))
        return wheel
        
    def front_sensor(self):
        return self.front[3]
        
    def side_sensor(self):
        return self.front[0]
                       
    def dump_status(self):
        time_ms = int(self.robot.getTime()*1000)
        status = 'time:%05d [ms] stage: %2d front[cm]: ' % (time_ms, self.stage)
        for sensor in self.front:
            distance = int(getDistance(sensor)*100)
            if distance >= 500:
                status += '--- '
            else:
                status += '%03d ' % distance
        status += 'back[cm]: '
        for sensor in self.back:
            distance = int(getDistance(sensor)*100)
            if distance >= 500:
                status += '--- '
            else:
                status += '%03d ' % distance
        status += 'wheels[cm/s]: '
        for wheel in self.wheels:
            velocity = int(wheel.getVelocity()*100)
            status += '%03d ' % velocity
        print status

    def update(self):
        next = self.robot.step(self.timestep)
        self.dump_status()
        return next
        
    def set_stage(self, stage):
        self.stage = stage
        return None
        
    def left(self):
        return self.wheels[0]
        
    def right(self):
        return self.wheels[1]
        
    def set_velocity(self, left_velocity, right_velocity):
        self.left().setVelocity(left_velocity*self.MAX_SPEED)
        self.right().setVelocity(right_velocity*self.MAX_SPEED)

# Get pointer to the robot.
robot = MyRobot()

# Get pointer to the robot wheels motors.
leftWheel = robot.left()
rightWheel = robot.right()

# Get and enable the distance sensors.
frontSensor = robot.front_sensor()
sideSensor = robot.side_sensor()

print 'MyRobot methods: ', dir(robot)
print 'Robot methods: ', dir(robot.robot)
print 'Wheel methods: ', dir(leftWheel)
print 'Sensor methods: ', dir(frontSensor)

# Move forward until we are 50 cm away from the wall.
robot.set_velocity(1, 1)
robot.set_stage(1)
while robot.update() != -1:
    if getDistance(frontSensor) < 0.5:
        break

# Rotate clockwise until the wall is to our left.
robot.set_velocity(1, -1)
robot.set_stage(2)
while robot.update() != -1:
    # Rotate until there is a wall to our left, and nothing in front of us.
    if getDistance(sideSensor) < 1:
        break

# Main loop.
robot.set_stage(10)
while robot.update() != -1:
    # Too close to the wall, we need to turn right.
    if getDistance(sideSensor) < 0.4:
        robot.set_stage(11)
        robot.set_velocity(1, 0.9)
    # Too far from the wall, we need to turn left.
    elif getDistance(sideSensor) > 0.6:
        robot.set_stage(12)
        robot.set_velocity(0.9, 1)
    # We are in the right direction.
    else:
        robot.set_stage(10)
        robot.set_velocity(1, 1)

# Stop the robot when we are done.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
