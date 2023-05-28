from controller import Robot

import sys

# Discovery of the robot

print 'Python version:', sys.version_info

class Dummy(object):
    pass

def particular_methods(an_object):
    all_methods = set(dir(an_object))
    all_methods -= set(dir(Dummy()))
    all_methods -= set(['__del__', '__getattr__'])
    all_methods -= set([m for m in all_methods if '__swig_' in m])
    return sorted(list(all_methods))
    
robot = Robot()
print 'Robot methods:', particular_methods(robot)

for i in range(100):
    device = robot.getDeviceByIndex(i)
    if device is None:
        break
    print 'device %d - %s -' % (i, device.getName()), particular_methods(device)
