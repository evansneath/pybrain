__author__ = 'Thomas Rueckstiess, ruecksti@in.tum.de'

import ode
import xode
from pybrain.utilities import Named
import sys
import warnings


class Actuator(Named):
    """Actuator class
    
    The base Actuator class. Every actuator has a name, and a list of values
    (even if it is only one value) with numValues entries. They can be added
    to the ODEEnvironment with its addActuator(...) function. Actuators
    receive the world model when added to the world.
    """
    def __init__(self, name, numValues):
        self._numValues = numValues
        self.name = name
        self._world = None

    def _update(self, action):
        pass

    def _connect(self, world):
        self._world = world

    def setNumValues(self, numValues):
        self._numValues = numValues

    def getNumValues(self):
        return self._numValues


class JointActuator(Actuator):
    """JointActuator class
    
    This actuator parses the xode root node for all joints and applies a torque
    to the angles of each of them. Different joints have a different number of
    values (e.g. a hinge2 joints has two degrees of freedom, whereas a slider
    joint has only one). However, calling the function getValues(), will return
    a flat list of all the degrees of freedom of all joints.

    Inherits:
        Actuator: The base actuator class
    """ 
    def __init__(self, name='JointActuator'):
        """Initialize

        Initializes the JointActuator class.

        Arguments:
            name: The given name for the actuator object.
                (Default: 'JointActuator')
        """
        Actuator.__init__(self, name, 0)

        self._joints = []
        self._numValues = 0

        return

    def _parseJoints(self, node):
        """Parse Joints

        Parses all joint objects in the given XODE node along with all child
        nodes of the given node recursively.

        Arguments:
            node: The node to parse for joints.
        """
        if isinstance(node, xode.joint.Joint):
            # append joints to joint vector
            joint = node.getODEObject()
            joint.name = node.getName()
            self._joints.append(joint)
        # recursive call for children
        for c in node.getChildren():
            self._parseJoints(c)

        return

    def _connect(self, world):
        """Connect

        Associates an ODE world with the actuator object. All joints in the
        world are parsed and the total number of degrees of freedom are
        calculated.

        Arguments:
            world: The ODE world object to associate with this actuator.
        """
        Actuator._connect(self, world)

        # get XODE Root and parse its joints
        self._joints = []
        self._parseJoints(self._world.getXODERoot())

        # do initial update to get numValues
        self._numValues = self._countValues()

        return

    def _countValues(self):
        """Count Values

        Calculates all degrees of freedom from the total of all joint axes
        in the actuator object.

        Returns:
            An integer representing the number of degrees of freedom from all
            joints combined. This is the number of list items to provide in
            the _update() method 'action' argument.
        """
        num = 0

        for j in self._joints:
            if type(j) == ode.BallJoint:
                pass
            elif type(j) == ode.UniversalJoint:
                pass
            elif type(j) == ode.AMotor:
                num += j.getNumAxes()
            elif type(j) == ode.HingeJoint:
                num += 1
            elif type(j) == ode.Hinge2Joint:
                num += 2
            elif type(j) == ode.SliderJoint:
                pass

        return num

    def _update(self, action):
        """Update

        Given an list of torque values, these values will be applied to each
        joint in the actuator in the order in which they were parsed. Note
        that the length of the torque list must equal the number of actuator
        degrees of freedom. The torques given apply for one time step in the
        ODE simulation.

        Arguments:
            action: A list of torques to apply to each joint for one time step.
        """
        assert len(action) == self._numValues

        for j in self._joints:
            if type(j) == ode.BallJoint:
                # ball joints can't be controlled yet
                pass
            elif type(j) == ode.UniversalJoint:
                # universal joints can't be controlled yet (use patch from mailing list)
                pass
            elif type(j) == ode.AMotor:
                num = j.getNumAxes()
                torques = []
                for _ in range(num):
                    torques.append(action[0])
                    action = action[1:]
                for _ in range(3 - num):
                    torques.append(0)
                (t1, t2, t3) = torques
                j.addTorques(t1, t2, t3)
            elif type(j) == ode.HingeJoint:
                # hinge joints have only one axis to add torque to
                j.addTorque(action[0])
                action = action[1:]
            elif type(j) == ode.Hinge2Joint:
                # hinge2 joints can receive 2 torques for their 2 axes
                t1, t2 = action[0:2]
                action = action[2:]
                j.addTorques(t1, t2)
            elif type(j) == ode.SliderJoint:
                # slider joints are currently only used for touch sensors
                # therefore, you can (must) set a torque but it is not applied
                # to the joint.
                pass

        return


class JointVelocityActuator(JointActuator):
    """JointVelocityActuator class

    Acts in a similar fashion to the JointActutator class, however, all
    actions given to the _update() method are velocities relative to the
    first connected body of each joint.

    Inherits:
        Joint Actuator: The torque actuation class.
    """
    def __init__(self, name='JointVelocityActuator'):
        """Initialize

        Initializes the JointVelocityActuator class object.

        Arguments:
            name: The given name for the actuator.
                (Default: 'JointVelocityActuator')
        """
        JointActuator.__init__(self, name)
        return

    def _update(self, action):
        """Update

        Actuates all joint degrees of freedom by giving velocities to each
        axis.

        Arguments:
            action: A list of velocities to apply to each joint for one time
                step.
        """
        assert len(action) == self._numValues

        for joint in self._joints:
            if type(joint) == ode.BallJoint:
                # ball joints can't be controlled yet
                pass
            elif type(joint) == ode.UniversalJoint:
                # universal joints can't be controlled yet (use patch from mailing list)
                pass
            elif type(joint) == ode.AMotor:
                num_axes = joint.getNumAxes()
                velocities = []

                for axis in range(num_axes):
                    velocities.append(action[0])
                    action = action[1:]

                for _ in range(3 - num_axes):
                    velocities.append(0.0)

                (v1, v2, v3) = velocities

                joint.setParam(ode.ParamVel, v1)
                joint.setParam(ode.ParamVel2, v2)
                joint.setParam(ode.ParamVel3, v3)
            elif type(joint) == ode.HingeJoint:
                # hinge joints have only one axis
                joint.setParam(ode.ParamVel, action[0])
                action = action[1:]
            elif type(joint) == ode.Hinge2Joint:
                # hinge2 joints can receive 2 torques for their 2 axes
                v1, v2 = action[0:2]
                action = action[2:]

                joint.setParam(ode.ParamVel, v1)
                joint.setParam(ode.ParamVel2, v2)
            elif type(joint) == ode.SliderJoint:
                # slider joints are currently only used for touch sensors
                # therefore, you can (must) set a torque but it is not applied
                # to the joint.
                pass

        return


class SpecificJointActuator(JointActuator):
    '''This sensor takes a list of joint names, and controlls only their values.'''

    def __init__(self, jointNames, name=None):
        Actuator.__init__(self, name, 0)
        self._names = jointNames
        self._joints = []

    def _parseJoints(self, node=None):
        for name in self._names:
            try:
                self._joints.append(self._world.getXODERoot().namedChild(name).getODEObject())
            except KeyError:
                # the given object name is not found. output warning and quit.
                warnings.warn("Joint with name '%s' not found." % (name))
                sys.exit()

    def _connect(self, world):
        Actuator._connect(self, world)

        # get XODE Root and parse its joints
        self._joints = []
        self._parseJoints()

        # do initial update to get numValues
        self._numValues = self._countValues()


class CopyJointActuator(JointActuator):
    ''' This sensor takes a list of joint names and controls all joints at once (one single value,
        even for multiple hinges/amotors) '''

    def __init__(self, jointNames, name=None):
        Actuator.__init__(self, name, 0)
        self._names = jointNames
        self._joints = []

    def _parseJoints(self, node=None):
        for name in self._names:
            try:
                self._joints.append(self._world.getXODERoot().namedChild(name).getODEObject())
            except KeyError:
                # the given object name is not found. output warning and quit.
                warnings.warn("Joint with name '%s' not found." % (name))
                sys.exit()

    def _connect(self, world):
        Actuator._connect(self, world)

        # get XODE Root and parse its joints
        self._joints = []
        self._parseJoints()

        # pretend to have only one single value
        self._numValues = 1

    def _update(self, action):
        assert (len(action) == self._numValues)
        for j in self._joints:
            if type(j) == ode.BallJoint:
                # ball joints can't be controlled yet
                pass
            elif type(j) == ode.UniversalJoint:
                # universal joints can't be controlled yet (use patch from mailing list)
                pass
            elif type(j) == ode.AMotor:
                num = j.getNumAxes()
                torques = []
                for _ in range(num):
                    torques.append(action[0])
                for _ in range(3 - num):
                    torques.append(0)
                (t1, t2, t3) = torques
                j.addTorques(t1 * 10, t2 * 10, t3 * 10)
            elif type(j) == ode.HingeJoint:
                # hinge joints have only one axis to add torque to
                j.addTorque(action[0])
            elif type(j) == ode.Hinge2Joint:
                # hinge2 joints can receive 2 torques for their 2 axes
                t1, t2 = (action[0], action[0])
                j.addTorques(t1, t2)
            elif type(j) == ode.SliderJoint:
                # slider joints are currently only used for touch sensors
                # therefore, you can (must) set a torque but it is not applied
                # to the joint.
                pass


if __name__ == '__main__':
    pass
