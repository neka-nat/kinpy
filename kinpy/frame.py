import numpy as np
import transformations as tf
from . import transform


class Visual(object):
    TYPES = ['box', 'cylinder', 'sphere', 'capsule', 'mesh']
    def __init__(self, offset=transform.Transform(),
                 geom_type=None, geom_param=None):
        self.offset = offset
        self.geom_type = geom_type
        self.geom_param = geom_param

    def __repr__(self):
        return "Visual(offset={0}, geom_type='{1}', geom_param={2})".format(self.offset,
                                                                            self.geom_type,
                                                                            self.geom_param)

class Link(object):
    def __init__(self, name=None, offset=transform.Transform(),
                 visuals=[]):
        self.name = name
        self.offset = offset
        self.visuals = visuals

    def __repr__(self):
        return "Link(name='{0}', offset={1}, visuals={2})".format(self.name,
                                                                  self.offset,
                                                                  self.visuals)


class Joint(object):
    TYPES = ['fixed', 'revolute', 'prismatic']
    def __init__(self, name=None, offset=transform.Transform(),
                 joint_type='fixed', axis=[0.0, 0.0, 1.0]):
        self.name = name
        self.offset = offset
        self.joint_type = joint_type
        if self.joint_type != 'fixed' and axis is None:
            self.axis = np.array([0.0, 0.0, 1.0])
        else:
            self.axis = np.array(axis)

    def __repr__(self):
        return "Joint(name='{0}', offset={1}, joint_type='{2}', axis={3})".format(self.name,
                                                                                  self.offset,
                                                                                  self.joint_type,
                                                                                  self.axis)


class Frame(object):
    def __init__(self, name=None, link=Link(),
                 joint=Joint(), children=[]):
        self.name = 'None' if name is None else name
        self.link = link
        self.joint = joint
        self.children = children

    def __str__(self, level=0):
        ret = " \t" * level + self.name + "\n"
        for child in self.children:
            ret += child.__str__(level + 1)
        return ret

    def add_child(self, child):
        self.children.append(child)

    def is_end(self):
        return (len(self.children) == 0)

    def get_transform(self, theta):
        if self.joint.joint_type == 'revolute':
            t = transform.Transform(tf.quaternion_about_axis(theta, self.joint.axis))
        elif self.joint.joint_type == 'prismatic':
            t = transform.Transform(pos=theta * self.joint.axis)
        elif self.joint.joint_type == 'fixed':
            t = transform.Transform()
        else:
            raise ValueError("Unsupported joint type %s." % self.joint.joint_type)
        return self.joint.offset * t