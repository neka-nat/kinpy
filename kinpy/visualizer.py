import os
import numpy as np
import vtk
from vtk.util.colors import tomato
import transformations as tf
from . import chain
from . import transform

class Visualizer(object):
    def __init__(self, win_size=(640, 480)):
        self._ren = vtk.vtkRenderer()
        self._ren.SetBackground(0.1, 0.2, 0.4)
        self._win = vtk.vtkRenderWindow()
        self._win.SetSize(*win_size)
        self._win.AddRenderer(self._ren)
        self._inter = vtk.vtkRenderWindowInteractor()
        self._inter.SetRenderWindow(self._win)

    def add_robot(self, transformations, visuals_map,
                  mesh_file_path='./', axes=False):
        for k, trans in transformations.items():
            if axes:
                self.add_axes(trans)
            for v in visuals_map[k]:
                tf = trans * v.offset
                if v.geom_type == 'mesh':
                    self.add_mesh(os.path.join(mesh_file_path, v.geom_param), tf)
                elif v.geom_type == 'cylinder':
                    self.add_cylinder(v.geom_param[0], v.geom_param[1], tf)
                elif v.geom_type == 'box':
                    self.add_box(v.geom_param, tf)
                elif v.geom_type == 'sphere':
                    self.add_sphere(v.geom_param, tf)
                elif v.geom_type == 'capsule':
                    self.add_capsule(v.geom_param[0], v.geom_param[1], tf)

    def add_shape_source(self, source, transform):
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(tomato)
        actor.SetPosition(transform.pos)
        rpy = np.rad2deg(tf.euler_from_quaternion(transform.rot, 'rxyz'))
        actor.RotateX(rpy[0])
        actor.RotateY(rpy[1])
        actor.RotateZ(rpy[2])
        self._ren.AddActor(actor)

    def add_axes(self, trans):
        transform = vtk.vtkTransform()
        transform.Translate(trans.pos)
        rpy = np.rad2deg(tf.euler_from_quaternion(trans.rot, 'rxyz'))
        transform.RotateX(rpy[0])
        transform.RotateY(rpy[1])
        transform.RotateZ(rpy[2])
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.1, 0.1, 0.1)
        axes.AxisLabelsOff()
        axes.SetUserTransform(transform)
        self._ren.AddActor(axes)

    def load_obj(self, filename):
        reader = vtk.vtkOBJReader()
        reader.SetFileName(filename)
        return reader

    def load_ply(self, filename):
        reader = vtk.vtkPLYReader()
        reader.SetFileName(filename)
        return reader

    def load_stl(self, filename):
        reader = vtk.vtkSTLReader()
        reader.SetFileName(filename)
        return reader

    def add_cylinder(self, radius, height, tf=transform.Transform()):
        cylinder = vtk.vtkCylinderSource()
        cylinder.SetResolution(20)
        cylinder.SetRadius(radius)
        cylinder.SetHeight(height)
        self.add_shape_source(cylinder, tf)

    def add_box(self, size, tf=transform.Transform()):
        cube = vtk.vtkCubeSource()
        cube.SetXLength(size[0])
        cube.SetYLength(size[1])
        cube.SetZLength(size[2])
        self.add_shape_source(cube, tf)

    def add_sphere(self, radius, tf=transform.Transform()):
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(radius)
        self.add_shape_source(sphere, tf)

    def add_capsule(self, radius, fromto, tf=transform.Transform(),
                    step=0.05):
        for t in np.arange(0.0, 1.0, step):
            trans = transform.Transform(pos= t * fromto[:3] + (1.0 - t) * fromto[3:])
            self.add_sphere(radius, tf * trans)

    def add_mesh(self, filename, tf=transform.Transform()):
        _, ext = os.path.splitext(filename)
        ext = ext.lower()
        if ext == '.stl':
            reader = self.load_stl(filename)
        elif ext == '.obj':
            reader = self.load_obj(filename)
        elif ext == '.ply':
            reader = self.load_ply(filename)
        else:
            raise ValueError("Unsupported file extension, '%s'." % ext)
        self.add_shape_source(reader, tf)

    def spin(self):
        self._win.Render()
        self._inter.Initialize()
        self._inter.Start()
