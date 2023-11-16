import os
from collections import defaultdict
from functools import partial
from typing import Dict, List, Optional, Tuple, Union

import numpy as np
import transformations as tf
import vtk
from vtk.util.colors import tomato

from . import transform
from .chain import Chain
from .frame import Visual


class Visualizer:
    def __init__(self, win_size: Tuple[int, int] = (640, 480)) -> None:
        self._actors: Dict[str, list] = defaultdict(list)
        self._ren = vtk.vtkRenderer()
        self._ren.SetBackground(0.1, 0.2, 0.4)
        self._win = vtk.vtkRenderWindow()
        self._win.SetSize(*win_size)
        self._win.AddRenderer(self._ren)
        self._inter = vtk.vtkRenderWindowInteractor()
        self._inter.SetRenderWindow(self._win)

    def add_robot(
        self,
        transformations: Dict[str, transform.Transform],
        visuals_map: Dict[str, List[Visual]],
        mesh_file_path: str = "./",
        axes: bool = False,
    ) -> None:
        for k, trans in transformations.items():
            if axes:
                self.add_axes(trans)
            for v in visuals_map[k]:
                tf = trans * v.offset
                if v.geom_type == "mesh":
                    self.add_mesh(os.path.join(mesh_file_path, v.geom_param), tf, geom_name=k)
                elif v.geom_type == "cylinder":
                    self.add_cylinder(v.geom_param[0], v.geom_param[1], tf, geom_name=k)
                elif v.geom_type == "box":
                    self.add_box(v.geom_param, tf, geom_name=k)
                elif v.geom_type == "sphere":
                    self.add_sphere(v.geom_param, tf, geom_name=k)
                elif v.geom_type == "capsule":
                    self.add_capsule(v.geom_param[0], v.geom_param[1], tf, geom_name=k)

    def add_shape_source(
        self, source: vtk.vtkAbstractPolyDataReader, transform: transform.Transform, geom_name: Optional[str] = None
    ) -> None:
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(tomato)
        actor.SetPosition(transform.pos)
        rpy = np.rad2deg(tf.euler_from_quaternion(transform.rot, "rxyz"))
        actor.RotateX(rpy[0])
        actor.RotateY(rpy[1])
        actor.RotateZ(rpy[2])
        if geom_name:
            self._actors[geom_name].append(actor)
        self._ren.AddActor(actor)

    def add_axes(self, trans: transform.Transform) -> None:
        transform = vtk.vtkTransform()
        transform.Translate(trans.pos)
        rpy = np.rad2deg(tf.euler_from_quaternion(trans.rot, "rxyz"))
        transform.RotateX(rpy[0])
        transform.RotateY(rpy[1])
        transform.RotateZ(rpy[2])
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.1, 0.1, 0.1)
        axes.AxisLabelsOff()
        axes.SetUserTransform(transform)
        self._ren.AddActor(axes)

    def load_obj(self, filename: str) -> vtk.vtkOBJReader:
        reader = vtk.vtkOBJReader()
        reader.SetFileName(filename)
        return reader

    def load_ply(self, filename: str) -> vtk.vtkPLYReader:
        reader = vtk.vtkPLYReader()
        reader.SetFileName(filename)
        return reader

    def load_stl(self, filename: str) -> vtk.vtkSTLReader:
        reader = vtk.vtkSTLReader()
        reader.SetFileName(filename)
        return reader

    def add_cylinder(
        self, radius: float, height: float, tf: Optional[transform.Transform] = None, geom_name: Optional[str] = None
    ) -> None:
        tf = tf or transform.Transform()
        cylinder = vtk.vtkCylinderSource()
        cylinder.SetResolution(20)
        cylinder.SetRadius(radius)
        cylinder.SetHeight(height)
        self.add_shape_source(cylinder, tf, geom_name)

    def add_box(
        self, size: List[float], tf: Optional[transform.Transform] = None, geom_name: Optional[str] = None
    ) -> None:
        tf = tf or transform.Transform()
        cube = vtk.vtkCubeSource()
        cube.SetXLength(size[0])
        cube.SetYLength(size[1])
        cube.SetZLength(size[2])
        self.add_shape_source(cube, tf, geom_name)

    def add_sphere(
        self, radius: float, tf: Optional[transform.Transform] = None, geom_name: Optional[str] = None
    ) -> None:
        tf = tf or transform.Transform()
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(radius)
        self.add_shape_source(sphere, tf, geom_name)

    def add_capsule(
        self,
        radius: float,
        fromto: np.ndarray,
        tf: Optional[transform.Transform] = None,
        step: float = 0.05,
        geom_name: Optional[str] = None,
    ) -> None:
        tf = tf or transform.Transform()
        for t in np.arange(0.0, 1.0, step):
            trans = transform.Transform(pos=t * fromto[:3] + (1.0 - t) * fromto[3:])
            self.add_sphere(radius, tf * trans)

    def add_mesh(
        self, filename: str, tf: Optional[transform.Transform] = None, geom_name: Optional[str] = None
    ) -> None:
        tf = tf or transform.Transform()
        _, ext = os.path.splitext(filename)
        ext = ext.lower()
        if ext == ".stl":
            reader = self.load_stl(filename)
        elif ext == ".obj":
            reader = self.load_obj(filename)
        elif ext == ".ply":
            reader = self.load_ply(filename)
        else:
            raise ValueError("Unsupported file extension, '%s'." % ext)
        self.add_shape_source(reader, tf, geom_name)

    def spin(self) -> None:
        self._win.Render()
        self._inter.Initialize()
        self._inter.Start()


class JointAngleEditor(Visualizer):
    def __init__(
        self,
        chain: Chain,
        mesh_file_path: str = "./",
        axes: bool = False,
        initial_state: Optional[Union[Dict[str, float], List[float]]] = None,
    ) -> None:
        super().__init__()
        if initial_state is None:
            initial_state = {}
        self._chain = chain
        if isinstance(initial_state, (list, np.ndarray)):
            initial_state = {k: v for k, v in zip(self._chain.get_joint_parameter_names(), initial_state)}
        self._joint_angles: Dict[str, float] = initial_state
        self._visuals_map = self._chain.visuals_map()
        self.add_robot(
            self._chain.forward_kinematics(self._joint_angles, end_only=False),  # type: ignore
            self._visuals_map,
            mesh_file_path,
            axes,
        )
        self._sliders = self._set_joint_slider(chain)

    def _update_joint_angle(self, obj: vtk.vtkSliderWidget, event: str, joint_name: str) -> None:
        slider_rep = obj.GetRepresentation()
        self._joint_angles[joint_name] = np.deg2rad(slider_rep.GetValue())
        positions = self._chain.forward_kinematics(self._joint_angles, end_only=False)  # type: ignore
        for k, position in positions.items():
            actors = self._actors[k]
            for i, actor in enumerate(actors):
                actor.SetOrientation(0, 0, 0)
                trans = position * self._visuals_map[k][i].offset
                actor.SetPosition(trans.pos)
                rpy = np.rad2deg(tf.euler_from_quaternion(trans.rot, "rxyz"))
                actor.RotateX(rpy[0])
                actor.RotateY(rpy[1])
                actor.RotateZ(rpy[2])
                actor.Modified()
        self._win.Render()

    def _set_joint_slider(self, chain: Chain) -> List[vtk.vtkSliderWidget]:
        sliders = []
        for i, frame in enumerate(chain):
            if frame.joint.joint_type == "fixed":
                continue
            slider_rep = vtk.vtkSliderRepresentation2D()
            slider_rep.SetMinimumValue(-180)
            slider_rep.SetMaximumValue(180)
            slider_rep.SetValue(np.rad2deg(self._joint_angles[frame.joint.name]))
            slider_rep.SetSliderLength(0.05)
            slider_rep.SetSliderWidth(0.02)
            slider_rep.SetEndCapLength(0.01)
            slider_rep.SetEndCapWidth(0.02)
            slider_rep.GetPoint1Coordinate().SetCoordinateSystemToNormalizedDisplay()
            slider_rep.GetPoint1Coordinate().SetValue(0.75, 1.0 - 0.05 * i)
            slider_rep.GetPoint2Coordinate().SetCoordinateSystemToNormalizedDisplay()
            slider_rep.GetPoint2Coordinate().SetValue(0.98, 1.0 - 0.05 * i)

            # Set the background color of the slider representation
            slider_rep.GetSliderProperty().SetColor(0.2, 0.2, 0.2)
            slider_rep.GetTubeProperty().SetColor(0.7, 0.7, 0.7)
            slider_rep.GetCapProperty().SetColor(0.2, 0.2, 0.2)

            slider_widget = vtk.vtkSliderWidget()
            slider_widget.SetInteractor(self._inter)
            slider_widget.SetRepresentation(slider_rep)
            slider_widget.AddObserver(
                vtk.vtkCommand.InteractionEvent,
                partial(self._update_joint_angle, joint_name=frame.joint.name),
            )
            sliders.append(slider_widget)
        return sliders

    def spin(self) -> None:
        self._win.Render()
        self._inter.Initialize()
        for slider in self._sliders:
            slider.SetEnabled(True)
            slider.On()
        self._inter.Start()
