import math
import PyKDL as kdl
from abc import ABC, abstractmethod
from typing import Union

# based on OROCOS Kinematics and Dynamics Library, licensed under LGPL v2.1
# https://github.com/orocos/orocos_kinematics_dynamics

class RotationalInterpolation(ABC):
    @abstractmethod
    def set_start_end(self, start: kdl.Rotation, end: kdl.Rotation) -> None:
        pass

    @abstractmethod
    def angle(self) -> float:
        pass

    @abstractmethod
    def position(self, theta: float) -> kdl.Rotation:
        pass

    @abstractmethod
    def velocity(self, theta: float, thetad: float) -> kdl.Vector:
        pass

    @abstractmethod
    def acceleration(self, theta: float, thetad: float, thetadd: float) -> kdl.Vector:
        pass

class RotationalInterpolationSingleAxis(RotationalInterpolation):
    def __init__(self):
        self._R_base_start = kdl.Rotation.Identity()
        self._R_base_end = kdl.Rotation.Identity()
        self._rot_start_end = kdl.Vector.Zero()
        self._angle = 0.0

    def set_start_end(self, start: kdl.Rotation, end: kdl.Rotation) -> None:
        self._R_base_start = kdl.Rotation(start)
        self._R_base_end = kdl.Rotation(end)
        R_start_end = self._R_base_start.Inverse() * self._R_base_end
        self._angle, self._rot_start_end = R_start_end.GetRotAngle()

    def angle(self) -> float:
        return self._angle

    def position(self, theta: float) -> kdl.Rotation:
        return self._R_base_start * kdl.Rotation.Rot2(self._rot_start_end, theta)

    def velocity(self, theta: float, thetad: float) -> kdl.Vector:
        return self._R_base_start * (self._rot_start_end * thetad)

    def acceleration(self, theta: float, thetad: float, thetadd: float) -> kdl.Vector:
        return self._R_base_start * (self._rot_start_end * thetadd)

class Path(ABC):
    @abstractmethod
    def position(self, s: float) -> kdl.Frame:
        pass

    @abstractmethod
    def velocity(self, s: float, sd: float) -> kdl.Twist:
        pass

    @abstractmethod
    def acceleration(self, s: float, sd: float, sdd: float) -> kdl.Twist:
        pass

    @abstractmethod
    def path_length(self) -> float:
        pass

    @abstractmethod
    def parameterized_length(self, s: float) -> float:
        pass

class PathLine(Path):
    def __init__(self, H_base_start: kdl.Frame, reference: Union[kdl.Frame, kdl.Twist], eqradius: float = 1.0):
        self._p_base_start = kdl.Vector(H_base_start.p)
        self._orient_interp = RotationalInterpolationSingleAxis()
        self._eqradius = eqradius

        if isinstance(reference, kdl.Frame):
            self._p_base_end = kdl.Vector(reference.p)
            self._orient_interp.set_start_end(H_base_start.M, reference.M)
        elif isinstance(reference, kdl.Twist):
            self._p_base_end = H_base_start.p + reference.vel
            self._orient_interp.set_start_end(H_base_start.M, H_base_start * kdl.Frame(kdl.Rotation.Rot(reference.rot, reference.rot.Norm()), reference.vel).M)
        else:
            raise ValueError('Reference must be either kdl.Frame or kdl.Twist')

        self._p_start_end = self._p_base_end - self._p_base_start

        distance = self._p_start_end.Normalize()
        alpha = self._orient_interp.angle()

        if alpha != 0 and alpha * self._eqradius > distance:
            self._pathlength = alpha * self._eqradius
            self._scale_rot = 1.0 / self._eqradius
            self._scale_lin = distance / self._pathlength
        elif distance != 0:
            self._pathlength = distance
            self._scale_rot = alpha / distance
            self._scale_lin = 1.0
        else:
            self._pathlength = 0.0
            self._scale_rot = 1.0
            self._scale_lin = 1.0

    def position(self, s: float) -> kdl.Frame:
        rot = self._orient_interp.position(s * self._scale_rot)
        pos = self._p_base_start + self._p_start_end * s * self._scale_lin
        return kdl.Frame(rot, pos)

    def velocity(self, s: float, sd: float) -> kdl.Twist:
        pos = self._p_start_end * sd * self._scale_lin
        rot = self._orient_interp.velocity(s * self._scale_rot, sd * self._scale_rot)
        return kdl.Twist(pos, rot)

    def acceleration(self, s: float, sd: float, sdd: float) -> kdl.Twist:
        pos = self._p_start_end * sdd * self._scale_lin
        rot = self._orient_interp.acceleration(s * self._scale_rot, sd * self._scale_rot, sdd * self._scale_rot)
        return kdl.Twist(pos, rot)

    def path_length(self) -> float:
        return self._pathlength

    def parameterized_length(self, s: float) -> float:
        return s * self._scale_lin

class PathCircle(Path):
    def __init__(self, H_base_start: kdl.Frame, p_base_center: kdl.Vector, p_base_p: kdl.Vector, R_base_end: kdl.Rotation, alpha: float, eqradius: float = 1.0):
        self._H_base_center = kdl.Frame.Identity()
        self._H_base_center.p = kdl.Vector(p_base_center)
        self._orient_interp = RotationalInterpolationSingleAxis()
        self._eqradius = eqradius

        self._orient_interp.set_start_end(H_base_start.M, R_base_end)
        oalpha = self._orient_interp.angle()
        self._radius_vector = H_base_start.p - self._H_base_center.p
        self._radius = self._radius_vector.Normalize()

        if self._radius < 1e-6:
            raise ValueError('Start point cannot be the same as the center of the circle')

        temp = p_base_p - self._H_base_center.p
        temp.Normalize()

        z = self._radius_vector * temp
        n = z.Normalize()

        if n < 1e-6:
            raise ValueError('Start point, center and point on the circle cannot be collinear')

        self._H_base_center.M = kdl.Rotation(self._radius_vector, z * self._radius_vector, z)
        distance = alpha * self._radius

        if oalpha * eqradius > distance:
            self._pathlength = oalpha * eqradius
            self._scale_rot = 1.0 / eqradius
            self._scale_lin = distance / self._pathlength
        else:
            self._pathlength = distance
            self._scale_rot = oalpha / distance
            self._scale_lin = 1.0

    def position(self, s: float) -> kdl.Frame:
        angle = s * self._scale_lin / self._radius
        rot = self._orient_interp.position(s * self._scale_rot)
        pos = self._H_base_center * kdl.Vector(self._radius * math.cos(angle), self._radius * math.sin(angle), 0.0)
        return kdl.Frame(rot, pos)

    def velocity(self, s: float, sd: float) -> kdl.Twist:
        p = s * self._scale_lin / self._radius
        v = sd * self._scale_lin / self._radius

        vel_pos = self._H_base_center.M * kdl.Vector(-self._radius * math.sin(p) * v, self._radius * math.cos(p) * v, 0.0)
        vel_rot = self._orient_interp.velocity(s * self._scale_rot, sd * self._scale_rot)

        return kdl.Twist(vel_pos, vel_rot)

    def acceleration(self, s: float, sd: float, sdd: float) -> kdl.Twist:
        p = s * self._scale_lin / self._radius
        v = sd * self._scale_lin / self._radius
        a = sdd * self._scale_lin / self._radius

        cos_p = math.cos(p)
        sin_p = math.sin(p)

        vel_pos = self._H_base_center.M * kdl.Vector(-self._radius * cos_p * v * v - self._radius * sin_p * a,
                                                      -self._radius * sin_p * v * v + self._radius * cos_p * a,
                                                      0.0)
        vel_rot = self._orient_interp.acceleration(s * self._scale_rot, sd * self._scale_rot, sdd * self._scale_rot)

        return kdl.Twist(vel_pos, vel_rot)

    def path_length(self) -> float:
        return self._pathlength

    def parameterized_length(self, s: float) -> float:
        return s / self._scale_lin

class VelocityProfile(ABC):
    @abstractmethod
    def position(self, time: float) -> float:
        pass

    @abstractmethod
    def velocity(self, time: float) -> float:
        pass

    @abstractmethod
    def acceleration(self, time: float) -> float:
        pass

    @abstractmethod
    def duration(self) -> float:
        pass

    @abstractmethod
    def set_profile(self, pos1: float, pos2: float) -> None:
        pass

    @abstractmethod
    def set_profile_duration(self, pos1: float, pos2: float, duration: float) -> None:
        pass

class VelocityProfileRectangular(VelocityProfile):
    def __init__(self, maxvel: float):
        self._maxvel = maxvel
        self._duration = 0
        self._position = 0
        self._velocity = 0

    def position(self, time: float) -> float:
        if time < 0:
            return self._position
        elif time > self._duration:
            return self._position + self._velocity * self._duration
        else:
            return self._position + self._velocity * time

    def velocity(self, time: float) -> float:
        if 0 <= time <= self._duration:
            return self._velocity
        else:
            return 0

    def acceleration(self, time: float) -> float:
        return 0

    def duration(self) -> float:
        return self._duration

    def set_max_velocity(self, maxvel: float) -> None:
        self._maxvel = maxvel

    def set_profile(self, pos1: float, pos2: float) -> None:
        diff = pos2 - pos1

        if diff != 0:
            self._velocity = self._maxvel if diff > 0 else -self._maxvel
            self._position = pos1
            self._duration = diff / self._velocity
        else:
            self._velocity = 0
            self._position = pos1
            self._duration = 0

    def set_profile_duration(self, pos1: float, pos2: float, duration: float) -> None:
        diff = pos2 - pos1

        if diff != 0:
            self._velocity = diff / duration

            if self._velocity > self._maxvel or duration == 0:
                self._velocity = self._maxvel

            self._position = pos1
            self._duration = diff / self._velocity
        else:
            self._velocity = 0
            self._position = pos1
            self._duration = 0

class Trajectory(ABC):
    @abstractmethod
    def position(self, time: float) -> kdl.Frame:
        pass

    @abstractmethod
    def velocity(self, time: float) -> kdl.Twist:
        pass

    @abstractmethod
    def acceleration(self, time: float) -> kdl.Twist:
        pass

    @abstractmethod
    def duration(self) -> float:
        pass

class TrajectorySegment(Trajectory):
    def __init__(self, path: Path, profile: VelocityProfile, duration: float):
        self._path = path
        self._profile = profile
        self._profile.set_profile_duration(0, path.path_length(), duration)

    def position(self, time: float) -> kdl.Frame:
        return self._path.position(self._profile.position(time))

    def velocity(self, time: float) -> kdl.Twist:
        return self._path.velocity(self._profile.position(time), self._profile.velocity(time))

    def acceleration(self, time: float) -> kdl.Twist:
        return self._path.acceleration(self._profile.position(time), self._profile.velocity(time), self._profile.acceleration(time))

    def duration(self) -> float:
        return self._profile.duration()

class TrajectoryComposite(Trajectory):
    def __init__(self):
        self._duration = 0.0
        self._trajectories: list[Trajectory] = []
        self._end_times: list[float] = []

    def add_segment(self, segment: Trajectory) -> None:
        self._trajectories.append(segment)
        self._duration += segment.duration()
        self._end_times.append(self._duration)

    def position(self, time: float) -> kdl.Frame:
        if time < 0:
            return self._trajectories[0].position(0)

        previous_time = 0.0

        for i, traj in enumerate(self._trajectories):
            if time < self._end_times[i]:
                return traj.position(time - previous_time)

            previous_time = self._end_times[i]

        last_traj = self._trajectories[-1]
        return last_traj.position(last_traj.duration())

    def velocity(self, time: float) -> kdl.Twist:
        if time < 0:
            return self._trajectories[0].velocity(0)

        previous_time = 0.0

        for i, traj in enumerate(self._trajectories):
            if time < self._end_times[i]:
                return traj.velocity(time - previous_time)

            previous_time = self._end_times[i]

        last_traj = self._trajectories[-1]
        return last_traj.velocity(last_traj.duration())

    def acceleration(self, time: float) -> kdl.Twist:
        if time < 0:
            return self._trajectories[0].acceleration(0)

        previous_time = 0.0

        for i, traj in enumerate(self._trajectories):
            if time < self._end_times[i]:
                return traj.acceleration(time - previous_time)

            previous_time = self._end_times[i]

        last_traj = self._trajectories[-1]
        return last_traj.acceleration(last_traj.duration())

    def duration(self) -> float:
        return self._duration
