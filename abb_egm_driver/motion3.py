import math
import PyKDL as kdl
from abc import ABC, abstractmethod

# based on OROCOS Kinematics and Dynamics Library, licensed under LGPL v2.1
# https://github.com/orocos/orocos_kinematics_dynamics

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

class PathLine(Path):
    def __init__(self, H_base_start: kdl.Frame, H_base_end: kdl.Frame):
        self._p_base_start = kdl.Vector(H_base_start.p)
        self._p_base_end = kdl.Vector(H_base_end.p)
        self._p_start_end = self._p_base_end - self._p_base_start
        self._pathlength = self._p_start_end.Normalize()

    def position(self, s: float) -> kdl.Frame:
        return kdl.Frame(self._p_base_start + self._p_start_end * s)

    def velocity(self, s: float, sd: float) -> kdl.Twist:
        return kdl.Twist(self._p_start_end * sd, kdl.Vector.Zero())

    def acceleration(self, s: float, sd: float, sdd: float) -> kdl.Twist:
        return kdl.Twist(self._p_start_end * sdd, kdl.Vector.Zero())

    def path_length(self) -> float:
        return self._pathlength

class PathCircle(Path):
    def __init__(self, H_base_start: kdl.Frame, p_base_center: kdl.Vector, alpha: float):
        self._H_base_start = kdl.Frame(H_base_start)
        self._H_base_center = kdl.Frame.Identity()
        self._H_base_center.p = kdl.Vector(p_base_center)
        self._alpha = alpha

        # vector from the center to the start point
        self._radius_vector = H_base_start.p - self._H_base_center.p
        self._radius = self._radius_vector.Norm()

        # arc that will be traveled
        self._pathlength = abs(alpha * self._radius)
        self._direction = 1 if alpha > 0 else -1

    def position(self, s: float) -> kdl.Frame:
        angle = self._direction * s / self._radius
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)

        # rotation in plane XY around the center
        p = self._H_base_center * kdl.Vector(self._radius_vector.x() * cos_angle - self._radius_vector.y() * sin_angle,
                                             self._radius_vector.x() * sin_angle + self._radius_vector.y() * cos_angle,
                                             self._radius_vector.z())

        return kdl.Frame(p)

    def velocity(self, s: float, sd: float) -> kdl.Twist:
        angle = self._direction * s / self._radius
        v = sd / self._radius
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)

        return kdl.Twist(kdl.Vector(-self._radius * sin_angle * v,
                                    self._radius * cos_angle * v,
                                    0), kdl.Vector.Zero())

    def acceleration(self, s: float, sd: float, sdd: float) -> kdl.Twist:
        angle = self._direction * s / self._radius
        v = sd / self._radius
        a = sdd / self._radius
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)

        return kdl.Twist(kdl.Vector(-self._radius * cos_angle * v * v - self._radius * sin_angle * a,
                                    -self._radius * sin_angle * v * v + self._radius * cos_angle * a,
                                    0), kdl.Vector.Zero())

    def path_length(self) -> float:
        return self._pathlength

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
