import time
from math import cos, radians, sin

from demosys.opengl.projection import Projection
from pyrr import Vector3, matrix33, matrix44, vector, vector3, Quaternion
from pyrr.vector import length, squared_length
from pyrr.quaternion import create_from_eulers, apply_to_vector, create_from_axis_rotation, create_from_matrix, inverse

from typing import Tuple

# Direction Definitions
RIGHT = 1
LEFT = 2
FORWARD = 3
BACKWARD = 4
UP = 5
DOWN = 6

# Movement Definitions
STILL = 0
POSITIVE = 1
NEGATIVE = 2


def inverse_rotate(q: Quaternion, v: Vector3) -> Vector3:
    """Returns the image of \p v by the Quaternion inverse() rotation.
        rotate() performs an inverse transformation. Same as inverse().rotate(v).

    :param q:
    :param v:
    :return:
    """
    return apply_to_vector(inverse(q), v)


def creative_modelview_from_position_orientation(position: Vector3, orientation: Quaternion) -> matrix44:
    """

    :param position:
    :param orientation:
    :return:
    """
    q = orientation

    q00 = 2.0 * q[0] * q[0]
    q11 = 2.0 * q[1] * q[1]
    q22 = 2.0 * q[2] * q[2]

    q01 = 2.0 * q[0] * q[1]
    q02 = 2.0 * q[0] * q[2]
    q03 = 2.0 * q[0] * q[3]

    q12 = 2.0 * q[1] * q[2]
    q13 = 2.0 * q[1] * q[3]

    q23 = 2.0 * q[2] * q[3]

    model_view_matrix = matrix44.create_identity()
    model_view_matrix[0][0] = 1.0 - q11 - q22
    model_view_matrix[0][1] = q01 - q23
    model_view_matrix[0][2] = q02 + q13
    model_view_matrix[0][3] = 0.0

    model_view_matrix[1][0] = q01 + q23
    model_view_matrix[1][1] = 1.0 - q22 - q00
    model_view_matrix[1][2] = q12 - q03
    model_view_matrix[1][3] = 0.0

    model_view_matrix[2][0] = q02 - q13
    model_view_matrix[2][1] = q12 + q03
    model_view_matrix[2][2] = 1.0 - q11 - q00
    model_view_matrix[2][3] = 0.0

    t = inverse_rotate(q, position)
    model_view_matrix[3][0] = -t[0]
    model_view_matrix[3][1] = -t[1]
    model_view_matrix[3][2] = -t[2]
    model_view_matrix[3][3] = 1.0

    return model_view_matrix


def create_from_view_direction(direction: Vector3, up: vector3=Vector3([0, 1, 0])) -> Quaternion:
    """Rotates the Camera so that its viewDirection() is \p direction (defined in
     the world coordinate system).
     The Camera position() is not modified. The Camera is rotated so that the
     horizon (defined by its upVector()) is preserved. See also lookAt() and
     setUpVector().

    :param direction:
    :param up
    :return:
    """
    if squared_length(direction) < 1e-10:
        raise ValueError("len2(direction)(={squared_length(direction)}) < 1e-10")

    xAxis = direction.cross(up)
    if squared_length(xAxis) < 1e-10:
        # TODO: [QGLViewer] Find equivalence !
        """
        // target is aligned with upVector, this means a rotation around X axis
        // X axis is then unchanged, let's keep it !
        xAxis = frame()->inverseTransformOf(Vec(1.0, 0.0, 0.0));
        """
        raise RuntimeError

    return create_from_rotated_basis(xAxis, xAxis.cross(direction), direction * -1)


def create_from_rotated_basis(x: Vector3, y: Vector3, z: Vector3) -> Quaternion:
    """Return a Quaternion from a (supposedly correct) 3x3 rotation matrix.
      The matrix is expressed in European format: its three \e columns are the
      images by the rotation of the three vectors of an orthogonal basis. Note that
      OpenGL uses a symmetric representation for its matrices.
      create_from_rotated_basis() return a Quaternion from the three axis of a rotated
      frame. It actually fills the three columns of a matrix with these rotated
      basis vectors and calls this method.

    :param x:
    :param y:
    :param z:
    :return:
    """
    m = matrix33.create_identity()

    norm_x = length(x)
    norm_y = length(y)
    norm_z = length(z)

    for i in range(3):
        m[i][0] = x[i] / norm_x
        m[i][1] = y[i] / norm_y
        m[i][2] = z[i] / norm_z

    return create_from_matrix(m)


class Camera:
    """Simple camera class containing projection"""
    def __init__(self, fov=60, aspect=1.0, near=1, far=100):
        """
        Initialize camera using a specific projection

        :param fov: Field of view
        :param aspect: Aspect ratio
        :param near: Near plane
        :param far: Far plane
        """
        self.position = Vector3([0.0, 0.0, 0.0])
        # Default camera placement
        self.up = Vector3([0.0, 1.0, 0.0])
        self.right = Vector3([1.0, 0.0, 0.0])
        self.dir = Vector3([0.0, 0.0, -1.0])
        # Yaw and Pitch
        self.yaw = -90.0
        self.pitch = 0.0

        # World up vector
        self._up = Vector3([0.0, 1.0, 0.0])

        # Projection
        self.projection = Projection(aspect, fov, near, far)

    def set_position(self, x, y, z):
        """
        Set the 3D position of the camera

        :param x: float
        :param y: float
        :param z: float
        """
        self.position = Vector3([x, y, z])

    @property
    def view_matrix(self):
        """
        :return: The current view matrix for the camera
        """
        self._update_yaw_and_pitch()
        return self._gl_look_at(self.position, self.position + self.dir, self._up)

    def _update_yaw_and_pitch(self):
        """
        Updates the camera vectors based on the current yaw and pitch
        """
        front = Vector3([0.0, 0.0, 0.0])
        front.x = cos(radians(self.yaw)) * cos(radians(self.pitch))
        front.y = sin(radians(self.pitch))
        front.z = sin(radians(self.yaw)) * cos(radians(self.pitch))

        self.dir = vector.normalise(front)
        self.right = vector.normalise(vector3.cross(self.dir, self._up))
        self.up = vector.normalise(vector3.cross(self.right, self.dir))

    def look_at(self, vec=None, pos=None):
        """
        Look at a specific point

        :param vec: Vector3 position
        :param pos: python list [x, y, x]
        :return: Camera matrix
        """
        if pos is None:
            vec = Vector3(pos)

        if vec is None:
            raise ValueError("vector or pos must be set")

        return self._gl_look_at(self.position, vec, self._up)

    def _gl_look_at(self, pos, target, up):
        """
        The standard lookAt method

        :param pos: current position
        :param target: target position to look at
        :param up: direction up
        """
        z = vector.normalise(pos - target)
        x = vector.normalise(vector3.cross(vector.normalise(up), z))
        y = vector3.cross(z, x)

        translate = matrix44.create_identity()
        translate[3][0] = -pos.x
        translate[3][1] = -pos.y
        translate[3][2] = -pos.z

        rotate = matrix44.create_identity()
        rotate[0][0] = x[0]  # -- X
        rotate[1][0] = x[1]
        rotate[2][0] = x[2]
        rotate[0][1] = y[0]  # -- Y
        rotate[1][1] = y[1]
        rotate[2][1] = y[2]
        rotate[0][2] = z[0]  # -- Z
        rotate[1][2] = z[1]
        rotate[2][2] = z[2]

        return matrix44.multiply(translate, rotate)


class CameraQuaternion(Camera):
    """
    void Camera::setViewDirection(const Vec &direction)
    https://github.com/GillesDebunne/libQGLViewer/blob/master/QGLViewer/camera.cpp#L1151


    https://github.com/GillesDebunne/libQGLViewer/blob/master/QGLViewer/quaternion.cpp#L136
    """

    def __init__(self, fov=60, aspect=1.0, near=1, far=100):
        """
        Initialize camera using a specific projection

        :param fov: Field of view
        :param aspect: Aspect ratio
        :param near: Near plane
        :param far: Far plane
        """
        super().__init__(fov=fov, aspect=aspect, near=near, far=far)

        # Eulers: +Roll
        self.roll = 0.0
        # self.yall = 0.0

        # Orientation: quaternion
        self.orientation = create_from_eulers((self.roll, self.pitch, self.yaw))

    @property
    def eulers(self) -> Tuple[float, float, float]:
        return self.yaw, self.pitch, self.roll

    def _update_yaw_and_pitch(self):
        """
        Updates the camera vectors based on the current yaw and pitch
        """
        self.orientation = create_from_eulers(self.eulers)


# class SystemCamera(Camera):
class SystemCamera(CameraQuaternion):
    """System camera controlled by mouse and keyboard"""
    def __init__(self, fov=60, aspect=1.0, near=1, far=100):
        # Position movement states
        self._xdir = STILL
        self._zdir = STILL
        self._ydir = STILL
        self._last_time = 0

        # Velocity in axis units per second
        self.velocity = 10.0
        self.mouse_sensitivity = 0.5
        self.mouse_sensitivity *= 0.01
        self.last_x = None
        self.last_y = None

        super().__init__(fov=fov, aspect=aspect, near=near, far=far)

    def move_left(self, activate):
        self.move_state(LEFT, activate)

    def move_right(self, activate):
        self.move_state(RIGHT, activate)

    def move_forward(self, activate):
        self.move_state(FORWARD, activate)

    def move_backward(self, activate):
        self.move_state(BACKWARD, activate)

    def move_up(self, activate):
        self.move_state(UP, activate)

    def move_down(self, activate):
        self.move_state(DOWN, activate)

    def move_state(self, direction, activate):
        """
        Set the camera position move state

        :param direction: What direction to update
        :param activate: Start or stop moving in the direction
        """
        if direction == RIGHT:
            self._xdir = POSITIVE if activate else STILL
        elif direction == LEFT:
            self._xdir = NEGATIVE if activate else STILL
        elif direction == FORWARD:
            self._zdir = NEGATIVE if activate else STILL
        elif direction == BACKWARD:
            self._zdir = POSITIVE if activate else STILL
        elif direction == UP:
            self._ydir = POSITIVE if activate else STILL
        elif direction == DOWN:
            self._ydir = NEGATIVE if activate else STILL

    def rot_state(self, x, y):
        """
        Set the rotation state of the camera

        :param x: viewport x pos
        :param y: viewport y pos
        """
        if self.last_x is None:
            self.last_x = x
        if self.last_y is None:
            self.last_y = y

        x_offset = self.last_x - x
        y_offset = self.last_y - y

        self.last_x = x
        self.last_y = y

        x_offset *= self.mouse_sensitivity
        y_offset *= self.mouse_sensitivity

        self.yaw -= y_offset
        self.pitch -= x_offset

        # if self.pitch > 85.0:
        #     self.pitch = 85.0
        # if self.pitch < -85.0:
        #     self.pitch = -85.0

        self._update_yaw_and_pitch()

    @property
    def view_matrix(self):
        """
        :return: The current view matrix for the camera
        """
        # Use separate time in camera so we can move it when the demo is paused
        now = time.time()
        # If the camera has been inactive for a while, a large time delta
        # can suddenly move the camera far away from the scene
        t = max(now - self._last_time, 0)
        self._last_time = now

        # X Movement
        if self._xdir == POSITIVE:
            self.position += self.right * self.velocity * t
        elif self._xdir == NEGATIVE:
            self.position -= self.right * self.velocity * t

        # Z Movement
        if self._zdir == NEGATIVE:
            self.position += self.dir * self.velocity * t
        elif self._zdir == POSITIVE:
            self.position -= self.dir * self.velocity * t

        # Y Movement
        if self._ydir == POSITIVE:
            self.position += self.up * self.velocity * t
        elif self._ydir == NEGATIVE:
            self.position -= self.up * self.velocity * t

        # return self._gl_look_at(self.position, self.position + self.dir, self._up)
        try:
            return creative_modelview_from_position_orientation(self.position, self.orientation)
            # return matrix44.create_from_translation(self.position)
        except Exception as e:
            print(f"Exception: {e}")
