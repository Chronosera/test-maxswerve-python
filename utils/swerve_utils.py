import math
import numpy


class SwerveUtils:
    def step_towards(self, current: float, target: float, step_size: float):
        """
         Steps a value towards a target with a specified step size.
        Returns the new value for <current> after performing the specified step towards the specified target.

        :param current: The current or starting value.  Can be positive or negative.
        :param target: The target value the algorithm will step towards.  Can be positive or negative.
        :param step_size: The maximum step size that can be taken.
        """
        if abs(current - target) <= step_size:
            return target
        elif target < current:
            return current - step_size
        else:
            return current + step_size

    def step_towards_circular(self, current: float, target: float, step_size: float):
        """
        Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
        Returns The new angle (in radians) for <current> after performing the specified step towards the specified target.
        This value will always lie in the range 0 to 2*PI (exclusive).

        :param current: The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
        :param target: The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
        :param step_size: The maximum step size that can be taken (in radians).
        """
        current = self.wrap_angle(current)
        target = self.wrap_angle(target)

        step_direction = math.copysign(1, target - current)
        difference = abs(current - target)

        if difference <= step_size:
            return target
        elif difference > numpy.pi:  # does the system need to wrap over eventually?
            # handle the special case where you can reach the target in one step while also wrapping
            if current + 2*numpy.pi - target < step_size or target + 2*numpy.pi - current < step_size:
                return target
            else:
                return self.wrap_angle(current - step_direction * step_size)  # this will handle wrapping gracefully
        else:
            return current + step_direction * step_size

    def angle_difference(self, angle_a: float, angle_b: float):
        """
        Finds the (unsigned) minimum difference between two angles including calculating across 0.
        Returns the (unsigned) minimum difference between the two angles (in radians).

        :param angle_a: An angle (in radians).
        :param angle_b: An angle (in radians).
        """
        difference = abs(angle_a - angle_b)

        return (2 * math.pi) - difference if difference > math.pi else difference

    def wrap_angle(self, angle: float):
        """
        Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
        Returns an angle (in radians) from 0 and 2*PI (exclusive).

        :param angle: The angle (in radians) to wrap. Can be positive or negative and can lie
         multiple wraps outside the output range.
        """
        two_pi = 2*numpy.pi

        if angle == two_pi:
            return 0.0
        elif angle > two_pi:
            return angle - two_pi*math.floor(angle / two_pi)
        elif angle < 0.0:
            return angle + two_pi*math.floor(((-angle) / two_pi)+1)
        else:
            return angle
