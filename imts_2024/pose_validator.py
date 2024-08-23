from robot_command.rpl import *
import rospy

def validate_pose(limits_low, limits_high):
    set_units("mm", "deg", "s")
    active_user_frame_str = get_active_user_frame()
    change_user_frame("my_world_frame")
    active_tool_frame_str = get_active_tool_frame()
    current_pose = get_pose()
    # rospy.logwarn(f"Pose now: {current_pose}")

    def normalize_angle(angle):
        """Normalize angle to be between -180 and 180 degrees."""
        return (angle + 180) % 360 - 180

    def angle_distance(a, b):
        """Calculate the shortest angular distance between two angles."""
        return min((a - b) % 360, (b - a) % 360)


    def is_angle_within_limits(angle, low, high):
        """Check if an angle is within limits, considering the shortest path."""
        angle = normalize_angle(angle)
        low = normalize_angle(low)
        high = normalize_angle(high)

        if ( high - low ) > 180:
            if angle >= -180 and angle <= low:
                return True
            if angle <= 180 and angle >= high:
                return True
            return False

        return low <= angle <= high

    def is_within_limits(value, low, high):
        return low <= value <= high

    within_limits = True
    out_of_bounds = []

    for i, (coord, low, high) in enumerate(zip(current_pose.to_list(), limits_low, limits_high)):
        if i < 3:  # x, y, z coordinates
            if not is_within_limits(coord, low, high):
                within_limits = False
                out_of_bounds.append(f"{'xyz'[i]}: {coord:.2f} (limits: {low} to {high})")
        else:  # a, b, c angles
            angle_deg = coord  # Angles are already in degrees
            normalized_angle = normalize_angle(angle_deg)
            # rospy.logwarn(f"Angle {'abc'[i-3]}: raw={angle_deg:.2f}°, normalized={normalized_angle:.2f}°")
            if not is_angle_within_limits(angle_deg, low, high):
                within_limits = False
                out_of_bounds.append(f"{'abc'[i-3]}: {normalized_angle:.2f}° (raw={angle_deg:.2f}°, limits: {low}° to {high}°)")

    if not within_limits:
        error_message = f"Active pose not within the limits:\n" + "\n".join(out_of_bounds)
        # error_message += f"\nActive user frame: {active_user_frame_str}"
        error_message += f"\nActive tool frame: {active_tool_frame_str}"
        notify(error_message, error=True)
        return False

    return True
