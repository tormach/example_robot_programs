# Example 10: Path_blending
Path blending makes the robot arm perform two moves in one continuous motion. To do this you use the `set_path_blending()` to enable or disable path blending in combination with `sync()` to force the execution of blended commands.

`set_path_blending(enable, blend_radius)` has two arguments:
* enable - Enable or disable path blending.
* blend_radius – The blend radius between moves in meters.

### Learn more about the Tormach Robot Programming Language (TRPL):
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language
[label](https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach%2BRobot%2BProgramming%2BLanguage#TormachRobotProgrammingLanguage-robot_command.rpl.sync)