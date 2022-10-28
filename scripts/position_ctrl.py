#!/usr/bin/python3
"""Print joint positions. Toggle between position- and zero-torque-control.

Runs a simple curses GUI that shows the current joint positions.  By pressing
"p" the user can toggle between position control to the current position and
zero-torque control.  So it can be used to manually move the joints to some
position (in zero-torque mode) and then lock them there by enabling the
position control.
"""
import argparse
import curses

from robot_interfaces_solo import solo12


class CursesGUI:
    """Displays robot data using curses."""

    def __init__(self, win):
        """Initialize.

        Args:
            win: Curses window.
        """
        self.win = win
        self.win.nodelay(True)

    def update(self, observation, position_control_enabled):
        """Update the displayed robot data.

        Args:
            observation:  Latest robot observation.
            position_control_enabled (bool):  Indicate whether position control
                mode is currently enabled or not.

        Returns:
            Code of the key pressed by the user or -1 if no key was pressed.
        """
        self.win.erase()

        try:

            # status line
            status_line = " q: quit | p: toggle position control on/off"
            max_rows, max_cols = self.win.getmaxyx()
            self.win.insstr(
                max_rows - 1,
                0,
                status_line.ljust(max_cols, " "),
                curses.A_STANDOUT,
            )

            # header line
            line = 0
            if position_control_enabled:
                self.win.addstr(line, 0, "Position Control Enabled", curses.A_BOLD)
            else:
                self.win.addstr(line, 0, "Position Control Disabled")
            line += 3

            line = self._print_joint_positions(line, observation.joint_positions)

            self.win.refresh()

        except curses.error:
            raise RuntimeError(
                "GUI rendering error.  Try increasing the terminal window."
            )

        return self.win.getch()

    def _print_joint_positions(self, line, positions):
        """Print positions of all joints without grouping."""
        for j, pos in enumerate(positions):
            self.win.addstr(
                line,
                4,
                "Joint %d: %.4f" % (j, pos),
            )
            line += 1

        return line

    def _print_joint_positions_per_finger(self, line, positions):
        """Print joint positions grouped by finger."""
        joints_per_finger = 3
        n_fingers = len(positions) // joints_per_finger

        for i in range(n_fingers):
            self.win.addstr(line, 0, "Finger %d" % i, curses.A_BOLD)
            line += 1

            for j, joint_name in enumerate(["upper", "middle", "lower"]):
                joint_idx = i * joints_per_finger + j
                self.win.addstr(
                    line,
                    4,
                    "%s: %.4f" % (joint_name, positions[joint_idx]),
                )
                line += 1

            line += 1

        return line

    def display_error(self, message):
        """Display error message and wait until user presses a key.

        Args:
            message:  The error message that is displayed.
        """
        self.win.nodelay(False)
        self.win.clear()

        self.win.addstr(1, 0, "ERROR:", curses.A_BOLD)
        self.win.addstr(3, 4, message)
        self.win.addstr(5, 0, "Press any key to exit.")
        self.win.refresh()

        self.win.getch()


def loop(win, robot_frontend):
    gui = CursesGUI(win)

    kp = [3.0] * 12
    kd = [0.05] * 12

    try:
        # get current position
        t = robot_frontend.append_desired_action(solo12.Action())
        initial_positions = robot_frontend.get_observation(t).joint_positions
        action = solo12.Action()
        action.joint_positions = initial_positions
        action.joint_position_gains = kp
        action.joint_velocity_gains = kd

        position_control_enabled = True

        while True:
            t = robot_frontend.append_desired_action(action)
            obs = robot_frontend.get_observation(t)

            pressed_key = gui.update(obs, position_control_enabled)
            if pressed_key == ord("q"):
                return
            elif pressed_key == ord("p"):
                if position_control_enabled:
                    action = solo12.Action.Zero()
                else:
                    action = solo12.Action()
                    action.joint_positions = obs.joint_positions
                    action.joint_position_gains = kp
                    action.joint_velocity_gains = kd

                position_control_enabled = not position_control_enabled

    except Exception as e:
        gui.display_error(str(e))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "config_file",
        type=str,
        help="YAML file with Solo12 driver configuration.",
    )
    args = parser.parse_args()

    # load robot configuration
    config = solo12.Config.from_file(args.config_file)

    robot_data = solo12.SingleProcessData()
    robot_backend = solo12.create_backend(robot_data, config)
    robot_frontend = solo12.Frontend(robot_data)
    robot_backend.initialize()

    curses.wrapper(lambda stdscr: loop(stdscr, robot_frontend))


if __name__ == "__main__":
    main()
