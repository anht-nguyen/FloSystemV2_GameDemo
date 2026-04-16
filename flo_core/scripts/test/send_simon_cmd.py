#!/usr/bin/env python3
"""
Send a repeatable sequence of dual-arm gesture commands to the FLO `/simon_cmd`
action server.
"""

import argparse
import sys
import time

import actionlib
import rospy

from flo_core_defs.msg import SimonCmdAction, SimonCmdGoal


def parse_actions(raw_value: str):
    actions = [token.strip() for token in raw_value.split(",") if token.strip()]
    if not actions:
        raise argparse.ArgumentTypeError("Expected a comma-separated action list")
    return actions


def build_pairs(shared_actions, left_actions, right_actions):
    if shared_actions is not None:
        return [(action, action) for action in shared_actions]

    if left_actions is None or right_actions is None:
        raise ValueError("Provide either --actions or both --left-actions and --right-actions")

    if len(left_actions) != len(right_actions):
        raise ValueError("--left-actions and --right-actions must have the same length")

    return list(zip(left_actions, right_actions))


def main():
    parser = argparse.ArgumentParser(description="Send scripted gesture goals to /simon_cmd")
    parser.add_argument("--actions", type=parse_actions, help="Comma-separated actions to run on both arms")
    parser.add_argument("--left-actions", type=parse_actions, help="Comma-separated left-arm actions")
    parser.add_argument("--right-actions", type=parse_actions, help="Comma-separated right-arm actions")
    parser.add_argument("--repeat", type=int, default=1, help="Number of full sequence repeats")
    parser.add_argument("--delay", type=float, default=1.0, help="Delay after each goal completes")
    parser.add_argument("--wait-for-server", type=float, default=60.0, help="Seconds to wait for /simon_cmd before failing")
    parser.add_argument("--simon-says", action="store_true", help="Set the Simon Says flag in the action goal")
    args = parser.parse_args()

    if args.repeat < 1:
      raise SystemExit("--repeat must be >= 1")
    if args.delay < 0:
      raise SystemExit("--delay must be >= 0")

    try:
        pairs = build_pairs(args.actions, args.left_actions, args.right_actions)
    except ValueError as exc:
        raise SystemExit(str(exc))

    rospy.init_node("send_simon_cmd_test", anonymous=True)
    client = actionlib.SimpleActionClient("/simon_cmd", SimonCmdAction)

    rospy.loginfo("Waiting for /simon_cmd action server...")
    if not client.wait_for_server(rospy.Duration.from_sec(args.wait_for_server)):
        raise SystemExit("Timed out waiting for /simon_cmd")

    total_goals = len(pairs) * args.repeat
    goal_index = 0

    for cycle in range(1, args.repeat + 1):
        for left_action, right_action in pairs:
            goal_index += 1
            gesture_name = f"{left_action}_left|{right_action}_right"
            goal = SimonCmdGoal(
                gesture_name=gesture_name,
                simon_says=args.simon_says,
            )

            rospy.loginfo(
                "Goal %d/%d: %s (cycle %d/%d)",
                goal_index,
                total_goals,
                gesture_name,
                cycle,
                args.repeat,
            )
            client.send_goal(goal)
            client.wait_for_result()

            state = client.get_state()
            result = client.get_result()
            success = bool(result and getattr(result, "success", False))
            rospy.loginfo("Result: state=%s success=%s", state, success)
            if not success:
                raise SystemExit(f"Gesture failed: {gesture_name}")

            if args.delay > 0:
                time.sleep(args.delay)

    rospy.loginfo("Completed %d gesture goals.", total_goals)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(130)
