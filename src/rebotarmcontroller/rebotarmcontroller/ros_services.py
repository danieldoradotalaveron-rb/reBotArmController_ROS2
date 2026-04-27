from __future__ import annotations

from rebotarm_msgs.srv import MoveToPoseIK, SetGripper, SetMode, SetZero
from std_srvs.srv import Trigger


class ArmServices:
    def __init__(self, node, hardware, namespace: str) -> None:
        self._node = node
        self._hardware = hardware
        self._namespace = namespace

        node.create_service(
            Trigger,
            self._service("enable"),
            self.enable,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            self._service("disable"),
            self.disable,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            self._service("safe_home"),
            self.safe_home,
            callback_group=node.slow_group,
        )
        node.create_service(
            SetZero,
            self._service("set_zero"),
            self.set_zero,
            callback_group=node.slow_group,
        )
        node.create_service(
            SetMode,
            self._service("set_mode"),
            self.set_mode,
            callback_group=node.slow_group,
        )
        node.create_service(
            MoveToPoseIK,
            self._service("move_to_pose_ik"),
            self.move_to_pose_ik,
            callback_group=node.reentrant_group,
        )
        node.create_service(
            SetGripper,
            self._service("gripper/set"),
            self.set_gripper,
            callback_group=node.reentrant_group,
        )

    def _service(self, name: str) -> str:
        return f"/{self._namespace}/{name}"

    def enable(self, _request, response):
        try:
            self._hardware.enable()
            response.success = True
            response.message = "enabled"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def disable(self, _request, response):
        try:
            self._hardware.disable()
            response.success = True
            response.message = "disabled"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def safe_home(self, _request, response):
        try:
            self._hardware.safe_home()
            response.success = True
            response.message = "safe_home complete"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def set_zero(self, request, response):
        try:
            ok = self._hardware.set_zero(request.joint_name)
            response.success = bool(ok)
            response.message = "set_zero complete" if ok else "set_zero failed"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def set_mode(self, request, response):
        try:
            ok = self._hardware.set_mode(request.mode)
            response.success = bool(ok)
            response.message = f"mode set to {request.mode}" if ok else "mode switch incomplete"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def move_to_pose_ik(self, request, response):
        try:
            ok, q_solution = self._hardware.move_to_pose_ik(request.target_pose)
            response.success = bool(ok)
            response.message = "IK target accepted" if ok else "IK failed"
            response.q_solution = [float(v) for v in q_solution]
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.q_solution = []
        self._node.publish_arm_status()
        return response

    def set_gripper(self, request, response):
        try:
            reached, reached_position = self._hardware.set_gripper_position(
                request.position,
                request.max_effort,
            )
            response.success = bool(reached)
            response.reached_position = float(reached_position)
        except Exception:
            response.success = False
            response.reached_position = 0.0
        self._node.publish_arm_status()
        return response
