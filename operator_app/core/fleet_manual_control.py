"""Manual-control commands for real and simulated operator fleets."""

from __future__ import annotations

from typing import Any


class FleetManualControlService:
    """Translate operator manual-control payloads to manager commands."""

    def __init__(self, owner: Any) -> None:
        self.owner = owner

    def manual_step_payload(
        self,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        owner = self.owner
        owner._sync_manager_mode()
        name = str(payload.get("name") or "").strip()
        if not name:
            raise ValueError("robot name is required")
        if owner.mode == "robots":
            result = owner.manager.teleop_robot(
                {
                    "name": name,
                    "linear": float(
                        payload.get("linear", 0.0) or 0.0
                    ),
                    "angular": float(
                        payload.get("angular", 0.0) or 0.0
                    ),
                    "timeoutMs": int(
                        payload.get("timeoutMs", 350) or 350
                    ),
                },
                include_state=False,
            )
            return {
                "ok": True,
                "blocked": False,
                "reason": "",
                "robot": result.get("robot"),
                "state": None,
            }

        poses = payload.get("poses", [])
        check = owner.manager.check_path(
            {"name": name, "poses": poses}
        )
        current_lm_key = (
            "blockedCurrentLm"
            if check.get("blocked")
            else "currentLm"
        )
        update_payload = {
            "name": name,
            "status": (
                "MANUAL_BLOCKED"
                if check.get("blocked")
                else "MANUAL"
            ),
            "targetLm": "",
            "currentLm": str(
                payload.get(current_lm_key)
                or payload.get("currentLm")
                or ""
            ),
        }
        pose_key = (
            "blockedPose"
            if check.get("blocked")
            else "nextPose"
        )
        pose = payload.get(pose_key)
        if isinstance(pose, dict):
            update_payload["pose"] = pose
        # Manual commands arrive at 30 Hz while rendering runs at 60 FPS.
        # Return only the changed robot; the websocket owns full snapshots.
        result = owner.manager.update_robot(
            update_payload,
            include_state=False,
        )
        return {
            "ok": True,
            "blocked": bool(check.get("blocked")),
            "reason": str(check.get("reason") or ""),
            "index": check.get("index"),
            "pose": check.get("pose"),
            "robot": result.get("robot"),
            "state": None,
        }

    def manual_stop_payload(
        self,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        owner = self.owner
        owner._sync_manager_mode()
        name = str(payload.get("name") or "").strip()
        if not name:
            raise ValueError("robot name is required")
        if owner.mode == "robots":
            result = owner.manager.teleop_stop_robot(
                {"name": name}
            )
            return {
                "ok": True,
                "robot": result.get("robot"),
                "state": owner._state_with_context(
                    result.get("state")
                ),
            }

        update_payload = {
            "name": name,
            "status": "IDLE",
            "targetLm": "",
            "currentLm": str(
                payload.get("currentLm") or ""
            ),
        }
        pose = payload.get("pose")
        if isinstance(pose, dict):
            update_payload["pose"] = pose
        result = owner.manager.update_robot(update_payload)
        return {
            "ok": True,
            "robot": result.get("robot"),
            "state": owner._state_with_context(
                result.get("state")
            ),
        }

    def acquire_control_payload(
        self,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        owner = self.owner
        owner._sync_manager_mode()
        if owner.mode != "robots":
            raise ValueError("control leases are available only for real robots")
        return owner._result_with_context(
            owner.manager.acquire_robot_control(payload)
        )

    def release_control_payload(
        self,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        owner = self.owner
        owner._sync_manager_mode()
        if owner.mode != "robots":
            raise ValueError("control leases are available only for real robots")
        return owner._result_with_context(
            owner.manager.release_robot_control(payload)
        )

    def note_external_control_takeover(
        self,
        endpoint: str,
        *,
        owner_id: str,
        owner_name: str,
    ) -> bool:
        owner = self.owner
        if owner.mode != "robots":
            return False
        owner._sync_manager_mode()
        return owner.manager.note_external_control_takeover(
            endpoint,
            owner_id=owner_id,
            owner_name=owner_name,
        )
