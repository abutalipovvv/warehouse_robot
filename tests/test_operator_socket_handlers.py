from __future__ import annotations

from urllib.parse import urlparse

from operator_app.web.handler import OperatorRequestHandler
from operator_app.web.socket_handlers import OperatorWebSocketHandlerMixin


def test_request_handler_inherits_websocket_capability() -> None:
    assert issubclass(
        OperatorRequestHandler,
        OperatorWebSocketHandlerMixin,
    )
    assert (
        OperatorRequestHandler._handle_robot_teleop_ws
        is OperatorWebSocketHandlerMixin._handle_robot_teleop_ws
    )


def test_stream_revision_index_tracks_current_robots_only() -> None:
    revisions = {"removed": 3, "kept": 2}

    OperatorWebSocketHandlerMixin._update_stream_route_revisions(
        revisions,
        {
            "state": {
                "robots": [
                    {"name": "kept", "routeRevision": "7"},
                    {"name": "new", "routeRevision": None},
                    {"routeRevision": 10},
                    "invalid",
                ]
            }
        },
    )

    assert revisions == {"kept": 7, "new": 0}


def test_websocket_query_values_are_normalized() -> None:
    parsed = urlparse(
        "/ws/robot/scan"
        "?robotId=%20robot-1%20"
        "&hz=invalid"
        "&includeIntensities=YES"
    )
    query = {
        "hz": ["invalid"],
        "includeIntensities": ["YES"],
    }

    assert (
        OperatorWebSocketHandlerMixin._robot_id_from_query(parsed)
        == "robot-1"
    )
    assert (
        OperatorWebSocketHandlerMixin._float_query_value(
            query,
            "hz",
            1.0,
        )
        == 1.0
    )
    assert OperatorWebSocketHandlerMixin._boolean_query_value(
        query,
        "includeIntensities",
        default=False,
    )
