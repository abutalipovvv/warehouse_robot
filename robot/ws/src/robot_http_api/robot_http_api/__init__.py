__all__ = [
    "RobotHttpApiBridge",
    "RobotRosClient",
    "parse_args",
    "resolve_map_dir",
    "serve_http_server",
]


def __getattr__(name):
    if name == "RobotRosClient":
        from .client import RobotRosClient

        return RobotRosClient
    if name in {"RobotHttpApiBridge", "parse_args", "resolve_map_dir", "serve_http_server"}:
        from . import server

        return getattr(server, name)
    raise AttributeError(name)
