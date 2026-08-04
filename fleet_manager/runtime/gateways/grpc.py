"""Real robot gateway backed by the existing gRPC tunnel."""

from ..grpc.api.client import GrpcRobotAdapter


class GrpcRobotGateway(GrpcRobotAdapter):
    """Named architecture boundary over the production gRPC adapter."""

    transport = "grpc"
