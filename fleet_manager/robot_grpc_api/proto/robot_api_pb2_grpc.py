# Generated-compatible gRPC helpers for robot_grpc_api/proto/robot_api.proto.
# grpcio-tools is intentionally not required at build time.

from __future__ import annotations

import grpc

from . import robot_api_pb2 as robot__api__pb2


class RobotApiStub:
    def __init__(self, channel):
        self.Health = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/Health",
            request_serializer=robot__api__pb2.HealthRequest.SerializeToString,
            response_deserializer=robot__api__pb2.HealthResponse.FromString,
        )
        self.Identity = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/Identity",
            request_serializer=robot__api__pb2.IdentityRequest.SerializeToString,
            response_deserializer=robot__api__pb2.IdentityResponse.FromString,
        )
        self.GetStatus = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/GetStatus",
            request_serializer=robot__api__pb2.StatusRequest.SerializeToString,
            response_deserializer=robot__api__pb2.StatusResponse.FromString,
        )
        self.WatchStatus = channel.unary_stream(
            "/robot.grpc.v1.RobotApi/WatchStatus",
            request_serializer=robot__api__pb2.WatchStatusRequest.SerializeToString,
            response_deserializer=robot__api__pb2.StatusResponse.FromString,
        )
        self.ExecuteRoute = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/ExecuteRoute",
            request_serializer=robot__api__pb2.ExecuteRouteRequest.SerializeToString,
            response_deserializer=robot__api__pb2.CommandResponse.FromString,
        )
        self.CancelRoute = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/CancelRoute",
            request_serializer=robot__api__pb2.CancelRouteRequest.SerializeToString,
            response_deserializer=robot__api__pb2.CommandResponse.FromString,
        )
        self.Teleop = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/Teleop",
            request_serializer=robot__api__pb2.TeleopRequest.SerializeToString,
            response_deserializer=robot__api__pb2.CommandResponse.FromString,
        )
        self.Stop = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/Stop",
            request_serializer=robot__api__pb2.StopRequest.SerializeToString,
            response_deserializer=robot__api__pb2.CommandResponse.FromString,
        )
        self.ListMaps = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/ListMaps",
            request_serializer=robot__api__pb2.ListMapsRequest.SerializeToString,
            response_deserializer=robot__api__pb2.ListMapsResponse.FromString,
        )
        self.GetMapBundle = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/GetMapBundle",
            request_serializer=robot__api__pb2.GetMapBundleRequest.SerializeToString,
            response_deserializer=robot__api__pb2.MapBundleResponse.FromString,
        )
        self.PutMapBundle = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/PutMapBundle",
            request_serializer=robot__api__pb2.PutMapBundleRequest.SerializeToString,
            response_deserializer=robot__api__pb2.MapBundleResponse.FromString,
        )
        self.LoadMap = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/LoadMap",
            request_serializer=robot__api__pb2.LoadMapRequest.SerializeToString,
            response_deserializer=robot__api__pb2.MapBundleResponse.FromString,
        )


class RobotApiServicer:
    def Health(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def Identity(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def GetStatus(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def WatchStatus(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def ExecuteRoute(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def CancelRoute(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def Teleop(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def Stop(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def ListMaps(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def GetMapBundle(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def PutMapBundle(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def LoadMap(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")


def add_RobotApiServicer_to_server(servicer, server):
    rpc_method_handlers = {
        "Health": grpc.unary_unary_rpc_method_handler(
            servicer.Health,
            request_deserializer=robot__api__pb2.HealthRequest.FromString,
            response_serializer=robot__api__pb2.HealthResponse.SerializeToString,
        ),
        "Identity": grpc.unary_unary_rpc_method_handler(
            servicer.Identity,
            request_deserializer=robot__api__pb2.IdentityRequest.FromString,
            response_serializer=robot__api__pb2.IdentityResponse.SerializeToString,
        ),
        "GetStatus": grpc.unary_unary_rpc_method_handler(
            servicer.GetStatus,
            request_deserializer=robot__api__pb2.StatusRequest.FromString,
            response_serializer=robot__api__pb2.StatusResponse.SerializeToString,
        ),
        "WatchStatus": grpc.unary_stream_rpc_method_handler(
            servicer.WatchStatus,
            request_deserializer=robot__api__pb2.WatchStatusRequest.FromString,
            response_serializer=robot__api__pb2.StatusResponse.SerializeToString,
        ),
        "ExecuteRoute": grpc.unary_unary_rpc_method_handler(
            servicer.ExecuteRoute,
            request_deserializer=robot__api__pb2.ExecuteRouteRequest.FromString,
            response_serializer=robot__api__pb2.CommandResponse.SerializeToString,
        ),
        "CancelRoute": grpc.unary_unary_rpc_method_handler(
            servicer.CancelRoute,
            request_deserializer=robot__api__pb2.CancelRouteRequest.FromString,
            response_serializer=robot__api__pb2.CommandResponse.SerializeToString,
        ),
        "Teleop": grpc.unary_unary_rpc_method_handler(
            servicer.Teleop,
            request_deserializer=robot__api__pb2.TeleopRequest.FromString,
            response_serializer=robot__api__pb2.CommandResponse.SerializeToString,
        ),
        "Stop": grpc.unary_unary_rpc_method_handler(
            servicer.Stop,
            request_deserializer=robot__api__pb2.StopRequest.FromString,
            response_serializer=robot__api__pb2.CommandResponse.SerializeToString,
        ),
        "ListMaps": grpc.unary_unary_rpc_method_handler(
            servicer.ListMaps,
            request_deserializer=robot__api__pb2.ListMapsRequest.FromString,
            response_serializer=robot__api__pb2.ListMapsResponse.SerializeToString,
        ),
        "GetMapBundle": grpc.unary_unary_rpc_method_handler(
            servicer.GetMapBundle,
            request_deserializer=robot__api__pb2.GetMapBundleRequest.FromString,
            response_serializer=robot__api__pb2.MapBundleResponse.SerializeToString,
        ),
        "PutMapBundle": grpc.unary_unary_rpc_method_handler(
            servicer.PutMapBundle,
            request_deserializer=robot__api__pb2.PutMapBundleRequest.FromString,
            response_serializer=robot__api__pb2.MapBundleResponse.SerializeToString,
        ),
        "LoadMap": grpc.unary_unary_rpc_method_handler(
            servicer.LoadMap,
            request_deserializer=robot__api__pb2.LoadMapRequest.FromString,
            response_serializer=robot__api__pb2.MapBundleResponse.SerializeToString,
        ),
    }
    generic_handler = grpc.method_handlers_generic_handler("robot.grpc.v1.RobotApi", rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))
