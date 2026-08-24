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
        self.TeleopStream = channel.stream_stream(
            "/robot.grpc.v1.RobotApi/TeleopStream",
            request_serializer=robot__api__pb2.TeleopRequest.SerializeToString,
            response_deserializer=robot__api__pb2.CommandResponse.FromString,
        )
        self.Stop = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/Stop",
            request_serializer=robot__api__pb2.StopRequest.SerializeToString,
            response_deserializer=robot__api__pb2.CommandResponse.FromString,
        )
        self.AcquireControl = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/AcquireControl",
            request_serializer=robot__api__pb2.ControlRequest.SerializeToString,
            response_deserializer=robot__api__pb2.CommandResponse.FromString,
        )
        self.ReleaseControl = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/ReleaseControl",
            request_serializer=robot__api__pb2.ControlRequest.SerializeToString,
            response_deserializer=robot__api__pb2.CommandResponse.FromString,
        )
        self.Relocate = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/Relocate",
            request_serializer=robot__api__pb2.RelocateRequest.SerializeToString,
            response_deserializer=robot__api__pb2.CommandResponse.FromString,
        )
        self.ConfirmLocalization = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/ConfirmLocalization",
            request_serializer=robot__api__pb2.ConfirmLocalizationRequest.SerializeToString,
            response_deserializer=robot__api__pb2.CommandResponse.FromString,
        )
        self.PauseRoute = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/PauseRoute",
            request_serializer=robot__api__pb2.PauseRouteRequest.SerializeToString,
            response_deserializer=robot__api__pb2.CommandResponse.FromString,
        )
        self.ResumeRoute = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/ResumeRoute",
            request_serializer=robot__api__pb2.ResumeRouteRequest.SerializeToString,
            response_deserializer=robot__api__pb2.CommandResponse.FromString,
        )
        self.WatchLaserScan = channel.unary_stream(
            "/robot.grpc.v1.RobotApi/WatchLaserScan",
            request_serializer=robot__api__pb2.WatchLaserScanRequest.SerializeToString,
            response_deserializer=robot__api__pb2.LaserScanFrame.FromString,
        )
        self.GetSlamDefaults = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/GetSlamDefaults",
            request_serializer=robot__api__pb2.SlamDefaultsRequest.SerializeToString,
            response_deserializer=robot__api__pb2.SlamDefaultsResponse.FromString,
        )
        self.StartSlam = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/StartSlam",
            request_serializer=robot__api__pb2.StartSlamRequest.SerializeToString,
            response_deserializer=robot__api__pb2.SlamStateResponse.FromString,
        )
        self.GetSlamState = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/GetSlamState",
            request_serializer=robot__api__pb2.SlamStateRequest.SerializeToString,
            response_deserializer=robot__api__pb2.SlamStateResponse.FromString,
        )
        self.WatchSlamMap = channel.unary_stream(
            "/robot.grpc.v1.RobotApi/WatchSlamMap",
            request_serializer=robot__api__pb2.WatchSlamMapRequest.SerializeToString,
            response_deserializer=robot__api__pb2.SlamMapFrame.FromString,
        )
        self.FinishSlam = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/FinishSlam",
            request_serializer=robot__api__pb2.FinishSlamRequest.SerializeToString,
            response_deserializer=robot__api__pb2.SlamFinishResponse.FromString,
        )
        self.CancelSlam = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/CancelSlam",
            request_serializer=robot__api__pb2.CancelSlamRequest.SerializeToString,
            response_deserializer=robot__api__pb2.SlamStateResponse.FromString,
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
        self.GetParams = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/GetParams",
            request_serializer=robot__api__pb2.ParamsRequest.SerializeToString,
            response_deserializer=robot__api__pb2.ParamsResponse.FromString,
        )
        self.PutParams = channel.unary_unary(
            "/robot.grpc.v1.RobotApi/PutParams",
            request_serializer=robot__api__pb2.PutParamsRequest.SerializeToString,
            response_deserializer=robot__api__pb2.ParamsResponse.FromString,
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

    def TeleopStream(self, request_iterator, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def Stop(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def AcquireControl(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def ReleaseControl(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def Relocate(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def ConfirmLocalization(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def PauseRoute(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def ResumeRoute(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def WatchLaserScan(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def GetSlamDefaults(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def StartSlam(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def GetSlamState(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def WatchSlamMap(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def FinishSlam(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def CancelSlam(self, request, context):
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

    def GetParams(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def PutParams(self, request, context):
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
        "TeleopStream": grpc.stream_stream_rpc_method_handler(
            servicer.TeleopStream,
            request_deserializer=robot__api__pb2.TeleopRequest.FromString,
            response_serializer=robot__api__pb2.CommandResponse.SerializeToString,
        ),
        "Stop": grpc.unary_unary_rpc_method_handler(
            servicer.Stop,
            request_deserializer=robot__api__pb2.StopRequest.FromString,
            response_serializer=robot__api__pb2.CommandResponse.SerializeToString,
        ),
        "AcquireControl": grpc.unary_unary_rpc_method_handler(
            servicer.AcquireControl,
            request_deserializer=robot__api__pb2.ControlRequest.FromString,
            response_serializer=robot__api__pb2.CommandResponse.SerializeToString,
        ),
        "ReleaseControl": grpc.unary_unary_rpc_method_handler(
            servicer.ReleaseControl,
            request_deserializer=robot__api__pb2.ControlRequest.FromString,
            response_serializer=robot__api__pb2.CommandResponse.SerializeToString,
        ),
        "Relocate": grpc.unary_unary_rpc_method_handler(
            servicer.Relocate,
            request_deserializer=robot__api__pb2.RelocateRequest.FromString,
            response_serializer=robot__api__pb2.CommandResponse.SerializeToString,
        ),
        "ConfirmLocalization": grpc.unary_unary_rpc_method_handler(
            servicer.ConfirmLocalization,
            request_deserializer=robot__api__pb2.ConfirmLocalizationRequest.FromString,
            response_serializer=robot__api__pb2.CommandResponse.SerializeToString,
        ),
        "PauseRoute": grpc.unary_unary_rpc_method_handler(
            servicer.PauseRoute,
            request_deserializer=robot__api__pb2.PauseRouteRequest.FromString,
            response_serializer=robot__api__pb2.CommandResponse.SerializeToString,
        ),
        "ResumeRoute": grpc.unary_unary_rpc_method_handler(
            servicer.ResumeRoute,
            request_deserializer=robot__api__pb2.ResumeRouteRequest.FromString,
            response_serializer=robot__api__pb2.CommandResponse.SerializeToString,
        ),
        "WatchLaserScan": grpc.unary_stream_rpc_method_handler(
            servicer.WatchLaserScan,
            request_deserializer=robot__api__pb2.WatchLaserScanRequest.FromString,
            response_serializer=robot__api__pb2.LaserScanFrame.SerializeToString,
        ),
        "GetSlamDefaults": grpc.unary_unary_rpc_method_handler(
            servicer.GetSlamDefaults,
            request_deserializer=robot__api__pb2.SlamDefaultsRequest.FromString,
            response_serializer=robot__api__pb2.SlamDefaultsResponse.SerializeToString,
        ),
        "StartSlam": grpc.unary_unary_rpc_method_handler(
            servicer.StartSlam,
            request_deserializer=robot__api__pb2.StartSlamRequest.FromString,
            response_serializer=robot__api__pb2.SlamStateResponse.SerializeToString,
        ),
        "GetSlamState": grpc.unary_unary_rpc_method_handler(
            servicer.GetSlamState,
            request_deserializer=robot__api__pb2.SlamStateRequest.FromString,
            response_serializer=robot__api__pb2.SlamStateResponse.SerializeToString,
        ),
        "WatchSlamMap": grpc.unary_stream_rpc_method_handler(
            servicer.WatchSlamMap,
            request_deserializer=robot__api__pb2.WatchSlamMapRequest.FromString,
            response_serializer=robot__api__pb2.SlamMapFrame.SerializeToString,
        ),
        "FinishSlam": grpc.unary_unary_rpc_method_handler(
            servicer.FinishSlam,
            request_deserializer=robot__api__pb2.FinishSlamRequest.FromString,
            response_serializer=robot__api__pb2.SlamFinishResponse.SerializeToString,
        ),
        "CancelSlam": grpc.unary_unary_rpc_method_handler(
            servicer.CancelSlam,
            request_deserializer=robot__api__pb2.CancelSlamRequest.FromString,
            response_serializer=robot__api__pb2.SlamStateResponse.SerializeToString,
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
        "GetParams": grpc.unary_unary_rpc_method_handler(
            servicer.GetParams,
            request_deserializer=robot__api__pb2.ParamsRequest.FromString,
            response_serializer=robot__api__pb2.ParamsResponse.SerializeToString,
        ),
        "PutParams": grpc.unary_unary_rpc_method_handler(
            servicer.PutParams,
            request_deserializer=robot__api__pb2.PutParamsRequest.FromString,
            response_serializer=robot__api__pb2.ParamsResponse.SerializeToString,
        ),
    }
    generic_handler = grpc.method_handlers_generic_handler("robot.grpc.v1.RobotApi", rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))
