import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _reset_odom_before_slam(context, *args, **kwargs):
    del args, kwargs
    service_name = LaunchConfiguration('reset_odom_service').perform(context).strip()
    if not service_name:
        return []
    timeout_sec = float(LaunchConfiguration('reset_odom_timeout').perform(context))

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Empty

    reset_context = Context()
    rclpy.init(args=[], context=reset_context)
    node = rclpy.create_node(
        'slam_toolbox_reset_odom',
        context=reset_context,
        cli_args=[],
        use_global_arguments=False,
        enable_rosout=False,
        start_parameter_services=False,
    )
    executor = SingleThreadedExecutor(context=reset_context)
    executor.add_node(node)
    try:
        client = node.create_client(Empty, service_name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f'{service_name} service is not available')
        future = client.call_async(Empty.Request())
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok(context=reset_context) and not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        if not future.done():
            raise RuntimeError(f'{service_name} service timed out')
        if future.exception() is not None:
            raise RuntimeError(f'{service_name} service failed: {future.exception()}')
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        reset_context.try_shutdown()
    return [LogInfo(msg=f'[LifecycleLaunch] Odometry reset via {service_name}.')]


def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("slam_toolbox"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    declare_reset_odom_service = DeclareLaunchArgument(
        'reset_odom_service', default_value='/reset_odom',
        description='std_srvs/Empty service called before online SLAM starts. Empty disables it.')
    declare_reset_odom_timeout = DeclareLaunchArgument(
        'reset_odom_timeout', default_value='3.0',
        description='Seconds to wait for reset_odom_service before failing launch.')
    reset_odom_action = OpaqueFunction(function=_reset_odom_before_slam)

    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
          slam_params_file,
          {
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    configure_event = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
          transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_reset_odom_service)
    ld.add_action(declare_reset_odom_timeout)
    ld.add_action(reset_odom_action)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld
