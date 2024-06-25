from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    start_rviz_rl = LaunchConfiguration("rviz")
    start_rviz_py = start_rviz_rl.perform(context)
    fake_rl = LaunchConfiguration("fake")
    fake_py = fake_rl.perform(context) == "true"
    gazebo_rl = LaunchConfiguration("gazebo")
    gazebo_py = gazebo_rl.perform(context) == "true"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("bipedal_description"), "urdf", "bipedal.urdf"]
            ),
            *(
                (" ", "use_fake_hardware:=true", " ")
                if fake_py
                else (
                    (" ", "use_fake_hardware:=true use_gazebo:=true", " ")
                    if gazebo_py
                    else (" ",)
                )
            ),
            " ",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("bipedal_description"),
            "controllers",
            f"bipedal_controllers.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("bipedal_description"),
            "config",
            f"{start_rviz_py}.rviz" if start_rviz_py != "true" else "bipedal.rviz",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(PythonExpression(f"'{start_rviz_py}' != 'false'")),
    )

    gazebo_state_broadcaster_params = PathJoinSubstitution(
        [
            FindPackageShare("bipedal_gazebo"),
            "config",
            "gz_state_broadcaster_params.yaml",
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *(
                ("joint_state_broadcaster", "-p", gazebo_state_broadcaster_params)
                if gazebo_py
                else ("joint_state_broadcaster",)
            ),
            "--controller-manager",
            "/controller_manager",
        ],
    )

    l_leg_forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["l_leg_forward_position_controller", "-c", "/controller_manager"],
    )

    r_leg_forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["r_leg_forward_position_controller", "-c", "/controller_manager"],
    )

    forward_torque_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_torque_controller", "-c", "/controller_manager"],
    )

    forward_torque_limit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_torque_limit_controller", "-c", "/controller_manager"],
    )

    forward_speed_limit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_speed_limit_controller", "-c", "/controller_manager"],
    )

    forward_pid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_pid_controller", "-c", "/controller_manager"],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
    )

    dynamic_state_router_node = Node(
        package="dynamic_state_router",
        executable="dynamic_state_router",
        arguments=[robot_controllers],
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("bipedal_gazebo"), "/launch", "/gazebo.launch.py"]
        ),
        condition=IfCondition(LaunchConfiguration("gazebo")),
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    l_leg_forward_position_controller_spawner,
                    r_leg_forward_position_controller_spawner,
                    forward_torque_controller_spawner,
                    forward_torque_limit_controller_spawner,
                    forward_speed_limit_controller_spawner,
                    forward_pid_controller_spawner,
                ],
            ),
        )
    )

    return [
        *((control_node,) if not gazebo_py else (SetUseSimTime(True), gazebo_node)),
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        dynamic_state_router_node,
    ]


def generate_launch_description():

    # for each file, if it is a .rviz file, add it to the list of choices without the .rviz extension
    # rviz_config_choices = []
    # for file in os.listdir(os.path.dirname(os.path.realpath(__file__)) + "/../../../../bipedal_description/config/"):
    #     if file.endswith(".rviz"):
    #         rviz_config_choices.append(file[:-5])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz2 automatically with this launch file.",
                # choices=["true", "false", *rviz_config_choices],
                choices=["true", "false", "bipedal.rviz"],
            ),
            DeclareLaunchArgument(
                "fake",
                default_value="false",
                description="Start on fake_reachy mode with this launch file.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "gazebo",
                default_value="false",
                description="Start a fake_hardware with gazebo as simulation tool.",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
