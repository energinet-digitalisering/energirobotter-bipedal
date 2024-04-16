from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction
)

def launch_setup(context, *args, **kwargs):
    ...



def generate_launch_description():

    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )

