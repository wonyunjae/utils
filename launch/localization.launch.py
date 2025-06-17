from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    glim_config_dir = DeclareLaunchArgument(
        'glim_config_dir',
        default_value='/home/smarthc/ros2_ws/src/glim_ros/config',
        description='GLIM config directory path'
    )
    
    foundation_stereo_path = DeclareLaunchArgument(
        'foundation_stereo_path',
        default_value='/home/smarthc/FoundationStereo',
        description='FoundationStereo project path'
    )

    # 1. GLIM Node
    glim_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_localization',
        output='screen',
        parameters=[{
            'dump_path': '/tmp/glim_dump'
        }]
    )

    # 2. Google Earth Visualizer Node
    google_earth_node = Node(
        package='local_pkg',
        executable='google_earth_visualizer',
        name='google_earth_visualizer',
        output='screen'
    )

    # 3. TAE Localization Node (for UTM publishing)
    tae_localization_node = Node(
        package='local_pkg',
        executable='tae_localization',
        name='tae_localization',
        output='screen'
    )

    # 4. FoundationStereo Process
    foundation_stereo_process = ExecuteProcess(
        cmd=[
            'python3',
            '/home/smarthc/FoundationStereo/scripts/trt_videostream.py',
            '--use_camera',
            '--left_camera_id', '4',
            '--right_camera_id', '6',
            '--pc_interval', '30',
            '--show_accumulated_pc',
            '--out_dir', '/test_outputs/output'
        ],
        name='foundation_stereo',
        output='screen'
    )

    # 5. Localization UI Node
    localization_ui_node = Node(
        package='utils',
        executable='localization_ui',
        name='localization_ui',
        output='screen',
        parameters=[{
            'foundation_stereo_path': LaunchConfiguration('foundation_stereo_path'),
            'glim_config_dir': LaunchConfiguration('glim_config_dir')
        }]
    )

    return LaunchDescription([
        glim_config_dir,
        foundation_stereo_path,
        
        LogInfo(msg="Starting GLIM Localization System..."),
        
        # 노드들을 순차적으로 실행
        tae_localization_node,
        glim_node,
        google_earth_node,
        foundation_stereo_process,
        localization_ui_node,
    ])
