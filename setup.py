from setuptools import setup

package_name = 'metu_rqt_plugin'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['resource/plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='volki',
    maintainer_email='volki@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "plugin_example = metu_rqt_plugin.plugin_example:main",
            "competition_map_plugin = metu_rqt_plugin.competition_map_plugin:main",
            "topic_controller_plugin = metu_rqt_plugin.topic_controller_plugin:main",
            "camera_data_publisher = metu_rqt_plugin.demo_nodes.camera_data_publisher:main",
            "rover_gps_publisher = metu_rqt_plugin.demo_nodes.rover_gps_publisher:main",
            "rover_gps_subscriber = metu_rqt_plugin.demo_nodes.rover_gps_subscriber:main",
            "gui_gps_publisher = metu_rqt_plugin.demo_nodes.gui_gps_publisher:main",
            "gui_gps_subscriber_thread = metu_rqt_plugin.gui_gps_subscriber_thread:main",
            "demo_sensor_publisher = metu_rqt_plugin.demo_nodes.demo_sensor_publisher:main"
        ],
    },
)
