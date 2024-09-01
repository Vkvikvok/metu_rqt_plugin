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
            "camera_data_publisher = metu_rqt_plugin.demo_nodes.camera_data_publisher:main",
            "rover_gps_publisher = metu_rqt_plugin.demo_nodes.rover_gps_publisher:main",
            "rover_gps_subscriber = metu_rqt_plugin.demo_nodes.rover_gps_subscriber:main",
            "gui_gps_publisher = metu_rqt_plugin.demo_nodes.gui_gps_publisher:main",
            "gui_gps_subscriber_thread = metu_rqt_plugin.gui_gps_subscriber_thread:main",
            "demo_sensor_publisher = metu_rqt_plugin.demo_nodes.demo_sensor_publisher:main",
            "demo_sensor_publisher2 = metu_rqt_plugin.demo_nodes.demo_sensor_publisher2:main",
            "topic_listener = metu_rqt_plugin.threads.topic_listener_thread:main",
            "flamable_gas_subscriber = metu_rqt_plugin.threads.sensor_subscribers_thread:main",
            "methane_gas_subscriber = metu_rqt_plugin.threads.sensor_subscribers_thread:main",
            "carbon_monoxide_subscriber = metu_rqt_plugin.threads.sensor_subscribers_thread:main",
            "env_temp_subscriber = metu_rqt_plugin.threads.sensor_subscribers_thread:main",
            "env_pressure_subscriber = metu_rqt_plugin.threads.sensor_subscribers_thread:main",
            "env_humidity_subscriber = metu_rqt_plugin.threads.sensor_subscribers_thread:main",
            "drill_control_node = metu_rqt_plugin.threads.sci_hub_publishers_thread:main",
            "sci_plat_control_node = metu_rqt_plugin.thread.sci_hub_publishers_thread:main",
            "drill_head_node = metu_rqt_plugin.threads.sci_hub_publishers_thread:main",
            "sci_plat_rotation_node = metu_rqt_plugin.threads.sci_hub_publishers_thread:main",
            "state_light_node = metu_rqt_plugin.threads.state_light_publisher_thread:main",
        ],
    },
)
