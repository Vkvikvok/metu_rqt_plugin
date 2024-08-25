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
            "competition_map_plugin = metu_rqt_plugin.competition_map_plugin:main"
        ],
    },
)
