from setuptools import find_packages, setup
from setuptools.command.develop import develop


class ColconDevelop(develop):
    """Accept colcon's legacy develop flags with modern setuptools."""

    user_options = develop.user_options + [
        ("editable", None, "accepted for colcon compatibility"),
        ("build-directory=", None, "accepted for colcon compatibility"),
        ("uninstall", None, "accepted for colcon compatibility"),
    ]
    boolean_options = develop.boolean_options + ["editable", "uninstall"]

    def initialize_options(self):
        super().initialize_options()
        self.editable = False
        self.build_directory = None
        self.uninstall = False

    def run(self):
        if self.uninstall:
            return
        super().run()

package_name = 'resense_hex21'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erik',
    maintainer_email='erik.helmut1@gmail.com',
    description='Resense Hex 21 ROS2 Jazzy Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'resense_hex21_node = resense_hex21.resense_hex21_node:main',
        ],
    },
    cmdclass={
        'develop': ColconDevelop,
    },
)
