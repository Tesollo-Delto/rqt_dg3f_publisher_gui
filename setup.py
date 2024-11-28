from setuptools import find_packages, setup
from glob import glob
package_name = 'rqt_dg3f_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # ("share/" + package_name + "/resource", ["resource/delto_rqt.ui"]),
        # ("share/" + package_name + "/resource", ["resource/icon.png"]),
        ('share/' + package_name + '/resource', glob('resource/*.*')),
        ('share/' + package_name + '/config', glob('config/*.*')),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["plugin.xml"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hong',
    maintainer_email='140060596+Kim-Hongcheol@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rqt_dg3f_publisher = rqt_dg3f_publisher.rqt_dg3f_publisher:main',

        ],
         'rqt_gui_plugins': [
        'rqt_dg3f_publisher = rqt_dg3f_publisher.rqt_dg3f_publisher:RqtDelto3FPublisher'
    ],
    },
)
