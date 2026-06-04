import os
import glob
from setuptools import find_packages, setup

package_name = 'robot_web_nav'


def _collect_web_files():
    result = []
    for f in glob.glob('web/**/*', recursive=True):
        if os.path.isfile(f):
            dir_rel = os.path.dirname(f)
            result.append((os.path.join('share', package_name, dir_rel), [f]))
    return result

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + _collect_web_files() + [
        (os.path.join('share', package_name, 'launch'),
            [f for f in glob.glob('launch/*.py') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools', 'aiohttp', 'Pillow', 'PyYAML'],
    zip_safe=True,
    maintainer='sanko',
    maintainer_email='sanko_working@163.com',
    description='Web-based 2D map visualization and click-to-navigate interface',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'web_nav_server = robot_web_nav.web_server:main',
        ],
    },
)
