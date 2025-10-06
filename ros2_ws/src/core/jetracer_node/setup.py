from setuptools import setup
from glob import glob


package_name = 'jetracer_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # launchディレクトリ内の全ての.pyと.xmlをインストール対象にする,
        ('share/' + package_name + '/launch', glob('launch/*.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetracer_node = jetracer_node.control_node:main'
        ],
    },
)
