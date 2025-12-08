from setuptools import find_packages
from setuptools import setup

package_name = 'ros2ai'

setup(
    name=package_name,
    version='0.1.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Tomoya Fujita',
    author_email='tomoya.fujita825@gmail.com',
    maintainer='Tomoya Fujita',
    maintainer_email='tomoya.fujita825@gmail.com',
    url='https://github.com/fujitatomoya/ros2ai',
    download_url='https://github.com/fujitatomoya/ros2ai/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The OpenAI command for ROS 2 command line tools.',
    long_description="""\
        The package provides the AI command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'ai = ros2ai.command.ai:AiCommand',
        ],
        'ros2ai.extension_point': [
            'ros2ai.verb = ros2ai.verb:VerbExtension',
        ],
        'ros2ai.verb': [
            'status = ros2ai.verb.status:StatusVerb',
            'query = ros2ai.verb.query:QueryVerb',
            'exec = ros2ai.verb.exec:ExecVerb',
        ],
    }
)