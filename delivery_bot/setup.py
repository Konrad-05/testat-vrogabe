from setuptools import setup

package_name = 'delivery_bot_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/delivery_bot']),
        ('share/delivery_bot', ['package.xml']),
        ('share/delivery_bot/launch', ['launch/delivery_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dein Name',
    maintainer_email='deine.email@addresse.com',
    description='Delivery bot with battery management',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navigator = delivery_bot_py.navigator:main',
            'battery_manager = delivery_bot_py.battery_manager:main',
            'order_server = delivery_bot_py.order_server:main',
            'status_display = delivery_bot_py.status_display:main',
        ],
    },
)