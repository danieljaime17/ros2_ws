from setuptools import setup

package_name = 'usb_camera_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fernando',
    maintainer_email='danijaime17@gmail.com',
    description='Nodo que publica imágenes desde cámara USB',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'usb_camera_node = usb_camera_publisher.usb_camera_node:main',
            'camera_front_node = usb_camera_publisher.camera_front_node:main',
            'camera_rear_node = usb_camera_publisher.camera_rear_node:main',
            'camera_ethernet_node = your_package.camera_ethernet_node:main',
            'record_node = usb_camera_publisher.record_node:main',
	        'detector_objetos = usb_camera_publisher.detector_objetos:main'
        ],
    },
)

