from setuptools import setup

package_name = "bag2_recorder"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="GeminiNinth",
    maintainer_email="gemini.ninth@gmail.com",
    description="ROS2 bag recording package with Python implementation",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bag2_recorder_node = bag2_recorder.bag2_launcher:main",
        ],
    },
)
