from setuptools import find_packages, setup

package_name = "route_manager"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "resource/map.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tobinio",
    maintainer_email="tobinio@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["route_manager = route_manager.route_manager:main"],
    },
)
