from setuptools import setup

package_name = "referee_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "plugin.xml"]),
        ("share/" + package_name + "/launch", ["launch/referee_sim.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nuc",
    maintainer_email="nuc@example.com",
    description="Referee system simulator node and rqt plugin.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "referee_sim_node = referee_sim.referee_sim_node:main",
        ],
        "rqt_gui_py.plugins": [
            "RefereeSimPlugin = referee_sim.referee_sim_plugin:RefereeSimPlugin",
        ],
    },
)
