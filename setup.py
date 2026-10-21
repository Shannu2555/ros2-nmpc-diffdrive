from glob import glob
from setuptools import setup

pkg = "mpc_diffdrive"
setup(
    name=pkg,
    version="0.1.0",
    packages=[pkg],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{pkg}"]),
        (f"share/{pkg}", ["package.xml"]),
        (f"share/{pkg}/launch", glob("launch/*.py")),
        (f"share/{pkg}/config", glob("config/*.yaml")),
        (f"share/{pkg}/urdf", glob("urdf/*")),
        (f"share/{pkg}/worlds", glob("worlds/*")),
        (f"share/{pkg}/rviz", glob("rviz/*")),
    ],
    install_requires=["setuptools", "casadi", "numpy"],
    zip_safe=True,
    maintainer="Shanmukh Narayana Chaganti",
    description="Nonlinear MPC for a diff-drive robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mpc_node = mpc_diffdrive.mpc_node:main",
            "pure_pursuit_node = mpc_diffdrive.pure_pursuit_node:main",
            "metrics_node = mpc_diffdrive.metrics_node:main",
        ],
    },
)
