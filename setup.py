from setuptools import setup, find_packages

package_name = "srdfdom"

setup(
    name=package_name,
    version="2.0.2",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    url="",
    license="",
    author="Ioan Sucan, Guillaume Walck",
    author_email="isucan@willowgarage.com, gwalck@techfak.uni-bielefeld.de",
    description="Parser for Semantic Robot Description Format (SRDF)",
    entry_points={
        "console_scripts": [
            "display_srdf = srdfdom.display_srdf:main",
        ],
    },
)
