#!/bin/bash

pip install -U rospkg

apt-get install python-rospkg

rosdep install --from-paths src --ignore-src -r -y
