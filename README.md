To run and build:

1) From your Structure Core SDK download, copy the .so library from:

Libraries/Structure/Linux/x86_64 to lib/libStructure.so

2) Copy the ST folder from

Libaries/Shared/Headers to include/ST

into this repo.

Then run:

2) source /opt/ros/DISTRO/setup.bash
(source /opt/ros/kinetic/setup.bash or source /opt/ros/melodic/setup.bash)

3) catkin_make -DCMAKE_BUILD_TYPE=Release

Release is important or the code will not optimize correctly and may crash or run very poorly.

