To run and build:

1) From your Structure Core SDK download, copy the include and lib folders from:

StructureSDK-CrossPlatform-0.6.1-2.0/Linux/Libraries/Structure/x86_64

into this repo.

Then run:

2) source /opt/ros/DISTRO/setup.bash
(source /opt/ros/kinetic/setup.bash or source /opt/ros/melodic/setup.bash)

3) catkin_make -DCMAKE_BUILD_TYPE=Release

Release is important or the code will not optimize correctly and may crash or run very poorly.

