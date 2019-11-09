#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/fizzer/enph353-team-grimm/grimm_gym_gazebo_ws/gym_gazebo/envs/enph353/src/gazebo_ros_pkgs/gazebo_ros"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/fizzer/enph353-team-grimm/grimm_gym_gazebo_ws/gym_gazebo/envs/enph353/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/fizzer/enph353-team-grimm/grimm_gym_gazebo_ws/gym_gazebo/envs/enph353/install/lib/python2.7/dist-packages:/home/fizzer/enph353-team-grimm/grimm_gym_gazebo_ws/gym_gazebo/envs/enph353/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/fizzer/enph353-team-grimm/grimm_gym_gazebo_ws/gym_gazebo/envs/enph353/build" \
    "/usr/bin/python2" \
    "/home/fizzer/enph353-team-grimm/grimm_gym_gazebo_ws/gym_gazebo/envs/enph353/src/gazebo_ros_pkgs/gazebo_ros/setup.py" \
    build --build-base "/home/fizzer/enph353-team-grimm/grimm_gym_gazebo_ws/gym_gazebo/envs/enph353/build/gazebo_ros_pkgs/gazebo_ros" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/fizzer/enph353-team-grimm/grimm_gym_gazebo_ws/gym_gazebo/envs/enph353/install" --install-scripts="/home/fizzer/enph353-team-grimm/grimm_gym_gazebo_ws/gym_gazebo/envs/enph353/install/bin"
