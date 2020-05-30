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

echo_and_run cd "/home/anna/catkin_ws/src/model_based_diagnosis/tug_observers/tug_observers"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/anna/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/anna/catkin_ws/install/lib/python2.7/dist-packages:/home/anna/catkin_ws/build/tug_observers/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/anna/catkin_ws/build/tug_observers" \
    "/usr/bin/python2" \
    "/home/anna/catkin_ws/src/model_based_diagnosis/tug_observers/tug_observers/setup.py" \
    build --build-base "/home/anna/catkin_ws/build/tug_observers" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/anna/catkin_ws/install" --install-scripts="/home/anna/catkin_ws/install/bin"
