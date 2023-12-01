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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/sebastien/ProjetVA50/VA50-navigation/src/geometry2/tf2_geometry_msgs"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sebastien/ProjetVA50/VA50-navigation/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sebastien/ProjetVA50/VA50-navigation/install/lib/python3/dist-packages:/home/sebastien/ProjetVA50/VA50-navigation/build/tf2_geometry_msgs/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sebastien/ProjetVA50/VA50-navigation/build/tf2_geometry_msgs" \
    "/usr/bin/python3" \
    "/home/sebastien/ProjetVA50/VA50-navigation/src/geometry2/tf2_geometry_msgs/setup.py" \
    egg_info --egg-base /home/sebastien/ProjetVA50/VA50-navigation/build/tf2_geometry_msgs \
    build --build-base "/home/sebastien/ProjetVA50/VA50-navigation/build/tf2_geometry_msgs" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sebastien/ProjetVA50/VA50-navigation/install" --install-scripts="/home/sebastien/ProjetVA50/VA50-navigation/install/bin"
