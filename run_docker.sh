#! /bin/bash

XAUTH=/tmp/.docker.xauth

PWD=`pwd`

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

xhost +local:docker
docker build --platform linux/amd64 -t robot-spot:latest .

# Original command (broke)
docker run -it --net=host --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH:rw" \
    --volume="$PWD:/host"\
    --volume="$PWD/.bash_history:/home/robot/.bash_history"\
    --volume="$HOME/.ssh:/home/robot/.ssh"\
    --device=/dev/input/js0\
    --privileged \
    --user="$(id -u):$(id -g)" \
    robot-spot:latest \
    /bin/bash
