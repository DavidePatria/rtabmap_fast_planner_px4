# intro

this ia sadjhskjfkfjlsajlkjlksadjvlkjnvkasnvsnvksnvlndd
# instructions

Install prerequisites

```
# Add the package repositories
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

Suppose the goal is to run the iris simulation in conjunction with planner, slam, controller and px4.

First the base images has to be built.
The files named `base-*` are the dockerfiles for an image containing ubuntu with number version and the coupled ros version and nvidia cuda capabilities.

Before building the final container (the one that actually provides the simulator needed), the intermediate container has to be built.

The name of the base images must be their file name for the successive steps to complete succesfully.

For `melodic-nvidia-18-04` the command to be run inside the main repositorium folder is

`docker build -f melodic-nvidia-18-04 -t melodic-nvidia-18-04:latest .`

specifying the file with `-f` and the tag with `-t`.

In reality there are no limitations on the name of the container, but the final container uses this intermediate one as a base, the same as being done here with another image invoked in the dockerfile, and a fix name has to be specified.
After building this image check that everything is correct by running `docker images` and verifying the presence of an image with the aforementioned tag name.

Next the image relative to the simulator can be built, even though in reality this two processes could be launched in parallel, by entering the folder `base-melodic-cuda-18-04` and running `docker build -t base-melodic-cuda-18-04 . ` to build the base image.
Again, the tag has to be the shown one as the next image will call it explicitly.
Then, to build the docker for `fast-planner` run `docker build -f fast-planner -t fast-planner:latest`, with a tag of choice, here `px4-planner` has been used.

Some notes: the containers are configured to use a different path for `catkin_ws` as ros workspace for containerization reasons.
This because the need for a way to edit the file with an external program (which is neovim of course) for development purposes has been felt as necessary.
The shared volumes are specified in `--volume="$HOME/volume_name:/home/docker/catkin_ws:rw" \` where the first path in the argument is the host folder.
An example name has been used to remember to use distinguishable. 

**NOTE:** shared volumes are cleaned when the container is run the first time, therefore another folder apart from catkin_ws has to be used. In case one wants to move there the ros workspace, simply copy src over to te new location (`shared_volume` in the example below) and build again the workspace. 
**Remember** to adjust the sourcing in the bashrc to the new workspace or source manually

Once the images are built the containers can by created by running the following command, paying attention to the necessary substitution

```
docker run -it \
    --network host \
    --user=docker \ 
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    -v /dev:/dev \
    --group-add=dialout \
    --group-add=video \
    --group-add=tty \
    --volume="$HOME/path/to/volume/on/host:/home/docker/volume:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --volume="/etc/localtime:/etc/localtime:ro" \
    --volume="/tmp/.docker.xauth:/tmp/.docker.xauth" \
    -env="XAUTHORITY=/tmp/.docker.xauth" \
    --workdir="/home/docker" \ 
    --name="fast-planner" \
    --privileged \
    --gpus all \
    fast-planner \
    bash -c "/bin/bash"
```
