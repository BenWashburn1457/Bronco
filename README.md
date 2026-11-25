# Bronco
My Bronco Robot Platform

## Docker development environment

Everything (ROS 2 Jazzy, build tools, dependencies) runs in Docker, so the
laptop and the Raspberry Pi get identical environments no matter which OS
they're running.

| Where        | Compose service | Image          | Has                                 |
|--------------|-----------------|----------------|-------------------------------------|
| Laptop       | `dev`           | `bronco:dev`   | ROS 2 + RViz, rqt, gdb (GUI tools)  |
| Raspberry Pi | `robot`         | `bronco:robot` | ROS 2 + hardware access (no GUI)    |

Your code in `ros2_ws/` stays on the host and is mounted into the container,
so you can edit it with normal tools and nothing is lost when a container is
deleted.

### One-time setup (on both laptop and Pi)

```bash
# Install Docker
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER      # run docker without sudo (log out/in after)

# Check your user/group IDs and make .env match them (usually 1000/1000)
id -u; id -g
```

Laptop only: let containers open windows on your screen (run once per login):

```bash
xhost +local:docker
```

### Daily use

```bash
# Laptop
docker compose up -d dev            # build (first time ~10+ min) and start
docker compose exec dev bash        # open a shell inside

# Raspberry Pi
docker compose up -d robot
docker compose exec robot bash
```

Inside the container:

```bash
colcon build --symlink-install      # build the workspace
source install/setup.bash           # load the freshly built packages
ros2 topic list
rviz2                               # laptop only
```

Stop with `docker compose down`. Rebuild the image (after changing the
Dockerfile or a package.xml dependency) with `docker compose build`.

### VS Code

Install the **Dev Containers** extension, open this folder, then run
"Dev Containers: Reopen in Container". VS Code will run inside the `dev`
container.

### Laptop <-> Pi communication

Both containers use host networking and the same `ROS_DOMAIN_ID` (set in
`.env`), so ROS 2 nodes on the Pi and laptop discover each other automatically
when they're on the same Wi-Fi network. Test it:

```bash
# On the Pi (robot container)
ros2 topic pub /chatter std_msgs/msg/String "data: hello from bronco"
# On the laptop (dev container)
ros2 topic echo /chatter
```
