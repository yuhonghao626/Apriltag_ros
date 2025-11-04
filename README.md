# EHR_AprilTag
此仓库包含了实现Apriltag-ros节点的实现，分支有apriltag源代码、apriltag-ros节点代码、apriltag_msgs代码以及realsense-ros源代码，分别对应四个分支，实际使用时需要都将这四个代码下载。可供参考的飞书文档：https://kcn7eg1rwa1s.feishu.cn/wiki/SJrewKKxOiKTLjkQLZ0cZhQtnkf


## Getting started
首先下载apriltag代码，包含apriltag源代码、apriltag-ros节点代码、apriltag_msgs代码
```
mkdir -p ~/ros2_ws/src

git clone http://192.168.10.100/algorithm_team/ehr_apriltag.git
cd ehr_apriltag
git checkout apriltag-ros 
```
```
cd ~/ros2_ws/src
#apriltag_msgs
git clone https://github.com/AprilRobotics/apriltag.git
#apriltag_msgs
git clone https://github.com/christianrauch/apriltag_msgs.git
```

然后需要安装realsen-ros:

先注册服务器的公钥：
```
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
sudo apt-get install apt-transport-https
```
```
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
```

```
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```
现在可以通过运行连接英特尔实感深度摄像头并运行以验证安装：
```
realsense-viewer 
```
这时输入
```
modinfo uvcvideo | grep "version:" 
```
应包含 RealSense 字符串

下载Intel® RealSense™ ROS2 wrapper  
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

```
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
cd ~/ros2_ws
```
安装依赖：
```
sudo apt-get install python3-rosdep -y
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
```
```
colcon build
```
```
export ROS_DISTRO=humble
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros2_ws
. install/local_setup.bash
```
最后的最后：配置一下环境变量：
```
nano ~/.bashrc
```
在最后新加上环境变量：
```
source /opt/ros/humble/setup.bash #这里要注意你的ros版本
source ~/ros2_ws/install/setup.bash #你的ros工作空间文件目录
```
```
source ~/.bashrc
```

## 启动realsense摄像头节点
```
ros2 run realsense2_camera realsense2_camera_node


# 可以加参数 for example - temporal and spatial filters are enabled:
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p spatial_filter.enable:=true -p temporal_filter.enable:=true
```

## Apriltag-ros使用
首先把所有相关包都编译，src目录下一定要是这个结构：
```
src/
├── apriltag/
├── apriltag-message/
├── ehr_apriltag/
├── realsense-ros/
```
如果缺少任意一个包，都会编译报错！！

```
cd ~/ros2_ws/src
colcon build
```
### 启动realsense相机ros2节点
```
ros2 run realsense2_camera realsense2_camera_node
```
现在看ros2topic应该会有
```
/camera/camera/color/camera_info
/camera/camera/color/image_raw
```
### 启动apriltag-ros节点进行检测
```
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/camera/camera/color/image_raw \
  -r camera_info:=/camera/camera/color/camera_info \
  --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml
```


# AprilTag ROS 2 Node

This ROS 2 node uses the AprilTag library to detect AprilTags in images and publish their pose, id and additional metadata.

For more information on AprilTag, the paper and the reference implementation: https://april.eecs.umich.edu/software/apriltag.html

## Topics

### Subscriptions:
The node subscribes via a `image_transport::CameraSubscriber` to rectified images on topic `image_rect`. The set of topic names depends on the type of image transport (parameter `image_transport`) selected (`raw` or `compressed`):
- `image_rect` (`raw`, type: `sensor_msgs/msg/Image`)
- `image_rect/compressed` (`compressed`, type: `sensor_msgs/msg/CompressedImage`)
- `camera_info` (type: `sensor_msgs/msg/CameraInfo`)

### Publisher:
- `/tf` (type: `tf2_msgs/msg/TFMessage`)
- `detections` (type: `apriltag_msgs/msg/AprilTagDetectionArray`)

The camera intrinsics `P` in `CameraInfo` are used to compute the marker tag pose `T` from the homography `H`. The image and the camera intrinsics need to have the same timestamp.

The tag poses are published on the standard TF topic `/tf` with the header set to the image header and `child_frame_id` set to either `tag<family>:<id>` (e.g. "tag36h11:0") or the frame name selected via configuration file. Additional information about detected tags is published as `AprilTagDetectionArray` message, which contains the original homography  matrix, the `hamming` distance and the `decision_margin` of the detection.

## Configuration

The node is configured via a yaml configurations file. For the complete ROS yaml parameter file syntax, see: https://github.com/ros2/rcl/tree/master/rcl_yaml_param_parser.

The configuration file has the format:
```yaml
apriltag:                 # node name
  ros__parameters:
    # setup (defaults)
    image_transport: raw  # image format: "raw" or "compressed"
    family: 36h11         # tag family name: 16h5, 25h9, 36h11
    size: 1.0             # default tag edge size in meter
    profile: false        # print profiling information to stdout

    # tuning of detection (defaults)
    max_hamming: 0        # maximum allowed hamming distance (corrected bits)
    detector:
      threads: 1          # number of threads
      decimate: 2.0       # decimate resolution for quad detection
      blur: 0.0           # sigma of Gaussian blur for quad detection
      refine: 1           # snap to strong gradients
      sharpening: 0.25    # sharpening of decoded images
      debug: 0            # write additional debugging images to current working directory

    pose_estimation_method: "pnp" # method for estimating the tag pose

    # (optional) list of tags
    # If defined, 'frames' and 'sizes' must have the same length as 'ids'.
    tag:
      ids:    [<id1>, <id2>, ...]         # tag IDs for which to publish transform
      frames: [<frame1>, <frame2>, ...]   # frame names
      sizes:  [<size1>, <size1>, ...]     # tag-specific edge size, overrides the default 'size'
```

The `family` (string) defines the tag family for the detector and must be one of `16h5`, `25h9`, `36h11`, `Circle21h7`, `Circle49h12`, `Custom48h12`, `Standard41h12`, `Standard52h13`. `size` (float) is the tag edge size in meters, assuming square markers.

Instead of publishing all tag poses, the list `tag.ids` can be used to only publish selected tag IDs. Each tag can have an associated child frame name in `tag.frames` and a tag specific size in `tag.sizes`. These lists must either have the same length as `tag.ids` or may be empty. In this case, a default frame name of the form `tag<family>:<id>` and the default tag edge size `size` will be used.

The remaining parameters are set to the their default values from the library. See `apriltag.h` for a more detailed description of their function.

See [tags_36h11.yaml](cfg/tags_36h11.yaml) for an example configuration that publishes specific tag poses of the 36h11 family.

## Nodes

### Standalone Executable

The `apriltag_node` executable can be launched with topic remappings and a configuration file:
```sh
ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/camera/image \
    -r camera_info:=/camera/camera_info \
    --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml
```

### Composable Node

For more efficient intraprocess communication, a composable node is provided:
```sh
$ ros2 component types
apriltag_ros
  AprilTagNode
```

This `AprilTagNode` component can be loaded with other nodes into a "container node" process where they used shared-memory communication to prevent unnecessary data copies. The example launch file [camera_36h11.launch.yml](launch/camera_36h11.launch.yml) loads the `AprilTagNode` component together with the `camera::CameraNode` component from the [`camera_ros` package](https://index.ros.org/p/camera_ros/) (`sudo apt install ros-$ROS_DISTRO-camera-ros`) into one container and enables `use_intra_process_comms` for both:
```sh
ros2 launch apriltag_ros camera_36h11.launch.yml
```

- [ ] [Create](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#create-a-file) or [upload](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#upload-a-file) files
- [ ] [Add files using the command line](https://docs.gitlab.com/ee/gitlab-basics/add-file.html#add-a-file-using-the-command-line) or push an existing Git repository with the following command:

```
cd existing_repo
git remote add origin http://192.168.10.100/algorithm_team/ehr_apriltag.git
git branch -M main
git push -uf origin main
```

## Integrate with your tools

- [ ] [Set up project integrations](http://192.168.10.100/algorithm_team/ehr_apriltag/-/settings/integrations)

## Collaborate with your team

- [ ] [Invite team members and collaborators](https://docs.gitlab.com/ee/user/project/members/)
- [ ] [Create a new merge request](https://docs.gitlab.com/ee/user/project/merge_requests/creating_merge_requests.html)
- [ ] [Automatically close issues from merge requests](https://docs.gitlab.com/ee/user/project/issues/managing_issues.html#closing-issues-automatically)
- [ ] [Enable merge request approvals](https://docs.gitlab.com/ee/user/project/merge_requests/approvals/)
- [ ] [Set auto-merge](https://docs.gitlab.com/ee/user/project/merge_requests/merge_when_pipeline_succeeds.html)

## Test and Deploy

Use the built-in continuous integration in GitLab.

- [ ] [Get started with GitLab CI/CD](https://docs.gitlab.com/ee/ci/quick_start/index.html)
- [ ] [Analyze your code for known vulnerabilities with Static Application Security Testing (SAST)](https://docs.gitlab.com/ee/user/application_security/sast/)
- [ ] [Deploy to Kubernetes, Amazon EC2, or Amazon ECS using Auto Deploy](https://docs.gitlab.com/ee/topics/autodevops/requirements.html)
- [ ] [Use pull-based deployments for improved Kubernetes management](https://docs.gitlab.com/ee/user/clusters/agent/)
- [ ] [Set up protected environments](https://docs.gitlab.com/ee/ci/environments/protected_environments.html)

***

# Editing this README

When you're ready to make this README your own, just edit this file and use the handy template below (or feel free to structure it however you want - this is just a starting point!). Thanks to [makeareadme.com](https://www.makeareadme.com/) for this template.

## Suggestions for a good README

Every project is different, so consider which of these sections apply to yours. The sections used in the template are suggestions for most open source projects. Also keep in mind that while a README can be too long and detailed, too long is better than too short. If you think your README is too long, consider utilizing another form of documentation rather than cutting out information.

## Name
Choose a self-explaining name for your project.

## Description
Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors.

## Badges
On some READMEs, you may see small images that convey metadata, such as whether or not all the tests are passing for the project. You can use Shields to add some to your README. Many services also have instructions for adding a badge.

## Visuals
Depending on what you are making, it can be a good idea to include screenshots or even a video (you'll frequently see GIFs rather than actual videos). Tools like ttygif can help, but check out Asciinema for a more sophisticated method.

## Installation
Within a particular ecosystem, there may be a common way of installing things, such as using Yarn, NuGet, or Homebrew. However, consider the possibility that whoever is reading your README is a novice and would like more guidance. Listing specific steps helps remove ambiguity and gets people to using your project as quickly as possible. If it only runs in a specific context like a particular programming language version or operating system or has dependencies that have to be installed manually, also add a Requirements subsection.

## Usage
Use examples liberally, and show the expected output if you can. It's helpful to have inline the smallest example of usage that you can demonstrate, while providing links to more sophisticated examples if they are too long to reasonably include in the README.

## Support
Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc.

## Roadmap
If you have ideas for releases in the future, it is a good idea to list them in the README.

## Contributing
State if you are open to contributions and what your requirements are for accepting them.

For people who want to make changes to your project, it's helpful to have some documentation on how to get started. Perhaps there is a script that they should run or some environment variables that they need to set. Make these steps explicit. These instructions could also be useful to your future self.

You can also document commands to lint the code or run tests. These steps help to ensure high code quality and reduce the likelihood that the changes inadvertently break something. Having instructions for running tests is especially helpful if it requires external setup, such as starting a Selenium server for testing in a browser.

## Authors and acknowledgment
Show your appreciation to those who have contributed to the project.

## License
For open source projects, say how it is licensed.

## Project status
If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers.
