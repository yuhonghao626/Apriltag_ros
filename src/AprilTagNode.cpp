//250710 by yuhonghao 
// ros
#include "pose_estimation.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>

// apriltag
#include "tag_functions.hpp"
#include <apriltag.h>

// new add tag_detections_image by.yuhonghao
#include <opencv2/imgproc.hpp>

// new add tag_odometry by.yuhonghao
#include <nav_msgs/msg/odometry.hpp>

#define IF(N, V) \
    if(assign_check(parameter, N, V)) continue;

template<typename T>
void assign(const rclcpp::Parameter& parameter, T& var) {
    var = parameter.get_value<T>();
}

template<typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var) {
    var = parameter.get_value<T>();
}

template<typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var) {
    if(parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false) {
    rcl_interfaces::msg::ParameterDescriptor descr;
    descr.description = description;
    descr.read_only = read_only;
    return descr;
}

geometry_msgs::msg::Pose transformToPose(const geometry_msgs::msg::Transform& t) {
    geometry_msgs::msg::Pose p;
    p.position.x = t.translation.x;
    p.position.y = t.translation.y;
    p.position.z = t.translation.z;
    p.orientation = t.rotation;
    return p;
}

class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions& options);
    ~AprilTagNode() override;

private:
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;
    apriltag_family_t* tf;
    apriltag_detector_t* const td;

    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;

    std::function<void(apriltag_family_t*)> tf_destructor;

    const image_transport::CameraSubscriber sub_cam;
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    const image_transport::Publisher pub_debug_image;
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    pose_estimation_f estimate_pose = nullptr;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);
};

RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)

AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
    sub_cam(image_transport::create_camera_subscription(
        this,
        this->get_node_topics_interface()->resolve_topic_name("image_rect"),
        std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
        declare_parameter("image_transport", "raw", descr({}, true)),
        rmw_qos_profile_sensor_data)),
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    pub_debug_image(image_transport::create_publisher(this, "tag_detections_image")),
    pub_odometry(create_publisher<nav_msgs::msg::Odometry>("tag_Odometry", rclcpp::QoS(10))),
    tf_broadcaster(this)
{
    const std::string tag_family = declare_parameter("family", "36h11", descr("tag family", true));
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames = declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));
    const std::string& pose_estimation_method =
        declare_parameter("pose_estimation_method", "pnp", descr("pose estimation method", true));

    if(!pose_estimation_method.empty()) {
        if(pose_estimation_methods.count(pose_estimation_method)) {
            estimate_pose = pose_estimation_methods.at(pose_estimation_method);
        } else {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown pose estimation method '" << pose_estimation_method << "'.");
        }
    }

    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur"));
    declare_parameter("detector.refine", td->refine_edges, descr("refine edges"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening"));
    declare_parameter("detector.debug", td->debug, descr("debug output"));
    declare_parameter("max_hamming", 0, descr("max corrected bits allowed"));
    declare_parameter("profile", false, descr("profiling info"));

    if(!frames.empty()) {
        if(ids.size() != frames.size()) {
            throw std::runtime_error("Mismatch between tag ids and frames");
        }
        for(size_t i = 0; i < ids.size(); i++) tag_frames[ids[i]] = frames[i];
    }

    if(!sizes.empty()) {
        if(ids.size() != sizes.size()) {
            throw std::runtime_error("Mismatch between tag ids and sizes");
        }
        for(size_t i = 0; i < ids.size(); i++) tag_sizes[ids[i]] = sizes[i];
    }

    if(tag_fun.count(tag_family)) {
        tf = tag_fun.at(tag_family).first();
        tf_destructor = tag_fun.at(tag_family).second;
        apriltag_detector_add_family(td, tf);
    } else {
        throw std::runtime_error("Unsupported tag family: " + tag_family);
    }
}

AprilTagNode::~AprilTagNode() {
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci) {
    const std::array<double, 4> intrinsics = {msg_ci->p[0], msg_ci->p[5], msg_ci->p[2], msg_ci->p[6]};
    const bool calibrated = msg_ci->width && msg_ci->height && intrinsics[0] && intrinsics[1] && intrinsics[2] && intrinsics[3];

    if(estimate_pose != nullptr && !calibrated) {
        RCLCPP_WARN(get_logger(), "Camera not calibrated! Pose estimation may fail.");
    }

    const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg_img, "bgr8");
    cv::Mat debug_image = cv_ptr->image.clone();
    cv::Mat gray;
    cv::cvtColor(debug_image, gray, cv::COLOR_BGR2GRAY);
    image_u8_t im{gray.cols, gray.rows, gray.cols, gray.data};

    mutex.lock();
    zarray_t* detections = apriltag_detector_detect(td, &im);
    mutex.unlock();

    if(profile)
        timeprofile_display(td->tp);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;
    std::vector<geometry_msgs::msg::TransformStamped> tfs;

    for(int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        if(!tag_frames.empty() && !tag_frames.count(det->id)) continue;
        if(det->hamming > max_hamming) continue;

        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
        msg_detection.decision_margin = det->decision_margin;
        msg_detection.centre.x = det->c[0];
        msg_detection.centre.y = det->c[1];
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8);
        std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9);
        msg_detections.detections.push_back(msg_detection);
        
        // Draw the tag detection corners
        cv::Point2d pts[4];
        for(int j = 0; j < 4; ++j) pts[j] = cv::Point2d(det->p[j][0], det->p[j][1]);
        for(int j = 0; j < 4; ++j)
            cv::line(debug_image, pts[j], pts[(j+1)%4], cv::Scalar(0,255,0), 2);
        cv::putText(debug_image, std::to_string(det->id), pts[0], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);

        if(estimate_pose != nullptr && calibrated) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = msg_img->header;
            tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : msg_detection.family + ":" + std::to_string(det->id);
            double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
            tf.transform = estimate_pose(det, intrinsics, size);
            tfs.push_back(tf);

            // Calculate the distance from camera to the tag
            double distance = sqrt(
                pow(tf.transform.translation.x, 2) +
                pow(tf.transform.translation.y, 2) +
                pow(tf.transform.translation.z, 2)
            );

            RCLCPP_INFO(get_logger(), "Distance to tag %d: %f meters", det->id, distance);

            // Publish the odometry message
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header = tf.header;
            odom_msg.child_frame_id = tf.child_frame_id;
            odom_msg.pose.pose = transformToPose(tf.transform);
            pub_odometry->publish(odom_msg);
        }
    }

    pub_detections->publish(msg_detections);
    if(pub_debug_image.getNumSubscribers() > 0)
        pub_debug_image.publish(cv_bridge::CvImage(msg_img->header, "bgr8", debug_image).toImageMsg());
    if(estimate_pose != nullptr)
        tf_broadcaster.sendTransform(tfs);

    apriltag_detections_destroy(detections);
}


rcl_interfaces::msg::SetParametersResult
AprilTagNode::onParameter(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    mutex.lock();
    for(const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);
        IF("detector.threads", td->nthreads)
        IF("detector.decimate", td->quad_decimate)
        IF("detector.blur", td->quad_sigma)
        IF("detector.refine", td->refine_edges)
        IF("detector.sharpening", td->decode_sharpening)
        IF("detector.debug", td->debug)
        IF("max_hamming", max_hamming)
        IF("profile", profile)
    }
    mutex.unlock();
    result.successful = true;
    return result;
}