#ifndef BENDER_PERCEPTION_VISION_H
#define BENDER_PERCEPTION_VISION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>


namespace bender_perception
{

using namespace cv;
using namespace std;

class LaneDetection
{
    public:
        /*
         * Constructor for reading from USB Camera
         */
        LaneDetection(ros::NodeHandle *nh, int device_id=0);


        /*
         * Constructor for reading from rostopic `input_topic_`
         */
        LaneDetection(ros::NodeHandle *nh, string input_topic, string output_topic="/bender_perception/image_quantized");


        /*
         * Desctructor
         */
        ~LaneDetection();


        /*
         * Update image source from incoming msg
         */
        void readImage(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr& info_msg);


        /*
         * Update image source by reading USB camera input
         */
        void readImage();


        /*
         * Compute the two colors to quantize to
         */
        void quantize();


        /*
         * Update the output with by processing the latest available source
         */
        void update();


        /*
         * Perform perspective transform 
         */
        void computeHomography();


        /*
         * Perform perspective transform 
         */
        void projectToGrid();


        /*
         * Display output in GUI window
         */
        void displayOutput();


        /*
         * Publish quantized image via image_transport to preserve bandwidth
         */
        void publishQuantized();

        
        /*
         * Variable for scaling the image size, must be between 0 and 1
         */
        float scale;


        /*
         * The `k` in the k-means algorithm
         */
        int num_colors = 2;

    private:

        VideoCapture cam_capture_;
        const uint8_t device_id_;
        const string wname_ = "bender_perception_vision";

        image_transport::ImageTransport it_;

        image_transport::CameraSubscriber input_sub_;
        const string input_topic_;

        image_transport::Publisher output_pub_;
        const string output_topic_;
        sensor_msgs::ImagePtr output_msg_; 

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        image_geometry::PinholeCameraModel cam_model_;

        Mat img_src_;
        Mat img_out_;
        
        bool has_homography_ = false;
        Matx33d H_;     // Homography matrix computed from extrinsic and intrinsic parameters

        void init(ros::NodeHandle *nh);
}; 

} // namespace bender_perception

#endif // BENDER_PERCEPTION_VISION_H