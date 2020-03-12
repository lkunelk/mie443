#include <imagePipeline.h>
#include <descriptor.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
// #define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"


ImagePipeline::ImagePipeline(ros::NodeHandle& n, int blank_threshold, double distance_threshold_coeff) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    std::cout<<"Initialized"<< std::endl;
    isValid = false;
    blank_thresh = blank_threshold;
    distance_thresh_coeff = distance_threshold_coeff;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    Descriptor descriptor; 

    int template_id = -1;
    int highest_match = -1;
    float curr_match = -1;

    if(!isValid) {
        //std::cout << "ERROR: INVALID IMAGE!" << std::endl;
        return -9;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
        return -9;
    } else {
        std::cout << "VALID IMAGE" << std::endl;
        for(int i = 0; i < boxes.templates.size(); i++) {
            std::cout << "Template Number: " << i << std::endl;

            // Get level of match to a certain template
            curr_match = descriptor.compareImages(img, boxes.templates[i], distance_thresh_coeff);
            std::cout << boxes.labels[i] + " -> " + std::to_string(curr_match) + " area" << std::endl;
            if (curr_match > highest_match){
                highest_match = curr_match;
                template_id = i;
            }
        }

        if (highest_match < blank_thresh){
            template_id = 3; // Corresponding to blank
        }

        // cv::imshow("view", img);
        // cv::waitKey(30);
    }  
    return template_id;
}

