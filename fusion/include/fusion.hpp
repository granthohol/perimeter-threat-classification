#pragma once

#include <string>
#include <opencv2/core.hpp>           
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fusion{

    // 1. Define a struct to hold all fused threat information
    struct ThreatObject {
        int id;
        float x, y, z;      // 3D location in sensor coordinates
        std::string type;   // e.g. "vehicle", "pedestrian"
        float confidence;   // classification models confidence [0, 1]
        float velocity;     // velocity of object
    }; 

    // 2. Declare our new fusion helper:
    //    Given an index, a frame, a cloud, and a timestamp
    //    produce a ThreatObject
    ThreatObject createThreatObject(
        int id,
        const cv::Mat &frame,
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        double timestamp
    );

    // called by main to run the pipeline
    void runPipeline();
    
}