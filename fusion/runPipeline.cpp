#include "fusion.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>
#include <cmath>
#include <ctime>      // std::tm, timegm
#include <cstdlib>    // for sscanf


// Parses either a raw numeric timestamp (e.g. "1317782378.123456")
// or an ISO string ("2011-09-26 13:02:25.951199337") into seconds since epoch.
static double parseTimestamp(const std::string &s) {
  if (s.find('-') != std::string::npos) {
    // ISO format: YYYY-MM-DD HH:MM:SS.NNNNNNNNN
    int Y, M, D, h, m;
    double sec;
    // sscanf extracts year, month, day, hour, min, seconds with fraction
    if (std::sscanf(s.c_str(), "%d-%d-%d %d:%d:%lf",
                    &Y, &M, &D, &h, &m, &sec) != 6) {
      return 0.0; // parse error fallback
    }
    std::tm tm = {};
    tm.tm_year = Y - 1900;
    tm.tm_mon  = M - 1;
    tm.tm_mday = D;
    tm.tm_hour = h;
    tm.tm_min  = m;
    tm.tm_sec  = static_cast<int>(std::floor(sec));
    // timegm interprets tm as UTC
    time_t t = timegm(&tm);
    double frac = sec - std::floor(sec);
    return static_cast<double>(t) + frac;
  }
  // Otherwise assume a plain floating-point string
  return std::stod(s); // stod = string to double
}



namespace fusion {


    ThreatObject createThreatObject(
        int id,
        const cv::Mat &frame,
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        double timestamp
    ){
        // implementation
        ThreatObject obj;
        obj.id = id;

        // 1. Compute 3D centroid of the cloud for the objects location
        //    aka condense a variable sized point cloud into a single x,y,z average location
        float sum_x = 0, sum_y = 0, sum_z = 0;

        for (const auto &pt : cloud->points){ // iterate over each point in the cloud
            sum_x += pt.x;
            sum_y += pt.y;
            sum_z += pt.z;
        }

        const float n = static_cast<float>(cloud->points.size()); // converts int (the cloud length) to float for float division later

        if (n > 0){ // calculate averages
            obj.x = sum_x / n;
            obj.y = sum_y / n;
            obj.z = sum_z / n;
        } else {
            obj.x = obj.y = obj.z = 0.0f;
        }

        // 2. Placeholder classification -- we will swap in the real model later
        obj.type = "unknown";
        obj.confidence = 0.0f;

        // 3. Placeholder velocity -- to be computed from timestamps and centroids later
        obj.velocity = 0.0f;

        return obj;

    }

    void runPipeline(){

        // ------ Stage 1: Camera Ingestion ----- 
        std::cout << "[fusion] Stage 1: Camera Ingestion\n";

        // 1. Define the path to a test image
        const std::string imgPath = "../data/camera/sequence_0001/0000000000.png";

        // 2. Load the image from disk into a cv:Mat
        // cv::imread reads the file and decodes into a matrix of pixels
        cv::Mat image = cv::imread(imgPath, cv::IMREAD_COLOR);

        // 3. Check if loading succeeded
        if (image.empty()) {
            std::cerr << "[fusion][ERROR] Failed to load image at " << imgPath << "\n";
            return; // abort this pipeline stage
        }

        // 4. Report the image's size (width x height)
        std::cout << "[fusion] Loaded image: "
                  << image.cols << "x" << image.rows << " pixels\n"; 

        
        // ----- Stage 2: LIDAR ----- 
        std::cout << "[fusion] Stage 2: LiDAR Ingestion\n";

        // a. Path to the first LiDAR scan (.bin)
        const std::string lidarPath = "../data/lidar/sequence_0001/0000000000.bin";

        // b. Open the file as binary
        std::ifstream input(lidarPath, std::ios::binary);
        if (!input) {
            std::cerr << "[fusion][ERROR] Failed to open the LiDAR file at " << lidarPath << "\n"; 
            return;
        }

        // c. Create a PCL point cloud to hold XYZ + intensity
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        // d. Read floats until EOF: .bin stores [x, y, z, intensity] per point
        // after these four calls, the four floats hold the next 16 bytes from the file
        while (true){
            float x, y, z, intensity;
            input.read(reinterpret_cast<char*>(&x), sizeof(float));
            input.read(reinterpret_cast<char*>(&y), sizeof(float));
            input.read(reinterpret_cast<char*>(&z), sizeof(float));
            input.read(reinterpret_cast<char*>(&intensity), sizeof(float));
        

        if (!input) break; // stop at EOF or read error

        pcl::PointXYZI pt; // this is a struct from the Point Cloud Library with members {float x, y, z; float intensity; }
        pt.x = x; pt.y = y; pt.z = z;
        pt.intensity = intensity; 
        cloud->points.push_back(pt); // cloud is a Point Cloud so cloud->points is a std::vector<PointXYZI>; push_back(pt) adds new point to end of in-memory point cloud

        } // end while

        // e. Set cloud metadata
        // PCL represents every point cloud as a 2D array of points; here we are making it a 1D list
        cloud->width = static_cast<uint32_t>(cloud->points.size()); // number of points
        cloud->height = 1; // unorganized cloud; now 1D
        cloud->is_dense = false; // says there may be some na points

        // f. report how many points were loaded
        std::cout << "[fusion] Loaded LiDAR scan: " << cloud->points.size() << " points\n"; 


        // ----- Stage 3: Timestamp Synchronization ------
        std::cout << "[fusion] Stage 3: Timestamp Synchronization\n";

        // a. Load camera timestamps
        const std::string camTsPath = "../data/camera/sequence_0001/timestamps.txt";
        std::ifstream camFile(camTsPath);
        if (!camFile) {
            std::cerr << "[fustion][ERROR] Cannot open camera timestamps at " << camTsPath << "\n"; 
            return;
        }

        std::vector<double> camTs; // vector of the camera timestamps
        { // these brackets limit 'line' scope to this block
            std::string line;
            while (std::getline(camFile, line)){
                camTs.push_back(parseTimestamp(line)); //parseTimestamp is method we wrote above
            }
        }

        // b. Load LiDAR timestamps
        const std::string lidarTsPath = "../data/lidar/sequence_0001/timestamps.txt"; 
        std::ifstream lidarFile(lidarTsPath);
        if (!lidarFile) {
            std::cerr << "[fusion][ERROR] Cannot open LiDAR timestamps at " << lidarTsPath << "\n"; 
            return;
        }

        std::vector<double> lidarTs; // vector of the lidar timestamps
        {
            std::string line;
            while (std::getline(lidarFile, line)) {
                lidarTs.push_back(parseTimestamp(line));
            }
        }

        // c. Simple nearest-neighbor sync: for each camera ts, find closest LiDAR ts
        size_t nSync = std::min(camTs.size(), lidarTs.size()); // number of syncs to do; minimum of two timestamp list lengths

        for (size_t i = 0; i < std::min(nSync, size_t(5)); ++i) { // loop over each camera timestamp

            double bestDiff = 1e9;
            size_t bestIdx = 0;

            for (size_t j = 0; j< lidarTs.size(); ++j) { // loop over every lidar timestamp

                double diff = std::abs(camTs[i] - lidarTs[j]); 

                if (diff < bestDiff) { // if this lidar time is closer to the curr camera time than any other seen so far
                    bestDiff = diff;
                    bestIdx = j;
                }
            }

            std::cout << "[fusion] Sync came frame " << i
                      << " (ts=" << camTs[i] << ") -> LiDAR scan " << bestIdx
                      << " (ts=" << lidarTs[bestIdx]
                      << "), Δ=" << bestDiff << "s\n";

        }


        // ----- Stage 4: Build ThreatObject for the first synced pair -----
        std::cout << "[fusion] Stage 4: Creating ThreatObject\n";

        // Used index 0 and the synced data from Stage 1 and 2 to create a ThreatObject
        auto threat = createThreatObject(
            0,          // unique ID
            image,      // the cv::Mat from camera ingestion
            cloud,      // the pcl::PointCloud from LiDAR ingestion
            camTs[0]    // timestamp for this frame
        );

        // Print out its fields
        std::cout << "[fusion] ThreatObject {\n"
              << "  id: "         << threat.id         << "\n"
              << "  location: ("  << threat.x << ", "
                                    << threat.y << ", "
                                    << threat.z << ")\n"
              << "  type: "       << threat.type       << "\n"
              << "  confidence: " << threat.confidence << "\n"
              << "  velocity: "   << threat.velocity   << "\n"
              << "}\n";    



    }
}