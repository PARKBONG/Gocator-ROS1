// read_depth.cpp
// Minimal example: Turn on Gocator sensor, print depth data for 5 seconds, then stop sensor
// Reference: extracted from gocator_profile.cpp (sensor connection and control only)

#include <ros/ros.h>
#include <GoSdk/GoSdk.h>
#include <chrono>
#include <thread>
#include <atomic>
// #include <gocator_bridge/Visualizer.h> // For macros and constants (NM_TO_MM, etc)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)
#define INVALID_RANGE_16BIT		((signed short)0x8000)
#define SENSOR_IP "192.168.1.10" // Change to your actual sensor IP

std::atomic<bool> running(true);

// Global publisher for ROS PointCloud2
ros::Publisher g_pcd_pub;

// Data callback: called automatically when sensor sends data
kStatus kCall onData(void* ctx, void* sys, void* dataset)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (unsigned int i = 0; i < GoDataSet_Count(dataset); ++i)
    {
        GoDataMsg dataObj = GoDataSet_At(dataset, i);
        switch (GoDataMsg_Type(dataObj))
        {
        
        case GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE:
        // Deprecated, use GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE_MSG
        // see file:///home/sms/bonggyeong_code/GO_SDK/doc/GoSdk/Gocator/html/struct_go_data_message_type.html
            {
                GoResampledProfileMsg profileMsg = dataObj;
                for (unsigned int k = 0; k < GoResampledProfileMsg_Count(profileMsg); ++k)
                {
                    short* data = GoResampledProfileMsg_At(profileMsg, k);
                    double XResolution = NM_TO_MM(GoResampledProfileMsg_XResolution(profileMsg));
                    double ZResolution = NM_TO_MM(GoResampledProfileMsg_ZResolution(profileMsg));
                    double XOffset = UM_TO_MM(GoResampledProfileMsg_XOffset(profileMsg));
                    double ZOffset = UM_TO_MM(GoResampledProfileMsg_ZOffset(profileMsg));
                    unsigned int width = GoResampledProfileMsg_Width(profileMsg);
                    for (unsigned int idx = 0; idx < width; ++idx)
                    {
                        if (data[idx] != INVALID_RANGE_16BIT)
                        {
                            double x = XOffset + XResolution * idx;
                            double z = ZOffset + ZResolution * data[idx];
                            double z_new = z -130; // Gocator 2330D sets zero point 130mm below the sensor
                            cloud->points.emplace_back(-x / 1000.0, 0.0, z_new / 1000.0); // x axis is inverted to align with standard coordinate frame
                        }
                    }
                }
            }
            break;
        default:
            break;
        }
    }
    if (!cloud->empty()) {
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "gocator_frame";
        g_pcd_pub.publish(msg);
    }
    GoDestroy(dataset);
    return kOK;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gocator_read_depth");
    ros::NodeHandle nh;

    // Add publisher
    g_pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("gocator_profile_pcd", 1);

    kAssembly api = kNULL;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    kStatus status;
    kIpAddress ipAddress;

    // 1. Initialize Gocator SDK
    if ((status = GoSdk_Construct(&api)) != kOK)
    {
        ROS_ERROR("Error: GoSdk_Construct: %d", status);
        return 1;
    }

    // 2. Create GoSystem object
    if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
    {
        ROS_ERROR("Error: GoSystem_Construct: %d", status);
        GoDestroy(api);
        return 1;
    }

    // 3. Parse sensor IP and get sensor object
    kIpAddress_Parse(&ipAddress, SENSOR_IP);
    if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
    {
        ROS_ERROR("Error: GoSystem_FindSensor: %d", status);
        GoDestroy(system);
        GoDestroy(api);
        return 1;
    }

    // 4. Connect to sensor
    if ((status = GoSystem_Connect(system)) != kOK)
    {
        ROS_ERROR("Error: GoSystem_Connect: %d", status);
        GoDestroy(system);
        GoDestroy(api);
        return 1;
    }

    // 5. Enable data channel (must be called for data callback to work)
    if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
    {
        ROS_ERROR("Error: GoSystem_EnableData: %d", status);
        GoSystem_Disconnect(system);
        GoDestroy(system);
        GoDestroy(api);
        return 1;
    }

    // 6. Register data handler (callback)
    if ((status = GoSystem_SetDataHandler(system, onData, NULL)) != kOK)
    {
        ROS_ERROR("Error: GoSystem_SetDataHandler: %d", status);
        GoSystem_Disconnect(system);
        GoDestroy(system);
        GoDestroy(api);
        return 1;
    }

    ROS_INFO("Sensor connected successfully.");

    // 7. Start sensor (data will now be sent and callback will be triggered)
    if ((status = GoSystem_Start(system)) != kOK)
    {
        ROS_ERROR("Error: GoSystem_Start: %d", status);
        GoSystem_Disconnect(system);
        GoDestroy(system);
        GoDestroy(api);
        return 1;
    }

    // Main loop: keep running as long as ROS is running
    while (ros::ok()) {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 8. Stop sensor
    if ((status = GoSystem_Stop(system)) != kOK)
    {
        ROS_ERROR("Error: GoSystem_Stop: %d", status);
    }
    else
    {
        ROS_INFO("Sensor stopped successfully.");
    }

    // 9. Disconnect and cleanup
    GoSystem_Disconnect(system);
    GoDestroy(system);
    GoDestroy(api);

    return 0;
}
