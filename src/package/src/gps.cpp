#include <global.h>

extern double ref_phi, ref_q, ref_x, ref_y, ref_z;
extern WGS84 ref_WGS;
ENU enu;
Euler enu_euler;

void getGPSData(const morai_msgs::GPSMessage::ConstPtr &data)
{
    //std::cout << "ego_wgs  " << data->latitude << " " << data->longitude << endl;

    enu = wgs84ToENU({data->latitude, data->longitude, data->altitude});

    std::cout << "enu  " << enu.East << " " << enu.North << endl;
}

void getImuData(const sensor_msgs::Imu::ConstPtr &data)
{
    enu_euler = quatToEuler({data->orientation.w, data->orientation.x, data->orientation.y, data->orientation.z});  
}

void saveResultToFile(const vector<ENU>& data, const std::string& filename) {
    ofstream file(filename);
    if (!file.is_open()) {
        cerr << "파일을 열 수 없습니다: " << filename << endl;
        return;
    }

    for (const auto& point : data) {
        file << point.East << " " << point.North << endl;
    }

    file.close();
    cout << "ENU 데이터가 " << filename << "에 저장되었습니다." << endl;    
}

int main(int argc, char** argv)
{
    initRef();
    
    ros::init(argc, argv, "GNSS");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

    ros::Subscriber gpsSub = node.subscribe<morai_msgs::GPSMessage>("/gps", 1, getGPSData);
    ros::Subscriber imuSub = node.subscribe<sensor_msgs::Imu>("/imu", 1, getImuData);

    ros::Publisher gnssPub = node.advertise<package::gnss>("/GNSS", 1);
    
    vector<ENU> path;

    while(ros::ok())
    {
        path.push_back(enu);

        package::gnss msg;
        msg.East = enu.East;
        msg.North = enu.North;
        msg.Yaw = radToDeg(enu_euler.Yaw);
        gnssPub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}