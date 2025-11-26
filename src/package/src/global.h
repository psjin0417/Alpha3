#ifndef GPS_H
#define GPS_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <climits>
#include <unistd.h>
#include <fstream>
#include <Eigen/Dense>
#include <sstream>
#include <vector>
#include <string>
#include <limits>
#include <chrono>
#include <thread>
#include <mutex>
#include <Eigen/Dense>
#include <algorithm>
#include <package/object.h>
#include <package/gnss.h>
#include <morai_msgs/GPSMessage.h>
#include <sensor_msgs/Imu.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/ObjectStatusList.h>
#include <morai_msgs/ObjectStatus.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <GeographicLib/UTMUPS.hpp>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include "morai_msgs/MoraiEventCmdSrv.h"
#include "morai_msgs/EventInfo.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Point.h>

using namespace std;

struct WGS84
{
    double latitude;
    double longitude;
    double altitude;
};

struct ENU
{
    double East;
    double North;
    double Up;
};

struct Euler
{
    double Roll;
    double Pitch;
    double Yaw;
};

struct Quaternion
{
    double w;
    double x;
    double y;
    double z;
};

double ref_phi, ref_q, ref_x, ref_y, ref_z;
WGS84 ref_WGS;
const double a = 6378137.0;
const double f = 1 / 298.257223563;
const double e2 = 2 * f - pow(f, 2);
// const double eastOffset = 302459.942;
// const double northOffset = 4122635.537;
const double eastOffset = 445130.363;
const double northOffset = 3945973.281;
const int zone = 52;
#define PI 3.14159265358979323846

string getCurrentDirectory() {
    char buffer[PATH_MAX];
    if (getcwd(buffer, sizeof(buffer)) != nullptr) {
        return std::string(buffer); 
    } else {
        perror("getcwd"); 
        return "";
    }
}

float Normalize(float x1, float y1, float x2, float y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

float Normalize(float *p1, float *p2)
{
    return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
}


double degToRad(double deg)
{
    return deg / 180 * M_PI;
}

double radToDeg(double rad)
{
    return rad / M_PI * 180;
}

Euler quatToEuler(Quaternion quat) {
    Euler enu_euler;
    enu_euler.Roll = atan2(2.0 * (quat.w * quat.x + quat.y * quat.z), 1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y));
    enu_euler.Pitch = asin(2.0 * (quat.w * quat.y - quat.z * quat.x));
    enu_euler.Yaw = atan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));

    return enu_euler;
}

ENU wgs84ToENU(WGS84 wgs84)
{
    double X, Y, Z, dx, dy, dz, latitude, longitude, h;
    double ref_latitude, ref_longitude, ref_h;
    double phi, lambda, N;

    latitude = wgs84.latitude;
    longitude = wgs84.longitude;
    h = wgs84.altitude;
    ref_latitude = degToRad(ref_WGS.latitude);
    ref_longitude = degToRad(ref_WGS.longitude);
    ref_h = ref_WGS.altitude;

    // rad로 변환
    phi = degToRad(latitude);
    lambda = degToRad(longitude);
    N = a / sqrt(1 - e2 * sin(phi) * sin(phi));

    // ECEF 좌표
    X = (N + h) * cos(phi) * cos(lambda);
    Y = (N + h) * cos(phi) * sin(lambda);
    Z = ((1 - e2) * N + h) * sin(phi);

    dx = X - ref_x;
    dy = Y - ref_y;
    dz = Z - ref_z;

    // ENU 변환
    ENU enu;
    enu.East = -sin(ref_longitude) * dx + cos(ref_longitude) * dy;
    enu.North = -sin(ref_latitude) * cos(ref_longitude) * dx - sin(ref_latitude) * sin(ref_longitude) * dy + cos(ref_latitude) * dz;
    enu.Up = cos(ref_latitude) * cos(ref_longitude) * dx + cos(ref_latitude) * sin(ref_longitude) * dy + sin(ref_latitude) * dz;

    return enu;
}

void initRef() {
    ifstream wgsRef(getCurrentDirectory() + "/src/package/src/ref.txt");
    wgsRef >> ref_WGS.latitude >> ref_WGS.longitude >> ref_WGS.altitude;
    ref_phi = sqrt(1 - e2 * pow(sin(degToRad(ref_WGS.latitude)), 2));        
    ref_q = (a / ref_phi + ref_WGS.altitude) * cos(degToRad(ref_WGS.latitude));
    ref_x = ref_q * cos(degToRad(ref_WGS.longitude));
    ref_y = ref_q * sin(degToRad(ref_WGS.longitude));
    ref_z = ((a * (1 - e2) / ref_phi) + ref_WGS.altitude) * sin(degToRad(ref_WGS.latitude));
}

vector<Eigen::Vector2d> load2DFile(const string& filename) {
    vector<Eigen::Vector2d> data;
    ifstream infile(filename);
    if (!infile.is_open()) {
        cerr << "파일 열기 실패: " << filename << endl;
        return data;
    }
    string line;
    while(getline(infile, line)) {
        if(line.empty()) continue;
        istringstream iss(line);
        double a, b;
        if (iss >> a >> b) {
            data.push_back(Eigen::Vector2d(a, b));
        }
    }
    infile.close();
    return data;
}


vector<double> load1DFile(const string& filename) {
    vector<double> data;
    ifstream infile(filename);
    if (!infile.is_open()) {
        cerr << "파일 열기 실패: " << filename << endl;
        return data;
    }
    string line;
    // 한 줄만 읽어옵니다.
    if(getline(infile, line)) {
        istringstream iss(line);
        double value;
        while (iss >> value) {
            data.push_back(value);
        }
    }
    infile.close();
    return data;
}

vector<Eigen::Vector2d> loadCSV(const string &filename)
{
    vector<Eigen::Vector2d> data;
    ifstream infile(filename);
    if (!infile.is_open())
    {
        cerr << "CSV 파일 열기 실패: " << filename << endl;
        return data;
    }
    string line;
    while (getline(infile, line))
    {
        if (line.empty())
            continue;
        istringstream ss(line);
        string token;
        vector<double> row;
        while (getline(ss, token, ','))
        {
            row.push_back(stod(token));
        }
        // 최소 2개의 값이 있을 경우만 Vector2d로 생성 (3번째 열은 무시)
        if (row.size() >= 2)
        {
            data.push_back(Eigen::Vector2d(row[0], row[1]));
        }
    }
    infile.close();
    return data;
}



vector<Eigen::Vector2d> load2DComaFile(const string& filename) {
    vector<Eigen::Vector2d> data;
    ifstream infile(filename);
    if (!infile.is_open()) {
        cerr << "파일 열기 실패: " << filename << endl;
        return data;
    }
    string line;
    while (getline(infile, line)) {
        if (line.empty()) continue;
        istringstream iss(line);
        string token;
        vector<double> values;
        
        while (getline(iss, token, ',')) {  // ',' 기준으로 데이터 나누기
            try {
                values.push_back(stod(token));  // 문자열을 double로 변환
            } catch (const invalid_argument& e) {
                cerr << "잘못된 데이터 형식: " << token << endl;
                continue;
            }
        }
        data.push_back(Eigen::Vector2d(values[0], values[1]));
    }
    infile.close();
    return data;
}




#endif