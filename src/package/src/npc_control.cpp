#include <global.h>

using namespace std;
using namespace Eigen;


#define PI 3.14159265358979323846

ros::Publisher ctrlPub;

float NPC_POS[2] = {0};
float Yaw = 0;
float TargetSteer = 0;
int NPC_clost_idx = 0;
int NPC_Target_idx = 0;
float NPC_VEL = 0;
const float WheelBase = 2.5;
const float Lookahead_Distance = 5.0;
float accelCmd = 0, brakeCmd = 0;
int NpcSize = 0;

// string filePath = "/home/autonav/alpha3/src/beginner_tutorials_answer/path";
// vector<Vector2d> ref_path = load2DComaFile(filePath + "control_npc.txt"); 


vector<Vector2d> ref_path = "/home/autonav/alpha3/src/beginner_tutorials_answer/path";

void FindTarget()
{
    double tmpDist = numeric_limits<double>::max();
    int tmpIdx = 0;
    for (size_t i = 0; i < ref_path.size(); ++i)
    {
        double dx = ref_path[i](0) - NPC_POS[0];
        double dy = ref_path[i](1) - NPC_POS[1];
        double dist = sqrt(dx*dx + dy*dy);

        if (dist < tmpDist) {
            tmpDist = dist;
            tmpIdx = static_cast<int>(i);
        }
    }
    NPC_clost_idx = tmpIdx;

    for ( int j = tmpIdx; j < ref_path.size(); j++)
    {
        double dx = ref_path[j](0) - NPC_POS[0];
        double dy = ref_path[j](1) - NPC_POS[1];
        double dist = sqrt(dx * dx + dy * dy);

        if (dist >= Lookahead_Distance)
        {
            NPC_Target_idx = j;
            break;
        }
    }
}

void GetSteer()
{
    float alpha = atan2(ref_path[NPC_Target_idx](1) - NPC_POS[1], ref_path[NPC_Target_idx](0) - NPC_POS[0]);
    alpha -= Yaw / 180 * PI;
    TargetSteer = atan2(2.0 * WheelBase * sin(alpha), Lookahead_Distance);
}

void PubCtrl()
{
    morai_msgs::CtrlCmd msg;
    msg.longlCmdType = 2;
    msg.steering = TargetSteer;
    msg.velocity = 60;

    ctrlPub.publish(msg);
}

void PurePursuit()
{
	FindTarget();
	GetSteer();
	PubCtrl();
}

void getEgoData(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
{
    double OBJ_LAT, OBJ_LON;
    GeographicLib::UTMUPS::Reverse(zone, true, msg->position.x + eastOffset, msg->position.y + northOffset, OBJ_LAT, OBJ_LON);

    ENU OBJ_ENU = wgs84ToENU({OBJ_LAT, OBJ_LON, 0});
    NPC_POS[0] = OBJ_ENU.East;
    NPC_POS[1] = OBJ_ENU.North;
    Yaw = msg->heading;
    NPC_VEL = sqrt(pow(msg->velocity.x,2) + pow(msg->velocity.y,2) + pow(msg->velocity.z,2)) * 3.6;

    cout << "NPC_POS : " << NPC_POS[0] << " " << NPC_POS[1] << endl; 
    cout << "Yaw : " << Yaw << endl;
    cout << "NPC_VEL : " << NPC_VEL << endl;
    PurePursuit();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Controller2");
    ros::NodeHandle node;

    ros::Rate rate(10);

    initRef();
    ros::Publisher objectPub = node.advertise<package::object>("/custom_npc", 1);
    ros::Subscriber objectSub = node.subscribe<morai_msgs::EgoVehicleStatus>("/ego_2/Ego_topic", 1, getEgoData);
    ctrlPub = node.advertise<morai_msgs::CtrlCmd>("/ego_2/ctrl_cmd_0", 1);

    while(ros::ok())   
    {
        package::object msg;
        msg.East = NPC_POS[0];
        msg.North = NPC_POS[1];
        msg.Yaw = Yaw;
        msg.Vel = NPC_VEL;
        objectPub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}