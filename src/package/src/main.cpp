#include <global.h>

using namespace std;
using namespace Eigen;


#define PI 3.14159265358979323846

ros::Publisher ctrlPub;

float CUR_POS[2] = {0};
float Yaw = 0;
float TargetSteer = 0;
int Clost_idx = 0;
int Target_idx = 0;
float CUR_VEL = 0;
const float WheelBase = 2.5;
const float Lookahead_Distance = 5.0;
float accelCmd = 0, brakeCmd = 0;
float Target_Vel = 20;

static double Goal_Point[2] = {999999,999999};

string filePath = "/home/autonav/alpha3/src/beginner_tutorials_answer/path/";
vector<Vector2d> ref_path = load2DFile(filePath + "global.txt"); 

static bool Start_Flag = false;
static bool Stop_Flag = false;

void FindTarget()
{
    double tmpDist = numeric_limits<double>::max();
    int tmpIdx = 0;
    for (size_t i = 0; i < ref_path.size(); ++i)
    {
        double dx = ref_path[i](0) - CUR_POS[0];
        double dy = ref_path[i](1) - CUR_POS[1];
        double dist = sqrt(dx*dx + dy*dy);

        if (dist < tmpDist) {
            tmpDist = dist;
            tmpIdx = static_cast<int>(i);
        }
    }
    Clost_idx = tmpIdx;

    cout << "clost" << Clost_idx << endl;

    for ( int j = tmpIdx; j < ref_path.size(); j++)
    {
        double dx = ref_path[j](0) - CUR_POS[0];
        double dy = ref_path[j](1) - CUR_POS[1];
        double dist = sqrt(dx * dx + dy * dy);

        if (dist >= Lookahead_Distance)
        {
            Target_idx = j;
            cout << "target" << Target_idx << endl;
            break;
        }
    }
}

void GetSteer()
{
    float alpha = atan2(ref_path[Target_idx](1) - CUR_POS[1], ref_path[Target_idx](0) - CUR_POS[0]);
    alpha -= Yaw / 180 * PI;
    TargetSteer = atan2(2.0 * WheelBase * sin(alpha), Lookahead_Distance);
}

void PID_Controll()
{
    
}

void PubCtrl()
{
    morai_msgs::CtrlCmd msg;
    msg.longlCmdType = 2;
    msg.steering = TargetSteer;
    msg.velocity = 20;

    ctrlPub.publish(msg);
}

void Stop()
{
    morai_msgs::CtrlCmd msg;
    msg.longlCmdType = 1;
    msg.steering = TargetSteer;

    msg.accel = 0;
    msg.brake = 1;

    ctrlPub.publish(msg);
}

void Arrival()
{
    double dx = Goal_Point[0] - CUR_POS[0];
    double dy = Goal_Point[1] - CUR_POS[1];
    double dist = sqrt(dx*dx + dy*dy);

    if (dist < 10.0)
    {
        Stop_Flag = true;
        cout << "Arrive" << endl;
    }
}


void PurePursuit()
{
	FindTarget();
	GetSteer();
    PID_Controll();

    Arrival();

    if (Stop_Flag)
    {
        Stop();
        return;
    }

	PubCtrl();
}

void getEgoData(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
{
    CUR_POS[0] = msg->position.x;
    CUR_POS[1] = msg->position.y;
    Yaw = msg->heading;

    CUR_VEL = sqrt(pow(msg->velocity.x,2) + pow(msg->velocity.y,2) + pow(msg->velocity.z,2)) * 3.6;
    
    if (!Start_Flag)
    {
        return;
    }

    PurePursuit();
}

void get_Start_Flag(const std_msgs::Int32::ConstPtr& msg)
{
    int num = msg->data;

    if (num != 1)
    {
        return;
    }

    Start_Flag = true;
}

void get_GoalPoint(const geometry_msgs::Point::ConstPtr& msg)
{
    Goal_Point[0] = msg->x;
    Goal_Point[1] = msg->y;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Controller");
    ros::NodeHandle node;

    ros::Rate rate(10);

    initRef();
    // ros::Publisher objectPub = node.advertise<package::object>("/custom_npc", 1);
    ros::Subscriber FlagSub = node.subscribe<std_msgs::Int32>("/Start_topic", 1, get_Start_Flag);
    ros::Subscriber Goal_Point_Sub = node.subscribe<geometry_msgs::Point>("/Goal_topic", 1, get_GoalPoint);
    ros::Subscriber egoSub = node.subscribe<morai_msgs::EgoVehicleStatus>("/Ego_topic", 1, getEgoData);
    ctrlPub = node.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 1);
    

    while(ros::ok())   
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

// 목적지 근처 도착하면 멈추기