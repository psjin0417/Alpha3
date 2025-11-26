#include <global.h>

#include <morai_msgs/MoraiEventCmdSrv.h>
#include <morai_msgs/EventInfo.h>

using namespace std;
using namespace Eigen;

#define PI 3.14159265358979323846

ros::Publisher ctrlPub, ego_pos_pub, static_pub, real_pos_pub, pred_pos_pub, pred_path_pub;
ros::ServiceClient obj_client, ego_client;
// ======================== 제어기 관련 변수 ======================== //
float CUR_POS[2] = {0}, OBJ_POS[2] = {0};
float CUR_POS_FRENET[2] = {0};
float OBJ_POS_FRENET[2] = {0};
float Yaw = 0;
float OBJ_Yaw = 0;
int NpcSize = 0;
int ego_lane = 0;
float TargetSteer = 0;

float CUR_VEL = 0;
float OBJ_VEL = 0;
const float WheelBase = 2.5;
const float Lookahead_Distance = 10.0;

float TargetSpeed = 60;
float accelCmd = 0, brakeCmd = 0;

float kp = 0.1;
float L = 4.63;
float W = 1.892;

double mean_PathGap_1 = -4.5;
double mean_PathGap_2 = 4.5;
double vehicle_offset = 2;
// ======================== 제어기 관련 변수 ======================== //

// ======================== 맵 데이터 추가 ======================== //
string filePath = "/home/autonav/new_spring_its/spring_its/kiapi_matlab/";
vector<Vector2d> ref_path = load2DFile(filePath + "Line2.txt");
vector<double> s_ref;
vector<Vector2d> PathGap = load2DFile(filePath + "PathGap.txt");
vector<Vector2d> Boundary1 = load2DFile(filePath + "Boundary1.txt");
vector<Vector2d> Boundary4 = load2DFile(filePath + "Boundary4.txt");
vector<Vector2d> Line1 = load2DFile(filePath + "Line1.txt");
vector<Vector2d> Line2 = load2DFile(filePath + "Line2.txt");
vector<Vector2d> Line3 = load2DFile(filePath + "Line3.txt");
vector<Vector2d> predPath;
vector<Vector2d> frenet_predPath;

vector<double> d2_1(PathGap.size());
vector<double> d2_1_left(PathGap.size());
vector<double> d2_1_right(PathGap.size());
// ======================== 맵 데이터 추가 ======================== //

// ======================== 칼만 데이터 추가 ======================== //
Eigen::MatrixXd H(4, 4);
Eigen::MatrixXd Q(4, 4);
Eigen::MatrixXd R(4, 4);
Eigen::MatrixXd P_t(4, 4);
Eigen::MatrixXd X_t(4, 1);
Eigen::MatrixXd X_p(4, 1);
Eigen::MatrixXd P_p(4, 4);
// ======================== 칼만 데이터 추가 ======================== //

Vector2d tanv;
Vector2d before_tanv;
Vector2d norv;

vector<Vector2d> ref_tanv, ref_norv;

double theta;
int EGO_clost_idx = 0;
int OBJ_clost_idx = 0;
int EGO_Target_idx = 0;
float dt = 0.1;
float target_lane_d = 0;

bool init = false;
bool obj_will_in_cur_line = false;
bool obj_in_cur_line = false;
bool isLaneChange = false;
bool isStart = true;

vector<double> get_S(const vector<Vector2d> &line)
{
    vector<double> s(line.size(), 0.0);
    for (size_t i = 1; i < line.size(); ++i)
    {
        double dx = line[i](0) - line[i - 1](0);
        double dy = line[i](1) - line[i - 1](1);
        s[i] = s[i - 1] + sqrt(dx * dx + dy * dy);
    }
    return s;
}

void get_OBJ_clost_idx()
{
    double tmpDist = numeric_limits<double>::max();
    int tmpIdx = 0;
    Eigen::Vector2d obj(OBJ_POS[0], OBJ_POS[1]);
    for (size_t i = 0; i < ref_path.size(); ++i)
    {
        double dist = (obj - ref_path[i]).norm();
        if (dist < tmpDist)
        {
            tmpDist = dist;
            tmpIdx = static_cast<int>(i);
        }
    }
    OBJ_clost_idx = tmpIdx;
}

void get_OBJ_s()
{
    double s = 0;

    if (OBJ_clost_idx < 1)
    {
        OBJ_POS_FRENET[0] = 0.0;
        return;
    }

    for (int i = 1; i <= OBJ_clost_idx; ++i)
    {
        double dx = ref_path[i](0) - ref_path[i - 1](0);
        double dy = ref_path[i](1) - ref_path[i - 1](1);
        s = s + sqrt(dx * dx + dy * dy);
    }

    OBJ_POS_FRENET[0] = s;
}

void get_tanv_norv()
{
    if (OBJ_clost_idx < 1)
    {
        return;
    }

    before_tanv = tanv;

    double dx = ref_path[OBJ_clost_idx](0) - ref_path[OBJ_clost_idx - 1](0);
    double dy = ref_path[OBJ_clost_idx](1) - ref_path[OBJ_clost_idx - 1](1);
    double dist = sqrt(dx * dx + dy * dy);

    if (dist < 1e-6)
    {
        tanv = Vector2d(1, 0);
        norv = Vector2d(0, 1);
        return;
    }

    Vector2d tangent(dx / dist, dy / dist);
    Vector2d normal(-tangent(1), tangent(0));

    tanv = tangent;
    norv = normal;
}

void get_OBJ_d()
{
    if (OBJ_clost_idx < 1)
    {
        OBJ_POS_FRENET[1] = 0.0;
        return;
    }

    double dx = OBJ_POS[0] - ref_path[OBJ_clost_idx](0);
    double dy = OBJ_POS[1] - ref_path[OBJ_clost_idx](1);
    double dist = sqrt(dx * dx + dy * dy);

    if (dist < 1e-6)
    {
        OBJ_POS_FRENET[1] = 0.0;
        return;
    }

    double expression = -(dx * norv(0) + dy * norv(1));
    double sign_factor = (expression > 0) ? 1.0 : ((expression < 0) ? -1.0 : 0.0);
    double d = sign_factor * dist;

    OBJ_POS_FRENET[1] = d;
}

void get_frenet_data()
{
    get_OBJ_clost_idx();
    get_OBJ_s();
    get_tanv_norv();
    get_OBJ_d();
}

void get_Tan(const vector<Vector2d> &line, vector<Vector2d> &tanv, vector<Vector2d> &norv)
{
    size_t n = line.size();
    tanv.resize(n);
    norv.resize(n);
    for (size_t i = 0; i < n - 1; ++i)
    {
        double dx = line[i + 1](0) - line[i](0);
        double dy = line[i + 1](1) - line[i](1);
        double dist = sqrt(dx * dx + dy * dy);
        tanv[i] = Vector2d(dx / dist, dy / dist);
        norv[i] = Vector2d(-tanv[i](1), tanv[i](0));
    }
    tanv[n - 1] = tanv[n - 2];
    norv[n - 1] = norv[n - 2];
}

vector<Vector2d> frenet_to_enu(const vector<Vector2d> &line, const vector<Vector2d> &norv, const vector<double> &d)
{
    vector<Vector2d> reLine(line.size());

    for (size_t i = 0; i < line.size(); ++i)
    {
        reLine[i] = line[i] - norv[i] * d[i];
    }
    return reLine;
}

Vector2d frenet_to_enu_point(double s, double d, const vector<double> &s_ref,
                             const vector<Vector2d> &ref_path, const vector<Vector2d> &norv)
{
    double minDiff = numeric_limits<double>::max();
    int idx = 0;
    for (size_t i = 0; i < s_ref.size(); ++i)
    {
        double diff = fabs(s_ref[i] - s);
        if (diff < minDiff)
        {
            minDiff = diff;
            idx = static_cast<int>(i);
        }
    }
    return ref_path[idx] - norv[idx] * d;
}

Vector2d frenet_to_enu_point(double s, double d)
{
    double minDiff = numeric_limits<double>::max();
    int idx = 0;
    for (size_t i = 0; i < s_ref.size(); ++i)
    {
        double diff = fabs(s_ref[i] - s);
        if (diff < minDiff)
        {
            minDiff = diff;
            idx = static_cast<int>(i);
        }
    }
    return ref_path[idx] - ref_norv[idx] * d;
}

double get_frenet_d(const vector<Vector2d> &line, const vector<Vector2d> &norv)
{
    if (line.empty() || norv.empty())
    {
        cerr << "경로 또는 법선 벡터 데이터가 비어 있습니다!" << endl;
        return 0.0;
    }
    Vector2d obj_pos_vector(OBJ_POS[0], OBJ_POS[1]);
    // 가장 가까운 점 찾기
    double minDist = numeric_limits<double>::max();
    int closest_idx = 0;
    for (size_t i = 0; i < line.size(); ++i)
    {
        double dist = (obj_pos_vector - line[i]).norm();
        if (dist < minDist)
        {
            minDist = dist;
            closest_idx = static_cast<int>(i);
        }
    }

    Vector2d diff = obj_pos_vector - line[closest_idx];
    double d = -diff.dot(norv[closest_idx]);

    return d;
}

double get_frenet_d_point(const Vector2d &point, const vector<Vector2d> &line, const vector<Vector2d> &norv)
{
    if (line.empty() || norv.empty())
    {
        cerr << "경로 또는 법선 벡터 데이터가 비어 있습니다!" << endl;
        return 0.0;
    }

    // 가장 가까운 점 찾기
    double minDist = numeric_limits<double>::max();
    int closest_idx = 0;
    for (size_t i = 0; i < line.size(); ++i)
    {
        double dist = (point - line[i]).norm();
        if (dist < minDist)
        {
            minDist = dist;
            closest_idx = static_cast<int>(i);
        }
    }

    Vector2d diff = point - line[closest_idx];
    double d = -diff.dot(norv[closest_idx]);

    return d;
}

pair<int, int> get_region(const Vector2d &point)
{

    vector<Vector2d> tanv1, norv1;
    vector<Vector2d> tanv2, norv2;
    vector<Vector2d> tanv3, norv3;
    get_Tan(Line1, tanv1, norv1);
    get_Tan(Line2, tanv2, norv2);
    get_Tan(Line3, tanv3, norv3);

    double d1 = get_frenet_d_point(point, Line1, norv1);
    double d2 = get_frenet_d_point(point, Line2, norv2);
    double d3 = get_frenet_d_point(point, Line3, norv3);
    double abs1 = fabs(d1), abs2 = fabs(d2), abs3 = fabs(d3);
    int idx_min = 0;
    double minVal = abs1;
    if (abs2 < minVal)
    {
        minVal = abs2;
        idx_min = 1;
    }
    if (abs3 < minVal)
    {
        minVal = abs3;
        idx_min = 2;
    }
    int line = idx_min + 1;
    int region = 0;
    if (idx_min == 0)
        region = (d1 < 0) ? 1 : 2;
    else if (idx_min == 1)
        region = (d2 < 0) ? 3 : 4;
    else
        region = (d3 < 0) ? 5 : 6;
    return make_pair(line, region);
}

pair<int, int> get_region_LR(const Vector2d &point)
{
    double yaw = OBJ_Yaw;
    Vector2d point_LF = {point[0] + (L / 2 * cos(yaw) - W / 2 * sin(yaw)),
                         OBJ_POS[1] + (L / 2 * sin(yaw) + W / 2 * cos(yaw))};
    Vector2d point_RF = {point[0] + (L / 2 * cos(yaw) + W / 2 * sin(yaw)),
                         point[1] + (L / 2 * sin(yaw) - W / 2 * cos(yaw))};

    auto LF_region = get_region(point_LF).first;
    auto RF_region = get_region(point_RF).first;

    return {LF_region, RF_region};
}

int isChangeLine(const Eigen::MatrixXd &X_t, float std)
{
    vector<Vector2d> tanv1, norv1;
    vector<Vector2d> tanv2, norv2;
    vector<Vector2d> tanv3, norv3;
    get_Tan(Line1, tanv1, norv1);
    get_Tan(Line2, tanv2, norv2);
    get_Tan(Line3, tanv3, norv3);

    Vector2d obj_pos_vector(OBJ_POS[0], OBJ_POS[1]);

    const double L = 4.63, W = 1.892;

    int line_cur = get_region(obj_pos_vector).first;

    double judge_d = X_t(2) + std * X_t(3);
    Eigen::Vector2d judge_point = frenet_to_enu_point(X_t(0), judge_d, get_S(Line2), Line2, norv2);

    int line_LF, line_RF;
    std::tie(line_LF, line_RF) = get_region_LR(judge_point);
    int futureLine = line_cur;

    if ((line_LF < line_cur) || (line_RF < line_cur))
    {
        if (X_t(3) < 0)
        {
            futureLine = std::min(line_LF, line_RF);
        }
        else if (X_t(3) > 0)
        {
            futureLine = line_cur;
        }
    }
    else if ((line_LF > line_cur) || (line_RF > line_cur))
    {
        if (X_t(3) > 0)
        {
            futureLine = std::max(line_LF, line_RF);
        }
        else if (X_t(3) < 0)
        {
            futureLine = line_cur;
        }
    }
    return futureLine;
}

pair<int, int> get_region_ego()
{
    vector<Vector2d> tanv1, norv1;
    vector<Vector2d> tanv2, norv2;
    vector<Vector2d> tanv3, norv3;
    get_Tan(Line1, tanv1, norv1);
    get_Tan(Line2, tanv2, norv2);
    get_Tan(Line3, tanv3, norv3);

    Vector2d ego_pos(CUR_POS[0], CUR_POS[1]);

    // lambda 함수: 주어진 라인에 대해 자차의 프레넷 d 값을 계산
    auto get_frenet_d_for_point = [&](const vector<Vector2d> &line, const vector<Vector2d> &norv) -> double
    {
        double minDist = numeric_limits<double>::max();
        int closest_idx = 0;
        for (size_t i = 0; i < line.size(); ++i)
        {
            double dist = (ego_pos - line[i]).norm();
            if (dist < minDist)
            {
                minDist = dist;
                closest_idx = static_cast<int>(i);
            }
        }
        Vector2d diff = ego_pos - line[closest_idx];
        double d = -diff.dot(norv[closest_idx]);
        return d;
    };

    double d1 = get_frenet_d_for_point(Line1, norv1);
    double d2 = get_frenet_d_for_point(Line2, norv2);
    double d3 = get_frenet_d_for_point(Line3, norv3);
    double abs1 = fabs(d1), abs2 = fabs(d2), abs3 = fabs(d3);
    int idx_min = 0;
    double minVal = abs1;
    if (abs2 < minVal)
    {
        minVal = abs2;
        idx_min = 1;
    }
    if (abs3 < minVal)
    {
        minVal = abs3;
        idx_min = 2;
    }
    int lane = idx_min + 1;
    int region = 0;
    if (idx_min == 0)
        region = (d1 < 0) ? 1 : 2;
    else if (idx_min == 1)
        region = (d2 < 0) ? 3 : 4;
    else
        region = (d3 < 0) ? 5 : 6;
    return make_pair(lane, region);
}

void init_Kalman()
{
    if (!init)
    {
        H << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        Q << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        R << 0.5, 0, 0, 0,
            0, 0.5, 0, 0,
            0, 0, 0.5, 0,
            0, 0, 0, 0.5;

        P_t << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        X_t << OBJ_POS_FRENET[0], OBJ_VEL / 3.6, OBJ_POS_FRENET[1], OBJ_VEL / 3.6 * sin(Yaw - theta);
        init = true;
    }
}

void Predict()
{
    float tau = 0.5;
    double s = X_t(0);
    double s_dot = X_t(1);
    double d = X_t(2);
    double d_dot = X_t(3);

    double s_pred = s + s_dot * dt;
    double s_dot_pred = s_dot;

    float alpha;

    alpha = (isLaneChange) ? 0.2 : 0.2;

    double exp = std::exp(-dt / tau);
    double d_pred = d * exp + (alpha * (d + d_dot * dt) + (1 - alpha) * target_lane_d) * (1 - exp);
    double d_dot_pred = d_dot;

    X_p << s_pred, s_dot_pred, d_pred, d_dot_pred;

    Eigen::Matrix4d F;
    F << 1, dt, 0, 0,
        0, 1, 0, 0,
        0, 0, exp + (1 - exp) * alpha, dt * alpha * (1 - exp),
        0, 0, 0, 1;

    P_p = F * P_t * F.transpose() + Q;
}

void Update()
{
    double minDiff = numeric_limits<double>::max();
    int idx = 1;
    for (size_t i = 0; i < s_ref.size(); ++i)
    {
        double diff = fabs(s_ref[i] - X_p(0));
        if (diff < minDiff)
        {
            minDiff = diff;
            idx = static_cast<int>(i);
        }
    }
    int OBJ_pred_clost_idx = idx;

    double dx = ref_path[idx](0) - ref_path[idx - 1](0);
    double dy = ref_path[idx](1) - ref_path[idx - 1](1);
    double dist = sqrt(dx * dx + dy * dy);

    Vector2d pred_tanv;

    if (dist < 1e-6)
    {
        pred_tanv = Vector2d(1, 0);
    }
    else
    {
        Vector2d tangent(dx / dist, dy / dist);
        pred_tanv = tangent;
    }
    theta = atan2(pred_tanv(1), pred_tanv(0));
    double d_dot_meas = -X_p(1) * sin(OBJ_Yaw - theta);

    Eigen::MatrixXd Z_t(4, 1);
    Z_t << OBJ_POS_FRENET[0], OBJ_VEL / 3.6, OBJ_POS_FRENET[1], d_dot_meas;

    Eigen::MatrixXd Z_p(4, 1);
    Z_p = H * X_p;

    Eigen::MatrixXd residual = Z_t - Z_p;

    Matrix4d S = H * P_p * H.transpose() + R;
    Matrix4d K = P_p * H.transpose() * S.inverse();
    X_t = X_p + K * residual;
    P_t = P_p - K * H * P_p;
}

MatrixXd makepath_dt(double dt_val, const MatrixXd &X_current)
{
    double tau = 1.0;
    double s = X_current(0);
    double s_dot = X_current(1);
    double d = X_current(2);
    double d_dot = X_current(3);

    double s_pred = s + s_dot * dt_val;
    double s_dot_pred = s_dot;

    double alpha = 0.2;
    double exp_val = std::exp(-dt_val / tau);
    double d_pred = d * exp_val + (alpha * (d + d_dot * dt_val) + (1 - alpha) * target_lane_d) * (1 - exp_val);
    double d_dot_pred = d_dot;
    MatrixXd X_pred(4, 1);
    X_pred << s_pred, s_dot_pred, d_pred, d_dot_pred;
    return X_pred;
}

void Kalman()
{
    const int predict_count = 50;
    Predict();
    Update();
    int future_line = isChangeLine(X_t, 1.5);
    Vector2d obj_pos_vector(OBJ_POS[0], OBJ_POS[1]);
    pair<int, int> ego_region = get_region_ego();
    ego_lane = ego_region.first;
    pair<int, int> obj_region = get_region(obj_pos_vector);
    int obj_lane = obj_region.first;

    obj_in_cur_line = (obj_lane == ego_lane);
    obj_will_in_cur_line = (future_line != obj_lane);

    vector<double> d2_1(PathGap.size());
    vector<double> d2_3(PathGap.size());
    double sum_gap = 0;
    for (const auto &p : PathGap)
        sum_gap += p(0);
    double mean_gap1 = sum_gap / PathGap.size();
    for (size_t i = 0; i < PathGap.size(); i++)
    {
        d2_1[i] = -PathGap[i](0);
    }

    for (size_t i = 0; i < PathGap.size(); i++)
    {
        d2_3[i] = PathGap[i](1);
    }
    int idx = OBJ_clost_idx;
    if (future_line == 1)
        target_lane_d = d2_1[idx];
    else if (future_line == 2)
        target_lane_d = 0;
    else if (future_line == 3)
        target_lane_d = d2_3[idx];

    Vector2d enu_pred = frenet_to_enu_point(X_t(0), X_t(2));
    Vector2d enu_real(OBJ_POS[0], OBJ_POS[1]);

    predPath.clear();
    frenet_predPath.clear();
    MatrixXd X_tmp_pred = X_t;
    for (int i = 1; i <= predict_count; i++)
    {
        double dt_t = dt;
        X_tmp_pred = makepath_dt(dt_t, X_tmp_pred);
        predPath.push_back(frenet_to_enu_point(X_tmp_pred(0), X_tmp_pred(2)));
        frenet_predPath.push_back({X_tmp_pred(0), X_tmp_pred(2)});
    }
}

double cal_TTC()
{
    float ttc;
    float dist = 999999999;
    for (int index = 0; index < frenet_predPath.size(); index++)
    {
        Vector2d frenet_point = frenet_predPath[index];
        frenet_point(1) += 1.5;
        Vector2d point = frenet_to_enu_point(frenet_point(0), frenet_point(1));
        pair<int, int> point_region = get_region(point);
        int point_line = point_region.first;
        if (point_line == ego_lane)
        {
            float cal_dist = sqrt(pow(point(0) - CUR_POS[0], 2) + pow(point(1) - CUR_POS[1], 2));

            if (dist <= cal_dist)
            {
                dist = cal_dist;
            }
        }
    }

    if (dist == 999999999)
    {
        dist = sqrt(pow(OBJ_POS[0] - CUR_POS[0], 2) + pow(OBJ_POS[1] - CUR_POS[1], 2));
    }
    //float dist = sqrt(pow(OBJ_POS[0] - CUR_POS[0], 2) + pow(OBJ_POS[1] - CUR_POS[1], 2));
    float dV = CUR_VEL - OBJ_VEL;
    if (dV < 1)
        dV = 1;

    if (obj_will_in_cur_line || obj_in_cur_line)
    {
        ttc = dist / dV;
    }
    else
    {
        ttc = 100;
    }

    return ttc;
}

void FindTarget()
{
    double tmpDist = numeric_limits<double>::max();
    int tmpIdx = 0;

    for (size_t i = 0; i < ref_path.size(); ++i)
    {
        double dx = ref_path[i](0) - CUR_POS[0];
        double dy = ref_path[i](1) - CUR_POS[1];
        double dist = sqrt(dx * dx + dy * dy);

        if (dist < tmpDist)
        {
            tmpDist = dist;
            tmpIdx = static_cast<int>(i);
        }
    }

    EGO_clost_idx = tmpIdx;

    for (int j = tmpIdx; j < ref_path.size(); j++)
    {
        double dx = ref_path[j](0) - CUR_POS[0];
        double dy = ref_path[j](1) - CUR_POS[1];
        double dist = sqrt(dx * dx + dy * dy);

        if (dist >= Lookahead_Distance)
        {
            EGO_Target_idx = j;
            break;
        }
    }
}

void GetSteer()
{
    float alpha = atan2(ref_path[EGO_Target_idx](1) - CUR_POS[1], ref_path[EGO_Target_idx](0) - CUR_POS[0]);
    alpha -= Yaw / 180 * PI;
    TargetSteer = atan2(2.0 * WheelBase * sin(alpha), Lookahead_Distance);
}

void PubCtrl_1()
{
    morai_msgs::CtrlCmd msg;
    msg.longlCmdType = 1;
    msg.steering = TargetSteer;
    msg.accel = accelCmd;
    msg.brake = brakeCmd;
    ctrlPub.publish(msg);
}

void PubCtrl_2()
{
    morai_msgs::CtrlCmd msg;
    msg.longlCmdType = 2;
    msg.steering = TargetSteer;
    msg.velocity = 60;

    ctrlPub.publish(msg);
}

void GetSteer_ACC()
{
    float target_x = OBJ_POS[0];
    float target_y = OBJ_POS[1];

    float Alpha = atan2(target_y - CUR_POS[1], target_x - CUR_POS[0]);
    Alpha -= Yaw / 180 * PI;
    TargetSteer = atan2(2.0 * WheelBase * sin(Alpha), Lookahead_Distance);
}

void ACC()
{
    float dist = sqrt(pow(CUR_POS[0] - OBJ_POS[0], 2) + pow(CUR_POS[1] - OBJ_POS[1], 2));
    float targetGap = 15.0;
    float dist_error = dist - targetGap;
    float vel_error = OBJ_VEL - CUR_VEL;

    if (fabs(dist_error) < 2.0)
    {
        TargetSpeed = CUR_VEL;
        if (accelCmd > 0.05)
            accelCmd -= 0.05;
        else
            accelCmd = 0;

        if (brakeCmd > 0.05)
            brakeCmd -= 0.05;
        else
            brakeCmd = 0;
    }
    else
    {
        TargetSpeed = OBJ_VEL;

        if (vel_error > 0)
        {
            accelCmd = kp * vel_error;
            brakeCmd = 0;
        }
        else
        {
            brakeCmd = kp * -vel_error;
            accelCmd = 0;
        }
    }

    if (fabs(vel_error) < 0.5)
    {
        accelCmd = 0;
        brakeCmd = 0;
    }

    if (accelCmd > 1.0)
        accelCmd = 1.0;
    if (brakeCmd > 1.0)
        brakeCmd = 1.0;
}

void PurePursuit()
{
    double ttc = cal_TTC();
    bool now = obj_in_cur_line;
    bool will = obj_will_in_cur_line;
    cout << "=============" << endl;
    cout << "TTC : " << ttc << endl;
    cout << "현재 자차선 진입 여부 : " << obj_in_cur_line << endl;
    // cout << "미래 자차선 진입 여부 : " << obj_will_in_cur_line << endl;
    if (ttc != 100)
        cout << "판단 시점에서의 거리 : " << sqrt(pow(OBJ_POS[0] - CUR_POS[0], 2) + pow(OBJ_POS[1] - CUR_POS[1], 2)) << endl;

    FindTarget();
    GetSteer();

    if (!now && !will)
    {
        PubCtrl_2();
    }
    else
    {
        if (ttc > 0 && ttc < 3)
        {
            accelCmd = 0;
            brakeCmd = 1;
            cout << "AEB" << endl;
        }
        else if (ttc > 0 && ttc < 6)
        {
            accelCmd = 0;
            brakeCmd = 0.2;
            cout << "AEB" << endl;
        }
        else
        {
            cout << "ACC" << endl;
            ACC();
        }
        PubCtrl_1();
    }
}

void showCarMarker()
{
    // ========================== 예측 OBJ 위치 ========================== //
    Vector2d enu_pred = frenet_to_enu_point(X_t(0), X_t(2), s_ref, ref_path, ref_norv);
    visualization_msgs::Marker pred_marker;
    pred_marker.header.frame_id = "map";
    pred_marker.header.stamp = ros::Time::now();
    pred_marker.ns = "obj_pred_pos";
    pred_marker.id = 8;
    pred_marker.type = visualization_msgs::Marker::CUBE;
    pred_marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Quaternion quat2 = tf::createQuaternionMsgFromYaw(OBJ_Yaw);
    pred_marker.pose.orientation = quat2;
    pred_marker.pose.position.x = enu_pred.x();
    pred_marker.pose.position.y = enu_pred.y();
    pred_marker.pose.position.z = 0;
    pred_marker.scale.x = 2.4;
    pred_marker.scale.y = 1.2;
    pred_marker.scale.z = 0.2;
    pred_marker.color.r = 1.0;
    pred_marker.color.g = 0.0;
    pred_marker.color.b = 0.0;
    pred_marker.color.a = 1.0;
    pred_pos_pub.publish(pred_marker);
    // ========================== 예측 OBJ 위치 ========================== //

    // ========================== 실제 OBJ 위치 ========================== //
    Vector2d enu_real(OBJ_POS[0], OBJ_POS[1]);
    visualization_msgs::Marker real_marker;
    real_marker.header.frame_id = "map";
    real_marker.header.stamp = ros::Time::now();
    real_marker.ns = "obj_real_pos";
    real_marker.id = 9;
    real_marker.type = visualization_msgs::Marker::CUBE;
    real_marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Quaternion quat1 = tf::createQuaternionMsgFromYaw(OBJ_Yaw);
    real_marker.pose.orientation = quat1;
    real_marker.pose.position.x = enu_real.x();
    real_marker.pose.position.y = enu_real.y();
    real_marker.pose.position.z = 0;
    real_marker.scale.x = 2.4;
    real_marker.scale.y = 1.2;
    real_marker.scale.z = 0.2;
    real_marker.color.r = 0.0;
    real_marker.color.g = 0.0;
    real_marker.color.b = 0.0;
    real_marker.color.a = 1.0;
    real_pos_pub.publish(real_marker); // 빨강
    // ========================== 실제 OBJ 위치 ========================== //

    // ========================== 실제 자차 위치 ========================== //
    visualization_msgs::Marker cur_pos_marker;
    cur_pos_marker.header.frame_id = "map";
    cur_pos_marker.header.stamp = ros::Time::now();
    cur_pos_marker.ns = "ego_cur_pos";
    cur_pos_marker.id = 10;
    cur_pos_marker.type = visualization_msgs::Marker::CUBE;
    cur_pos_marker.action = visualization_msgs::Marker::ADD;
    cur_pos_marker.pose.position.x = CUR_POS[0];
    cur_pos_marker.pose.position.y = CUR_POS[1];
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(Yaw / 180 * PI);
    cur_pos_marker.pose.orientation = quat;
    cur_pos_marker.scale.x = 2.4;
    cur_pos_marker.scale.y = 1.2;
    cur_pos_marker.scale.z = 0.2;
    cur_pos_marker.color.r = 0.0;
    cur_pos_marker.color.g = 1.0;
    cur_pos_marker.color.b = 0.0;
    cur_pos_marker.color.a = 1.0;
    ego_pos_pub.publish(cur_pos_marker);
    // ========================== 실제 자차 위치 ========================== //

    // ========================== 예측 경로 ========================== //
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "pred_path";
    path_marker.id = 11;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.2;
    path_marker.scale.z = 0.4;
    path_marker.color.r = 0.8;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.8;
    path_marker.color.a = 1.0;
    for (const auto &p : predPath)
    {
        geometry_msgs::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = 0;
        path_marker.points.push_back(pt);
    }
    pred_path_pub.publish(path_marker);
    // ========================== 예측 경로 ========================== //
}

void showLine()
{
    ros::Time current_time = ros::Time::now();
    vector<Eigen::Vector2d> middle_lane1, middle_lane2;

    for (size_t i = 0; i < Line1.size() - 1; i++)
    {
        Vector2d mid_point = (Line1[i] + Line2[i]) / 2.0;
        middle_lane1.push_back(mid_point);
    }
    for (size_t i = 0; i < Line2.size() - 1; i++)
    {
        Vector2d mid_point = (Line2[i] + Line3[i]) / 2.0;
        middle_lane2.push_back(mid_point);
    }

    visualization_msgs::Marker line1_marker;
    line1_marker.header.frame_id = "map";
    line1_marker.header.stamp = current_time;
    line1_marker.ns = "line1";
    line1_marker.id = 200;
    line1_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line1_marker.action = visualization_msgs::Marker::ADD;
    line1_marker.scale.x = 0.2;
    line1_marker.color.a = 1.0;
    line1_marker.color.r = 0.5;
    line1_marker.color.g = 1.0;
    line1_marker.color.b = 0.5;

    for (size_t i = 0; i < Line1.size(); ++i)
    {
        geometry_msgs::Point pt;
        pt.x = Line1[i](0);
        pt.y = Line1[i](1);
        pt.z = 0;
        line1_marker.points.push_back(pt);
    }

    visualization_msgs::Marker out_boundary = line1_marker;
    out_boundary.id = 20102;
    out_boundary.ns = "out_boundary";
    out_boundary.color.r = 1.0;
    out_boundary.color.g = 1.0;
    out_boundary.color.b = 1.0;
    out_boundary.points.clear();
    out_boundary.header.stamp = current_time;
    for (size_t i = 0; i < Boundary1.size(); i++)
    {
        geometry_msgs::Point pt;
        pt.x = Boundary1[i](0);
        pt.y = Boundary1[i](1);
        pt.z = 0;
        out_boundary.points.push_back(pt);
    }

    visualization_msgs::Marker in_boundary = line1_marker;
    in_boundary.id = 22011;
    in_boundary.ns = "in_boundary";
    in_boundary.color.r = 1.0;
    in_boundary.color.g = 1.0;
    in_boundary.color.b = 1.0;
    in_boundary.points.clear();
    in_boundary.header.stamp = current_time;
    for (size_t i = 0; i < Boundary4.size(); i++)
    {
        geometry_msgs::Point pt;
        pt.x = Boundary4[i](0);
        pt.y = Boundary4[i](1);
        pt.z = 0;
        in_boundary.points.push_back(pt);
    }

    visualization_msgs::Marker middle_lane1_marker = line1_marker;
    middle_lane1_marker.id = 400;
    middle_lane1_marker.type = visualization_msgs::Marker::LINE_LIST;
    middle_lane1_marker.scale.x = 0.2;
    middle_lane1_marker.ns = "line12";
    middle_lane1_marker.points.clear();
    middle_lane1_marker.color.r = 1.0;
    middle_lane1_marker.color.g = 1.0;
    middle_lane1_marker.color.b = 1.0;
    middle_lane1_marker.color.a = 1.0;
    middle_lane1_marker.header.stamp = current_time;

    for (int i = 0; i < middle_lane1.size() - 10; i += 10)
    {
        geometry_msgs::Point pt1, pt2;
        pt1.x = middle_lane1[i](0);
        pt1.y = middle_lane1[i](1);
        pt1.z = 0;

        pt2.x = middle_lane1[i + 5](0); // 짧은 선을 그림
        pt2.y = middle_lane1[i + 5](1);
        pt2.z = 0;

        middle_lane1_marker.points.push_back(pt1);
        middle_lane1_marker.points.push_back(pt2);
    }

    visualization_msgs::Marker middle_lane2_marker = line1_marker;
    middle_lane2_marker.id = 401;
    middle_lane2_marker.type = visualization_msgs::Marker::LINE_LIST;
    middle_lane2_marker.scale.x = 0.2;
    middle_lane2_marker.ns = "line23";
    middle_lane2_marker.points.clear();
    middle_lane2_marker.color.r = 1.0;
    middle_lane2_marker.color.g = 1.0;
    middle_lane2_marker.color.b = 1.0;
    middle_lane2_marker.header.stamp = current_time;

    for (int i = 0; i < middle_lane2.size() - 10; i += 10)
    {
        geometry_msgs::Point pt1, pt2;
        pt1.x = middle_lane2[i](0);
        pt1.y = middle_lane2[i](1);
        pt1.z = 0;

        pt2.x = middle_lane2[i + 5](0); // 짧은 선을 그림
        pt2.y = middle_lane2[i + 5](1);
        pt2.z = 0;

        middle_lane2_marker.points.push_back(pt1);
        middle_lane2_marker.points.push_back(pt2);
    }

    static_pub.publish(middle_lane1_marker);
    static_pub.publish(middle_lane2_marker);

    static_pub.publish(in_boundary);
    static_pub.publish(out_boundary);
}

void visualization()
{
    showLine();
    showCarMarker();
}

void getGNSSData(const package::gnss::ConstPtr &msg)
{
    CUR_POS[0] = msg->East;
    CUR_POS[1] = msg->North;
    Yaw = msg->Yaw;
    visualization();
}

void getObjectData(const package::object::ConstPtr &msg)
{
    OBJ_VEL = msg->Vel;
    OBJ_POS[0] = msg->East;
    OBJ_POS[1] = msg->North;
    OBJ_Yaw = msg->Yaw * M_PI / 180;

    get_frenet_data();
    Kalman();
    PurePursuit();

    if (isStart)
    {
        morai_msgs::MoraiEventCmdSrv ego_srv, obj_srv;
        morai_msgs::EventInfo ego_info, obj_info;

        ego_info.option = 2;
        ego_info.gear = 4;
        ego_srv.request.request = ego_info;
        ego_client.call(ego_srv);

        obj_info.option = 2;    
        obj_info.gear = 4;
        obj_srv.request.request = obj_info;
        obj_client.call(obj_srv);

        ego_info.option = 1;
        ego_info.ctrl_mode = 3;
        ego_srv.request.request = ego_info;
        ego_client.call(ego_srv);

        obj_info.option = 1;    
        obj_info.ctrl_mode = 3;
        obj_srv.request.request = obj_info;
        obj_client.call(obj_srv);
        isStart = false;    
    }
}

void getEgoData(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
{
    CUR_VEL = sqrt(pow(msg->velocity.x, 2) + pow(msg->velocity.y, 2) + pow(msg->velocity.z, 2)) * 3.6;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Controller");
    ros::NodeHandle node;
    ros::Rate rate(10);

    initRef();
    init_Kalman();

    s_ref = get_S(ref_path);
    get_Tan(ref_path, ref_tanv, ref_norv);

    ego_pos_pub = node.advertise<visualization_msgs::Marker>("/ego_marker", 1);
    static_pub = node.advertise<visualization_msgs::Marker>("/path", 100);
    real_pos_pub = node.advertise<visualization_msgs::Marker>("/real_pos", 1);
    pred_pos_pub = node.advertise<visualization_msgs::Marker>("/pred_pos", 1);
    pred_path_pub = node.advertise<visualization_msgs::Marker>("/pred_path", 1);

    ros::Subscriber gnssSub = node.subscribe<package::gnss>("/GNSS", 1, getGNSSData);
    ros::Subscriber objectSub = node.subscribe<package::object>("/custom_npc", 1, getObjectData);
    ros::Subscriber egoSub = node.subscribe<morai_msgs::EgoVehicleStatus>("/Ego_topic", 1, getEgoData);
    ctrlPub = node.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 1);

    ego_client = node.serviceClient<morai_msgs::MoraiEventCmdSrv>("/Service_MoraiEventCmd");
    obj_client = node.serviceClient<morai_msgs::MoraiEventCmdSrv>("/ego_2/Service_MoraiEventCmd");

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}