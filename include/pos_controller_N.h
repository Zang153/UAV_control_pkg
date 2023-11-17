
#ifndef POS_CONTROLLER_N_H
#define POS_CONTROLLER_N_H

#include <math.h>
#include <command_to_mavros.h>
#include <px4_command_utils.h>
#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>
#include <px4_command/ControlOutput.h>
#include <px4_command/VectorPayloadtoQuad.h>
#include <math_utils.h>

using namespace std;

class pos_controller_N
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        pos_controller_N(void):
            pos_pid_n_nh("~")
        {
        /*    
        //wait for editing
            pos_pid_n_nh.param<float>("Quad/mass", Quad_MASS, 1.0);

            pos_pid_n_nh.param<float>("Pos_pid/Kp_xy", Kp[0], 1.0);
            pos_pid_n_nh.param<float>("Pos_pid/Kp_xy", Kp[1], 1.0);
            pos_pid_n_nh.param<float>("Pos_pid/Kp_z" , Kp[2], 2.0);
            pos_pid_n_nh.param<float>("Pos_pid/Kd_xy", Kd[0], 0.5);
            pos_pid_n_nh.param<float>("Pos_pid/Kd_xy", Kd[1], 0.5);
            pos_pid_n_nh.param<float>("Pos_pid/Kd_z" , Kd[2], 0.5);
            pos_pid_n_nh.param<float>("Pos_pid/Ki_xy", Ki[0], 0.2);
            pos_pid_n_nh.param<float>("Pos_pid/Ki_xy", Ki[1], 0.2);
            pos_pid_n_nh.param<float>("Pos_pid/Ki_z" , Ki[2], 0.2);
            
            pos_pid_n_nh.param<float>("Limit/pxy_error_max", pos_error_max[0], 0.6);
            pos_pid_n_nh.param<float>("Limit/pxy_error_max", pos_error_max[1], 0.6);
            pos_pid_n_nh.param<float>("Limit/pz_error_max" , pos_error_max[2], 1.0);
            pos_pid_n_nh.param<float>("Limit/vxy_error_max", vel_error_max[0], 0.3);
            pos_pid_n_nh.param<float>("Limit/vxy_error_max", vel_error_max[1], 0.3);
            pos_pid_n_nh.param<float>("Limit/vz_error_max" , vel_error_max[2], 1.0);
            pos_pid_n_nh.param<float>("Limit/pxy_int_max"  , int_max[0], 0.5);
            pos_pid_n_nh.param<float>("Limit/pxy_int_max"  , int_max[1], 0.5);
            pos_pid_n_nh.param<float>("Limit/pz_int_max"   , int_max[2], 0.5);
            pos_pid_n_nh.param<float>("Limit/tilt_max", tilt_max, 20.0);
            pos_pid_n_nh.param<float>("Limit/int_start_error"  , int_start_error, 0.3);

            integral = Eigen::Vector3f(0.0,0.0,0.0);
            */
            // subscribe the direction of vector.
            vector_sub = pos_pid_n_nh.subscribe<px4_command::VectorPayloadtoQuad>("/PtoQVector/pose", 10, &pos_controller_N::vect_cb, this);
            pos_pid_n_nh.param<float>("Limit/tilt_max", tilt_max, 20.0);
            Eigen::MatrixXd integal = Eigen::MatrixXd::Zero(30,3);
        }

        //Quadrotor Parameter
        float Quad_MASS;
        float Payload_MASS;
        //PD and neuron coefficients for the control law
        //*******************************************************************
        // Fx= -Kp * tanh (error_pose) - Kd * tanh (error_vel) + Nx 
        // Fy= -Kp * tanh (error_pose) - Kd * tanh (error_vel) + Ny 
        // Fz= -Kp * tanh (error_pose) - Kd * tanh (error_vel) + Nz 
        
        Eigen::Vector3d Kp;
        Eigen::Vector3d Kd;
        Eigen::Vector3d Kn;

        //Limitation
        Eigen::Vector3f pos_error_max;
        Eigen::Vector3f vel_error_max;
        Eigen::Vector3f int_max;
        float tilt_max;
        float int_start_error;

        //积分项
        //Eigen::Vector3f integral;

        //输出
        px4_command::ControlOutput _ControlOutput;

        px4_command::VectorPayloadtoQuad _VectorPtoQ;

        //Printf the PID parameter
        //void printf_param();

        //void printf_result();

        // Position control main function 
        // [Input: Current state, Reference state, sub_mode, vector(payload to quad), dt; Output: AttitudeReference;]
        px4_command::ControlOutput pos_controller(const px4_command::DroneState& _DroneState, const px4_command::TrajectoryPoint& _Reference_State, const px4_command::VectorPayloadtoQuad& _VectorPtoQ, float dt);

        Eigen::Vector3d cal_es_Neuron(Eigen::MatrixXd Pos_d, Eigen::MatrixXd PosE, Eigen::MatrixXd VelE, Eigen::MatrixXd Vector_pos, Eigen::MatrixXd Vector_vel);
        Eigen::Vector3d cal_des_Neuron(Eigen::MatrixXd Pos_d, Eigen::MatrixXd PosE, Eigen::MatrixXd VelE, Eigen::MatrixXd Vector_pos, Eigen::MatrixXd Vector_vel);
        //Eigen::Vector3d cal_neuron(const px4_command::DroneState& _DroneState, const px4_command::TrajectoryPoint& _Reference_State, const px4_command::VectorPayloadtoQuad& _VectorPtoQ );

        Eigen::MatrixXd fcn(Eigen::MatrixXd Pos_d, Eigen::MatrixXd PosE, Eigen::MatrixXd VelE, Eigen::MatrixXd Vector_pos, Eigen::MatrixXd Vector_vel);
        
        Eigen::MatrixXd integ(Eigen::MatrixXd dW, double dt);

    private:
        ros::NodeHandle pos_pid_n_nh;

        ros::Subscriber vector_sub;

        void vect_cb(const px4_command::VectorPayloadtoQuad::ConstPtr& msg)
        {
            _VectorPtoQ = *msg;
            
        }

};

px4_command::ControlOutput pos_controller_N::pos_controller(const px4_command::DroneState& _DroneState, const px4_command::TrajectoryPoint& _Reference_State, const px4_command::VectorPayloadtoQuad& _VectorPtoQ, float dt)
{
    Eigen::Vector3d accel_sp;
    Eigen::Vector3d Force_sp;
    // 计算误差项
    Eigen::Vector3d es;
    Eigen::Vector3d des;
    // 系数
    Eigen::MatrixXd Pos_d(3,1);
    Eigen::MatrixXd PosE(3,1);
    Eigen::MatrixXd VelE(3,1);

    Eigen::MatrixXd Vector_pos(3,1);
    Eigen::MatrixXd Vector_vel(3,1);

    Pos_d << _Reference_State.position_ref[0], _Reference_State.position_ref[1], _Reference_State.position_ref[2];
    PosE  << _DroneState.position[0], _DroneState.position[1], _DroneState.position[2];
    VelE  << _DroneState.velocity[0], _DroneState.velocity[1], _DroneState.velocity[2];

    Vector_pos << _VectorPtoQ.position[0], _VectorPtoQ.position[1], _VectorPtoQ.position[2];
    Vector_vel << _VectorPtoQ.velocity[0], _VectorPtoQ.velocity[1], _VectorPtoQ.velocity[2];

    Eigen::Vector3d Neuron;

    Neuron = fcn(Pos_d, PosE, VelE, Vector_pos, Vector_vel);
    
    es = cal_es_Neuron(Pos_d, PosE, VelE, Vector_pos, Vector_vel);

    des = cal_des_Neuron(Pos_d, PosE, VelE, Vector_pos, Vector_vel);

    Eigen::Vector3f vel_error_max, pos_error_max;
    vel_error_max[0] = 0.3;
    vel_error_max[1] = 0.3;
    vel_error_max[2] = 1.0;
    
    pos_error_max[0] = 0.6;
    pos_error_max[1] = 0.6;
    pos_error_max[2] = 1.0;
    /*for(int i=0; i<3; i++)
    {
        es[i] = constrain_function(es[i],pos_error_max[i]);
        des[i] = constrain_function(des[i],vel_error_max[i]);
    }*/

    // 误差项限幅
    /*
    for (int i=0; i<3; i++)
    {
        pos_error[i] = constrain_function(pos_error[i], pos_error_max[i]);
        vel_error[i] = constrain_function(vel_error[i], vel_error_max[i]);
    }
    */
    // 期望加速度 = (PD + N)/Mass
    
    Quad_MASS = 1.5;
    Payload_MASS = 0.0;
//kp=100; kd=40;
    Kp[0] = 5.0;
    Kp[1] = 5.0;
    Kp[2] = 10.0;

    Kd[0] = 5.0;
    Kd[1] = 5.0;
    Kd[2] = 9.0;
  
    Force_sp[0] = -Kp[0] * tanh(es[0]) - Kd[0] * tanh(des[0]) + Neuron[0] ;
    Force_sp[1] = -Kp[1] * tanh(es[1]) - Kd[1] * tanh(des[1]) + Neuron[1] ;
    Force_sp[2] = -Kp[2] * tanh(es[2]) - Kd[2] * tanh(des[2]) + Neuron[2] + (Quad_MASS + Payload_MASS) * 9.8 - 2.8;
    cout << "direction[X] =" << es[0] <<' '<< des[0] << ' ' << Force_sp[0] << "N = " << Neuron[0] << endl;
    cout << "direction[Y] =" << es[1] <<' '<< des[1] << ' ' << Force_sp[1] << "N = " << Neuron[1] << endl;
    cout << "direction[Z] =" << es[2] <<' '<< des[2] << ' ' << Force_sp[2] << "N = " << Neuron[2] << endl;

    for (int i=0; i<3; i++)
    {
        accel_sp[i] = Force_sp[i]/(Quad_MASS + Payload_MASS);
    }
    // 期望推力 = 期望加速度 × 质量
    // 归一化推力 ： 根据电机模型，反解出归一化推力
    Eigen::Vector3d thrust_sp;
    Eigen::Vector3d throttle_sp;
    float tilt_limitation;
    tilt_limitation = 8.0;
    thrust_sp =  px4_command_utils::accelToThrust(accel_sp, Quad_MASS, tilt_limitation);
    throttle_sp = px4_command_utils::thrustToThrottle(thrust_sp);
    cout << "thr_direction[X] ="  << thrust_sp[0] << endl;
    cout << "thr_direction[Y] ="  << thrust_sp[1] << endl;
    cout << "thr_direction[Z] ="  << thrust_sp[2] << endl;
    for (int i=0; i<3; i++)
    {
        _ControlOutput.Thrust[i] = thrust_sp[i];
        _ControlOutput.Throttle[i] = throttle_sp[i];
        _ControlOutput.Force[i] = Force_sp[i];
    }

    return _ControlOutput;

}
Eigen::MatrixXd pos_controller_N::fcn(Eigen::MatrixXd Pos_d, Eigen::MatrixXd PosE, Eigen::MatrixXd VelE, Eigen::MatrixXd Vector_pos, Eigen::MatrixXd Vector_vel)
{
    double x1, x2, x3, x4, x5, x6, x7, x8, x9, y1, y2, y3, y4, y5, y6;

    x1 = Pos_d(0);
    x2 = Pos_d(1);
    x3 = Pos_d(2);
    x4 = PosE(0);
    x5 = PosE(1);
    x6 = PosE(2);
    x7 = VelE(0);
    x8 = VelE(1);
    x9 = VelE(2);

    y1 = Vector_pos(0);
    y2 = Vector_pos(1);
    y3 = Vector_pos(2);
    y4 = Vector_vel(0);
    y5 = Vector_vel(1);
    y6 = Vector_vel(2);

    double alpha_x, alpha_y, alpha_z;
    alpha_x = 0.01;
    alpha_y = 0.01;
    alpha_z = 0.01;

    Eigen::MatrixXd mu = Eigen::MatrixXd::Zero(3,30);

    double sigmal, iota, g, k;

    sigmal = 0.003;
    iota = 3.0;
    g = 9.81;
    k = 0.0;

    double dex, dey, dez, dvx, dvy, dvz;
    dex = x7;
    dey = x8;
    dez = x9;

    dvx = y4;
    dvy = y5;
    dvz = y6;
    
    double ds1x, ds1y, ds1z;
    ds1x = dex + k * dvx;
    ds1y = dey + k * dvy;
    ds1z = dez + k * dvz;

    Eigen::MatrixXd xi(3,1);
    Eigen::MatrixXd yi(3,1);
    Eigen::MatrixXd zi(3,1);
    xi << x1, x4, x7;
    yi << x2, x5, x8;
    zi << x3, x6, x9;

    Eigen::MatrixXd h1(30,1);
    Eigen::MatrixXd h2(30,1);
    Eigen::MatrixXd h3(30,1);

    for (int j = 0; j<30; j++)
    {
        h1(j,0) = exp(-pow(((xi - mu.col(j)).norm()),2) / (2 * iota * iota));
        h2(j,0) = exp(-pow(((yi - mu.col(j)).norm()),2) / (2 * iota * iota));
        h3(j,0) = exp(-pow(((zi - mu.col(j)).norm()),2) / (2 * iota * iota));
    }

    Eigen::MatrixXd dWx(30,1);
    Eigen::MatrixXd dWy(30,1);
    Eigen::MatrixXd dWz(30,1);
    dWx = -alpha_x * ds1x * h1;
    dWy = -alpha_y * ds1y * h2;
    dWz = -alpha_z * ds1z * h3;

    Eigen::MatrixXd dW(30, 3);
    dW.col(0) = dWx;
    dW.col(1) = dWy;
    dW.col(2) = dWz;

    Eigen::MatrixXd W(30, 3);
    
    double dt = 0.02;
    W = integ(dW, dt);
    cout << "W = " << W << endl;
    Eigen::Vector3d Neuron;
    Eigen::VectorXd Wx;
    Eigen::VectorXd Wy;
    Eigen::VectorXd Wz;
    Eigen::VectorXd H1;
    Eigen::VectorXd H2;
    Eigen::VectorXd H3;
    H1 = h1.col(0);
    H2 = h2.col(0);
    H3 = h3.col(0);
    Wx = W.col(0);
    Neuron[0] = Wx.dot(H1);
    Wy = W.col(1);
    Neuron[1] = Wy.dot(H2);
    Wz = W.col(2);
    Neuron[2] = Wz.dot(H3);
    return Neuron;
}
Eigen::MatrixXd integal = Eigen::MatrixXd::Zero(30,3);
Eigen::MatrixXd pos_controller_N::integ(Eigen::MatrixXd dW, double dt)
{
    Eigen::MatrixXd w_mid = Eigen::MatrixXd::Zero(30,3);
    w_mid = dW * dt;
    integal = integal + w_mid;
    return integal;
}

Eigen::Vector3d pos_controller_N::cal_es_Neuron(Eigen::MatrixXd Pos_d, Eigen::MatrixXd PosE, Eigen::MatrixXd VelE, Eigen::MatrixXd Vector_pos, Eigen::MatrixXd Vector_vel)
{
    double x1, x2, x3, x4, x5, x6, x7, x8, x9, y1, y2, y3, y4, y5, y6;

  
    x1 = Pos_d(0);
    x2 = Pos_d(1);
    x3 = Pos_d(2);
    x4 = PosE(0);
    x5 = PosE(1);
    x6 = PosE(2);
    x7 = VelE(0);
    x8 = VelE(1);
    x9 = VelE(2);

    y1 = Vector_pos(0);
    y2 = Vector_pos(1);
    y3 = Vector_pos(2);
    y4 = Vector_vel(0);
    y5 = Vector_vel(1);
    y6 = Vector_vel(2);

    double k;
    k = 0.0;

    double ex, ey, ez, vx, vy, vz;
    ex = x4 - x1;
    ey = x5 - x2;
    ez = x6 - x3;
    cout << "ex = " << ex << " " << "ey = " << ey << endl;

    vx = y1;
    vy = y2;
    vz = y3;
    
    double s1x, s1y, s1z;
    s1x = ex + k * vx;
    s1y = ey + k * vy;
    s1z = ez + k * (vz-1);
    
    Eigen::Vector3d es;
    es[0] = s1x;
    es[1] = s1y;
    es[2] = s1z;

    return es;
}
Eigen::Vector3d pos_controller_N::cal_des_Neuron(Eigen::MatrixXd Pos_d, Eigen::MatrixXd PosE, Eigen::MatrixXd VelE, Eigen::MatrixXd Vector_pos, Eigen::MatrixXd Vector_vel)
{
    double x1, x2, x3, x4, x5, x6, x7, x8, x9, y1, y2, y3, y4, y5, y6;

 
    x1 = Pos_d(0);
    x2 = Pos_d(1);
    x3 = Pos_d(2);
    x4 = PosE(0);
    x5 = PosE(1);
    x6 = PosE(2);
    x7 = VelE(0);
    x8 = VelE(1);
    x9 = VelE(2);

    y1 = Vector_pos(0);
    y2 = Vector_pos(1);
    y3 = Vector_pos(2);
    y4 = Vector_vel(0);
    y5 = Vector_vel(1);
    y6 = Vector_vel(2);


    double k;
    k = 0.0;

    double dex, dey, dez, dvx, dvy, dvz;
    dex = x7;
    dey = x8;
    dez = x9;

    dvx = y4;
    dvy = y5;
    dvz = y6;
    
    double ds1x, ds1y, ds1z;
    ds1x = dex + k * dvx;
    ds1y = dey + k * dvy;
    ds1z = dez + k * dvz;
    
    Eigen::Vector3d des;
    des[0] = ds1x;
    des[1] = ds1y;
    des[2] = ds1z;

    return des;
}

/*
Eigen::Vector3d pos_controller_N::cal_neuron(const px4_command::DroneState& _DroneState, const px4_command::TrajectoryPoint& _Reference_State, const px4_command::VectorPayloadtoQuad& _VectorPtoQ )
{
    return 0;
}

void pos_controller_PID::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>  PID Position Controller  <<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);
}

// 【打印参数函数】
void pos_controller_PID::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PID Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"Quad_MASS : "<< Quad_MASS << endl;

    cout <<"Kp_x : "<< Kp[0] << endl;
    cout <<"Kp_y : "<< Kp[1] << endl;
    cout <<"Kp_z : "<< Kp[2] << endl;

    cout <<"Kd_x : "<< Kd[0] << endl;
    cout <<"Kd_y : "<< Kd[1] << endl;
    cout <<"Kd_z : "<< Kd[2] << endl;

    cout <<"Ki_x : "<< Ki[0] << endl;
    cout <<"Ki_y : "<< Ki[1] << endl;
    cout <<"Ki_z : "<< Ki[2] << endl;

    cout <<"Limit:  " <<endl;
    cout <<"pxy_error_max : "<< pos_error_max[0] << endl;
    cout <<"pz_error_max :  "<< pos_error_max[2] << endl;
    cout <<"vxy_error_max : "<< vel_error_max[0] << endl;
    cout <<"vz_error_max :  "<< vel_error_max[2] << endl;
    cout <<"pxy_int_max : "<< int_max[0] << endl;
    cout <<"pz_int_max : "<< int_max[2] << endl;
    cout <<"tilt_max : "<< tilt_max << endl;
    cout <<"int_start_error : "<< int_start_error << endl;

}
*/
#endif

//下面这部分函数要加到px4_command_utils.h中
/*
Eigen::Vector3f cal_neuron(const px4_command::DroneState& _DroneState, const px4_command::TrajectoryPoint& _Reference_State)
{
    Eigen::Vector3f pos_error; 
    Eigen::Vector3f vel_error;
    Eigen::Vector3f pos_des;

    Eigen::Vector10d dWx;
    Eigen::Vector10d dWy;
    Eigen::Vector10d dWz;

    Eigen::Vector10d h1;
    Eigen::Vector10d h2;
    Eigen::Vector10d h3;

    for (int i=0; i<3; i++)
    {
        vel_error[i] = _Reference_State.velocity_ref[i] - _DroneState.velocity[i];
        pos_error[i] = _Reference_State.position_ref[i] - _DroneState.position[i];
        pos_des[i]   = _Reference_State.position_ref[i];
    }

    // 这部分是h1、h2、h3 的计算
    for (int j=0; j<10; j++)
    {
        h1[j] = exp(-(pow(pos_error[0],2) + pow(vel_error[0],2) + pow(pos_des[0],2)) / 2 * 3 * 3);
        h2[j] = exp(-(pow(pos_error[0],2) + pow(vel_error[0],2) + pow(pos_des[0],2)) / 2 * 3 * 3);
        h3[j] = exp(-(pow(pos_error[0],2) + pow(vel_error[0],2) + pow(pos_des[0],2)) / 2 * 3 * 3);
        
    }
    
    // dW计算部分
    for (int i=0; i<10; i++)
    {
        dWx[i] = alpha_x * vel_error[0] * h1[i];
        dWy[i] = alpha_y * vel_error[1] * h2[i];
        dWz[i] = alpha_z * vel_error[2] * h3[i];

    }

    for (int i=0; i<10; i++)
    {
        Wx[i] = 0.02 * dWx[i];
        Wy[i] = 0.02 * dWy[i];
        Wz[i] = 0.02 * dWz[i];
    }

    // dW部分的积分有点问题，如果时间间隔为0.02，直接乘。
    
    double mid_cal_x;
    double mid_cal_y;
    double mid_cal_z;
    mid_cal_x = 0;
    mid_cal_y = 0;
    mid_cal_z = 0;
    for (int i=0; i<10; i++)
    {
        mid_cal_x = Wx[i] * h1[i];
        Neuron[0] = Neuron[0] + mid_cal_x;

        mid_cal_y = Wy[i] * h2[i];
        Neuron[1] = Neuron[1] + mid_cal_y;

        mid_cal_z = Wz[i] * h3[i];
        Neuron[2] = Neuron[2] + mid_cal_z;
    }
    return Neuron;
}
*/
