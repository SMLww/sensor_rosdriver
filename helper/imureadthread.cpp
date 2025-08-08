#include "imureadthread.h"
#include <QDebug>
#include "math.h"
#include "QDateTime"
#define PI acos(-1)
using namespace Eigen;

ImuReadThread::ImuReadThread(QObject*parent):QThread(parent)
{
    ImuPort = new QSerialPort();
    ImuTimer = new QTimer();
    OpenPort("/dev/ttyAMA3",115200); // 开启端口
}

ImuReadThread::~ImuReadThread()
{
    if(ImuPort->isOpen())
    {
        ImuPort->clear();
        ImuPort->close();
    }
    delete ImuPort;
    ImuTimer->stop();
    delete ImuTimer;
}


void ImuReadThread::OpenPort(const QString com, int baudrate)
{
    if(ImuPort->isOpen())
    {
        ImuPort->clear();
        ImuPort->close();
    }
    ImuPort->setPortName(com);
    if(!ImuPort->open(QIODevice::ReadWrite))
    {
        qDebug()<<"open serialport faliure";
        return;
    }
    qDebug()<<"open serialport success";

    ImuPort->setBaudRate(baudrate,QSerialPort::AllDirections);
    ImuPort->setDataBits(QSerialPort::Data8);
    ImuPort->setFlowControl(QSerialPort::NoFlowControl);
    ImuPort->setParity(QSerialPort::NoParity);
    ImuPort->setStopBits(QSerialPort::OneStop);
}

void ImuReadThread::ImuDataReceive()
{
    ImuTimer->start(5);
    QByteArray data = ImuPort->readAll();
    ImuDataBuf.append(data);
}

void ImuReadThread::ImuDataUpdate()
{
    ImuTimer->stop();
    //qDebug()<<"imuDateSize: "<<ImuDataBuf.size();
    if(ImuDataBuf.size() == 30 && ImuDataBuf.at(0) == 0x49 && ImuDataBuf.at(29)==0x4D)
    {
        short BITENUM[30];//根据规则转换为对应的有符号值
        for(int i = 0 ; i < 30 ; i++)
        {
              BITENUM[i]=(short)ImuDataBuf[i];
        }

        double Acc_X = (short)(BITENUM[11] << 8 | BITENUM[10])  * scaleAccel;
        double Acc_Y = (short)(BITENUM[13] << 8 | BITENUM[12]) * scaleAccel;
        double Acc_Z = (short)(BITENUM[15] << 8 | BITENUM[14]) * scaleAccel;
        double Gyro_X = (short)(BITENUM[17] << 8 | BITENUM[16]) * scaleAngleSpeed;
        double Gyro_Y = (short)(BITENUM[19] << 8 | BITENUM[18]) * scaleAngleSpeed;
        double Gyro_Z = (short)(BITENUM[21] << 8 | BITENUM[20]) * scaleAngleSpeed;
        double Mag_X = (short)(BITENUM[23] << 8 | BITENUM[22]) * scaleMag;
        double Mag_Y = (short)(BITENUM[25] << 8 | BITENUM[24]) * scaleMag;
        double Mag_Z = (short)(BITENUM[27] << 8 | BITENUM[26]) * scaleMag;

        double Acc[3] = {Acc_X - 0.0164, Acc_Y - 0.0574, Acc_Z - 0.0462};
        double Gyro[3] = {Gyro_X, Gyro_Y + 0.0014, Gyro_Z};
        double Mag[3] = {Mag_X + 3.9243, Mag_Y + 46.1862, Mag_Z + 113.5359};

        double Acc_X_1 = 0.9957 * Acc[0] + 0.0058 * Acc[1] + 0.0008 * Acc[2];
        double Acc_Y_1 = 0.9901 * Acc[1] - 0.0066 * Acc[2];
        double Acc_Z_1 = 0.9985 * Acc[2];

        double Gyro_X_1 = 0.0175 * Gyro[0] + 0.0001 * Gyro[1] - 0.0001 * Gyro[2];
        double Gyro_Y_1 = -0.0001 * Gyro[0] + 0.0175 * Gyro[1];
        double Gyro_Z_1 = 0.0001 * Gyro[0] - 0.0001 * Gyro[1] + 0.0175 * Gyro[2];

        double Mag_X_1 = -1.1308 * Mag[0] + 0.0180 * Mag[1] - 0.0123 * Mag[2];
        double Mag_Y_1 = -0.0045 * Mag[0] - 1.0672 * Mag[1] - 0.0194 * Mag[2];
        double Mag_Z_1 = -0.0114 * Mag[0] - 0.0111 * Mag[1] - Mag[2];


        //qDebug()<<Acc_X_1<<Acc_Y_1<<Acc_Z_1<<Gyro_X_1<<" "<<Gyro_Y_1<<" "<<Gyro_Z_1<<Mag_X_1<<Mag_Y_1<<Mag_Z_1;
        if (count < 10)
        {
            count++;
            double Data[9] = {Acc_X_1, Acc_Y_1, Acc_Z_1, Gyro_X_1, Gyro_Y_1, Gyro_Z_1, Mag_X_1, Mag_Y_1, Mag_Z_1};
            double Q[4];
            out_q(Data, Q);
            q0 = Q[0];
            q1 = Q[1];
            q2 = Q[2];
            q3 = Q[3];
            //qDebug()<<q0<<" "<<q1<<" "<<q2<<" "<<q3;
//            double Yaw = atan2(2. * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;
//            double Pitch = -asin(2. * (q1 * q3 - q0 * q2)) * 57.3;
//            double Roll = atan2(2. * q2*q3 + 2. * q0*q1, q0*q0 - q1*q1 - q2*q2 + q3*q3)* 57.3;
        }
        else
        {
            MahonyAHRSupdate(Gyro_X_1, Gyro_Y_1, Gyro_Z_1, Acc_X_1, Acc_Y_1, Acc_Z_1, Mag_X_1, Mag_Y_1, Mag_Z_1);
        }
    }

    ImuDataBuf.clear();
}

QByteArray ImuReadThread::GetImuData()
{
    QByteArray data = ImuAngData;
    return data;
}

// cross函数实现 -- 计算两个三维向量的叉积
double *ImuReadThread::cross(double *V_1, double *V_2)
{
    double *Output = new double[3];
    Output[0] = V_1[1] * V_2[2] - V_1[2] * V_2[1];
    Output[1] = V_1[2] * V_2[0] - V_1[0] * V_2[2];
    Output[2] = V_1[0] * V_2[1] - V_1[1] * V_2[0];
    //qDebug() << QString::asprintf("cross : %.2f %.2f %.2f", V_1[0], V_1[1], V_1[2]);
    return Output;
}

// Norm函数实现 -- 返回三维向量的模长
double ImuReadThread::Norm(double *V)
{
    return (sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]));
}

// qUtoV函数实现 -- 生成四元数
void ImuReadThread::qUtoV(double *u, double *v, double *Q)
{
    double Norm_U = Norm(u);
    double Nu[3] = {u[0] / Norm_U, u[1] / Norm_U, u[2] / Norm_U};
    double Norm_V = Norm(v);
    double Nv[3] = {v[0] / Norm_V, v[1] / Norm_V, v[2] / Norm_V};

    double N_C[3] = {v[0] + u[0], v[1] + u[1], v[2] + u[2]};

    if (Norm(N_C) == 0) {
        double V_3[3] = {Nu[1], Nu[2], Nu[0]};
        double *V_4 = cross(Nu, V_3);
        double Nor_V_4 = Norm(V_4);
        double NV_4[3] = {V_4[0] / Nor_V_4, V_4[1] / Nor_V_4, V_4[2] / Nor_V_4};
        Q[0] = 0;
        Q[1] = NV_4[0];
        Q[2] = NV_4[1];
        Q[3] = NV_4[2];

        delete[] V_4;
    }
    else {
        double Nor_N_C = Norm(N_C);
        double Half[3] = {N_C[0] / Nor_N_C, N_C[1] / Nor_N_C, N_C[2] / Nor_N_C};
        double *Q_T = cross(Nu, Half);
        Q[0] = Nu[0] * Half[0] + Nu[1] * Half[1] + Nu[2] * Half[2];
        Q[1] = Q_T[0];
        Q[2] = Q_T[1];
        Q[3] = Q_T[2];

        delete[] Q_T;
    }
}

// qMultiVec函数实现
void ImuReadThread::qMultiVec(double *vec, double *q, double *vector)
{
    double x_ = q[0] * vec[0] + q[2] * vec[2] - q[3] * vec[1];
    double y_ = q[0] * vec[1] + q[3] * vec[0] - q[1] * vec[2];
    double z_ = q[0] * vec[2] + q[1] * vec[1] - q[2] * vec[0];
    double w_ = -q[1] * vec[0] - q[2] * vec[1] - q[3] * vec[2];


    vector[0] = (x_ * q[0]) - (w_ * q[1]) - (y_ * q[3]) + (z_ * q[2]);
    vector[1] = (y_ * q[0]) - (w_ * q[2]) - (z_ * q[1] )+ (x_ * q[3]);
    vector[2] = (z_ * q[0]) - (w_ * q[3]) - (x_ * q[2]) + (y_ * q[1]);

}

// qMulyiQ函数实现
void ImuReadThread::qMulyiQ(double *p, double *q, double *qq)
{
    qq[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
    qq[1] = p[1] * q[0] + p[0] * q[1] - p[3] * q[2] + p[2] * q[3];
    qq[2] = p[2] * q[0] + p[3] * q[1] + p[0] * q[2] - p[1] * q[3];
    qq[3] = p[3] * q[0] - p[2] * q[1] + p[1] * q[2] + p[0] * q[3];
}

// out_q函数实现
void ImuReadThread::out_q(double *data, double *Q)
{
    double mag[3] = {data[6], data[7], data[8]};
    double acc[3] = {data[0], data[1], data[2]};
    double *V_x = cross(mag, acc);
    double V_X_T[3] = {V_x[0], V_x[1], V_x[2]};
    V_x[0] = -V_X_T[1];
    V_x[1] = V_X_T[0];


    double Norm_V_x = Norm(V_x);
    for (int i = 0; i < 3; i++) {
        V_x[i] = V_x[i] / Norm_V_x;
    }

    double *V_y = cross(acc, V_x);
    double Norm_V_y = Norm(V_y);
    for (int i = 0; i < 3; i++) {
        V_y[i] = V_y[i] / Norm_V_y;
    }

    double axis_X[3] = {1, 0, 0};
    double Q_x[4];
    qUtoV(V_x, axis_X, Q_x);

    double axis_Y[3];
    qMultiVec(V_y, Q_x, axis_Y);


    double y[3] = {0, 1, 0};

    double Q_y[4];
    qUtoV(axis_Y, y, Q_y);

    Q_x[0] = -Q_x[0];
    Q_y[0] = -Q_y[0];
    qMulyiQ(Q_x, Q_y, Q);
    Q[1] = -Q[1];
    Q[2] = -Q[2];
    Q[3] = -Q[3];

    if (Q[0] < 0) {
        for (int i = 0; i < 4; i++) {
            Q[i] = -Q[i];
        }
    }

    delete[] V_x;
    delete[] V_y;
    //return Q;
}


// MahonyAHRSupdate函数实现 -- 陀螺仪、加速度计、磁力计数据融合出姿态四元数
void ImuReadThread::MahonyAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz)
{
    //qDebug()<<q0<<" "<<q1<<" "<<q2<<" "<<q3;
    double norm;
    double hx, hy, hz, bx, bz;
    double vx, vy, vz, wx, wy, wz;
    double ex, ey, ez;

    // 计算相关乘积项
    double q0q0 = q0 * q0;
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q1 = q1 * q1;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;

    // 标准化测量值
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    norm = sqrt(mx * mx + my * my + mz * mz);
    mx = mx / norm;
    my = my / norm;
    mz = mz / norm;

    // 计算地磁场参考方向
    hx = 2 * mx * (0.5 - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5 - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5 - q1q1 - q2q2);

    // 计算bx和bz
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;

    // 计算重力和磁场估计方向
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);

    // 计算误差
    //ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    //ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    //ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    if(Is_Correct)
    {
        ez = (ax * vy - ay * vx) + DotPos[2];
        Is_Correct = false;
    }
    else {
        ez = (ax * vy - ay * vx);
    }

    // 积分误差并缩放积分增益
    exInt = exInt + ex * Ki * (1.0 / sampleFreq);
    eyInt = eyInt + ey * Ki * (1.0 / sampleFreq);
    ezInt = ezInt + ez * Ki * (1.0 / sampleFreq);

    // 调整陀螺仪测量值
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;

    // 积分四元数速率并归一化
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    // 归一化四元数
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    if (q0 < 0) {
        q0 = -q0;
        q1 = -q1;
        q2 = -q2;
        q3 = -q3;
    }

    Yaw = atan2(2. * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;
    Pitch = -asin(2. * (q1 * q3 - q0 * q2)) * 57.3;
    Roll = atan2(2. * q2*q3 + 2. * q0*q1, q0*q0 - q1*q1 - q2*q2 + q3*q3)* 57.3;
    // qDebug()<<Yaw<<" "<<Pitch<<" "<<Roll;

    // 加速度完成修正，投影至大地坐标系下。
    Matrix<double ,3 , 3> A,B,C,T;
    A<<     cos(Yaw*PI/180.0),-sin(Yaw*PI/180.0),0,
            sin(Yaw*PI/180.0),cos(Yaw*PI/180.0),0,
            0,0,1;
    B<<     cos(Pitch*PI/180.0),0,sin(Pitch*PI/180.0),
            0,1,0,
            -sin(Pitch*PI/180.0),0,cos(Pitch*PI/180.0);
    C<<     1,0,0,
            0,cos(Roll*PI/180.0),-sin(Roll*PI/180.0),
            0,sin(Roll*PI/180.0),cos(Roll*PI/180.0);

    T = A*B*C;
    Vector3d Vec_Acc;
    Vec_Acc<< ax, ay ,az-9.78;
    Vector3d Acc_To_N;
    Acc_To_N = T.inverse()*Vec_Acc;
    //积分求位置
    V[0] = 0.01*(Acc_To_N[0]+Acc_Next[0])/2.0;
    V[1] = 0.01*(Acc_To_N[1]+Acc_Next[1])/2.0;

    Acc_Next[0] = Acc_To_N[0];
    Acc_Next[1] = Acc_To_N[1];

    Pos[0] += 0.01*(V[0]+V_Next[0])/2.0;
    Pos[1] += 0.01*(V[1]+V_Next[1])/2.0;


    V_Next[0] = V[0];
    V_Next[1] = V[1];


    //qDebug()<<"PosX: "<<Pos[0]<<"PosY"<<Pos[1] ;
}


void ImuReadThread::MahonyPos(double Pos_X,double Pos_Y)
{
    if(Is_First)
    {
        Pos[0] = Pos_X;
        Pos[1] = Pos_Y;
        Pos_Next[0] = Pos[0];
        Pos_Next[1] = Pos[1];
        Is_First = false;
        QDateTime MyTime = QDateTime::currentDateTime();
        //QString Time_Str = MyTime.toString("yyyy-MM-dd HH:mm:ss.zzz");
        Time_Now = MyTime.toMSecsSinceEpoch();
        Time_Next = Time_Now;

    }
    else {
        double err_Px = Pos_X-Pos[0];
        double err_Py = Pos_Y-Pos[1];
        PxInt += Ki*err_Px/30.0;
        PyInt += Ki*err_Py/30.0;

        Pos[0] = Pos[0] + Kp*err_Px + PxInt;
        Pos[1] = Pos[1] + Kp*err_Py + PyInt;
        QDateTime MyTime = QDateTime::currentDateTime();
        Time_Now = MyTime.toMSecsSinceEpoch();
        double Dot_Time = (Time_Now-Time_Next)/1000.0;
        double V_x = (Pos[0]-Pos_Next[0])/Dot_Time;
        double V_y = (Pos[1]-Pos_Next[1])/Dot_Time;

        double err_Vx = V_x-V[0];
        double err_Vy = V_y-V[1];

        VxInt += Ki*err_Vx/30.0;
        VyInt += Ki*err_Vy/30.0;

        V[0] = V[0] + Kp*err_Vx + VxInt;
        V[1] = V[1] + Kp*err_Vy + VyInt;

    }

}

void ImuReadThread::GetUwb1Pos(double Pos_X,double Pos_Y)
{
    D_Uwb1_x = Pos_X-Pos[0];
    D_Uwb1_y = Pos_Y-Pos[1];

    Matrix<double ,3 , 3> A;
    A<<     cos(Yaw*PI/180.0),-sin(Yaw*PI/180.0),0,
            sin(Yaw*PI/180.0),cos(Yaw*PI/180.0),0,
            0,0,1;

    DotPos << D_Uwb1_x, D_Uwb1_y, 0;
    DotPos = A * DotPos;
    DotPos = DotPos/ DotPos.norm();
    Vector3d aix_y(0, 1, 0);
    DotPos = DotPos.cross(aix_y);

    //qDebug()<<DotPos[0]<<" "<<DotPos[1]<<" "<<DotPos[2];

    Is_Correct = true;
}

void ImuReadThread::run()
{
    connect(ImuPort,&QSerialPort::readyRead,this,&ImuReadThread::ImuDataReceive,Qt::DirectConnection);
    connect(ImuTimer,&QTimer::timeout,this,&ImuReadThread::ImuDataUpdate,Qt::DirectConnection);
    this->exec();
}

/*IMU 读取*/

//        double Acc_X,Acc_Y,Acc_Z,Gyro_X,Gyro_Y,Gyro_Z,Mag_X,Mag_Y,Mag_Z;

//        if ((uchar)ImuDataBuf[11] > 127){
//            Acc_X = (0xFF<<24 | 0xFF<<16 | (uchar)ImuDataBuf[11] << 8  | (uchar)ImuDataBuf[10]) * scaleAccel;
//        }else{
//            Acc_X = ((uchar)ImuDataBuf[11] << 8 | (uchar)ImuDataBuf[10])  * scaleAccel;
//        }
//        if ((uchar)ImuDataBuf[13] > 127){
//            Acc_Y = (0xFF<<24 | 0xFF<<16 | (uchar)ImuDataBuf[13] << 8  | (uchar)ImuDataBuf[12])  * scaleAccel;
//        }else{
//            Acc_Y = ((uchar)ImuDataBuf[13] << 8 | (uchar)ImuDataBuf[12]) * scaleAccel;
//        }
//        if ((uchar)ImuDataBuf[15] > 127){
//            Acc_Z = (0xFF<<24 | 0xFF<<16 | (uchar)ImuDataBuf[15] << 8  | (uchar)ImuDataBuf[14])  * scaleAccel;
//        }else{
//            Acc_Z = ((uchar)ImuDataBuf[15] << 8 | (uchar)ImuDataBuf[14])  * scaleAccel;
//        }


//        if ((uchar)ImuDataBuf[17] > 127){
//            Gyro_X = (0xFF<<24 | 0xFF<<16 | (uchar)ImuDataBuf[17] << 8  | (uchar)ImuDataBuf[16])  * scaleAngleSpeed;
//        }else{
//            Gyro_X = ((uchar)ImuDataBuf[17] << 8 | (uchar)ImuDataBuf[16]) * scaleAngleSpeed;
//        }
//        if ((uchar)ImuDataBuf[19] > 127){
//            Gyro_Y = (0xFF<<24 | 0xFF<<16 | (uchar)ImuDataBuf[19] << 8  | (uchar)ImuDataBuf[18])  * scaleAngleSpeed;
//        }else{
//            Gyro_Y = ((uchar)ImuDataBuf[19] << 8 | (uchar)ImuDataBuf[18])  * scaleAngleSpeed;
//        }
//        if ((uchar)ImuDataBuf[21] > 127){
//            Gyro_Z = (0xFF<<24 | 0xFF<<16 | (uchar)ImuDataBuf[21] << 8  | (uchar)ImuDataBuf[20])  * scaleAngleSpeed;
//        }else{
//            Gyro_Z = ((uchar)ImuDataBuf[21] << 8 | (uchar)ImuDataBuf[20])  * scaleAngleSpeed;
//        }


//        if ((uchar)ImuDataBuf[23] > 127){
//            Mag_X = (0xFF<<24 | 0xFF<<16 | (uchar)ImuDataBuf[23] << 8  | (uchar)ImuDataBuf[22]) * scaleMag;
//        }else{
//            Mag_X = ((uchar)ImuDataBuf[23] << 8 | (uchar)ImuDataBuf[22]) * scaleMag;
//        }
//        if ((uchar)ImuDataBuf[25] > 127){
//            Mag_Y = (0xFF<<24 | 0xFF<<16 | (uchar)ImuDataBuf[25] << 8  | (uchar)ImuDataBuf[24]) * scaleMag;
//        }else{
//            Mag_Y = ((uchar)ImuDataBuf[25] << 8 | (uchar)ImuDataBuf[24]) * scaleMag;
//        }
//        if ((uchar)ImuDataBuf[27] > 127){
//            Mag_Z =(0xFF<<24 | 0xFF<<16 | (uchar)ImuDataBuf[27] << 8  | (uchar)ImuDataBuf[26])  * scaleMag;
//        }else{
//            Mag_Z = ((uchar)ImuDataBuf[27] << 8 | (uchar)ImuDataBuf[26]) * scaleMag;
//        }
