#include "uwb1_data.h"
#include "QDebug"
#include "QDateTime"

Uwb1_Data::Uwb1_Data(QObject *parent) : QThread (parent)
{
    Uwb1_Port = new QSerialPort();
    Uwb1_Timer =  new QTimer();

    Open_Serial("/dev/ttyUSB0",115200);
}

Uwb1_Data::~Uwb1_Data()
{
    delete Uwb1_Port;
    delete Uwb1_Timer;
}


void Uwb1_Data::Open_Serial(const QString Com, int Baudrate)
{
    if(Uwb1_Port->isOpen())
    {
        Uwb1_Port->clear();
        Uwb1_Port->close();
    }
    Uwb1_Port->setPortName(Com);
    if(!Uwb1_Port->open(QIODevice::ReadWrite))
    {
        qDebug()<<"Open Uwb1 Port Faliure";
        return;
    }
    qDebug()<<"Open Uwb1 Port Success";
    Uwb1_Port->setBaudRate(Baudrate,QSerialPort::AllDirections);
    Uwb1_Port->setDataBits(QSerialPort::Data8);
    Uwb1_Port->setFlowControl(QSerialPort::NoFlowControl);
    Uwb1_Port->setParity(QSerialPort::NoParity);
    Uwb1_Port->setStopBits(QSerialPort::OneStop);
}

void Uwb1_Data::Uwb1_Data_Receive()
{
    Uwb1_Timer->start(6);
    QByteArray Data = Uwb1_Port->readAll();
    Uwb1_Port->write(Buf);
    Uwb1_Buf.append(Data);
    //qDebug()<<Lidar_Buf.size();
    // if(Lidar_Buf.size()>106)
    //     Lidar_Unpack();
}


void Uwb1_Data::Uwb1_Unpack()
{
    Uwb1_Timer->stop();
    QDateTime MyTime = QDateTime::currentDateTime();
    //QString Time_Str = MyTime.toString("yyyy-MM-dd HH:mm:ss.zzz");
    qint64 t = MyTime.toMSecsSinceEpoch();
    //qDebug()<<t;

    if(t- t_Next>20)
    {
        int dis_A,dis_B,dis_C;
        t_Next = t;
        for(int i = 0; i < Uwb1_Buf.size() - 11; i++)
        {
            if ((uchar)Uwb1_Buf.at(i) == 0xAC && (uchar)Uwb1_Buf.at(i+1) == 0xDA && (uchar)Uwb1_Buf.at(i+2) == 0x00 && (uchar)Uwb1_Buf.at(i+5) == 0x03)
            {
                dis_A = (uchar)Uwb1_Buf[i+6] << 8  | (uchar)Uwb1_Buf[i + 7];
                dis_B = (uchar)Uwb1_Buf[i+8] << 8  | (uchar)Uwb1_Buf[i + 9];
                //dis_C = (uchar)Lidar_Buf[i+10] << 8  | (uchar)Lidar_Buf[i + 11];
                double pos_X_1 = (dis_A * dis_A - dis_B * dis_B + long_X * long_X) / (2 * long_X);
                if((dis_A * dis_A - pos_X_1 * pos_X_1)>0)
                {
                double pos_Y_1 = sqrt(dis_A * dis_A - pos_X_1 * pos_X_1);
                //double pos_Y_2 = (dis_A*dis_A-dis_C*dis_C+long_Y*long_Y)/(2*long_Y);


                //double pos_X = (pos_X_1+pos_X_2)/2.0;
                //double pos_Y = (pos_Y_1+pos_Y_2)/2.0;
                //qDebug()<<"uwb1"<<pos_X_1<<", "<<pos_Y_1;
                //qDebug()<<dis_A<<" "<<dis_B<<" "<<dis_C;
                emit(Point1_Send(pos_X_1, pos_Y_1));
                }
            }
        }

    }

//    double PosX,PosY;
//    for(int i = 0; i<Lidar_Buf.size()-13;i++)

//    {
//        if ((uchar)Lidar_Buf.at(i) == 0xCA && (uchar)Lidar_Buf.at(i+1) == 0xDA && (uchar)Lidar_Buf.at(i+2) == 0x00 && (uchar)Lidar_Buf.at(i+3) == 0x03 &&(uchar)Lidar_Buf.at(i+7) == 0x01 && (uchar)Lidar_Buf.at(i+9) == 0x07)
//        {
//            i += 10;
//            qDebug()<<"begin";
//            //printf("%x",str[i+3]);
//            //printf("\n");
//            if ((uchar)Lidar_Buf[i] > 127)
//            {
//                PosX =0xFF<<24 | 0xFF<<16 | (uchar)Lidar_Buf[i] << 8  | (uchar)Lidar_Buf[i + 1];

//            }
//            else
//            {
//                PosX = (uchar)Lidar_Buf[i] << 8 | (uchar)Lidar_Buf[i + 1];
//            }
//            Real_PosX = Real_PosX+(PosX-Real_PosX)*0.1;
//            if ((uchar)Lidar_Buf[i + 2] > 127)
//            {
//                PosY =0xFF<<24 | 0xFF<<16 | (uchar)Lidar_Buf[i + 2] << 8 | (uchar)Lidar_Buf[i + 3];
//            }
//            else
//            {
//                PosY = (uchar)Lidar_Buf[i + 2] << 8 | (uchar)Lidar_Buf[i + 3];
//            }
//            Real_PosY = Real_PosY+(PosY-Real_PosY)*0.1;

//            //return true;
//            break;

//        }



//        //int k = Lidar_Buf.size();

//    }
    Uwb1_Buf.clear();
//    qDebug()<<"Posx"<<Real_PosX<<"PosY"<<Real_PosY;
//    if(PosX<5&&PosY<5)
//        return;
//    emit(Point_Send(Real_PosX/100.0,Real_PosY/100.0));
}

void Uwb1_Data::run()
{
    connect(Uwb1_Port, &QSerialPort::readyRead, this, &Uwb1_Data::Uwb1_Data_Receive, Qt::DirectConnection);
    connect(Uwb1_Timer, &QTimer::timeout, this, &Uwb1_Data::Uwb1_Unpack, Qt::DirectConnection);
    this->exec();
}
