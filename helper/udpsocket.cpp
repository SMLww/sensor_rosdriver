#include "udpsocket.h"
#include<QDebug>

UDPSocket::UDPSocket(QObject *parent):QThread(parent)
{
    UdpSock = new QUdpSocket();


    if(Bind(BindIP, BindPort))
    {
        qDebug()<<"Bind Ip Successes ";
    }
}

UDPSocket::~UDPSocket()
{
    delete UdpSock;
}

/*
 * UDP端口绑定
*/
bool UDPSocket::Bind(QString IP, ushort Port)
{
    return UdpSock->bind(QHostAddress(IP), Port);
}

/*
 * UDP数据读取
*/
void UDPSocket::UdpDataReceive()
{
    QByteArray DataBuf;
    while(UdpSock->hasPendingDatagrams())
    {
        DataBuf.resize(UdpSock->pendingDatagramSize());
        UdpSock->readDatagram(DataBuf.data(),DataBuf.size());
    }
    if(DataBuf.at(0) == 0x00 && DataBuf.size() == 27)
    {
        emit(MotorControlSignal(DataBuf));
    }
    //qDebug()<<"DataBuf: "<<DataBuf.toHex();
}

void UDPSocket::UdpDateSend(const QByteArray Data, QString IP,  ushort Port)
{
    UdpSock->writeDatagram(Data, QHostAddress(IP), Port);
}

void UDPSocket::run()
{
    connect(UdpSock,&QUdpSocket::readyRead,this,&UDPSocket::UdpDataReceive);
    this->exec();
}
