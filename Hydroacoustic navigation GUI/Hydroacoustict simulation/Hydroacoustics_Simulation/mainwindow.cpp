#include "mainwindow.h"
#include "qglobal.h"
#include "qserialport.h"
#include "qstringalgorithms.h"
#include "ui_mainwindow.h"
#include "QPalette"
#include <QPlainTextEdit>
#include <QTime>
#include <QTimer>
#include <iterator>
#include <cmath>
#include <iostream>

#define PI 3.14
QByteArray from_mds;
QByteArray from_rov;
int x_mo = 0;
int y_mo = 0;
int z_mo = 0;
int x_mds = 0;
int y_mds = 0;
int z_mds = 0;

QByteArray send;
unsigned short dist_mo = 0;
uint8_t elevation_angle_mo = 0;
short peleng_mo = 0;
unsigned short dist_mds = 0;
short peleng_mds = 0;
unsigned char new_data = 0;
qint8 send_1[14];
bool mds_coordinates_updated = false;
bool mo_coordinates_updated = false;
bool custom_message_in_progress = false;
bool custom_message_transmission = false;
bool custom_responce_in_progress = false;
bool custom_responce_transmission = false;
int current_custom_message_size = 0;
int current_custom_responce_size = 0;
int custom_message_size = 0;
int custom_responce_size = 0;

float distance = 0; // distance between ROV and MDS in meters

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    serial = new QSerialPort(this);
    serial->setPortName("com7");
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->open(QIODevice::ReadWrite);

    connect(serial, SIGNAL(readyRead()), this, SLOT(serialRecieve()));

    serial_1 = new QSerialPort(this);
    serial_1->setPortName("com8");
    serial_1->setBaudRate(QSerialPort::Baud9600);
    serial_1->setDataBits(QSerialPort::Data8);
    serial_1->setParity(QSerialPort::NoParity);
    serial_1->setStopBits(QSerialPort::OneStop);
    serial_1->setFlowControl(QSerialPort::NoFlowControl);
    serial_1->open(QIODevice::ReadWrite);

    connect(serial_1, SIGNAL(readyRead()), this, SLOT(serialRecieve_1()));

    timer_responce = new QTimer(this);
    connect(timer_responce, SIGNAL(timeout()), this, SLOT(update_responce()));

    timer_timeout = new QTimer(this);
    connect(timer_timeout, SIGNAL(timeout()), this, SLOT(update_timeout()));
    timer_timeout->start(4000);

    timer_transmission = new QTimer(this);
    connect(timer_timeout, SIGNAL(timeout()), this, SLOT(update_transmission()));
}

MainWindow::~MainWindow()
{
    delete ui;
}
//void MainWindow::send(QByteArray a)
//{
//    a.push_front(qint8(170));
//    a.push_front(qint8(170));
//    serial->write(a);
//    //ui->plainTextEdit_2->appendPlainText(a);
//}
//void MainWindow::send_1(QByteArray a)
//{
//    a.push_front(qint8(170));
//    a.push_front(qint8(170));
//    serial->write(a);
//   // ui->plainTextEdit_2->appendPlainText(a);
//}

void MainWindow::transmission_delay(int n) // this function delays transmission based on distance and number of bytes transmitted
{
    // n is the number of bytes transmitted
    distance = sqrt(x_mds*x_mds + y_mds*y_mds + z_mds*z_mds);
    float delay = n * 8 * (40 + distance/1500 * 1000); // 40 ms is the time one bit is emitted
    ui->Trans_info->appendPlainText("Current distance is: " + QString::number(distance) + "m");
    ui->Trans_info->appendPlainText("Transmission will take: " + QString::number(delay, 'g', 6) + "ms");
    QTime dieTime= QTime::currentTime().addMSecs(delay);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}




//void MainWindow::on_Change_dist_clicked()
//{
//    distance = ui->new_dist->toPlainText().toInt();
//    if (distance != 0)
//    {
//        ui->curr_dist->setPlainText(QString::number(distance));
//    }
//    ui->new_dist->clear();
//}


void MainWindow::on_mds_x_textChanged()
{
    int temp = ui->mds_x->toPlainText().toInt();

    if (temp != 0)
    {
        x_mds = temp;
    }
    else
    {
        ui->Trans_info->appendPlainText("Ошибка! Не правильное значение координаты х МДС!");
    }
}


void MainWindow::on_mds_y_textChanged()
{
    int temp = ui->mds_y->toPlainText().toInt();

    if (temp != 0)
    {
        y_mds = temp;
    }
    else
    {
        ui->Trans_info->appendPlainText("Ошибка! Не правильное значение координаты y МДС!");
    }

}


void MainWindow::on_mds_z_textChanged()
{
    int temp = ui->mds_z->toPlainText().toInt();
    if (temp != 0)
    {
        z_mds = temp;
    }
    else
    {
        ui->Trans_info->appendPlainText("Ошибка! Не правильное значение координаты z МДС!");
    }
}


void MainWindow::on_mo_x_textChanged()
{
    int temp = ui->mo_x->toPlainText().toInt();
    if (temp != 0)
    {
        x_mo = temp;
    }
    else
    {
        ui->Trans_info->appendPlainText("Ошибка! Не правильное значение координаты х МО!");
    }
}


void MainWindow::on_mo_y_textChanged()
{
    int temp = ui->mo_y->toPlainText().toInt();
    if (temp != 0)
    {
        y_mo = temp;
    }
    else
    {
        ui->Trans_info->appendPlainText("Ошибка! Не правильное значение координаты y МО!");
    }
}


void MainWindow::on_mo_z_textChanged()
{
    int temp = ui->mo_z->toPlainText().toInt();
    if (temp != 0)
    {
        z_mo = temp;
    }
    else
    {
        ui->Trans_info->appendPlainText("Ошибка! Не правильное значение координаты z МО!");
    }
}

void MainWindow::serialRecieve()
{
    if(custom_message_in_progress)
    {
        if(serial->bytesAvailable() < custom_message_size)
        {
            custom_message_size -= serial->bytesAvailable();
            from_rov += serial->read(serial->bytesAvailable());
            custom_message_in_progress = true;
        }
        else
        {
            from_rov += serial->read(custom_message_size);
            custom_message_size = 0;
            custom_message_in_progress = false;
            QByteArray from_rov_temp = from_rov;
            from_rov_temp.remove(0, 2);
            ui->Trans_info->appendPlainText("Custom transmission: " + from_rov_temp);
            //ui->Trans_info->appendPlainText("Custom message fully recieved");
        }
    }
    else if (serial->bytesAvailable() >= 4)
    {
        QByteArray sink = serial->read(2);
        from_rov += serial->read(2);
        int size = from_rov.size();
        if (sink[0] == char(0b10101010) && sink[1] == char(0b10101010))
        {
            ui->tr_from_rov->appendPlainText("Transmission has occured!");
            if (from_rov[size - 1 - 1] == char(0b10011011) && from_rov[size - 1] == char(0) )
            {
                ui->tr_from_rov->appendPlainText("Turn the lights on!");
            }

            if (from_rov[size - 1 - 1] == char(0b10011100) && from_rov[size - 1] == char(0))
            {
                ui->tr_from_rov->appendPlainText("Turn the lights off!");
            }
            if(from_rov[size - 1 - 1] == char(qint8(60)))
            {
                std::cout << "Custom message recieved" << std::endl;

                custom_message_size = from_rov[size-1];
                ui->Trans_info->appendPlainText("Custom message size: " + QString::number(custom_message_size));
                if(serial->bytesAvailable() < custom_message_size)
                {
                    custom_message_size -= serial->bytesAvailable();
                    from_rov += serial->read(serial->bytesAvailable());
                    custom_message_in_progress = true;
                }
                else
                {
                    from_rov += serial->read(custom_message_size);
                    custom_message_size = 0;
                    custom_message_in_progress = false;
                }
              }

        }
        else
        {
            ui->tr_from_rov->appendPlainText("Transmission error!");
            from_rov.remove(size - 1 - 1, 2);
        }
    }
}

void MainWindow::serialRecieve_1()
{
    if(custom_responce_in_progress)
    {
        if(serial_1->bytesAvailable() < custom_responce_size)
        {
            custom_responce_size -= serial_1->bytesAvailable();
            from_mds += serial_1->read(serial_1->bytesAvailable());
            custom_responce_in_progress = true;
        }
        else
        {
            from_mds += serial_1->read(custom_responce_size);
            custom_responce_size = 0;
            custom_responce_in_progress = false;
            QByteArray from_mds_temp = from_mds;
            from_mds_temp.remove(0, 2);
            ui->Trans_info->appendPlainText("Custom responce: " + from_mds_temp);
            //ui->Trans_info->appendPlainText("Custom message fully recieved");
        }
    }
    else if (serial_1->bytesAvailable() >= 4)
    {
        QByteArray sink = serial_1->read(2);
        from_mds += serial_1->read(2);
        int size = from_mds.size();
        if (sink[0] == char(0b10101010) && sink[1] == char(0b10101010))
        {
            ui->tr_from_rov->appendPlainText("Transmission has occured!");
            if (from_mds[size - 1 - 1] == char(0b10011011) && from_mds[size - 1] == char(0))
            {
                ui->tr_from_mds->appendPlainText("Lights are on!");
            }
            if (from_mds[size - 1 - 1] == char(0b10011100) && from_mds[size - 1] == char(0))
            {

                ui->tr_from_mds->appendPlainText("Lights are off!");
            }
            if(from_mds[size - 1 - 1] == char(qint8(60)))
            {
                std::cout << "Custom responce recieved" << std::endl;

                custom_responce_size = from_mds[size-1];
                ui->Trans_info->appendPlainText("Custom responce size: " + QString::number(custom_responce_size));
                if(serial_1->bytesAvailable() < custom_responce_size)
                {
                    custom_responce_size -= serial_1->bytesAvailable();
                    from_mds += serial->read(serial_1->bytesAvailable());
                    custom_responce_in_progress = true;
                }
                else
                {
                    from_mds += serial_1->read(custom_responce_size);
                    custom_responce_size = 0;
                    custom_responce_in_progress = false;
                    QByteArray from_mds_temp = from_mds;
                    from_mds_temp.remove(0, 2);
                    ui->Trans_info->appendPlainText("Custom responce: " + from_mds_temp);

                }
              }
            //serial->write(from_mds);
        }
        else
        {
            ui->tr_from_mds->appendPlainText("Transmission error!");
            from_mds.remove(size - 1 - 1, 2);
        }
    }
}

void MainWindow::update_transmission()
{
    timer_transmission->stop();
    if(from_rov.size() != 0)
    {
        if (custom_message_transmission)
        {
            if(current_custom_message_size > 2)
            {
                send.append(from_rov[0]);
                send.append(from_rov[1]);
                serial_1->write(send);
                send.clear();
                from_rov.remove(0, 2);
                current_custom_message_size -=2;
                return;
            }
            else
            {
                for (int i = 0; i < current_custom_message_size; i++)
                {
                    send.append(from_rov[i]);
                }
                serial_1->write(send);
                send.clear();
                from_rov.remove(0, current_custom_message_size);
                custom_message_transmission = false;
                current_custom_message_size = 0;
                ui->Trans_info->appendPlainText("Custom message transmission has ended!");
                return;
            }

        }
        else
        {
            send.append(qint8(170));
            send.append(qint8(170));
            send.append(from_rov[0]);
            send.append(from_rov[1]);
            if(from_rov[0] == char(qint8(60)))
            {
                custom_message_transmission = true;
                current_custom_message_size = from_rov[1];
                ui->Trans_info->appendPlainText("Custom message recieved!");
            }
            serial_1->write(send);
            send.clear();
            from_rov.remove(0, 2);
        }
    }
}

void MainWindow::update_responce()
{
       std::cout << "Responce timer has triggered" << std::endl;
       timer_responce->stop();
       bool coordinates_not_zero = ((x_mo != 0 && y_mo != 0) && z_mo != 0) && ((x_mds != 0 && y_mds != 0) && z_mds != 0);
        mds_coordinates_updated = true;
    if (coordinates_not_zero)
    {

         dist_mo = sqrt(x_mo*x_mo + y_mo*y_mo + z_mo*z_mo);
         elevation_angle_mo = acos(double(y_mo)/dist_mo) * double (180.0 / PI);
        peleng_mo = acos(double(x_mo)/sqrt(x_mo*x_mo + y_mo*y_mo)) * 180.0 / PI;
         dist_mds = sqrt(x_mds*x_mds + y_mds*y_mds + z_mds*z_mds);
         peleng_mds = acos(double(x_mds)/sqrt(x_mds*x_mds + y_mds*y_mds)) * 180.0 / PI;
         new_data = 3;

        send_1[0] = qint8(170);
        send_1[1] = qint8(170);
        send_1[2] = *reinterpret_cast<qint8*>(&peleng_mo);
        send_1[3] = *(reinterpret_cast<qint8*>(&peleng_mo) + 1);
        send_1[4] = *reinterpret_cast<qint8*>(&elevation_angle_mo);
        send_1[5] = *reinterpret_cast<qint8*>(&dist_mo);
        send_1[6] = *(reinterpret_cast<qint8*>(&dist_mo) + 1);
        send_1[7] = *reinterpret_cast<qint8*>(&peleng_mds);
        send_1[8] = *(reinterpret_cast<qint8*>(&peleng_mds) + 1);
        send_1[9] = *reinterpret_cast<qint8*>(&dist_mds);
        send_1[10] = *(reinterpret_cast<qint8*>(&dist_mds) + 1);
        send_1[11] = *reinterpret_cast<qint8*>(&new_data);
        if (from_mds.size() != 0)
        {
            if (custom_responce_transmission)
            {
                if(current_custom_responce_size > 2)
                {
                    send_1[12] = from_mds[0];
                    send_1[13] = from_mds[1];

                    from_mds.remove(0, 2);
                    current_custom_responce_size -=2;
                    return;
                }
                else
                {
                    for (int i = 0; i < current_custom_responce_size; i++)
                    {
                        send_1[12+i] = from_mds[0];
                        from_mds.remove(0, 1);
                    }
                    //serial_1->write(send);
                    //send.clear();
                    //from_rov.remove(0, current_custom_message_size);
                    custom_message_transmission = false;
                    current_custom_message_size = 0;
                    ui->Trans_info->appendPlainText("Custom responce transmission has ended!");
                    return;
                }

            }
            else
            {
            if(from_mds[0] == char(qint8(60)))
            {
                custom_responce_transmission = true;
                current_custom_responce_size = from_mds[1];
                ui->Trans_info->appendPlainText("Custom responce recieved!");
            }
            send_1[12] = from_mds[0];
            send_1[13] = from_mds[1];
            from_mds.remove(0, 2);
            }
        }
        else
        {
            send_1[12] = 0;
            send_1[13] = 0;
        }
        for(int i = 0; i < 14; i++)
        {
            send.push_back(send_1[i]);
        }
        serial->write(send);
        send_1[12] = 0;
        send_1[13] = 0;
    }
    else
    {
        ui->Trans_info->appendPlainText("Not all of the coordinates are diled in!");
        std::cout << "Responce timer! Not all of the coordinates are diled in!" << std::endl;
    }
//    std::cout << "Four seconds elapsed exit!" << std::endl;
    //timer_timeout->start(4000);
    std::cout << "transmission time: " << 2*sqrt(x_mds*x_mds + y_mds*y_mds + z_mds*z_mds)/1500*1000 << " ms." << std::endl;
    //timer_responce->start(2*sqrt(x_mds*x_mds + y_mds*y_mds + z_mds*z_mds)/1500*1000);
    send.clear();
}



void MainWindow::update_timeout()
{
    std::cout << "Timeout timer has triggered!" << std::endl;
    timer_responce->stop();
    timer_transmission->stop();

    bool coordinates_not_zero = ((x_mo != 0 && y_mo != 0) && z_mo != 0) && ((x_mds != 0 && y_mds != 0) && z_mds != 0);

    if (coordinates_not_zero)
    {
        if (!mds_coordinates_updated)
        {
            dist_mo = sqrt(x_mo*x_mo + y_mo*y_mo + z_mo*z_mo);
            elevation_angle_mo = acos(double(y_mo)/dist_mo) * double (180.0 / PI);
            peleng_mo = acos(double(x_mo)/sqrt(x_mo*x_mo + y_mo*y_mo)) * 180.0 / PI;
            new_data = 1;
            send_1[0] = qint8(170);
            send_1[1] = qint8(170);
            send_1[2] = *reinterpret_cast<qint8*>(&peleng_mo);
            send_1[3] = *(reinterpret_cast<qint8*>(&peleng_mo) + 1);
            send_1[4] = *reinterpret_cast<qint8*>(&elevation_angle_mo);
            send_1[5] = *reinterpret_cast<qint8*>(&dist_mo);
            send_1[6] = *(reinterpret_cast<qint8*>(&dist_mo) + 1);
            send_1[7] = *reinterpret_cast<qint8*>(&peleng_mds);
            send_1[8] = *(reinterpret_cast<qint8*>(&peleng_mds) + 1);
            send_1[9] = *reinterpret_cast<qint8*>(&dist_mds);
            send_1[10] = *(reinterpret_cast<qint8*>(&dist_mds) + 1);
            send_1[11] = *reinterpret_cast<qint8*>(&new_data);
            if (from_mds.size() != 0)
            {
                send_1[12] = 0;
                send_1[13] = 0;
                //rom_mds.clear();
            }
            else
            {
                send_1[12] = 0;
                send_1[13] = 0;
            }
            for(int i = 0; i < 14; i++)
            {
                send.push_back(send_1[i]);
            }
            serial->write(send);
            send.clear();
        }
        if (2*sqrt(x_mds*x_mds + y_mds*y_mds + z_mds*z_mds)/1500*1000 < 4000)
        {
        timer_transmission->start(sqrt(x_mds*x_mds + y_mds*y_mds + z_mds*z_mds)/1500*1000 + 80);
        timer_responce->start(2*sqrt(x_mds*x_mds + y_mds*y_mds + z_mds*z_mds)/1500*1000);
        }
        else
        {
            timer_responce->stop();
            timer_transmission->stop();
            std::cout << "MDS is out of reach! Transmission time: " << 2*sqrt(x_mds*x_mds + y_mds*y_mds + z_mds*z_mds)/1500*1000 << std::endl;
            ui->Trans_info->appendPlainText("MDS is out of reach!");
        }
        mds_coordinates_updated = false;
        std::cout << "transmission time: " << 2*sqrt(x_mds*x_mds + y_mds*y_mds + z_mds*z_mds)/1500*1000 << " ms." << std::endl;
    }
    else
    {
        std::cout << "Timeout timer! Not all of the coordinates are diled in!" << std::endl;
        ui->Trans_info->appendPlainText("Not all of the coordinates are diled in!");
    }
}

void MainWindow::on_pushButton_clicked()
{
    ui->Trans_info->clear();
    ui->tr_from_mds->clear();
    ui->tr_from_rov->clear();
}

