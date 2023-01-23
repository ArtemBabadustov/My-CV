#include "mainwindow.h"
#include "qglobal.h"
#include "qnamespace.h"
#include "qserialport.h"
#include "qstringalgorithms.h"
#include "ui_mainwindow.h"
#include "QPalette"
#include <QPlainTextEdit>



#include <My_additions.h>

QString test = 0;
qint8 lights = 0;
QByteArray output;
const char lights_on[] = {char(0b10101010), char(0b10101010), char(0b10011011), char(0) };
//const QByteArray lights_on = {};
const char lights_off[] = {char(0b10101010), char(0b10101010), char(0b10011100), char(0) };
// const QByteArray lights_off = {};
bool custom_message_in_progress = false;
int custom_message_size;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    serial = new QSerialPort(this);
    serial->setPortName("com6");
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->open(QIODevice::ReadWrite);

    connect(serial, SIGNAL(readyRead()), this, SLOT(serialRecieve()));

}


MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_clicked()
{
    QByteArray re;
    re = ui->plainTextEdit->toPlainText().toLatin1();
    re.push_front(qint8(re.size()));
    re.push_front(qint8(60));
    re.push_front(qint8(170));
    re.push_front(qint8(170));
    serial->write(re);
    ui->plainTextEdit->clear();
}

void MainWindow::on_plainTextEdit_textChanged()
{

}

void MainWindow::serialRecieve()
{
    QByteArray from_sim;
    short peleng_mo = -200;
    signed char elevation_angle_mo = 200;
    unsigned short dist_mo = 20000;
    short peleng_mds = 200;
    unsigned short dist_mds = 20000;
    unsigned char new_data = 0;
    char temp[2];

    if (serial->bytesAvailable() >= 14)
    {
        from_sim = serial->read(14);
        if (from_sim[0] == char(0b10101010) && from_sim[1] == char(0b10101010))
        {
            temp[0] = from_sim[2];
            temp[1] = from_sim[3];
            peleng_mo = *reinterpret_cast<short*>(temp);
            ui->peleng_mo->setPlainText(QString::number(peleng_mo));

            temp[0] = from_sim[4];
            //temp[1] = from_sim[5];
            elevation_angle_mo = *reinterpret_cast<signed char*>(temp);
            ui->elevation_angle_mo->setPlainText(QString::number(elevation_angle_mo));

            temp[0] = from_sim[5];
            temp[1] = from_sim[6];
            dist_mo = *reinterpret_cast<unsigned short*>(temp);
            ui->dist_mo->setPlainText(QString::number(dist_mo));

            temp[0] = from_sim[7];
            temp[1] = from_sim[8];
            peleng_mds = *reinterpret_cast<short*>(temp);
            ui->peleng_mds->setPlainText(QString::number(peleng_mds));


            temp[0] = from_sim[9];
            temp[1] = from_sim[10];
            dist_mds = *reinterpret_cast<short*>(temp);
            ui->dis_mds->setPlainText(QString::number(dist_mds));

            temp[0] = from_sim[11];
            //temp[1] = from_sim[10];
            new_data = *reinterpret_cast<unsigned char*>(temp);

            if(new_data == 1)
            {
                ui->Status_window->appendPlainText("MDS is unreachable");
            }
            temp[0] = from_sim[12];
            temp[1] = from_sim[13];

            if(custom_message_in_progress && (temp[0] != 0 || temp[1] != 0))
            {
                if(serial->bytesAvailable() < custom_message_size)
                {

                    custom_message_size -= serial->bytesAvailable();

                    from_sim += serial->read(serial->bytesAvailable());

                    custom_message_in_progress = true;

                    return;
                }
                else
                {
                    from_sim += serial->read(custom_message_size);
                    custom_message_size = 0;
                    custom_message_in_progress = false;
                    QByteArray from_sim_temp = from_sim;
                    from_sim_temp.remove(0, 2);
                    ui->Data_from_sim->appendPlainText(from_sim_temp);
                    from_sim.clear();
                    ui->Data_from_sim->appendPlainText("Custom message has been resieved");
                    return;
                }
            }
            else if (temp[0] == char(0b10011011) && temp[1] == char(0))
            {
                ui->Data_from_sim->appendPlainText("Lights are on!");
            }
            else if (temp[0] == char(0b10011100) && temp[1] == char(0))
            {
                ui->Data_from_sim->appendPlainText("Lights are off!");
            }
            else if(temp[0] == char(qint8(60)))
            {
                //std::cout << "Custom message recieved" << std::endl;

                custom_message_size = temp[1];
                ui->Data_from_sim->appendPlainText("Cuctom message size: " + QString::number(custom_message_size));
                custom_message_in_progress = true;
            }
        }
        else
        {
            ui->Data_from_sim->appendPlainText("Transmission error!");
        }
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    ui->Status_window->clear();
    ui->Data_from_sim->clear();
    ui->dis_mds->clear();
    ui->dist_mo->clear();
    ui->elevation_angle_mo->clear();
    ui->peleng_mds->clear();
    ui->peleng_mo->clear();
}

void MainWindow::on_Lights_clicked()
{

    if (lights == 0)
    {
        lights = 1;
        serial->write(lights_on, 4);
       // ui->Status_window->appendPlainText(QString(serial->bytesToWrite()));

    }
    else
    {
        lights = 0;
        serial->write(lights_off, 4);
    }
}

