#include "mainwindow.h"
#include "qserialport.h"
#include "ui_mainwindow.h"
#include "QPalette"
#include <QPlainTextEdit>


//#include <My_additions.h>
//#include<Global.cpp>
QByteArray output;
QByteArray from_sim;
bool custom_message_in_progress = false;
int custom_message_size;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    serial = new QSerialPort(this);
    serial->setPortName("com9");
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


void MainWindow::serialRecieve()
{
    if(custom_message_in_progress)
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
    else if (serial->bytesAvailable() >= 4)
    {
        QByteArray sink = serial->read(2);
        from_sim += serial->read(2);
        int size = from_sim.size();
        if (sink[0] == char(0b10101010) && sink[1] == char(0b10101010))
        {
            ui->Data_from_sim->appendPlainText("Transmission has occured!");
            if (from_sim[size - 1 - 1] == char(0b10011011) && from_sim[size - 1] == char(0) )
            {
                ui->Data_from_sim->appendPlainText("Turn the lights on!");
                serial->write(sink + from_sim);
                from_sim.clear();
            }

            if (from_sim[size - 1 - 1] == char(0b10011100) && from_sim[size - 1] == char(0) )
            {
                ui->Data_from_sim->appendPlainText("Turn the lights off!");
                serial->write(sink + from_sim);
                from_sim.clear();
            }
            if(from_sim[size - 1 - 1] == char(qint8(60)))
            {
                //std::cout << "Custom message recieved" << std::endl;

                custom_message_size = from_sim[size-1];
                 ui->Data_from_sim->appendPlainText("Cuctom message size: " + QString::number(custom_message_size));
                if(serial->bytesAvailable() < custom_message_size)
                {
                    custom_message_size -= serial->bytesAvailable();
                    from_sim += serial->read(serial->bytesAvailable());
                    custom_message_in_progress = true;
                }
                else
                {
                    from_sim += serial->read(custom_message_size);
                    custom_message_size = 0;
                    custom_message_in_progress = false;
                }
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
    ui->Data_from_sim->clear();
}

