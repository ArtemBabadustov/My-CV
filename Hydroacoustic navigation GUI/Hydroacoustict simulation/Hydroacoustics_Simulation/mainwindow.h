#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTimer>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    QSerialPort* serial;
    QSerialPort* serial_1;
    QTimer *timer_responce;
    QTimer* timer_timeout;
    QTimer* timer_transmission;
    //void send(QByteArray a);
    //void send_1(QByteArray a);
private slots:
    void serialRecieve(); //получаем данные от ТНПА'
    void serialRecieve_1(); //получаем данные от МДС'
    void update_responce();
    void update_timeout();
    void update_transmission();
    void transmission_delay(int n);
   // void on_Change_dist_clicked();
    void on_mds_x_textChanged();
    void on_mds_y_textChanged();
    void on_mds_z_textChanged();
    void on_mo_x_textChanged();
    void on_mo_y_textChanged();
    void on_mo_z_textChanged();

    void on_pushButton_clicked();
};
#endif // MAINWINDOW_H
