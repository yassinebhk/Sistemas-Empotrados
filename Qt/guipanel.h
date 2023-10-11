#ifndef GUIPANEL_H
#define GUIPANEL_H

#include <QWidget>
#include <QSerialPort>
#include <QMessageBox>
#include <QGraphicsItem>
#include <QGraphicsItemAnimation>
#include <QGraphicsScene>
#include <QPixmap>
#include <QPoint>
#include <QMediaPlayer>
#include <QAudioOutput>


namespace Ui {
class GUIPanel;
}


class GUIPanel : public QWidget
{
    Q_OBJECT
    
public:
    //GUIPanel(QWidget *parent = 0);
    explicit GUIPanel(QWidget *parent = 0);
    ~GUIPanel(); // Da problemasz
    QPixmap rotatePixmap(int angle);
    void initdial();
    
private slots:
    void readRequest();
    void on_pingButton_clicked();
    void on_runButton_clicked();
    void on_statusButton_clicked();


    void on_Motor_Izq_sliderReleased();

    void on_Motor_Dcha_sliderReleased();

    void on_caida_checkChanged(bool val);

    void on_motores_2_checkChanged(bool val);

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void on_horizontalSlider_valueChanged(int value);

    void on_verticalSlider_valueChanged(int value);

    void on_Motor_Izq_valueChanged(double value);

    void on_Motor_Dcha_valueChanged(double value);

private: // funciones privadas
    void pingDevice();
    void startSlave();
    void processError(const QString &s);
    void activateRunButton();
    void pingResponseReceived();
private:
    Ui::GUIPanel *ui;
    int transactionCount;
    bool fConnected;
    QSerialPort serial;
    QByteArray incommingDataBuffer;
    QString LastError;
    QMediaPlayer * sound;
    QAudioOutput *audioOutput;
    QMessageBox ventanaPopUp;
    QPixmap Tank;
    QPoint pos;
    int angle;
    double varX;
    double varY;
};

#endif // GUIPANEL_H
