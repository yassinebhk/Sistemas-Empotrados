#include "guipanel.h"
#include "ui_guipanel.h"
#include <QSerialPort>      // Comunicacion por el puerto serie
#include <QSerialPortInfo>  // Comunicacion por el puerto serie
#include <QGraphicsDropShadowEffect>
#include <QRandomGenerator>
#include<stdint.h>      // Cabecera para usar tipos de enteros con tamaño
#include<stdbool.h>     // Cabecera para usar booleanos

#include<QKeyEvent>
#include<QMessageBox>
#include <QPainter>
#include <math.h>
#include <QtMath>

extern "C" {
#include "serial2USBprotocol.h"    // Cabecera de funciones de gestión de tramas; se indica que está en C, ya que QTs
// se integra en C++, y el C puede dar problemas si no se indica.
}

#include "usb_messages_table.h"
#include "config.h"

#include<qwt_compass.h>
#include<qwt_dial_needle.h>

GUIPanel::GUIPanel(QWidget *parent) :  // Constructor de la clase
    QWidget(parent),
    ui(new Ui::GUIPanel)               // Indica que guipanel.ui es el interfaz grafico de la clase
  , transactionCount(0)
{
    ui->setupUi(this);                // Conecta la clase con su interfaz gráfico.
    setWindowTitle(tr("Control de MiniRobot (2022/2023)")); // Título de la ventana

    // Conexion por el puerto serie-USB
    fConnected=false;                 // Todavía no hemos establecido la conexión USB
    ui->serialPortComboBox->clear(); // Vacía de componentes la comboBox
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())

        if ((info.vendorIdentifier()==0x1CBE) && (info.productIdentifier()==0x0002))
        {
            ui->serialPortComboBox->addItem(info.portName());
        }
    ui->serialPortComboBox->setFocus();   // Componente del GUI seleccionado de inicio

    connect(&serial, SIGNAL(readyRead()), this, SLOT(readRequest()));

    ui->pingButton->setEnabled(false);    // Se deshabilita el botón de ping del interfaz gráfico, hasta que
    // se haya establecido conexión

    //Inicializa la ventana pop-up PING
    ventanaPopUp.setIcon(QMessageBox::Information);
    ventanaPopUp.setText(tr("Status: RESPUESTA A PING RECIBIDA"));
    ventanaPopUp.setStandardButtons(QMessageBox::Ok);
    ventanaPopUp.setWindowTitle(tr("Evento"));
    ventanaPopUp.setParent(this,Qt::Popup);

    initdial();

    Tank=(QPixmap) (ui->robot->pixmap());
    pos=ui->robot->pos();
    angle=90;
    varX=0;
    varY=0;
}

void GUIPanel::initdial(){

    ui->dial->setWrapping(false);  // La aguja no puede superar los valores
    // maximo o minimo
    ui->dial->setReadOnly(true); // La aguja no se puede mover con el raton

    // Configuracion de la aguja del instrumento, y de los ticks
    QwtDialSimpleNeedle *needle = new QwtDialSimpleNeedle(
                QwtDialSimpleNeedle::Arrow, true, Qt::red,
                QColor(Qt::gray).lighter(130));
    ui->dial->setNeedle(needle);

    sound= new QMediaPlayer();
    audioOutput = new QAudioOutput();
    sound->setAudioOutput(audioOutput);
}


GUIPanel::~GUIPanel() // Destructor de la clase
{
    delete ui;   // Borra el interfaz gráfico asociado a la clase
}

void GUIPanel::readRequest()
{
    int StopCharPosition,StartCharPosition,tam;   // Solo uso notacin hungara en los elementos que se van a
    uint8_t *pui8Frame; // Puntero a zona de memoria donde reside la trama recibida
    void *ptrtoparam;
    uint8_t ui8Message; // Para almacenar el mensaje de la trama entrante
    PARAM_MENSAJE_DISTANCIA distancia;
    uint8_t pui8Frame2[MAX_FRAME_SIZE];
    int size;

    double x;
    double y;
    static double x_ini = ui->robot->pos().x();
    static double y_ini = ui->robot->pos().y();
    double dist = 0;

    incommingDataBuffer.append(serial.readAll()); // Añade el contenido del puerto serie USB al array de bytes 'incommingDataBuffer'

    StopCharPosition=incommingDataBuffer.indexOf((char)STOP_FRAME_CHAR,0);
    while (StopCharPosition>=0)
    {
        //Ahora buscamos el caracter de inicio correspondiente.
        StartCharPosition=incommingDataBuffer.lastIndexOf((char)START_FRAME_CHAR,0); //Este seria el primer caracter de inicio que va delante...

        if (StartCharPosition<0)
        {
            //En caso de que no lo encuentre, no debo de hacer nada, pero debo vaciar las primeras posiciones hasta STOP_FRAME_CHAR (inclusive)
            incommingDataBuffer.remove(0,StopCharPosition+1);
            LastError=QString("Status:Fallo trozo paquete recibido");
        } else
        {
            incommingDataBuffer.remove(0,StartCharPosition); //Si hay datos anteriores al caracter de inicio, son un trozo de trama incompleto. Los tiro.
            tam=StopCharPosition-StartCharPosition+1;//El tamanio de la trama es el numero de bytes desde inicio hasta fin, ambos inclusive.
            if (tam>=MINIMUM_FRAME_SIZE)
            {
                pui8Frame=(uint8_t*)incommingDataBuffer.data(); // Puntero de trama al inicio del array de bytes
                pui8Frame++; //Nos saltamos el caracter de inicio.
                tam-=2; //Descontamos los bytes de inicio y fin del tamanio del paquete

                // Paso 1: Destuffing y cálculo del CRC. Si todo va bien, obtengo la trama
                // con valores actualizados y sin bytes de CRC.
                tam=destuff_and_check_checksum((unsigned char *)pui8Frame,tam);
                if (tam>=0)
                {
                    //El paquete está bien, luego procedo a tratarlo.
                    ui8Message=decode_message_type(pui8Frame); // Obtencion del byte de Mensaje
                    tam=get_message_param_pointer(pui8Frame,tam,&ptrtoparam);
                    switch(ui8Message) // Segun el mensaje tengo que hacer cosas distintas
                    {
                    /** A PARTIR AQUI ES DONDE SE DEBEN AÑADIR NUEVAS RESPUESTAS ANTE LOS MENSAJES QUE SE ENVIEN DESDE LA TIVA **/
                    case MENSAJE_PING:  // Algunos mensajes no tiene parametros
                        // Crea una ventana popup con el texto indicado
                        pingResponseReceived();
                        break;

                    case MENSAJE_MVTO_MOTOR:
                    {
                        float heading;
                        float pi = MPI;

                        PARAM_MENSAJE_MVTO_MOTOR parametro;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(parametro),&parametro)>0){

                            if (parametro.heading < 0) {

                                if (parametro.heading < -2*pi) {

                                    float grados = (180 * parametro.heading) / pi;
                                    heading = (int32_t) (grados) % 360 + 360;
                                }

                                else {

                                    heading = (180 * parametro.heading) / (pi) + 360;
                                }

                            }

                            else {

                                if (parametro.heading > 2*pi) {

                                    float grados = (180 * parametro.heading) / pi;
                                    //heading = ((grados / 360) - (int8_t) (grados / 360)) * 360;
                                    heading = (int32_t) (grados) % 360;
                                }

                                else {

                                    heading = (180 * parametro.heading) / (pi);
                                }
                            }

                            ui->dial->setValue(heading);
                            ui->Velocimetro->setValue(parametro.Vact);
                            ui->Distancia_recorrida->setValue(parametro.distancia_recorrida);

                            x=ui->robot->pos().x();
                            y=ui->robot->pos().y();
                            x = x + parametro.Vact * cos (parametro.heading);
                            y = y + parametro.Vact * sin (parametro.heading);

                            ui->robot->move(x,y);
                            ui->robot->setPixmap(rotatePixmap(heading));

                            dist = sqrt ((x_ini - x)*(x_ini - x) + (y_ini - y)*(y_ini - y));
                            ui->distancia->display(dist);

                            if(fConnected)
                            {
                                distancia.distancia  = dist;       // Se crea la trama con n de secuencia 0; mensaje MENSAJE_LEDS; se le pasa la
                                // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
                                // de trama
                                size=create_frame((uint8_t *)pui8Frame2, MENSAJE_DISTANCIA, &distancia, sizeof(distancia), MAX_FRAME_SIZE);
                                // Se se pudo crear correctamente, se envia la trama
                                if (size>0) serial.write((char *)pui8Frame2,size);
                            }

                            if (dist > 300) {

                                sound->setSource(QUrl("qrc:/sonidos/evacuacion.mp3"));
                                sound->play();
                                ui->groupBox_9->setEnabled(false);
                                ui->Distancia->setEnabled(false);
                            }

                        }// else TRATAMIENTO DE ERRORES
                        break;
                    }

                    case MENSAJE_SENSORES:
                    {
                        PARAM_MENSAJE_SENSORES parametro;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(parametro),&parametro)>0){
                            // OJO! Propiedad "autoexclusive" de los botones debe estar desactivada!!!
                            ui->rightButton->setChecked(parametro.fRight);
                            ui->leftButton->setChecked(parametro.fLeft);
                            ui->backButton->setChecked(parametro.fBack);

                            if (parametro.fBack != 0 || parametro.fLeft != 0 || parametro.fRight != 0) {

                                sound->setSource(QUrl("qrc:/sonidos/boton.mp3"));
                                sound->play();
                            }
                            else {

                                //sound->stop();
                            }
                            float distancia = (70 * parametro.distancia) / 4095;
                            ui->thermometer->setValue(distancia);

                        }// else TRATAMIENTO DE ERRORES
                        break;
                    }
                    case MENSAJE_ENERGIA:
                    {
                        PARAM_MENSAJE_ENERGIA parametro;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(parametro),&parametro)>0){

                            float energia = parametro.energia/1000;
                            ui->energia->setValue(energia);
                            ui->progressBar->setValue(energia);

                            if (parametro.energia >= 0) {

                                ui->cont->display(parametro.energia);
                            }
                        }// else TRATAMIENTO DE ERRORES
                        break;
                    }

                    case MENSAJE_ALARM:
                    {
                        PARAM_MENSAJE_ALARM parametro;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(parametro),&parametro)>0){

                            if (parametro.energia40 == true) {

                                ui->energia40->setChecked(true);
                                sound->setSource(QUrl("qrc:/sonidos/alarma.mp3"));
                                sound->play();

                            }

                            if (parametro.energia40 == false) {

                                ui->energia40->setChecked(false);
                                sound->setSource(QUrl("qrc:/sonidos/alarma.mp3"));
                                sound->stop();
                            }

                            if (parametro.energia0 == true) {

                                sound->stop();

                            }

                            if (parametro.caida == true) {

                                ui->caida->setChecked(true);
                            }

                            if (parametro.peligro == true) {

                                ui->motores->setChecked(true);
                                sound->setSource(QUrl(":qrc:/sonidos/alarma.mp3"));
                                sound->play();
                            }

                            if (parametro.solucionado == true) {

                                ui->motores->setChecked(false);
                                sound->stop();
                            }

                            if (parametro.quemado == true) {

                                ui->motores_2->setChecked(true);

                                //ui->label_9->setPixmap(QPixmap(":/sonidos/alarma.mp3"));
                            }
                        }// else TRATAMIENTO DE ERRORES
                        break;
                    }

                    case MENSAJE_AUTODESTRUCCION :
                    {
                        //PARAM_MENSAJE_AUTODESTRUCCION destruir;
                        //if (check_and_extract_message_param(ptrtoparam, tam, sizeof(destruir),&destruir)>0){

                            ui->planeLabel_2->setPixmap(QPixmap(":/imagenes/smoke.png"));
                            ui->groupBox_9->setEnabled(false);

                        //}// else TRATAMIENTO DE ERRORES
                        break;
                    }
                    case MENSAJE_NO_IMPLEMENTADO:
                    {
                        // En otros mensajes hay que extraer los parametros de la trama y copiarlos
                        // a una estructura para poder procesar su informacion
                        PARAM_MENSAJE_NO_IMPLEMENTADO parametro;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(parametro),&parametro)>0)
                        {
                            // Muestra en una etiqueta (statuslabel) del GUI el mensaje
                            ui->statusLabel->setText(tr("  Mensaje rechazado,"));
                        }
                        else
                        {
                            // TRATAMIENTO DE ERRORES
                        }
                    }
                        break;

                        //Falta por implementar la recepcion de mas tipos de mensajes
                        //habria que decodificarlos y emitir las señales correspondientes con los parametros que correspondan

                    default:
                        //Este error lo notifico mediante la señal statusChanged
                        LastError=QString("Status: Recibido paquete inesperado");
                        ui->statusLabel->setText(tr("  Recibido paquete inesperado,"));
                        break;
                    }
                }
                else
                {
                    LastError=QString("Status: Error de stuffing o CRC");
                    ui->statusLabel->setText(tr(" Error de stuffing o CRC"));
                 }
            }
            else
            {

                // B. La trama no está completa o no tiene el tamano adecuado... no lo procesa
                //Este error lo notifico mediante la señal statusChanged
                LastError=QString("Status: Error trozo paquete recibido");
                ui->statusLabel->setText(tr(" Fallo trozo paquete recibido"));
            }
            incommingDataBuffer.remove(0,StopCharPosition-StartCharPosition+1); //Elimino el trozo que ya he procesado
        }

        StopCharPosition=incommingDataBuffer.indexOf((char)STOP_FRAME_CHAR,0); //Compruebo si el se ha recibido alguna trama completa mas. (Para ver si tengo que salir del bucle o no
    } //Fin del while....
}

// Funciones auxiliares a la gestión comunicación USB

// Establecimiento de la comunicación USB serie a través del interfaz seleccionado en la comboBox, tras pulsar el
// botón RUN del interfaz gráfico. Se establece una comunicacion a 9600bps 8N1 y sin control de flujo en el objeto
// 'serial' que es el que gestiona la comunicación USB serie en el interfaz QT
void GUIPanel::startSlave()
{
    if (serial.portName() != ui->serialPortComboBox->currentText()) {
        serial.close();
        serial.setPortName(ui->serialPortComboBox->currentText());

        if (!serial.open(QIODevice::ReadWrite)) {
            processError(tr("No puedo abrir el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setBaudRate(9600)) {
            processError(tr("No puedo establecer tasa de 9600bps en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setDataBits(QSerialPort::Data8)) {
            processError(tr("No puedo establecer 8bits de datos en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setParity(QSerialPort::NoParity)) {
            processError(tr("NO puedo establecer parida en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setStopBits(QSerialPort::OneStop)) {
            processError(tr("No puedo establecer 1bitStop en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setFlowControl(QSerialPort::NoFlowControl)) {
            processError(tr("No puedo establecer el control de flujo en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }
    }

    ui->runButton->setEnabled(false);

    // Se indica que se ha realizado la conexión en la etiqueta 'statusLabel'
    ui->statusLabel->setText(tr("Estado: Ejecucion, conectado al puerto %1.")
                             .arg(ui->serialPortComboBox->currentText()));

    // Y se habilitan los controles
    ui->pingButton->setEnabled(true);

    // Variable indicadora de conexión a TRUE, para que se permita enviar mensajes en respuesta
    // a eventos del interfaz gráfico
    fConnected=true;
}

// Funcion auxiliar de procesamiento de errores de comunicación (usada por startSlave)
void GUIPanel::processError(const QString &s)
{
    activateRunButton(); // Activa el botón RUN
    // Muestra en la etiqueta de estado la razón del error (notese la forma de pasar argumentos a la cadena de texto)
    ui->statusLabel->setText(tr("Status: Not running, %1.").arg(s));
}

// Funcion de habilitacion del boton de inicio/conexion
void GUIPanel::activateRunButton()
{
    ui->runButton->setEnabled(true);
}

// Funciones SLOT que se crean automaticamente desde QTDesigner al activar una señal de un Widget del interfaz gráfico
// Se suelen asociar a funciones auxiliares, en muchos caso, por comodidad.

// SLOT asociada a pulsación del botón RUN
void GUIPanel::on_runButton_clicked()
{
    startSlave();
}

// SLOT asociada a pulsación del botón PING
void GUIPanel::on_pingButton_clicked()
{
    pingDevice();
}

// SLOT asociada al borrado del mensaje de estado al pulsar el boton
void GUIPanel::on_statusButton_clicked()
{
    ui->statusLabel->setText(tr(""));
}

// Funciones de usuario asociadas a la respuesta a mensajes. La estructura va a ser muy parecida en casi todos los
// casos. Se va a crear una trama de un tamaño maximo (100), y se le van a introducir los elementos de
// num_secuencia, mensaje, y parametros.

// Envío de un mensaje PING

void GUIPanel::pingDevice()
{
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;

    if (fConnected) // Para que no se intenten enviar datos si la conexion USB no esta activa
    {
        // El mensaje PING no necesita parametros; de ahí el NULL, y el 0 final.
        // No vamos a usar el mecanismo de numeracion de tramas; pasamos un 0 como n de trama
        size=create_frame(paquete, MENSAJE_PING, nullptr, 0, MAX_FRAME_SIZE);
        // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
        if (size>0) serial.write((const char*)paquete,size);
    }
}

void GUIPanel::pingResponseReceived()

{
    // Ventana popUP para el caso de mensaje PING; no te deja definirla en un "caso"
    ventanaPopUp.setStyleSheet("background-color: lightgrey");
    ventanaPopUp.setModal(true);
    ventanaPopUp.show();
}


void GUIPanel::on_Motor_Izq_sliderReleased()
{

    PARAM_MENSAJE_MOTOR parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(fConnected)
    {
        // Se rellenan los parametros del paquete (en este caso, el estado de los LED)
        parametro.potencia_mi  = ui->Motor_Izq->value();       // Se crea la trama con n de secuencia 0; mensaje MENSAJE_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, MENSAJE_MOTOR_IZQUIERDO, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}


void GUIPanel::on_Motor_Dcha_sliderReleased()
{

    PARAM_MENSAJE_MOTOR parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(fConnected)
    {
        // Se rellenan los parametros del paquete (en este caso, el estado de los LED)
        parametro.potencia_md  = ui->Motor_Dcha->value();       // Se crea la trama con n de secuencia 0; mensaje MENSAJE_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, MENSAJE_MOTOR_DERECHO, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}


void GUIPanel::on_caida_checkChanged(bool val)
{

    ui->planeLabel->setPixmap(QPixmap(":/imagenes/explosion.png"));
    //sound->setSource(QUrl("qrc:/sonidos/explosion.mp3"));
    //sound->play();

    //ui->label_8->setPixmap(QPixmap(":/sonidos/explosion.mp3"));
    ui->groupBox_9->setEnabled(false);
}

void GUIPanel::on_motores_2_checkChanged(bool val)
{
    ui->planeLabel_2->setPixmap(QPixmap(":/imagenes/smoke.png"));
    ui->groupBox_9->setEnabled(false);
}


void GUIPanel::on_pushButton_3_clicked()
{
    //PARAM_MENSAJE_ENCENDER parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    ui->pushButton_3->setIcon(QPixmap(":/imagenes/Palanca2On.png"));

    if(fConnected)
    {
        sound->setSource(QUrl("qrc:/sonidos/encendido.mp3"));
        sound->play();

        //ui->label_10->setPixmap(QPixmap(":/sonidos/encendido.mp3"));
        // Se rellenan los parametros del paquete (en este caso, el estado de los LED)
        //parametro.encendido  = true;       // Se crea la trama con n de secuencia 0; mensaje MENSAJE_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, MENSAJE_ENCENDER, 0, 0, MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}


void GUIPanel::on_pushButton_4_clicked()
{
    PARAM_MENSAJE_PANEL_SOLAR parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    bool conectado;
    if(fConnected)
    {
        sound->setSource(QUrl("qrc:/sonidos/energia.mp3"));
        sound->play();
        // Se rellenan los parametros del paquete (en este caso, el estado de los LED)
        ui->pushButton_4->setChecked(conectado);
        parametro.encendido  = true;       // Se crea la trama con n de secuencia 0; mensaje MENSAJE_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, MENSAJE_PANEL_SOLAR, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}


void GUIPanel::on_pushButton_5_clicked()
{
    PARAM_MENSAJE_PANEL_SOLAR parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    bool conectado;
    if(fConnected)
    {
        sound->stop();
        // Se rellenan los parametros del paquete (en este caso, el estado de los LED)
        ui->pushButton_4->setChecked(conectado);
        parametro.encendido  = false;       // Se crea la trama con n de secuencia 0; mensaje MENSAJE_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, MENSAJE_PANEL_SOLAR, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}

QPixmap GUIPanel::rotatePixmap(int angle){
    QSize size = Tank.size();
    QPixmap rotatedPixmap(size);
    rotatedPixmap.fill(QColor::fromRgb(0, 0, 0, 0)); //the new pixmap must be transparent.
    QPainter* p = new QPainter(&rotatedPixmap);
    p->translate(size.height()/2,size.height()/2);
    p->rotate(angle);
    p->translate(-size.height()/2,-size.height()/2);
    p->drawPixmap(0, 0, Tank);
    p->end();
    delete p;
    return rotatedPixmap;
}

void GUIPanel::on_horizontalSlider_valueChanged(int value)
{
    int vertical = ui->verticalSlider->value();

    if (value <= 100 && value >= 90 && abs (vertical) <= 10) {

        ui->Motor_Izq->setValue(-100);
        ui->Motor_Dcha->setValue(100);
    }

    if (value >= -100 && value <= -90  && abs (vertical) <= 10) {

        ui->Motor_Izq->setValue(100);
        ui->Motor_Dcha->setValue(-100);
    }

    if (value >= -100 && value <= -90 && vertical >= -100 && vertical <= -90) {

        ui->Motor_Izq->setValue(-50);
        ui->Motor_Dcha->setValue(-100);
    }

    if (value >= -100 && value <= -90 && vertical <= 100 && vertical >= 90) {

        ui->Motor_Izq->setValue(50);
        ui->Motor_Dcha->setValue(100);
    }

    if (value <= 100 && value >= 90 && vertical >= -100 && vertical <= -90) {

        ui->Motor_Izq->setValue(-100);
        ui->Motor_Dcha->setValue(-50);
    }

    if (abs (value) <= 10 && vertical <= 100 && vertical >= 90) {

        ui->Motor_Izq->setValue(100);
        ui->Motor_Dcha->setValue(100);
    }

    if (abs (value) <= 10 && vertical >= -100 && vertical <= -90) {

        ui->Motor_Izq->setValue(-100);
        ui->Motor_Dcha->setValue(-100);
    }

    if (value <= 100 && value >= 90 && vertical <= 100 && vertical >= 90) {

        ui->Motor_Izq->setValue(100);
        ui->Motor_Dcha->setValue(50);
    }

    if (abs (value) <= 10 && abs (vertical) <= 10) {

        ui->Motor_Izq->setValue(0);
        ui->Motor_Dcha->setValue(0);
    }
}


void GUIPanel::on_verticalSlider_valueChanged(int value)
{
    int horizontal = ui->horizontalSlider->value();

    if (value <= 100 && value >= 90 && abs (horizontal) <= 10) {

        ui->Motor_Izq->setValue(100);
        ui->Motor_Dcha->setValue(100);
    }

    if (value >= -100 && value <= -90  && abs (horizontal) <= 10) {

        ui->Motor_Izq->setValue(-100);
        ui->Motor_Dcha->setValue(-100);
    }

    if (value >= -100 && value <= -90 && horizontal >= -100 && horizontal <= -90) {

        ui->Motor_Izq->setValue(-50);
        ui->Motor_Dcha->setValue(-100);
    }

    if (value >= -100 && value <= -90 && horizontal <= 100 && horizontal >= 90) {

        ui->Motor_Izq->setValue(-100);
        ui->Motor_Dcha->setValue(-50);
    }

    if (value <= 100 && value >= 90 && horizontal >= -100 && horizontal <= -90) {

        ui->Motor_Izq->setValue(50);
        ui->Motor_Dcha->setValue(100);
    }

    if (abs (value) <= 10 && horizontal <= 100 && horizontal >= 90) {

        ui->Motor_Izq->setValue(100);
        ui->Motor_Dcha->setValue(0);
    }

    if (abs (value) <= 10 && horizontal >= -100 && horizontal <= -90) {

        ui->Motor_Izq->setValue(0);
        ui->Motor_Dcha->setValue(100);
    }

    if (value <= 100 && value >= 90 && horizontal <= 100 && horizontal >= 90) {

        ui->Motor_Izq->setValue(100);
        ui->Motor_Dcha->setValue(50);
    }

    if (abs (value) <= 10 && abs (horizontal) <= 10) {

        ui->Motor_Izq->setValue(0);
        ui->Motor_Dcha->setValue(0);
    }
}


void GUIPanel::on_Motor_Izq_valueChanged(double value)
{
    PARAM_MENSAJE_MOTOR parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(fConnected)
    {
        // Se rellenan los parametros del paquete (en este caso, el estado de los LED)
        parametro.potencia_mi  = value;       // Se crea la trama con n de secuencia 0; mensaje MENSAJE_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, MENSAJE_MOTOR_IZQUIERDO, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}


void GUIPanel::on_Motor_Dcha_valueChanged(double value)
{
    PARAM_MENSAJE_MOTOR parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(fConnected)
    {
        // Se rellenan los parametros del paquete (en este caso, el estado de los LED)
        parametro.potencia_md  = value;       // Se crea la trama con n de secuencia 0; mensaje MENSAJE_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, MENSAJE_MOTOR_DERECHO, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}

