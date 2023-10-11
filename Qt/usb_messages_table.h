/*
 * Listado de los tipos de mensajes empleados en la aplicación, así como definiciones de sus parámetros.
*/
#ifndef __USB_MESSAGES_TABLE_H
#define __USB_MESSAGES_TABLE_H

#include<stdint.h>

//Codigos de los mensajes. El estudiante deberá definir los códigos para los mensajes que vaya
// a crear y usar. Estos deberan ser compatibles con los usados en la parte Qt

typedef enum {
    MENSAJE_NO_IMPLEMENTADO,
    MENSAJE_PING,
    MENSAJE_MOTOR_IZQUIERDO,
    MENSAJE_MOTOR_DERECHO,
    MENSAJE_MVTO_MOTOR,
    MENSAJE_SENSORES,
    MENSAJE_ENERGIA,
    MENSAJE_ALARM,
    MENSAJE_ENCENDER,
    MENSAJE_PANEL_SOLAR,
    MENSAJE_AUTODESTRUCCION,
    MENSAJE_DISTANCIA,
    //etc, etc...
} messageTypes;

//Estructuras relacionadas con los parametros de los mensajes. El estuadiante debera crear las
// estructuras adecuadas a los mensajes usados, y asegurarse de su compatibilidad con el extremo Qt

#pragma pack(1)   //Con esto consigo que el alineamiento de las estructuras en memoria del PC (32 bits) no tenga relleno.
//Con lo de abajo consigo que el alineamiento de las estructuras en memoria del microcontrolador no tenga relleno
#define PACKED //__attribute__ ((packed))

typedef struct {
    int8_t potencia_md;
    int8_t potencia_mi;
} PACKED PARAM_MENSAJE_MOTOR;

typedef struct {

    float Vact;
    float heading;
    float distancia_recorrida;
} PACKED PARAM_MENSAJE_MVTO_MOTOR;

typedef struct {
        uint8_t fLeft:1;
        uint8_t fRight:1;
        uint8_t fBack:1;
        uint32_t distancia;
} PACKED PARAM_MENSAJE_SENSORES;

typedef struct {
    int32_t energia;
} PACKED PARAM_MENSAJE_ENERGIA;

typedef struct {

    bool energia40;
    bool energia0;
    bool caida;
    bool peligro;
    bool solucionado;
    bool quemado;
} PACKED PARAM_MENSAJE_ALARM;

//typedef struct {
//    bool encendido;
//} PACKED PARAM_MENSAJE_ENCENDER;

typedef struct {
    bool encendido;
} PACKED PARAM_MENSAJE_PANEL_SOLAR;

//typedef struct {
//    bool destruir;
//} PACKED PARAM_MENSAJE_AUTODESTRUCCION;

typedef struct {
    float distancia;
} PACKED PARAM_MENSAJE_DISTANCIA;

typedef struct {
    uint8_t message;
}PACKED PARAM_MENSAJE_NO_IMPLEMENTADO;

#pragma pack()    //...Pero solo para los mensajes que voy a intercambiar, no para el resto





#endif
