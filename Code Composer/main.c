
/* Yassine Bouhaik Bouhoussaine */
        // Proyecto 1 //


    #include<stdbool.h>
    #include<stdint.h>

    #include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
    #include "inc/hw_types.h"        // TIVA: Definiciones API
    #include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
    #include "inc/hw_uart.h"
    #include "inc/hw_adc.h"

    #include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
    #include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
    #include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (MAP_)
    #include "driverlib/rom_map.h"   // TIVA: Mapeo automatico de funciones API incluidas en ROM de micro (MAP_)
    #include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
    #include "driverlib/uart.h"      // TIVA: Funciones API manejo UART
    #include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
    #include "driverlib/adc.h"       // TIVA: Funciones API manejo de ADC
    #include "driverlib/timer.h"     // TIVA: Funciones API manejo de timers
    #include "driverlib/pwm.h"       // Libreria PWM
    #include "driverlib/debug.h"
    #include "driverlib/sysctl.h"
    #include "drivers/rgb.h"

    #include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
    #include "drivers/rgb.h"         // TIVA: Funciones API manejo de leds con PWM
    #include "FreeRTOS.h"            // FreeRTOS: definiciones generales
    #include "semphr.h"              // FreeRTOS: definiciones para uso de semaforos
    #include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
    #include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos
    #include "queue.h"               // FreeRTOS: definiciones relacionadas con colas de mensajes
    #include "timers.h"              // FreeRTOS: definiciones relacionadas con timers
    #include <utils/uartstdioMod.h>     // TIVA: Funciones API UARTSTDIO (printf)
    #include "utils/cpu_usage.h"
    #include "commands.h"
    #include "event_groups.h"        // FreeRTOS: definiciones relacionadas con grupos de eventos

    #include <serial2USBprotocol.h>
    #include <usb_dev_serial.h>
    #include "usb_messages_table.h"

    #define PULSADO 0x0001
    #define SOLTADO 0X0000

    #define ENERGIA40_FLAG 0x0001
    #define ENERGIA0_FLAG 0x0002
    #define CAIDA_FLAG 0x0004
    #define PELIGRO_FLAG 0x0020
    #define QUEMADO_FLAG 0x0010
    #define SOLUCIONADO_FLAG 0x0008
    #define FUERA_ALCANCE_FLAG 0x0040

    #define MITASKPRIO 1
    #define MITASKSTACKSIZE 128
    #define MDTASKPRIO 1
    #define MDTASKSTACKSIZE 128
    #define MVTASKPRIO 1
    #define MVTASKSTACKSIZE 128
    #define BTTASKPRIO 1
    #define BTTASKSTACKSIZE 128
    #define ENTASKPRIO 1
    #define ENTASKSTACKSIZE 128
    #define ALTASKPRIO 2
    #define ALTASKSTACKSIZE 128
    #define VTASKPRIO 2
    #define VTASKSTACKSIZE 128
    #define PSTASKPRIO 2
    #define PSTASKSTACKSIZE 128

    #define DIV_RELOJ_PWM 16
    #define PERIOD_PWM (SysCtlClockGet()*0.02)/DIV_RELOJ_PWM //50.000 (20 ms)

    // Variables globales "main"
    uint32_t g_ui32CPUUsage;
    uint32_t g_ui32SystemClock;

    TaskHandle_t manejadora_usb;
    TaskHandle_t manejadora_iz;
    TaskHandle_t manejadora_dch;
    TaskHandle_t manejadora_mvto;
    TaskHandle_t manejadora_energia;
    TaskHandle_t manejadora_sensor;
    TaskHandle_t manejadora_alarma;
    TaskHandle_t manejadora_panel;

    static QueueHandle_t cola_motor_izq;
    static QueueHandle_t cola_motor_dcha;
    static QueueHandle_t cola_mvto_izq;
    static QueueHandle_t cola_mvto_dcha;
    static QueueHandle_t cola_botones;
    static QueueHandle_t cola_infrarrojo;
    static QueueHandle_t cola_velocidad_energia;
    static QueueHandle_t cola_velocidad_mvto;
    static QueueHandle_t cola_panel_solar;

    static QueueHandle_t mailbox_energia_izq;
    static QueueHandle_t mailbox_energia_dch;
    static QueueHandle_t mailbox_sensor_caida;
    static QueueHandle_t mailbox_panel_solar;

    QueueHandle_t mailbox_modo_traza;
    QueueHandle_t mailbox_cambio_parametros;
    QueueHandle_t cola_traza;

    static QueueSetHandle_t grupo_mvto;
    static QueueSetHandle_t grupo_sensor;
//    static QueueSetHandle_t grupo_energia;

    TimerHandle_t xTimer;
    TimerHandle_t xTimer_en;
    TimerHandle_t xTimer_peligro;
    TimerHandle_t xTimer_quemado;
    TimerHandle_t xTimer_panel;
    TimerHandle_t xTimer_autodestruccion;

    static SemaphoreHandle_t mutex_Qt;
    static SemaphoreHandle_t semaforo_timer;
    static SemaphoreHandle_t semaforo_energia;

    EventGroupHandle_t FlagsEventos;

    #define PACKED __attribute__ ((packed))

    typedef struct {
        int32_t energia;
        int32_t distancia;
    } PACKED parametros;

    //*****************************************************************************
    //
    // The error routine that is called if the driver library encounters an error.
    //
    //*****************************************************************************
    #ifdef DEBUG
    void
    __error__(char *pcFilename, unsigned long ulLine)
    {
    }

    #endif

    //*****************************************************************************
    //
    // Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
    //
    //*****************************************************************************

    //Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
    //
    void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
    {
        //
        // This function can not return, so loop forever.  Interrupts are disabled
        // on entry to this function, so no processor interrupts will interrupt
        // this loop.
        //
        while(1)
        {
        }
    }

    //Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
    void vApplicationTickHook( void )
    {
        static uint8_t ui8Count = 0;

        if (++ui8Count == 10)
        {
            g_ui32CPUUsage = CPUUsageTick();
            ui8Count = 0;
        }
        //return;
    }

    //Esto se ejecuta cada vez que entra a funcionar la tarea Idle
    void vApplicationIdleHook (void)
    {
        SysCtlSleep();
    }


    //Esto se ejecuta cada vez que se produce un fallo de asignacio de heap
    void vApplicationMallocFailedHook (void)
    {
        while(1);
    }




/****************************************************************************************************************************************/
/*                                                                                                                                      */
/*                                                     Canal USB                                                                        */
/*                                                                                                                                      */
/****************************************************************************************************************************************/

    static portTASK_FUNCTION( USBMessageProcessingTask, pvParameters ){

        uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila
        int32_t i32Numdatos;
        uint8_t ui8Message;
        void *ptrtoreceivedparam;
        uint32_t ui32Errors=0;

        PARAM_MENSAJE_MOTOR parametro;
        PARAM_MENSAJE_PANEL_SOLAR panel;
        PARAM_MENSAJE_DISTANCIA distancia;

        int8_t potencia = 0;
        uint32_t periodPWM=PERIOD_PWM; // Ciclos para 20ms con el reloj delsistema
        uint32_t ui32DutyCycle = periodPWM;

        MAP_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        MAP_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, periodPWM); // Establece periodo
        MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3,ui32DutyCycle); //Establece el ciclo de trabajo
        MAP_PWMGenEnable(PWM1_BASE, PWM_GEN_1); // Habilita generador
        //MAP_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true); // Habiloita salidas PWM

        /* The parameters are not used. */
        ( void ) pvParameters;

        //
        // Mensaje de bienvenida inicial.
        //
        UARTprintf("\n\nBienvenido a la aplicacion control de MiniRobot (curso 2022/23)!\n");
        UARTprintf("\nAutores: XXXXXX y XXXXX ");

        for(;;)
        {
            //Espera hasta que se reciba una trama con datos serializados por el interfaz USB
            i32Numdatos=receive_frame(pui8Frame,MAX_FRAME_SIZE); //Esta funcion es bloqueante
            if (i32Numdatos>0)
            {   //Si no hay error, proceso la trama que ha llegado.
                i32Numdatos=destuff_and_check_checksum(pui8Frame,i32Numdatos); // Primero, "destuffing" y comprobaciï¿½n checksum
                if (i32Numdatos<0)
                {
                    //Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
                    ui32Errors++;
                    // Procesamiento del error
                }
                else
                {
                    //El paquete esta bien, luego procedo a tratarlo.
                    //Obtiene el valor del campo mensaje
                    ui8Message=decode_message_type(pui8Frame);
                    //Obtiene un puntero al campo de parametros y su tamanio.
                    i32Numdatos=get_message_param_pointer(pui8Frame,i32Numdatos,&ptrtoreceivedparam);
                    switch(ui8Message)
                    {
                    case MENSAJE_PING :
                        //A un mensaje de ping se responde con el propio mensaje
                        i32Numdatos=create_frame(pui8Frame,ui8Message,0,0,MAX_FRAME_SIZE);
                        if (i32Numdatos>=0)
                        {
                            send_frame(pui8Frame,i32Numdatos);
                        }else{
                            //Error de creacion de trama: determinar el error y abortar operacion
                            ui32Errors++;
                            // Procesamiento del error
                            //                      // Esto de aqui abajo podria ir en una funcion "createFrameError(numdatos)  para evitar
                            //                      // tener que copiar y pegar todo en cada operacion de creacion de paquete
                            switch(i32Numdatos){
                            case PROT_ERROR_NOMEM:
                                // Procesamiento del error NO MEMORY
                                break;
                            case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
                                //                          // Procesamiento del error STUFFED_FRAME_TOO_LONG
                                break;
                            case PROT_ERROR_MESSAGE_TOO_LONG:
                                //                          // Procesamiento del error MESSAGE TOO LONG
                                break;
                            }
                            case PROT_ERROR_INCORRECT_PARAM_SIZE:
                            {
                                // Procesamiento del error INCORRECT PARAM SIZE
                            }
                            break;
                        }
                        break; // case: MENSAJE_PING

                    case MENSAJE_MOTOR_IZQUIERDO :

                        if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0){

                            potencia = parametro.potencia_mi;
                            xQueueSend (cola_motor_izq,&potencia,portMAX_DELAY);
                            xQueueSend (cola_mvto_izq,&potencia,portMAX_DELAY);
                            xQueueOverwrite (mailbox_energia_izq,&potencia);

                        }else//Error de tamaï¿½o de parametro
                            ui32Errors++; // Tratamiento del error

                    break; // case: MENSAJE_MOTOR_IZQUIERDO

                    case MENSAJE_MOTOR_DERECHO :

                        if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0){

                            potencia = parametro.potencia_md;
                            xQueueSend (cola_motor_dcha,&potencia,portMAX_DELAY);
                            xQueueSend (cola_mvto_dcha,&potencia,portMAX_DELAY);
                            xQueueOverwrite (mailbox_energia_dch,&potencia);

                        }else//Error de tamaï¿½o de parametro
                            ui32Errors++; // Tratamiento del error

                    break; // case: MENSAJE_MOTOR_DERECHO

                    case MENSAJE_ENCENDER :

                        //if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, 0,NULL)>0){

                            vTaskResume( manejadora_usb);
                            vTaskResume( manejadora_iz);
                            vTaskResume( manejadora_dch);
                            vTaskResume( manejadora_mvto);
                            vTaskResume( manejadora_energia);
                            vTaskResume( manejadora_sensor);
                            vTaskResume( manejadora_alarma);
                            vTaskResume( manejadora_panel);
                            vTaskResume( manejadora_usb);
                            MAP_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true); // Habiloita salidas PWM

                        //}else//Error de tamaï¿½o de parametro
                            //ui32Errors++; // Tratamiento del error

                    break; // case: Encender motores

                    case MENSAJE_PANEL_SOLAR :

                        if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(panel),&panel)>0){

                            bool encendido = panel.encendido;
                            xQueueSend (cola_panel_solar,&encendido, portMAX_DELAY);

                        }else//Error de tamaï¿½o de parametro
                            ui32Errors++; // Tratamiento del error

                    break; // case: Encender panel solar

                    case MENSAJE_DISTANCIA :


                        if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(distancia),&distancia)>0){

                            if ((500 * distancia.distancia) / 3 >= periodPWM) {

                                MAP_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, false); // Habiloita salidas PWM
                            }

                            else {

                                ui32DutyCycle = (uint32_t) (periodPWM - (500 * distancia.distancia) / 3);
                                MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3,ui32DutyCycle); //Establece el ciclo de trabajo
                            }

                            if (distancia.distancia > 300) {

                                xEventGroupSetBits(FlagsEventos, FUERA_ALCANCE_FLAG);
                            }

                        } else//Error de tamaï¿½o de parametro
                            ui32Errors++; // Tratamiento del error

                    break; // case: Distancia del robot al innciio

                    default:
                    {
                        PARAM_MENSAJE_NO_IMPLEMENTADO parametro;
                        parametro.message=ui8Message;
                        //El mensaje esta bien pero no esta implementado
                        i32Numdatos=create_frame(pui8Frame,MENSAJE_NO_IMPLEMENTADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                        if (i32Numdatos>=0)
                        {
                            send_frame(pui8Frame,i32Numdatos);
                        }
                        break;
                    } // default
                    }// switch
                }
            }else{ // if (ui32Numdatos >0)
                ui32Errors++;
            }
        }
    }

/****************************************************************************************************************************************/
/*                                                                                                                                      */
/*                                                     Motor Izquierdo                                                                  */
/*                                                                                                                                      */
/****************************************************************************************************************************************/

    static portTASK_FUNCTION( MITask, pvParameters ){

        vTaskSuspend( manejadora_iz);
        int8_t potencia_iz = 0;
        uint32_t periodPWM=PERIOD_PWM; // Ciclos para 20ms con el reloj del sistema

        while(1)
        {
            if (xQueueReceive(cola_motor_izq,&potencia_iz,portMAX_DELAY)==pdTRUE)
            {

                if (potencia_iz == 101) {

                    MAP_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, false); // Habiloita salidas PWM
                    MAP_PWMGenDisable (PWM1_BASE, PWM_GEN_1);
                    vTaskDelete (manejadora_iz);

                }

                else {

                    uint32_t ui32DutyCycle = (periodPWM * abs (potencia_iz)) / 100;

                    if (ui32DutyCycle == 0) {

                        PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, false); // Habiloita salidas PWM
                    }

                    else {

                        MAP_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
                        MAP_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, periodPWM); // Establece periodo
                        MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2,ui32DutyCycle); //Establece el ciclo de trabajo
                        MAP_PWMGenEnable(PWM1_BASE, PWM_GEN_1); // Habilita generador
                        MAP_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true); // Habiloita salidas PWM
                        }
                    }
                }
            }
        }

/****************************************************************************************************************************************/
/*                                                                                                                                      */
/*                                                     Motor Derecho                                                                    */
/*                                                                                                                                      */
/****************************************************************************************************************************************/

    static portTASK_FUNCTION( MDTask, pvParameters ){

        vTaskSuspend( manejadora_dch);
        int8_t potencia_dch = 0;
        uint32_t periodPWM = PERIOD_PWM; // Ciclos para 20ms con el reloj del sistema

        while(1)
        {
            if (xQueueReceive(cola_motor_dcha,&potencia_dch,portMAX_DELAY)==pdTRUE)
            {

                if (potencia_dch == 101) {

                    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false); // Habiloita salidas PWM
                    MAP_PWMGenDisable (PWM0_BASE, PWM_GEN_1);
                    vTaskDelete (manejadora_dch);
                }

                else {

                    uint32_t ui32DutyCycle = (uint32_t) ((periodPWM * abs (potencia_dch)) / 100);

                    if (ui32DutyCycle == 0) {

                        PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false); // Habiloita salidas PWM
                    }

                    else {

                        MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
                        MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, periodPWM); // Establece periodo
                        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,ui32DutyCycle); //Establece el ciclo de trabajo
                        MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1); // Habilita generador
                        MAP_PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true); // Habiloita salidas PWM
                    }
                }
            }
        }
    }

/****************************************************************************************************************************************/
/*                                                                                                                                      */
/*                                                     Parametros de mvto                                                               */
/*                                                                                                                                      */
/****************************************************************************************************************************************/

    void vTimerCallback( TimerHandle_t pxTimer )
        {

            xSemaphoreGive(semaforo_timer);
        }

    static portTASK_FUNCTION( MVTOTask, pvParameters ){

            vTaskSuspend( manejadora_mvto);
            float Vact = 0;
            float heading = 0;
            float distancia_recorrida = 0;

            float Vmax = 10; //10 cm/s para el 100% de la onda PWM
            float tiempo = 0.2; // 200 ms
            uint8_t longitud = 10;// 10 cm

            int8_t potencia_dch = 0;
            int8_t potencia_izq = 0;
            uint8_t pui8Frame[MAX_FRAME_SIZE];
            int32_t i32Numdatos;

            parametros variables;
            QueueSetMemberHandle_t  Activado;

            xQueueOverwrite (mailbox_cambio_parametros,0);


            for( ;; )
                {
                Activado = xQueueSelectFromSet( grupo_mvto, portMAX_DELAY);

                   if (Activado == cola_mvto_izq) { //motor izquierdo

                        xQueueReceive(cola_mvto_izq,&potencia_izq,portMAX_DELAY);
                        Vact =  Vmax * (potencia_dch + potencia_izq) / (2*100);
                        heading = heading + (tiempo * Vmax * (- potencia_dch + potencia_izq)) / (longitud * 100);
                        distancia_recorrida = distancia_recorrida + abs (Vact) * tiempo;

                    }

                    else if (Activado == cola_mvto_dcha) { //motor derecho

                        xQueueReceive(cola_mvto_dcha,&potencia_dch,portMAX_DELAY);
                        Vact = (Vmax * (potencia_dch + potencia_izq)) / (2*100);
                        heading = heading + (tiempo * Vmax * (- potencia_dch + potencia_izq)) / (longitud * 100);
                        distancia_recorrida = distancia_recorrida + abs (Vact) * tiempo;

                    }

                    else if (Activado == semaforo_timer) { //motor derecho

                        PARAM_MENSAJE_MVTO_MOTOR parametro;
                        xSemaphoreTake(semaforo_timer,0);

                        if (potencia_izq == 101 || potencia_dch == 101) {

                            Vact = (Vmax * (potencia_dch + potencia_izq)) / (2*100);
                            heading = heading + (tiempo * Vmax * (- potencia_dch + potencia_izq)) / (longitud * 100);
                            distancia_recorrida = distancia_recorrida + abs (Vact) * tiempo;


                            parametro.Vact=Vact;
                            parametro.distancia_recorrida = distancia_recorrida;
                            parametro.heading = heading;
                            i32Numdatos=create_frame(pui8Frame,MENSAJE_MVTO_MOTOR,&parametro,sizeof(parametro),MAX_FRAME_SIZE);

                           if (i32Numdatos>=0)
                           {
                               xSemaphoreTake(mutex_Qt,portMAX_DELAY);
                               send_frame(pui8Frame,i32Numdatos);
                               xSemaphoreGive(mutex_Qt);

                           }

                            vTaskDelete (manejadora_mvto);
                        }

                        else {

                            Vact = (Vmax * (potencia_dch + potencia_izq)) / (2*100);
                            heading = heading + (tiempo * Vmax * (- potencia_dch + potencia_izq)) / (longitud * 100);
                            distancia_recorrida = distancia_recorrida + abs (Vact) * tiempo;

                            if (xQueuePeek (mailbox_cambio_parametros, &variables, portMAX_DELAY) == pdTRUE) {

                               if (variables.energia == -1) {

                                   distancia_recorrida = variables.distancia;
                                   variables.energia = -2;
                                   variables.distancia = -2;
                                   xQueueOverwrite (mailbox_cambio_parametros,&variables);
                               }
                            }

                            parametro.Vact=Vact;
                            parametro.distancia_recorrida = distancia_recorrida;
                            parametro.heading = heading;
                            i32Numdatos=create_frame(pui8Frame,MENSAJE_MVTO_MOTOR,&parametro,sizeof(parametro),MAX_FRAME_SIZE);

                           if (i32Numdatos>=0)
                           {
                               xSemaphoreTake(mutex_Qt,portMAX_DELAY);
                               send_frame(pui8Frame,i32Numdatos);
                               xSemaphoreGive(mutex_Qt);

                           }
                       }
                    }
                }
            }

/****************************************************************************************************************************************/
/*                                                                                                                                      */
/*                                                     Sensores                                                                         */
/*                                                                                                                                      */
/****************************************************************************************************************************************/
    void vTimerCallback_autodestruccion( TimerHandle_t pxTimer )
        {

        //PARAM_MENSAJE_AUTODESTRUCCION destruir;
        //destruir.destruir = true;
        uint8_t pui8Frame[MAX_FRAME_SIZE];
        int32_t i32Numdatos;
        i32Numdatos=create_frame(pui8Frame,MENSAJE_AUTODESTRUCCION,NULL,0,MAX_FRAME_SIZE);

       if (i32Numdatos>=0)
       {
           xSemaphoreTake(mutex_Qt,portMAX_DELAY);
           send_frame(pui8Frame,i32Numdatos);
           xSemaphoreGive(mutex_Qt);
       }

       vTaskDelete (manejadora_dch);
       vTaskDelete (manejadora_iz);
       vTaskDelete (manejadora_mvto);
       vTaskDelete (manejadora_energia);
       vTaskDelete (manejadora_alarma);
       vTaskDelete (manejadora_sensor);
       vTaskDelete (manejadora_panel);

       MAP_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, false);
       IntDisable(INT_ADC0SS1);
       MAP_IntDisable(INT_GPIOD);
       MAP_IntDisable(INT_GPIOF);
       RGBBlinkRateSet(0);

       xTimerDelete (xTimer, portMAX_DELAY);
      xTimerDelete (xTimer_en, portMAX_DELAY);
      xTimerDelete (xTimer_peligro, portMAX_DELAY);
      xTimerDelete (xTimer_quemado, portMAX_DELAY);
      xTimerDelete (xTimer_panel, portMAX_DELAY);
      xTimerDelete (xTimer_autodestruccion, portMAX_DELAY);
    }

    static portTASK_FUNCTION( SensorTask, pvParameters ){

        vTaskSuspend( manejadora_sensor);

        uint8_t pui8Frame[MAX_FRAME_SIZE];
        PARAM_MENSAJE_SENSORES parametro;
        parametro.fLeft = 0;
        parametro.fRight = 0;
        parametro.fBack = 0;
        parametro.distancia = 0;

        int32_t valor_act = 0;
        int32_t valor_ant = 2048;
        int32_t i32Numdatos;
        uint32_t distancia;
        int32_t i32Status [2] = {0, 0};
        bool soltado = true;
        uint8_t activado = 0;

        QueueSetMemberHandle_t  Activado;

        for( ;; )
            {

            Activado = xQueueSelectFromSet( grupo_sensor, portMAX_DELAY);

                if (Activado == cola_botones) {

                    xQueueReceive(cola_botones,&i32Status,portMAX_DELAY);
                    parametro.fLeft = 0;
                    parametro.fRight = 0;
                    parametro.fBack = 0;
                    soltado = true;

                    if((i32Status [0] & LEFT_BUTTON) == 0) {
                        parametro.fLeft = 1;
                        soltado = false;

                        if (xQueuePeek (mailbox_modo_traza, &activado, portMAX_DELAY) == pdTRUE) {

                           if (activado == 1) {

                               UARTprintf("Sensor izquierdo activado\r\n");
                           }
                        }

                        xTimerReset( xTimer_peligro, 0 );
                        xTimerStart (xTimer_peligro, 0);
                    }

                    if((i32Status [0] & RIGHT_BUTTON) == 0) {
                        parametro.fRight = 1;
                        soltado = false;

                        if (xQueuePeek (mailbox_modo_traza, &activado, portMAX_DELAY) == pdTRUE) {

                           if (activado == 1) {

                               UARTprintf("Sensor derecho activado\r\n");
                           }
                        }

                        xTimerReset( xTimer_peligro, 0 );
                        xTimerStart (xTimer_peligro, 0);
                    }

                    if((i32Status [1] & BACK_BUTTON) == 0) {
                        parametro.fBack = 1;
                        soltado = false;

                        if (xQueuePeek (mailbox_modo_traza, &activado, portMAX_DELAY) == pdTRUE) {

                           if (activado == 1) {

                               UARTprintf("Sensor trasero activado\r\n");
                           }
                        }

                        xTimerReset( xTimer_autodestruccion, 0 );
                        xTimerStart (xTimer_autodestruccion, 0);
                    }

                    i32Numdatos=create_frame(pui8Frame,MENSAJE_SENSORES,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                    if (i32Numdatos>=0)
                    {
                        xSemaphoreTake(mutex_Qt,portMAX_DELAY);
                        send_frame(pui8Frame,i32Numdatos);
                        xSemaphoreGive(mutex_Qt);
                    }

                    if (soltado) {

                        xTimerReset(xTimer_autodestruccion, 0 );
                        xTimerStop (xTimer_autodestruccion, 0);
                        xTimerReset(xTimer_peligro, 0 );
                        xTimerStop (xTimer_peligro, 0);
                        xTimerReset(xTimer_quemado, 0 );
                        xTimerStop (xTimer_quemado, 0);
                        xEventGroupSetBits(FlagsEventos, SOLUCIONADO_FLAG);
                    }
                }

                if (Activado == cola_infrarrojo) {

                    xQueueReceive(cola_infrarrojo,&distancia,portMAX_DELAY);
                    parametro.distancia = distancia;

                    i32Numdatos=create_frame(pui8Frame,MENSAJE_SENSORES,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                    if (i32Numdatos>=0)
                    {
                        xSemaphoreTake(mutex_Qt,portMAX_DELAY);
                        send_frame(pui8Frame,i32Numdatos);
                        xSemaphoreGive(mutex_Qt);
                    }
                }

                if (Activado = mailbox_sensor_caida) {

                    xQueuePeek (mailbox_sensor_caida, &valor_act, portMAX_DELAY);

                    if (abs (valor_act - valor_ant) > 2048) {

                        xEventGroupSetBits(FlagsEventos, CAIDA_FLAG);
                    }

                    valor_ant = valor_act;
                }
            }
        }

/****************************************************************************************************************************************/
/*                                                                                                                                      */
/*                                                    Energia                                                                           */
/*                                                                                                                                      */
/****************************************************************************************************************************************/


    void vTimerCallback_en( TimerHandle_t pxTimer )
        {

            xSemaphoreGive(semaforo_energia);
        }


    static portTASK_FUNCTION( EnergiaTask, pvParameters ){

        int32_t an = 0;

        vTaskSuspend( manejadora_energia);
        xQueueOverwrite (mailbox_energia_izq,&an);
        xQueueOverwrite (mailbox_energia_dch,&an);
        xQueueOverwrite (mailbox_panel_solar,0);

        int32_t energia = 100000;
        int8_t potencia = 101;
        int8_t potencia_izq = 0;
        int8_t potencia_dch = 0;
        int8_t pot = 0;
        uint8_t panel = 0;
        int8_t salvado = 0;
        uint32_t verde[3] = {0, 255 << 8, 0};
        float intensidad = 0.99;

        PARAM_MENSAJE_ENERGIA parametro;
        parametros variables;

//        QueueSetMemberHandle_t  Activado;
        uint8_t pui8Frame[MAX_FRAME_SIZE];
        int32_t i32Numdatos;

        PARAM_MENSAJE_ALARM alarma;

        alarma.caida = false;
        alarma.energia0 = false;
        alarma.energia40 = false;
        alarma.peligro = false;
        alarma.solucionado = false;
        alarma.quemado = false;

        RGBInit(0);
        RGBColorSet(verde);
        RGBIntensitySet (intensidad);
        RGBEnable();


        for( ;; )
        {

        //Activado = xQueueSelectFromSet( grupo_energia, portMAX_DELAY);

            if (xSemaphoreTake(semaforo_energia,0) == pdTRUE) {

                if (xQueuePeek (mailbox_energia_dch, &pot, portMAX_DELAY) == pdTRUE) {

                    potencia_dch = pot;
                }

                if (xQueuePeek (mailbox_energia_izq, &pot, portMAX_DELAY) == pdTRUE) {

                    potencia_izq = pot;
                }

                if (xQueuePeek (mailbox_cambio_parametros, &variables, portMAX_DELAY) == pdTRUE) {

                   if ((variables.distancia == -1) && (variables.energia >= 0) && (variables.energia <= 100000)) {

                       energia = variables.energia;
                       variables.energia = -2;
                       variables.distancia = -2;
                       xQueueOverwrite (mailbox_cambio_parametros,&variables);
                   }
                }

                if (xQueuePeek (mailbox_panel_solar, &panel, portMAX_DELAY) == pdTRUE) {

                    if (panel == 100) {

                        energia = energia + panel;

                        if (energia > 100000) {

                            energia = 100000;
                        }
                    }
                }

                if ((energia < 4000) && (energia > 0)) {

                    xEventGroupSetBits(FlagsEventos, ENERGIA40_FLAG);
                    salvado = 1;
                }

                if (energia <= 0) {

                    xEventGroupSetBits(FlagsEventos, ENERGIA0_FLAG);

                    xQueueSend (cola_motor_dcha,&potencia,portMAX_DELAY);
                    xQueueSend (cola_motor_izq,&potencia,portMAX_DELAY);
                    vTaskDelete (manejadora_mvto);

                }

                energia = energia - abs (potencia_izq) - abs (potencia_dch);
                parametro.energia = energia;

                i32Numdatos=create_frame(pui8Frame,MENSAJE_ENERGIA,&parametro,sizeof(parametro),MAX_FRAME_SIZE);

                if (i32Numdatos>=0)
                {

                    if (energia <= 0) {

                        parametro.energia = 0;
                    }

                    xSemaphoreTake(mutex_Qt,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(mutex_Qt);

                    if (energia > 4000) {

                        if (salvado != 0) {

                            salvado = 0;
                            RGBBlinkRateSet(10000.0f);
                            intensidad = (0.99 * energia) / 100000;
                            RGBIntensitySet (intensidad);
                            RGBColorSet(verde);

                            alarma.energia40 = false;
                            i32Numdatos=create_frame(pui8Frame,MENSAJE_ALARM,&alarma,sizeof(alarma),MAX_FRAME_SIZE);
                            if (i32Numdatos>=0)
                            {
                                xSemaphoreTake(mutex_Qt,portMAX_DELAY);
                                send_frame(pui8Frame,i32Numdatos);
                                xSemaphoreGive(mutex_Qt);
                            }

                            UARTprintf("Energia reestablecida\r\n");
                        }

                        else {

                              RGBBlinkRateSet(10000.0f);
                              intensidad = (0.99 * energia) / 100000;
                              RGBIntensitySet (intensidad);
                        }
                    }
                }

                if (potencia_izq == 101 || potencia_dch == 101) {

                    vTaskDelete (manejadora_energia);
                }
            }
        }
    }



/****************************************************************************************************************************************/
/*                                                                                                                                      */
/*                                                     Alarma                                                                           */
/*                                                                                                                                      */
/****************************************************************************************************************************************/


    void vTimerCallback_peligro( TimerHandle_t pxTimer )
    {

        xEventGroupSetBits(FlagsEventos, PELIGRO_FLAG);
        xTimerStop ( xTimer_peligro, 0);
        xTimerReset (xTimer_quemado, 0);
        xTimerStart (xTimer_quemado, 0);

    }

    void vTimerCallback_quemado( TimerHandle_t pxTimer )
    {

        xEventGroupSetBits(FlagsEventos, QUEMADO_FLAG);

    }

    static portTASK_FUNCTION( AlarmTask, pvParameters ){

        vTaskSuspend( manejadora_alarma);
        xQueueOverwrite (mailbox_modo_traza,0);

        EventBits_t uxBits;
        PARAM_MENSAJE_ALARM parametro;
        int32_t i32Numdatos;
        uint8_t pui8Frame[MAX_FRAME_SIZE];
        int8_t potencia = 101;
        uint8_t activado = 0;

        parametro.caida = false;
        parametro.energia0 = false;
        parametro.energia40 = false;
        parametro.peligro = false;
        parametro.solucionado = false;
        parametro.quemado = false;

        uint32_t rojo[3] = {255 << 8, 0, 0};
        uint32_t azul[3] = {0, 0, 255 << 8};

        int8_t en40 =  0;
        int8_t en0 = 0;

        for( ;; )
        {
            uxBits = xEventGroupWaitBits(FlagsEventos, FUERA_ALCANCE_FLAG|SOLUCIONADO_FLAG|PELIGRO_FLAG|CAIDA_FLAG|QUEMADO_FLAG|ENERGIA40_FLAG|ENERGIA0_FLAG, pdTRUE, pdFALSE, portMAX_DELAY);

            if ((uxBits & ENERGIA40_FLAG) == ENERGIA40_FLAG) {

                MAP_PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);
                parametro.energia40 = true;
                i32Numdatos=create_frame(pui8Frame,MENSAJE_ALARM,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                if (i32Numdatos>=0)
                {
                    xSemaphoreTake(mutex_Qt,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(mutex_Qt);
                }

                RGBBlinkRateSet(128.0f);
                RGBColorSet(rojo);
                RGBIntensitySet(0.99);

                if (xQueuePeek (mailbox_modo_traza, &activado, portMAX_DELAY) == pdTRUE) {

                   if (activado == 1 && en40 == 0) {

                       UARTprintf("Energia menor del 4 por ciento\r\n");
                       en40 = 1;
                   }
                }
            }


            if ((uxBits & ENERGIA0_FLAG) == ENERGIA0_FLAG) {

                parametro.energia0 = true;
                i32Numdatos=create_frame(pui8Frame,MENSAJE_ALARM,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                if (i32Numdatos>=0)
                {
                    xSemaphoreTake(mutex_Qt,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(mutex_Qt);
                }

                RGBBlinkRateSet(64.0f);

                if (xQueuePeek (mailbox_modo_traza, &activado, portMAX_DELAY) == pdTRUE) {

                   if (activado == 1 && en0 == 0) {

                       UARTprintf("Energia a 0\r\n");
                       en0 = 1;
                   }
                }
            }

            if ((uxBits & CAIDA_FLAG) == CAIDA_FLAG) {

                parametro.caida = true;
                i32Numdatos=create_frame(pui8Frame,MENSAJE_ALARM,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                if (i32Numdatos>=0)
                {
                    xSemaphoreTake(mutex_Qt,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(mutex_Qt);
                }

                RGBColorSet(rojo);
                RGBIntensitySet(0.99);

                if (xQueuePeek (mailbox_modo_traza, &activado, portMAX_DELAY) == pdTRUE) {

                   if (activado == 1 ) {

                       UARTprintf("Robot cayedose\r\n");
                   }
                }

                ADCSequenceDisable(ADC0_BASE, 1);
                MAP_IntDisable(INT_GPIOD);
                MAP_IntDisable(INT_GPIOF);

                xQueueSend (cola_motor_dcha,&potencia,portMAX_DELAY);
                xQueueSend (cola_motor_izq,&potencia,portMAX_DELAY);

                vTaskDelete (manejadora_mvto);
                vTaskDelete (manejadora_energia);
                vTaskDelete (manejadora_alarma);
                vTaskDelete (manejadora_sensor);

                xTimerDelete (xTimer, portMAX_DELAY);
                xTimerDelete (xTimer_en, portMAX_DELAY);
                xTimerDelete (xTimer_peligro, portMAX_DELAY);
                xTimerDelete (xTimer_quemado, portMAX_DELAY);
                xTimerDelete (xTimer_panel, portMAX_DELAY);
                xTimerDelete (xTimer_autodestruccion, portMAX_DELAY);
            }

            if ((uxBits & PELIGRO_FLAG) == PELIGRO_FLAG) {

                parametro.peligro = true;
                i32Numdatos=create_frame(pui8Frame,MENSAJE_ALARM,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                if (i32Numdatos>=0)
                {
                    xSemaphoreTake(mutex_Qt,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(mutex_Qt);
                }

                parametro.peligro = false;

                if (xQueuePeek (mailbox_modo_traza, &activado, portMAX_DELAY) == pdTRUE) {

                   if (activado == 1) {

                       UARTprintf("Peligro activado\r\n");
                   }
                }
            }

            if ((uxBits & SOLUCIONADO_FLAG) == SOLUCIONADO_FLAG) {

                parametro.solucionado = true;
                i32Numdatos=create_frame(pui8Frame,MENSAJE_ALARM,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                if (i32Numdatos>=0)
                {
                    xSemaphoreTake(mutex_Qt,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(mutex_Qt);
                }

                parametro.solucionado = false;
            }

            if ((uxBits & QUEMADO_FLAG) == QUEMADO_FLAG) {

                parametro.quemado = true;
                i32Numdatos=create_frame(pui8Frame,MENSAJE_ALARM,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                if (i32Numdatos>=0)
                {
                    xSemaphoreTake(mutex_Qt,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(mutex_Qt);
                }

                RGBColorSet(azul);
                RGBIntensitySet(0.99);

                if (xQueuePeek (mailbox_modo_traza, &activado, portMAX_DELAY) == pdTRUE) {

                   if (activado == 1) {

                       UARTprintf("Flag quemado\r\n");
                   }
                }

                vTaskDelete (manejadora_energia);
                vTaskDelete (manejadora_mvto);
                vTaskDelete (manejadora_dch);
                vTaskDelete (manejadora_iz);

//                xTimerDelete (xTimer, portMAX_DELAY);
//                xTimerDelete (xTimer_en, portMAX_DELAY);
//                xTimerDelete (xTimer_peligro, portMAX_DELAY);
//                xTimerDelete (xTimer_quemado, portMAX_DELAY);
//                xTimerDelete (xTimer_panel, portMAX_DELAY);
//                xTimerDelete (xTimer_autodestruccion, portMAX_DELAY);

                parametro.quemado = false;
            }

            if ((uxBits & FUERA_ALCANCE_FLAG) == FUERA_ALCANCE_FLAG) {

               PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, false); // Habiloita salidas PWM
               PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false); // Habiloita salidas PWM
               RGBDisable ();

               vTaskDelete (manejadora_usb);
               vTaskDelete (manejadora_dch);
               vTaskDelete (manejadora_iz);
               vTaskDelete (manejadora_mvto);
               vTaskDelete (manejadora_energia);
               vTaskDelete (manejadora_alarma);
               vTaskDelete (manejadora_sensor);
               vTaskDelete (manejadora_panel);

               IntDisable(INT_ADC0SS1); // Habilitación a nivel global del sistema
               MAP_IntDisable(INT_GPIOD);
               MAP_IntDisable(INT_GPIOF);

               xTimerDelete (xTimer, portMAX_DELAY);
               xTimerDelete (xTimer_en, portMAX_DELAY);
               xTimerDelete (xTimer_peligro, portMAX_DELAY);
               xTimerDelete (xTimer_quemado, portMAX_DELAY);
               xTimerDelete (xTimer_panel, portMAX_DELAY);
               xTimerDelete (xTimer_autodestruccion, portMAX_DELAY);
            }
        }
    }

/****************************************************************************************************************************************/
/*                                                                                                                                      */
/*                                                         Panel Solar                                                                  */
/*                                                                                                                                      */
/****************************************************************************************************************************************/

    void vTimerCallback_panel( TimerHandle_t pxTimer )
        {

            uint8_t energia = 100;
            xQueueOverwrite (mailbox_panel_solar,&energia);

        }

    static portTASK_FUNCTION(PanelTask,pvParameters)
    {

        vTaskSuspend( manejadora_panel);
        PARAM_MENSAJE_PANEL_SOLAR panel;

        while(1)
        {
            xQueueReceive (cola_panel_solar,&panel,portMAX_DELAY);

            if (panel.encendido) {

                xTimerReset( xTimer_panel, 0 );
                xTimerStart (xTimer_panel, 0);
            }

            else {

                xTimerStop (xTimer_panel, 0);
                xQueueOverwrite (mailbox_panel_solar,0);

            }
        }
    }

/****************************************************************************************************************************************/
/*                                                                                                                                      */
/*                                                     MAIN                                                                             */
/*                                                                                                                                      */
/****************************************************************************************************************************************/


    int main(void)
    {

// ------------------------------------------------------------------------------------------------------------------------------------ //
// -------------------------------------------------- RELOJ --------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------ //


        // Set the clocking to run at 40 MHz from the PLL.
        //
        MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |SYSCTL_OSC_MAIN);  //Ponermos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)

        //Configure PWM Clock
        MAP_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

// ------------------------------------------------------------------------------------------------------------------------------------ //
// --------------------------------------------------- PWM ---------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------ //


        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // NHR: Se usara salidas puerto B para PWM
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // NHR: Se usara salidas puerto B para PWM
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // NHR: Se usara salidas puerto E para PWM
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  // NHR: Se usarï¿½ el PWM1
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  // NHR: Se usarï¿½ el PWM1

        MAP_SysCtlPeripheralSleepEnable (SYSCTL_PERIPH_PWM1);
        MAP_SysCtlPeripheralSleepEnable (SYSCTL_PERIPH_PWM0);

        MAP_GPIOPinConfigure(GPIO_PB5_M0PWM3); // NHR: Configura PB5 como salida 3 del modulo PWM 0(ver pinmap.h)
        MAP_GPIOPinConfigure(GPIO_PE5_M1PWM3); // NHR: Configura PE5 como salida 3 del modulo PWM 1(ver pinmap.h)
        MAP_GPIOPinConfigure(GPIO_PE4_M1PWM2); // NHR: Configura PE4 como salida 2 del modulo PWM 1(ver pinmap.h)

        MAP_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5); // NHR: PB5  van a tener funciï¿½n PWM
        MAP_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5); // NHR: PE4  van a tener funciï¿½n PWM

        MAP_SysCtlPeripheralSleepEnable (SYSCTL_PERIPH_TIMER0);
        MAP_SysCtlPeripheralSleepEnable (SYSCTL_PERIPH_TIMER1);


// ------------------------------------------------------------------------------------------------------------------------------------ //
// ---------------------------------------------------- CPU --------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------ //

        // Get the system clock speed.
        g_ui32SystemClock = SysCtlClockGet();

        //Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
        //                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
        MAP_SysCtlPeripheralClockGating(true);
        CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 3);

// ------------------------------------------------------------------------------------------------------------------------------------ //
// --------------------------------------------------- BOTONES ------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------ //

        //Inicializa los botones (tambien en el puerto F) y habilita sus interrupciones
        ButtonsInit();
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0);
        MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_BOTH_EDGES);
        MAP_IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);// Misma prioridad que configMAX_SYSCALL_INTERRUPT_PRIORITY
        MAP_SysCtlPeripheralSleepEnable (INT_GPIOF);

        //Inicializa los botones (tambien en el puerto D) y habilita sus interrupciones
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // PUERTO D
        MAP_GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_1,GPIO_BOTH_EDGES);
        MAP_SysCtlPeripheralSleepEnable (SYSCTL_PERIPH_GPIOD);
        MAP_GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_DIR_MODE_IN);
        MAP_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_1,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
        MAP_IntPrioritySet(INT_GPIOD,configMAX_SYSCALL_INTERRUPT_PRIORITY);// Misma prioridad que configMAX_SYSCALL_INTERRUPT_PRIORITY
        MAP_SysCtlPeripheralSleepEnable (INT_GPIOD);

        // Una prioridad menor (mayor numero) podria dar problemas si la interrupcion ejecuta llamadas a funciones de FreeRTOS
        MAP_GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
        MAP_IntEnable(INT_GPIOF);

        MAP_GPIOIntEnable(GPIO_PORTD_BASE,GPIO_PIN_1 );
        MAP_IntEnable(INT_GPIOD);


// ------------------------------------------------------------------------------------------------------------------------------------ //
// ---------------------------------------------------- ADC --------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------ //

        MAP_GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2,GPIO_BOTH_EDGES);
        MAP_GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2, GPIO_DIR_MODE_IN);
        MAP_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);   // Habilita ADC0
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);// Habilita periferico Timer2
        MAP_SysCtlPeripheralSleepEnable (SYSCTL_PERIPH_TIMER2);
        MAP_SysCtlPeripheralSleepEnable (SYSCTL_PERIPH_ADC0);

        // Deshabilita el secuenciador 1 (SS1) del ADC0 para su configuracion
        ADCSequenceDisable(ADC0_BASE, 1);

        // Disparo de muestreo por instrucciones de procesador
        ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);

        // Timer
        TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
        uint32_t ui32Period = SysCtlClockGet() / 4;
        TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period -1);


        // El conversor 1 es el ultimo, y  se genera un aviso de interrupcion
        ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH5);
        ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH7 | ADC_CTL_IE | ADC_CTL_END);

        // Para permitir que un timer funcione como disparo de los conversores ADC
        TimerControlTrigger (TIMER2_BASE, TIMER_A, 1);

        // Instrucciones relacionadas con la habilitación de interrupciones

        IntEnable(INT_ADC0SS1); // Habilitación a nivel global del sistema
        ADCIntEnable(ADC0_BASE,1); // Habilitación del secuenciador dentro del periférico
        ADCIntClear(ADC0_BASE,1); // Borramos posibles interrupciones pendientes

        // Tras configurar el secuenciador, se vuelve a habilitar
        ADCSequenceEnable(ADC0_BASE, 1);

        // SOBREMUESTREO
        ADCHardwareOversampleConfigure (ADC0_BASE, 64); //64 MUESTRAS, EN VEZ DE 4

        // Activa el Timer2A (empezara a funcionar)
        TimerEnable(TIMER2_BASE, TIMER_A);

// ------------------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------- Creacion de tareas --------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------ //

        // Inicializa el sistema de depuraciï¿½n por terminal UART
        if (initCommandLine(256,tskIDLE_PRIORITY + 1) != pdTRUE)
        {
            while(1);
        }

        USBSerialInit(32,32);   //Inicializo el  sistema USB

        if(xTaskCreate(USBMessageProcessingTask,"usbser",512, NULL, tskIDLE_PRIORITY + 2,  &manejadora_usb) != pdTRUE)
                {
                    while(1);
                }

        if((xTaskCreate(MITask, "Motor_izquierdo", MITASKSTACKSIZE,NULL,tskIDLE_PRIORITY + MITASKPRIO, &manejadora_iz) != pdTRUE))
                {
                    while(1);
                }

        if((xTaskCreate(MDTask, "Motor_derecho", MDTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + MDTASKPRIO, &manejadora_dch) != pdTRUE))
                {
                    while(1);
                }

        if((xTaskCreate(MVTOTask, "Calculo_movimiento", MVTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + MVTASKPRIO, &manejadora_mvto) != pdTRUE))
                {
                    while(1);
                }

        if((xTaskCreate(SensorTask, "Control_sensores", BTTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + BTTASKPRIO, &manejadora_sensor) != pdTRUE))
                {
                    while(1);
                }

        if((xTaskCreate(EnergiaTask, "Calculo_energia", ENTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + ENTASKPRIO, &manejadora_energia) != pdTRUE))
                {
                    while(1);
                }

        if((xTaskCreate(AlarmTask, "Alarmas", ALTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + ALTASKPRIO, &manejadora_alarma) != pdTRUE))
                {
                    while(1);
                }

        if((xTaskCreate(PanelTask, "Panel_solar", PSTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + PSTASKPRIO, &manejadora_panel) != pdTRUE))
                {
                    while(1);
                }

// ------------------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------ Creacion de colas ----------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------ //


        cola_motor_izq = xQueueCreate(3,sizeof(int32_t));  //espacio para 3items de tamaï¿½o ulong
                if (NULL==cola_motor_izq)
                    while(1);

        cola_motor_dcha = xQueueCreate(3,sizeof(int32_t));  //espacio para 3items de tamaï¿½o ulong
                if (NULL==cola_motor_dcha)
                    while(1);

        cola_mvto_izq = xQueueCreate(3,sizeof(int8_t));  //espacio para 3items de tamaï¿½o ulong
                        if (NULL==cola_mvto_izq)
                            while(1);

        cola_mvto_dcha = xQueueCreate(3,sizeof(int8_t));  //espacio para 3items de tamaï¿½o ulong
                if (NULL==cola_mvto_dcha)
                    while(1);

        cola_botones = xQueueCreate(3,2*sizeof(int32_t));  //espacio para 3items de tamaï¿½o ulong
                if (NULL==cola_botones)
                    while(1);

        cola_infrarrojo = xQueueCreate(3,sizeof(int32_t));  //espacio para 3items de tamaï¿½o ulong
                if (NULL==cola_infrarrojo)
                    while(1);

        mailbox_energia_dch = xQueueCreate(1,sizeof(int8_t));  //espacio para 1items de tamaï¿½o ulong
                if (NULL==mailbox_energia_dch)
                    while(1);

        mailbox_energia_izq = xQueueCreate(1,sizeof(int8_t));  //espacio para 3items de tamaï¿½o ulong
                if (NULL==mailbox_energia_izq)
                    while(1);

        cola_velocidad_energia = xQueueCreate(3,sizeof(float));  //espacio para 3items de tamaï¿½o ulong
                if (NULL==cola_velocidad_energia)
                    while(1);

        cola_velocidad_mvto = xQueueCreate(3,sizeof(float));  //espacio para 3items de tamaï¿½o ulong
                if (NULL==cola_velocidad_mvto)
                    while(1);

        cola_panel_solar = xQueueCreate(1,sizeof(bool));  //espacio para 1items de tamaï¿½o ulong
                if (NULL==cola_panel_solar)
                    while(1);

        mailbox_sensor_caida = xQueueCreate(1,sizeof(int32_t));  //espacio para 1items de tamaï¿½o ulong
                if (NULL==mailbox_sensor_caida)
                    while(1);

        mailbox_modo_traza = xQueueCreate(1,sizeof(int8_t));  //espacio para 1items de tamaï¿½o ulong
                if (NULL==mailbox_modo_traza)
                    while(1);

        mailbox_cambio_parametros = xQueueCreate(1,sizeof(parametros));  //espacio para 1items de tamaï¿½o ulong
                if (NULL==mailbox_cambio_parametros)
                    while(1);

        mailbox_panel_solar = xQueueCreate(1,sizeof(uint8_t));  //espacio para 1items de tamaï¿½o ulong
                if (NULL==mailbox_panel_solar)
                    while(1);
// ------------------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------ Creacion de semáforos ------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------ //

        semaforo_timer = xSemaphoreCreateBinary();
                if ((NULL == semaforo_timer))
                {
                    while (1);  //No hay memoria para los semaforo
                }

        semaforo_energia = xSemaphoreCreateBinary();
                if ((NULL == semaforo_energia))
                {
                    while (1);  //No hay memoria para los semaforo
                }

        mutex_Qt=xSemaphoreCreateMutex();
                if ((NULL==mutex_Qt))
                {
                    while (1);  //No hay memoria para los semaforo
                }

// ------------------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------- Flags de eventos ----------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------ //

        FlagsEventos = xEventGroupCreate();
                if( NULL ==  FlagsEventos)
                {
                    while(1);
                }

// ------------------------------------------------------------------------------------------------------------------------------------ //
// -------------------------------------------------- Grupo de colas ------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------ //

        grupo_mvto = xQueueCreateSet(10);
                if (NULL == grupo_mvto) {
                    while(1);
                }
        if (xQueueAddToSet(cola_mvto_izq, grupo_mvto)!=pdPASS)
                {
                    while(1);
                }

        if (xQueueAddToSet(cola_mvto_dcha, grupo_mvto)!=pdPASS)
                {
                    while(1);
                }


        if (xQueueAddToSet(semaforo_timer, grupo_mvto)!=pdPASS)
                {
                    while(1);
                }
//----------------------------------------------------------------------------------------//

        grupo_sensor = xQueueCreateSet(7);
                if (NULL == grupo_sensor) {
                    while(1);
                }
        if (xQueueAddToSet(cola_infrarrojo, grupo_sensor)!=pdPASS)
                {
                    while(1);
                }

        if (xQueueAddToSet(cola_botones, grupo_sensor)!=pdPASS)
                {
                    while(1);
                }
        if (xQueueAddToSet(mailbox_sensor_caida, grupo_sensor)!=pdPASS)
                {
                    while(1);
                }

//----------------------------------------------------------------------------------------//

//        grupo_energia = xQueueCreateSet(8);
//                if (NULL == grupo_energia) {
//                    while(1);
//                }

//        if (xQueueAddToSet(semaforo_energia, grupo_energia)!=pdPASS)
//                {
//                    while(1);
//                }

// ------------------------------------------------------------------------------------------------------------------------------------ //
// ----------------------------------------------- Timer Software --------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------ //

        // Create and start timer SW      1/5 * 1s;     TIMER PERIODICO; NULL; que queremos que haga cuando expire el tiempo
        xTimer = xTimerCreate("TimerSW", 0.2 * configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback);

                if( NULL == xTimer )
                     {
                        while(1);
                     }
                else{

                    if( xTimerStart( xTimer, 0 ) != pdPASS )
                    {

                        while(1);
                    }
                }

        // Create and start timer SW       1s;     TIMER PERIODICO; NULL; que queremos que haga cuando expire el tiempo
        xTimer_en = xTimerCreate("TimerSW_en", configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback_en);

                if( NULL == xTimer_en )
                     {

                        while(1);
                     }
                else{


                    if( xTimerStart( xTimer_en, 0 ) != pdPASS )
                    {
                        /* The timer could not be set into the Active state. */
                        while(1);
                    }
                }

        // Create and start timer SW       4s;     TIMER PERIODICO; NULL; que queremos que haga cuando expire el tiempo
        xTimer_peligro = xTimerCreate("TimerSW_peligro", 4 * configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback_peligro);

                if( NULL == xTimer_peligro )
                     {

                        while(1);
                     }


        // Create and start timer SW       6s;     TIMER PERIODICO; NULL; que queremos que haga cuando expire el tiempo
        xTimer_quemado = xTimerCreate("TimerSW_quemado", 6 * configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback_quemado);

                if( NULL == xTimer_quemado )
                     {
                        while(1);
                     }
        // Create and start timer SW       1s;     TIMER PERIODICO; NULL; que queremos que haga cuando expire el tiempo
        xTimer_panel = xTimerCreate("TimerSW_panel", configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback_panel);

                if( NULL == xTimer_quemado )
                     {
                        while(1);
                     }

        // Create and start timer SW       1s;     TIMER PERIODICO; NULL; que queremos que haga cuando expire el tiempo
        xTimer_autodestruccion = xTimerCreate("TimerSW_autodestruccion", 3 * configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback_autodestruccion);

                if( NULL == xTimer_autodestruccion)
                     {
                        while(1);
                     }
// ------------------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------  Scheduler ------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------ //


        vTaskStartScheduler();  //el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

        //De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.

        while(1)
        {
            //Si llego aqui es que algo raro ha pasado
        }

    } // main


// ------------------------------------------------------------------------------------------------------------------------------------ //
// -------------------------------------------- Rutinas de Interrupción --------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------ //


    void GPIOFIntHandler(void)
    {
        signed portBASE_TYPE higherPriorityTaskWoken=pdFALSE;
        int32_t i32Status [2] = {0, 0};
        i32Status [0] = MAP_GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS);
        i32Status [1] = MAP_GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_1);

        static uint32_t tiempo_ant;
        uint32_t tiempo_act;

        if (i32Status [1] == 0) {

            tiempo_ant = xTaskGetTickCountFromISR ();
        }

        if (i32Status [1] != 0) {

            tiempo_act = xTaskGetTickCountFromISR ();

            if (tiempo_act - tiempo_ant < 0.00003) { // en ms es menor que 30, no hacemos nada

                i32Status [1] = 0;
            }
        }

        xQueueSendFromISR (cola_botones,&i32Status,&higherPriorityTaskWoken);

        MAP_GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);
        MAP_GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_1);

        portEND_SWITCHING_ISR(higherPriorityTaskWoken);
    }

    void sensorInfrarrojo(void){

        signed portBASE_TYPE higherPriorityTaskWoken=pdFALSE;
        uint32_t ui32ADC0Value  [2]= {0, 0};

        uint32_t distancia = 0;
        uint32_t valor = 0;

        ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
        ADCIntClear(ADC0_BASE, 1);


        distancia = ui32ADC0Value [0]; // max 4095
        valor = ui32ADC0Value [1];

        xQueueSendFromISR (cola_infrarrojo,&distancia,&higherPriorityTaskWoken);
        xQueueOverwriteFromISR (mailbox_sensor_caida, &valor, &higherPriorityTaskWoken);

        portEND_SWITCHING_ISR(higherPriorityTaskWoken);
    }



