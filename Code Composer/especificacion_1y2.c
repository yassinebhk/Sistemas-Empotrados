    //*****************************************************************************
    //
    // Codigo de partida comunicacion TIVA-QT (Abril2021)
    // Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
    //
    //  Estructura de aplicacion basica para el desarrollo de aplicaciones genericas
    //  basada en la TIVA, en las que existe un intercambio de mensajes con un interfaz
    //  gráfico (GUI) Qt.
    //  La aplicacion se basa en un intercambio de mensajes con ordenes e informacion, a traves  de la
    //  configuracion de un perfil CDC de USB (emulacion de puerto serie) y un protocolo
    //  de comunicacion con el PC que permite recibir ciertas ordenes y enviar determinados datos en respuesta.
    //   En el ejemplo basico de partida se implementara la recepcion de un mensaje
    //  generico que permite el apagado y encendido de los LEDs de la placa; asi como un segundo
    //  mensaje enviado desde la placa al GUI, para mostrar el estado de los botones.
    //
    //*****************************************************************************
    #include<stdbool.h>
    #include<stdint.h>
    #include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
    #include "inc/hw_types.h"        // TIVA: Definiciones API
    #include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
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
    #include <utils/uartstdioMod.h>     // TIVA: Funciones API UARTSTDIO (printf)
    #include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
    #include "drivers/rgb.h"         // TIVA: Funciones API manejo de leds con PWM
    #include "FreeRTOS.h"            // FreeRTOS: definiciones generales
    #include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
    #include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos
    #include "queue.h"               // FreeRTOS: definiciones relacionadas con colas de mensajes
    #include "timers.h"              // FreeRTOS: definiciones relacionadas con timers
    #include "utils/cpu_usage.h"
    #include "commands.h"
    #include <serial2USBprotocol.h>
    #include <usb_dev_serial.h>
    #include "usb_messages_table.h"


    #define MITASKPRIO 1
    #define MITASKSTACKSIZE 128
    #define MDTASKPRIO 1
    #define MDTASKSTACKSIZE 128
    #define MVTASKPRIO 1
    #define MVTASKSTACKSIZE 128
    #define BTTASKPRIO 1
    #define BTTASKSTACKSIZE 128

    #define DIV_RELOJ_PWM 16
    #define PERIOD_PWM (SysCtlClockGet()*0.02)/DIV_RELOJ_PWM //50.000 (20 ms)

    // Variables globales "main"
    uint32_t g_ui32CPUUsage;
    uint32_t g_ui32SystemClock;


    static QueueHandle_t cola_motor_izq;
    static QueueHandle_t cola_motor_dcha;
    static QueueHandle_t cola_mvto_izq;
    static QueueHandle_t cola_mvto_dcha;
    static QueueHandle_t cola_timer;
    static QueueHandle_t cola_botones;
    static QueueSetHandle_t grupo_colas;
    static TimerHandle_t xTimer;
    static SemaphoreHandle_t semaforo_uart;


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



    //*****************************************************************************
    //
    // A continuacion van las tareas...
    //
    //*****************************************************************************

    //// Codigo para procesar los mensajes recibidos a traves del canal USB

    static portTASK_FUNCTION( USBMessageProcessingTask, pvParameters ){

        uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila
        int32_t i32Numdatos;
        uint8_t ui8Message;
        void *ptrtoreceivedparam;
        uint32_t ui32Errors=0;
        PARAM_MENSAJE_MOTOR parametro;
        int8_t potencia = 0;

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
                i32Numdatos=destuff_and_check_checksum(pui8Frame,i32Numdatos); // Primero, "destuffing" y comprobaci�n checksum
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

                        }else//Error de tama�o de parametro
                            ui32Errors++; // Tratamiento del error

                    break; // case: MENSAJE_MOTOR_IZQUIERDO

                    case MENSAJE_MOTOR_DERECHO :

                        if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0){

                            potencia = parametro.potencia_md;
                            xQueueSend (cola_motor_dcha,&potencia,portMAX_DELAY);
                            xQueueSend (cola_mvto_dcha,&potencia,portMAX_DELAY);

                        }else//Error de tama�o de parametro
                            ui32Errors++; // Tratamiento del error

                    break; // case: MENSAJE_MOTOR_DERECHO

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
                //Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
                ui32Errors++;
                // Procesamiento del error
            }
        }
    }

    // Codigo para los motores

    static portTASK_FUNCTION( MITask, pvParameters ){

        int8_t potencia_iz;
        uint32_t periodPWM=PERIOD_PWM; // Ciclos para 20ms con el reloj del sistema

        //
        // Loop forever.
        //
        while(1)
        {
            if (xQueueReceive(cola_motor_izq,&potencia_iz,portMAX_DELAY)==pdTRUE)
            {

                uint32_t ui32DutyCycle = (periodPWM * abs (potencia_iz)) / 100;

                if (ui32DutyCycle == 0) {

                    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false); // Habiloita salidas PWM
                }

                else {

                    // NHR: Usa GEN3 de PWM1. Modo alineacion izquierda con otros PWM; no sincroniza en caso de cambio de onda

                    MAP_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
                    MAP_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, periodPWM); // Establece periodo
                    MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,ui32DutyCycle); //Establece el ciclo de trabajo
                    MAP_PWMGenEnable(PWM1_BASE, PWM_GEN_3); // Habilita generador
                    MAP_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habiloita salidas PWM
                }
            }
        }
    }

    static portTASK_FUNCTION( MDTask, pvParameters ){

        int8_t potencia_dch;
        uint32_t periodPWM = PERIOD_PWM; // Ciclos para 20ms con el reloj del sistema

        //
        // Loop forever.
        //
        while(1)
        {
            if (xQueueReceive(cola_motor_dcha,&potencia_dch,portMAX_DELAY)==pdTRUE)
            {

                uint32_t ui32DutyCycle = (uint32_t) ((periodPWM * abs (potencia_dch)) / 100);

                if (ui32DutyCycle == 0) {

                    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false); // Habiloita salidas PWM
                }

                else {

                    // NHR: Usa GEN3 de PWM1. Modo alineacion izquierda con otros PWM; no sincroniza en caso de cambio de onda

                    MAP_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
                    MAP_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, periodPWM); // Establece periodo
                    MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,ui32DutyCycle); //Establece el ciclo de trabajo
                    MAP_PWMGenEnable(PWM1_BASE, PWM_GEN_3); // Habilita generador
                    MAP_PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habiloita salidas PWM

                }
            }
        }
    }

    void vTimerCallback( TimerHandle_t pxTimer )
    {

        //uint8_t fin = 1;
        xQueueSend (cola_timer,1,portMAX_DELAY);

    }

    static portTASK_FUNCTION( MVTOTask, pvParameters ){

            float Vact = 0;
            float heading = 0;
            float distancia_recorrida = 0;

            float Vmax = 10; //10 cm/s para el 100% de la onda PWM
            float tiempo = 0.2; // 200 ms
            uint8_t longitud = 10;// 10 cm

            int8_t potencia_dch = 0;
            int8_t potencia_izq = 0;
            int8_t timer;
            uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila
            int32_t i32Numdatos;

            QueueSetMemberHandle_t  Activado;


            for( ;; )
                {
                //espera a que se desbloquee uno de los IPC
                Activado = xQueueSelectFromSet( grupo_colas, portMAX_DELAY);

                //Comprueba quien ha sido

                    if (Activado == cola_mvto_izq) { //motor izquierdo

                        xQueueReceive(cola_mvto_izq,&potencia_izq,portMAX_DELAY);
                        Vact = (Vmax * (potencia_dch + potencia_izq)) / (2*100);
                        heading = (tiempo * Vmax * (- potencia_dch + potencia_izq)) / (longitud * 100);
                        distancia_recorrida = abs (Vact) * tiempo;

                    }

                    else if (Activado == cola_mvto_dcha) { //motor derecho

                        xQueueReceive(cola_mvto_dcha,&potencia_dch,portMAX_DELAY);
                        Vact = (Vmax * (potencia_dch + potencia_izq)) / (2*100);
                        heading = (tiempo * Vmax * (- potencia_dch + potencia_izq)) / (longitud * 100);
                        distancia_recorrida = abs (Vact) * tiempo;

                    }

                    else if (Activado == cola_timer) { //motor derecho

                        xQueueReceive(cola_timer,&timer,portMAX_DELAY);
                        Vact = (Vmax * (potencia_dch + potencia_izq)) / (2*100);
                        heading = (tiempo * Vmax * (- potencia_dch + potencia_izq)) / (longitud * 100);
                        distancia_recorrida = abs (Vact) * tiempo;

                        PARAM_MENSAJE_MVTO_MOTOR parametro;
                        parametro.Vact=Vact;
                        parametro.distancia_recorrida = distancia_recorrida;
                        parametro.heading = heading;
                        i32Numdatos=create_frame(pui8Frame,MENSAJE_MVTO_MOTOR,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                       if (i32Numdatos>=0)
                       {
                           xSemaphoreTake(semaforo_uart,portMAX_DELAY);
                           send_frame(pui8Frame,i32Numdatos);
                           xSemaphoreGive(semaforo_uart);

                       }
                    }
                }
            }

    static portTASK_FUNCTION( SensorTask, pvParameters ){

        uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila
        PARAM_MENSAJE_BUTTONS parametro;
        int32_t i32Numdatos;
        int32_t i32Status;
        //
        // Loop forever.
        //
        while(1)
        {
            if (xQueueReceive(cola_botones,&i32Status,portMAX_DELAY)==pdTRUE)
            {
                parametro.ui8Buttons=0;
                if((i32Status & LEFT_BUTTON) == 0)
                    parametro.button.fLeft = 1;
                if((i32Status & RIGHT_BUTTON) == 0)
                    parametro.button.fRight = 1;
                //(if((i32Status & BACK_BUTTON) == 0)
                    //parametro.button.fBack = 1;
                i32Numdatos=create_frame(pui8Frame,MENSAJE_BUTTONS,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                if (i32Numdatos>=0)
                {
                    xSemaphoreTake(semaforo_uart,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(semaforo_uart);
                }
            }
        }
    }

    //*****************************************************************************
    //
    // Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
    //
    //*****************************************************************************
    int main(void)
    {

        //
        // Set the clocking to run at 40 MHz from the PLL.
        //
        MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |SYSCTL_OSC_MAIN);  //Ponermos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)

        //Configure PWM Clock
        MAP_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // NHR: Se usara salidas puerto F para PWM
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  // NHR: Se usar� el PWM1
        MAP_SysCtlPeripheralSleepEnable (SYSCTL_PERIPH_PWM1);

        MAP_GPIOPinConfigure(GPIO_PF2_M1PWM6); // NHR: Configura PF2 como salida 6 del modulo PWM 1(ver pinmap.h)
        MAP_GPIOPinConfigure(GPIO_PF3_M1PWM7); // NHR: Configura PF3 como salida 7 del modulo PWM 1(ver pinmap.h)
        MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2); // NHR: PF2 y PF3 van a tener funci�n PWM

        // Codigo principal, (poner en bucle infinito o bajo consumo)

        // Get the system clock speed.
        g_ui32SystemClock = SysCtlClockGet();


        //Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
        //                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
        MAP_SysCtlPeripheralClockGating(true);
        CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 3);

        //Inicializa el puerto F (LEDs)
        //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        //MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
        //MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0); //LEDS APAGADOS

        //Inicializa la biblioteca RGB (configurando las salidas como RGB)
//        RGBInit(1);
//        MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);  //Esto es necesario para que el timer0 siga funcionando en bajo consumo
//        MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);  //Esto es necesario para que el timer1 siga funcionando en bajo consumo
//
        //Inicializa los botones (tambien en el puerto F) y habilita sus interrupciones
        ButtonsInit();
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0);
        MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_BOTH_EDGES);
        MAP_IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);// Misma prioridad que configMAX_SYSCALL_INTERRUPT_PRIORITY

        // Una prioridad menor (mayor numero) podria dar problemas si la interrupcion ejecuta llamadas a funciones de FreeRTOS
        MAP_GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
        MAP_IntEnable(INT_GPIOF);

        // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
        // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
        // (y por tanto este no se deberia utilizar para otra cosa).


        /**                                              Creacion de tareas                                     **/

        // Inicializa el sistema de depuraci�n por terminal UART
        if (initCommandLine(256,tskIDLE_PRIORITY + 1) != pdTRUE)
        {
            while(1);
        }

        USBSerialInit(32,32);   //Inicializo el  sistema USB

        //
        // Crea la tarea que gestiona los mensajes USB (definidos en USBMessageProcessingTask)
        //

        if(xTaskCreate(USBMessageProcessingTask,"usbser",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
                {
                    while(1);
                }

        if((xTaskCreate(MITask, "Motor_izquierdo", MITASKSTACKSIZE,NULL,tskIDLE_PRIORITY + MITASKPRIO, NULL) != pdTRUE))
                {
                    while(1);
                }

        if((xTaskCreate(MDTask, "Motor_derecho", MDTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + MDTASKPRIO, NULL) != pdTRUE))
                {
                    while(1);
                }

        if((xTaskCreate(MVTOTask, "Calculo_movimiento", MVTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + MVTASKPRIO, NULL) != pdTRUE))
                {
                    while(1);
                }

        if((xTaskCreate(SensorTask, "Control_sensores", BTTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + BTTASKPRIO, NULL) != pdTRUE))
                {
                    while(1);
                }

        cola_motor_izq = xQueueCreate(3,sizeof(int32_t));  //espacio para 3items de tama�o ulong
                if (NULL==cola_motor_izq)
                    while(1);

        cola_motor_dcha = xQueueCreate(3,sizeof(int32_t));  //espacio para 3items de tama�o ulong
                if (NULL==cola_motor_dcha)
                    while(1);

        cola_mvto_izq = xQueueCreate(3,sizeof(int8_t));  //espacio para 3items de tama�o ulong
                        if (NULL==cola_mvto_izq)
                            while(1);

        cola_mvto_dcha = xQueueCreate(3,sizeof(int8_t));  //espacio para 3items de tama�o ulong
                if (NULL==cola_mvto_dcha)
                    while(1);

        cola_timer = xQueueCreate(3,sizeof(int32_t));  //espacio para 3items de tama�o ulong
                if (NULL==cola_timer)
                    while(1);

        cola_botones = xQueueCreate(3,sizeof(int32_t));  //espacio para 3items de tama�o ulong
                if (NULL==cola_timer)
                    while(1);

        //Crea mutex
        semaforo_uart=xSemaphoreCreateMutex();
                if ((NULL==semaforo_uart))
                {
                    while (1);  //No hay memoria para los semaforo
                }

        grupo_colas = xQueueCreateSet(9);
                if (NULL == grupo_colas) {
                    while(1);
                }
        if (xQueueAddToSet(cola_mvto_izq, grupo_colas)!=pdPASS)
                {
                    while(1);
                }

        if (xQueueAddToSet(cola_mvto_dcha, grupo_colas)!=pdPASS)
                {
                    while(1);
                }


        if (xQueueAddToSet(cola_timer, grupo_colas)!=pdPASS)
                {
                    while(1);
                }
        // Create and start timer SW      1/5 * 1s;     TIMER PERIODICO; NULL; que queremos que haga cuando expire el tiempo
        xTimer = xTimerCreate("TimerSW", 0.2 * configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback);

                if( NULL == xTimer ) //COMPROBRAMOS SI HA IDO BIEN
                     {
                         /* The timer was not created. */
                        while(1);
                     }
            else{

                /* Start the timer.  No block time is specified, and even if one was
                it would be ignored because the RTOS scheduler has not yet been
                started. */
                    if( xTimerStart( xTimer, 0 ) != pdPASS )
                    {
                        /* The timer could not be set into the Active state. */
                        while(1);
                    }
                }


        //
        // Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
        //

        vTaskStartScheduler();  //el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

        //De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.

        while(1)
        {
            //Si llego aqui es que algo raro ha pasado
        }
    }


    void GPIOFIntHandler(void)
    {
        signed portBASE_TYPE higherPriorityTaskWoken=pdFALSE;   //Hay que inicializarlo a False!!
        int32_t i32Status = MAP_GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS);
        xQueueSendFromISR (cola_botones,&i32Status,&higherPriorityTaskWoken);
        MAP_GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);              //limpiamos flags
        //Cesion de control de CPU si se ha despertado una tarea de mayor prioridad
        portEND_SWITCHING_ISR(higherPriorityTaskWoken);
    }


/////////////////////////////////////////////////////////////////////////////////////////////////////////////7






    //*****************************************************************************
    //
    // Codigo de partida comunicacion TIVA-QT (Abril2021)
    // Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
    //
    //  Estructura de aplicacion basica para el desarrollo de aplicaciones genericas
    //  basada en la TIVA, en las que existe un intercambio de mensajes con un interfaz
    //  gráfico (GUI) Qt.
    //  La aplicacion se basa en un intercambio de mensajes con ordenes e informacion, a traves  de la
    //  configuracion de un perfil CDC de USB (emulacion de puerto serie) y un protocolo
    //  de comunicacion con el PC que permite recibir ciertas ordenes y enviar determinados datos en respuesta.
    //   En el ejemplo basico de partida se implementara la recepcion de un mensaje
    //  generico que permite el apagado y encendido de los LEDs de la placa; asi como un segundo
    //  mensaje enviado desde la placa al GUI, para mostrar el estado de los botones.
    //
    //*****************************************************************************
    #include<stdbool.h>
    #include<stdint.h>

    #include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
    #include "inc/hw_types.h"        // TIVA: Definiciones API
    #include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones

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

#include <utils/uartstdioMod.h>     // TIVA: Funciones API UARTSTDIO (printf)
    #include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
    #include "drivers/rgb.h"         // TIVA: Funciones API manejo de leds con PWM
    #include "FreeRTOS.h"            // FreeRTOS: definiciones generales
    #include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
    #include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos
    #include "queue.h"               // FreeRTOS: definiciones relacionadas con colas de mensajes
    #include "timers.h"              // FreeRTOS: definiciones relacionadas con timers
    #include "utils/cpu_usage.h"
    #include "commands.h"

    #include <serial2USBprotocol.h>
    #include <usb_dev_serial.h>
    #include "usb_messages_table.h"


    #define MITASKPRIO 1
    #define MITASKSTACKSIZE 128
    #define MDTASKPRIO 1
    #define MDTASKSTACKSIZE 128
    #define MVTASKPRIO 1
    #define MVTASKSTACKSIZE 128
    #define BTTASKPRIO 1
    #define BTTASKSTACKSIZE 128

    #define DIV_RELOJ_PWM 16
    #define PERIOD_PWM (SysCtlClockGet()*0.02)/DIV_RELOJ_PWM //50.000 (20 ms)

    // Variables globales "main"
    uint32_t g_ui32CPUUsage;
    uint32_t g_ui32SystemClock;


    static QueueHandle_t cola_motor_izq;
    static QueueHandle_t cola_motor_dcha;
    static QueueHandle_t cola_mvto_izq;
    static QueueHandle_t cola_mvto_dcha;
    static QueueHandle_t cola_timer;
    static QueueHandle_t cola_botones;
    static QueueSetHandle_t grupo_colas;
    static TimerHandle_t xTimer;
    static SemaphoreHandle_t semaforo_uart;


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



    //*****************************************************************************
    //
    // A continuacion van las tareas...
    //
    //*****************************************************************************

    //// Codigo para procesar los mensajes recibidos a traves del canal USB

    static portTASK_FUNCTION( USBMessageProcessingTask, pvParameters ){

        uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila
        int32_t i32Numdatos;
        uint8_t ui8Message;
        void *ptrtoreceivedparam;
        uint32_t ui32Errors=0;
        PARAM_MENSAJE_MOTOR parametro;
        int8_t potencia = 0;

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
                i32Numdatos=destuff_and_check_checksum(pui8Frame,i32Numdatos); // Primero, "destuffing" y comprobaci�n checksum
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

                        }else//Error de tama�o de parametro
                            ui32Errors++; // Tratamiento del error

                    break; // case: MENSAJE_MOTOR_IZQUIERDO

                    case MENSAJE_MOTOR_DERECHO :

                        if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0){

                            potencia = parametro.potencia_md;
                            xQueueSend (cola_motor_dcha,&potencia,portMAX_DELAY);
                            xQueueSend (cola_mvto_dcha,&potencia,portMAX_DELAY);

                        }else//Error de tama�o de parametro
                            ui32Errors++; // Tratamiento del error

                    break; // case: MENSAJE_MOTOR_DERECHO

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
                //Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
                ui32Errors++;
                // Procesamiento del error
            }
        }
    }

    // Codigo para los motores

    static portTASK_FUNCTION( MITask, pvParameters ){

        int8_t potencia_iz;
        uint32_t periodPWM=PERIOD_PWM; // Ciclos para 20ms con el reloj del sistema

        //
        // Loop forever.
        //
        while(1)
        {
            if (xQueueReceive(cola_motor_izq,&potencia_iz,portMAX_DELAY)==pdTRUE)
            {

                uint32_t ui32DutyCycle = (periodPWM * abs (potencia_iz)) / 100;

                if (ui32DutyCycle == 0) {

                    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false); // Habiloita salidas PWM
                }

                else {

                    // NHR: Usa GEN3 de PWM1. Modo alineacion izquierda con otros PWM; no sincroniza en caso de cambio de onda

                    MAP_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
                    MAP_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, periodPWM); // Establece periodo
                    MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,ui32DutyCycle); //Establece el ciclo de trabajo
                    MAP_PWMGenEnable(PWM1_BASE, PWM_GEN_3); // Habilita generador
                    MAP_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habiloita salidas PWM
                }
            }
        }
    }

    static portTASK_FUNCTION( MDTask, pvParameters ){

        int8_t potencia_dch;
        uint32_t periodPWM = PERIOD_PWM; // Ciclos para 20ms con el reloj del sistema

        //
        // Loop forever.
        //
        while(1)
        {
            if (xQueueReceive(cola_motor_dcha,&potencia_dch,portMAX_DELAY)==pdTRUE)
            {

                uint32_t ui32DutyCycle = (uint32_t) ((periodPWM * abs (potencia_dch)) / 100);

                if (ui32DutyCycle == 0) {

                    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false); // Habiloita salidas PWM
                }

                else {

                    // NHR: Usa GEN3 de PWM1. Modo alineacion izquierda con otros PWM; no sincroniza en caso de cambio de onda

                    MAP_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
                    MAP_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, periodPWM); // Establece periodo
                    MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,ui32DutyCycle); //Establece el ciclo de trabajo
                    MAP_PWMGenEnable(PWM1_BASE, PWM_GEN_3); // Habilita generador
                    MAP_PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habiloita salidas PWM

                }
            }
        }
    }

    void vTimerCallback( TimerHandle_t pxTimer )
    {

        //uint8_t fin = 1;
        xQueueSend (cola_timer,1,portMAX_DELAY);

    }

    static portTASK_FUNCTION( MVTOTask, pvParameters ){

            float Vact = 0;
            float heading = 0;
            float distancia_recorrida = 0;

            float Vmax = 10; //10 cm/s para el 100% de la onda PWM
            float tiempo = 0.2; // 200 ms
            uint8_t longitud = 10;// 10 cm

            int8_t potencia_dch = 0;
            int8_t potencia_izq = 0;
            int8_t timer;
            uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila
            int32_t i32Numdatos;

            QueueSetMemberHandle_t  Activado;


            for( ;; )
                {
                //espera a que se desbloquee uno de los IPC
                Activado = xQueueSelectFromSet( grupo_colas, portMAX_DELAY);

                //Comprueba quien ha sido

                    if (Activado == cola_mvto_izq) { //motor izquierdo

                        xQueueReceive(cola_mvto_izq,&potencia_izq,portMAX_DELAY);
                        Vact = (Vmax * (potencia_dch + potencia_izq)) / (2*100);
                        heading = (tiempo * Vmax * (- potencia_dch + potencia_izq)) / (longitud * 100);
                        distancia_recorrida = abs (Vact) * tiempo;

                    }

                    else if (Activado == cola_mvto_dcha) { //motor derecho

                        xQueueReceive(cola_mvto_dcha,&potencia_dch,portMAX_DELAY);
                        Vact = (Vmax * (potencia_dch + potencia_izq)) / (2*100);
                        heading = (tiempo * Vmax * (- potencia_dch + potencia_izq)) / (longitud * 100);
                        distancia_recorrida = abs (Vact) * tiempo;

                    }

                    else if (Activado == cola_timer) { //motor derecho

                        xQueueReceive(cola_timer,&timer,portMAX_DELAY);
                       Vact = (Vmax * (potencia_dch + potencia_izq)) / (2*100);
                        heading = (tiempo * Vmax * (- potencia_dch + potencia_izq)) / (longitud * 100);
                        distancia_recorrida = abs (Vact) * tiempo;

                        PARAM_MENSAJE_MVTO_MOTOR parametro;
                        parametro.Vact=Vact;
                        parametro.distancia_recorrida = distancia_recorrida;
                        parametro.heading = heading;
                        i32Numdatos=create_frame(pui8Frame,MENSAJE_MVTO_MOTOR,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                       if (i32Numdatos>=0)
                       {
                           xSemaphoreTake(semaforo_uart,portMAX_DELAY);
                           send_frame(pui8Frame,i32Numdatos);
                           xSemaphoreGive(semaforo_uart);

                       }
                    }
                }
            }

    static portTASK_FUNCTION( SensorTask, pvParameters ){

        uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila
        PARAM_MENSAJE_SENSORES parametro;
        int32_t i32Numdatos;
        int32_t i32Status;
        //
        // Loop forever.
        //
        while(1)
        {
            if (xQueueReceive(cola_botones,&i32Status,portMAX_DELAY)==pdTRUE)
            {
                if((i32Status & LEFT_BUTTON) == 0)
                    parametro.fLeft = 1;
                if((i32Status & RIGHT_BUTTON) == 0)
                    parametro.fRight = 1;
                //(if((i32Status & BACK_BUTTON) == 0)
                    //parametro.button.fBack = 1;
                i32Numdatos=create_frame(pui8Frame,MENSAJE_SENSORES,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                if (i32Numdatos>=0)
                {
                    xSemaphoreTake(semaforo_uart,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(semaforo_uart);
                }
            }
        }
    }

    //*****************************************************************************
    //
    // Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
    //
    //*****************************************************************************
    int main(void)
    {

        //
        // Set the clocking to run at 40 MHz from the PLL.
        //
        MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |SYSCTL_OSC_MAIN);  //Ponermos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)

        //Configure PWM Clock
        MAP_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // NHR: Se usara salidas puerto F para PWM
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  // NHR: Se usar� el PWM1
        MAP_SysCtlPeripheralSleepEnable (SYSCTL_PERIPH_PWM1);

        MAP_GPIOPinConfigure(GPIO_PF2_M1PWM6); // NHR: Configura PF2 como salida 6 del modulo PWM 1(ver pinmap.h)
        MAP_GPIOPinConfigure(GPIO_PF3_M1PWM7); // NHR: Configura PF3 como salida 7 del modulo PWM 1(ver pinmap.h)
        MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2); // NHR: PF2 y PF3 van a tener funci�n PWM

        // Codigo principal, (poner en bucle infinito o bajo consumo)

        // Get the system clock speed.
        g_ui32SystemClock = SysCtlClockGet();


        //Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
        //                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
        MAP_SysCtlPeripheralClockGating(true);
        CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 3);

        //Inicializa el puerto F (LEDs)
        //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        //MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
        //MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0); //LEDS APAGADOS

        //Inicializa la biblioteca RGB (configurando las salidas como RGB)
//        RGBInit(1);
//        MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);  //Esto es necesario para que el timer0 siga funcionando en bajo consumo
//        MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);  //Esto es necesario para que el timer1 siga funcionando en bajo consumo
//
        //Inicializa los botones (tambien en el puerto F) y habilita sus interrupciones
        ButtonsInit();
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0);
        MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_BOTH_EDGES);
        MAP_IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);// Misma prioridad que configMAX_SYSCALL_INTERRUPT_PRIORITY

        // Una prioridad menor (mayor numero) podria dar problemas si la interrupcion ejecuta llamadas a funciones de FreeRTOS
        MAP_GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
        MAP_IntEnable(INT_GPIOF);

        // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
        // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
        // (y por tanto este no se deberia utilizar para otra cosa).


        /**                                              Creacion de tareas                                     **/

        // Inicializa el sistema de depuraci�n por terminal UART
        if (initCommandLine(256,tskIDLE_PRIORITY + 1) != pdTRUE)
        {
            while(1);
        }

        USBSerialInit(32,32);   //Inicializo el  sistema USB

        //
        // Crea la tarea que gestiona los mensajes USB (definidos en USBMessageProcessingTask)
        //

        if(xTaskCreate(USBMessageProcessingTask,"usbser",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
                {
                    while(1);
                }

        if((xTaskCreate(MITask, "Motor_izquierdo", MITASKSTACKSIZE,NULL,tskIDLE_PRIORITY + MITASKPRIO, NULL) != pdTRUE))
                {
                    while(1);
                }

        if((xTaskCreate(MDTask, "Motor_derecho", MDTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + MDTASKPRIO, NULL) != pdTRUE))
                {
                    while(1);
                }

        if((xTaskCreate(MVTOTask, "Calculo_movimiento", MVTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + MVTASKPRIO, NULL) != pdTRUE))
                {
                    while(1);
                }

        if((xTaskCreate(SensorTask, "Control_sensores", BTTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + BTTASKPRIO, NULL) != pdTRUE))
                {
                    while(1);
                }

        cola_motor_izq = xQueueCreate(3,sizeof(int32_t));  //espacio para 3items de tama�o ulong
                if (NULL==cola_motor_izq)
                    while(1);

        cola_motor_dcha = xQueueCreate(3,sizeof(int32_t));  //espacio para 3items de tama�o ulong
                if (NULL==cola_motor_dcha)
                    while(1);

        cola_mvto_izq = xQueueCreate(3,sizeof(int8_t));  //espacio para 3items de tama�o ulong
                        if (NULL==cola_mvto_izq)
                            while(1);

        cola_mvto_dcha = xQueueCreate(3,sizeof(int8_t));  //espacio para 3items de tama�o ulong
                if (NULL==cola_mvto_dcha)
                    while(1);

        cola_timer = xQueueCreate(3,sizeof(int32_t));  //espacio para 3items de tama�o ulong
                if (NULL==cola_timer)
                    while(1);

        cola_botones = xQueueCreate(3,sizeof(int32_t));  //espacio para 3items de tama�o ulong
                if (NULL==cola_timer)
                    while(1);

        //Crea mutex
        semaforo_uart=xSemaphoreCreateMutex();
                if ((NULL==semaforo_uart))
                {
                    while (1);  //No hay memoria para los semaforo
                }

        grupo_colas = xQueueCreateSet(9);
                if (NULL == grupo_colas) {
                    while(1);
                }
        if (xQueueAddToSet(cola_mvto_izq, grupo_colas)!=pdPASS)
                {
                    while(1);
                }

        if (xQueueAddToSet(cola_mvto_dcha, grupo_colas)!=pdPASS)
                {
                    while(1);
                }


        if (xQueueAddToSet(cola_timer, grupo_colas)!=pdPASS)
                {
                    while(1);
                }
        // Create and start timer SW      1/5 * 1s;     TIMER PERIODICO; NULL; que queremos que haga cuando expire el tiempo
        xTimer = xTimerCreate("TimerSW", 0.2 * configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback);

                if( NULL == xTimer ) //COMPROBRAMOS SI HA IDO BIEN
                     {
                         /* The timer was not created. */
                        while(1);
                     }
            else{

                /* Start the timer.  No block time is specified, and even if one was
                it would be ignored because the RTOS scheduler has not yet been
                started. */
                    if( xTimerStart( xTimer, 0 ) != pdPASS )
                    {
                        /* The timer could not be set into the Active state. */
                        while(1);
                    }
                }


        //
        // Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
        //

        vTaskStartScheduler();  //el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

        //De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.

        while(1)
        {
            //Si llego aqui es que algo raro ha pasado
        }
    }


    void GPIOFIntHandler(void)
    {
        signed portBASE_TYPE higherPriorityTaskWoken=pdFALSE;   //Hay que inicializarlo a False!!
        int32_t i32Status = MAP_GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS);
        xQueueSendFromISR (cola_botones,&i32Status,&higherPriorityTaskWoken);
        MAP_GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);              //limpiamos flags
        //Cesion de control de CPU si se ha despertado una tarea de mayor prioridad
        portEND_SWITCHING_ISR(higherPriorityTaskWoken);
    }



