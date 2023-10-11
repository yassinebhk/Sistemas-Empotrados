//*****************************************************************************
//
// commands.c - FreeRTOS porting example on CCS4
//
// Este fichero contiene errores que seran explicados en clase
//
//*****************************************************************************


#include <stdbool.h>
#include <stdint.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Standard TIVA includes */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip

/* Other TIVA includes */
#include "utils/cpu_usage.h"
#include "utils/cmdline.h"
#include "utils/uartstdioMod.h"

#include "drivers/rgb.h"
#include "event_groups.h"        // FreeRTOS: definiciones relacionadas con grupos de eventos


// ==============================================================================
// Colas externas
// ==============================================================================
extern QueueHandle_t mailbox_modo_traza;
extern QueueHandle_t mailbox_cambio_parametros;
extern TimerHandle_t xTimer;
extern TimerHandle_t xTimer_en;
extern TimerHandle_t xTimer_peligro;
extern TimerHandle_t xTimer_quemado;
extern TimerHandle_t xTimer_panel;
extern TimerHandle_t xTimer_autodestruccion;
extern SemaphoreHandle_t mutex_uart;


#define PACKED __attribute__ ((packed))

typedef struct {
    int32_t energia;
    int32_t distancia;
} PACKED parametros;

// ==============================================================================
// The CPU usage in percent, in 16.16 fixed point format.
// ==============================================================================
extern uint32_t g_ui32CPUUsage;

// ==============================================================================
// Implementa el comando cpu que muestra el uso de CPU
// ==============================================================================
static int  Cmd_cpu(int argc, char *argv[])
{
    //
    // Print some header text.
    //
    UARTprintf("ARM Cortex-M4F %u MHz - ",SysCtlClockGet() / 1000000);
    UARTprintf("%2u%% de uso\r\n", (g_ui32CPUUsage+32768) >> 16);

    // Return success.
    return(0);
}

// ==============================================================================
// Implementa el comando free, que muestra cuanta memoria "heap" le queda al FreeRTOS
// ==============================================================================
static int Cmd_free(int argc, char *argv[])
{
    //
    // Print some header text.
    //
    UARTprintf("%d bytes libres\r\n", xPortGetFreeHeapSize());

    // Return success.
    return(0);
}

// ==============================================================================
// Implementa el comando task. Sï¿½lo es posible si la opciï¿½n configUSE_TRACE_FACILITY de FreeRTOS estï¿½ habilitada
// ==============================================================================
#if ( configUSE_TRACE_FACILITY == 1 )

extern char *__stack;
static  int Cmd_tasks(int argc, char *argv[])
{
	char*	pcBuffer;
	uint8_t*	pi8Stack;
	portBASE_TYPE	x;
	
	pcBuffer = pvPortMalloc(1024);
	vTaskList(pcBuffer);
	UARTprintf("\t\t\t\tUnused\r\nTaskName\tStatus\tPri\tStack\tTask ID\r\n");
	UARTprintf("=======================================================\r\n");
	UARTprintf("%s", pcBuffer);
	
	// Calculate kernel stack usage
	x = 0;
	pi8Stack = (uint8_t *) &__stack;
	while (*pi8Stack++ == 0xA5)
	{
		x++;	//Esto sï¿½lo funciona si hemos rellenado la pila del sistema con 0xA5 en el arranque
	}
	sprintf((char *) pcBuffer, "%%%us", configMAX_TASK_NAME_LEN);
	sprintf((char *) &pcBuffer[10], (const char *) pcBuffer, "kernel");
	UARTprintf("%s\t-\t*%u\t%u\t-\r\n", &pcBuffer[10], configKERNEL_INTERRUPT_PRIORITY, x/sizeof(portBASE_TYPE));
	vPortFree(pcBuffer);
	return 0;
}

#endif /* configUSE_TRACE_FACILITY */

#if configGENERATE_RUN_TIME_STATS
// ==============================================================================
// Implementa el comando "Stats"
// ==============================================================================
static Cmd_stats(int argc, char *argv[])
{
	char*	pBuffer;

	pBuffer = pvPortMalloc(1024);
	if (pBuffer)
	{
		vTaskGetRunTimeStats(pBuffer); //Es un poco inseguro, pero por ahora nos vale...
		UARTprintf("TaskName\tCycles\t\tPercent\r\n");
		UARTprintf("===============================================\r\r\n");
		UARTprintf("%s", pBuffer);
		vPortFree(pBuffer);
	}
	return 0;
}
#endif

// ==============================================================================
// Implementa el comando help
// ==============================================================================
static int Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Print some header text.
    //
    UARTprintf("Comandos disponibles\r\n");
    UARTprintf("------------------\r\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_psCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        UARTprintf("%s%s\r\n", pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    //
    // Return success.
    //
    return(0);
}

static int Cmd_velocidad(int argc, char *argv[])
{

    float velocidad = 1;
    if (argc != 2)
        {
            //Si los parametros no son suficientes o son demasiados, muestro la ayuda
            UARTprintf(" velocidad: [x0.5] [x1] [x2] [x4] [x8] [x16] [x32]\r\n");
        }
        else
        {
            if (0==strncmp( argv[1], "x0.5",3))
            {
                UARTprintf("Cambio a velocidad x0.5\r\n");
                velocidad = 0.5;

            }
            else if (0==strncmp( argv[1], "x16",3))
            {
                UARTprintf("Cambio a velocidad x16\r\n");
                velocidad = 16;
            }
            else if (0==strncmp( argv[1], "x32",3))
            {
                UARTprintf("Cambio a velocidad x32\r\n");
                velocidad = 32;
            }
            else if (0==strncmp( argv[1], "x1",2))
            {
               UARTprintf("Cambio a velocidad x1\r\n");
               velocidad = 1;
            }
            else if (0==strncmp( argv[1], "x2",2))
            {
                UARTprintf("Cambio a velocidad x2\r\n");
                velocidad = 2;
            }
            else if (0==strncmp( argv[1], "x4",2))
            {
                UARTprintf("Cambio a velocidad x4\r\n");
                velocidad = 4;
            }
            else if (0==strncmp( argv[1], "x8",2))
            {
                UARTprintf("Cambio a velocidad x8\r\n");
                velocidad = 8;
            }

            xTimerChangePeriod (xTimer,  (0.2 / velocidad) * configTICK_RATE_HZ, portMAX_DELAY);
            xTimerChangePeriod (xTimer_en,  (1 / velocidad) * configTICK_RATE_HZ, portMAX_DELAY);

            xTimerChangePeriod (xTimer_peligro, (4 / velocidad) * configTICK_RATE_HZ, portMAX_DELAY);
            xTimerStop (xTimer_peligro, 0);

            xTimerChangePeriod (xTimer_quemado, (6 * velocidad) * configTICK_RATE_HZ, portMAX_DELAY);
            xTimerStop (xTimer_quemado, 0);

            xTimerChangePeriod (xTimer_panel, (1 / velocidad) * configTICK_RATE_HZ, portMAX_DELAY);
            xTimerStop (xTimer_panel, 0);
        }

    return(0);
}

static int Cmd_traza(int argc, char *argv[])
{

    int8_t activado = 0;

    if (argc != 2)
    {
        //Si los parametros no son suficientes o son demasiados, muestro la ayuda
        UARTprintf("traza [on|off]\r\n");
    }
    else
    {
        if (0==strncmp( argv[1], "on",2))
        {
            UARTprintf("Activando modo traza\r\n");
            activado = 1;
            xQueueOverwrite (mailbox_modo_traza,&activado);
        }
        else if (0==strncmp( argv[1], "off",3))
        {
            activado = 0;
            xQueueOverwrite (mailbox_modo_traza,&activado);
            UARTprintf("Desactivando modo traza\r\n");
        }
        else
        {
            //Si el parametro no es correcto, muestro la ayuda
            UARTprintf(" traza [on|off]\r\n");
        }
    }

    return(0);
}

static int Cmd_variable(int argc, char *argv[])
{

    parametros variables;
    variables.distancia = 0;
    variables.energia = 0;

    if (argc != 3)
    {
        UARTprintf("variable energia [numero]");
    }
    else
    {

        if (0==strncmp( argv[1], "energia",7)) {

            variables.energia = strtol(argv [2], NULL, 10);
            variables.distancia = -1;
            xQueueOverwrite (mailbox_cambio_parametros,&variables);

            UARTprintf("Cambiando energia\r\n");

        }

        else if (0==strncmp( argv[1], "distancia",9)) {

            variables.distancia = strtol (argv [2], NULL, 10);
            variables.energia = -1;
            xQueueOverwrite (mailbox_cambio_parametros,&variables);

            UARTprintf("Cambiando distancia\r\n");
        }

        else {

            UARTprintf("variable energia [numero]");
        }
    }

    return(0);
}



// ==============================================================================
// Tabla con los comandos y su descripcion. Si quiero anadir alguno, debo hacerlo aqui
// ==============================================================================
//Este array tiene que ser global porque es utilizado por la biblioteca cmdline.c para implementar el interprete de comandos
//No es muy elegante, pero es lo que ha implementado Texas Instruments.
tCmdLineEntry g_psCmdTable[] =
{
    { "help",     Cmd_help,      "     : Lista de comandos" },
    { "?",        Cmd_help,      "        : lo mismo que help" },
    { "cpu",      Cmd_cpu,       "      : Muestra el uso de  CPU " },
    { "free",     Cmd_free,      "     : Muestra la memoria libre" },
    { "velocidad",     Cmd_velocidad,      "     : Regula la velocidad de simulación" },
    { "traza",     Cmd_traza,      "     : Activa o desactiva el modo traza" },
    { "variable",     Cmd_variable,      "     : Modifica el valor del parametro que se le pasa como entrada" },

#if ( configUSE_TRACE_FACILITY == 1 )
	{ "tasks",    Cmd_tasks,     "    : Muestra informacion de las tareas" },
#endif
#if (configGENERATE_RUN_TIME_STATS)
	{ "stats",    Cmd_stats,      "     : Muestra estadisticas de las tareas" },
#endif
    { 0, 0, 0 }
};

// ==============================================================================
// Tarea UARTCommandTask.  Espera la llegada de comandos por el puerto serie y los ejecuta al recibirlos...
// ==============================================================================
static void UARTCommandTask( void *pvParameters )
{
    char    pcCmdBuf[64];
    int32_t i32Status;
    parametros variables;
    variables.energia = 100000;
    variables.distancia = 0;
    //
    // Mensaje de bienvenida inicial.
    //
    UARTprintf("\r\n\r\nWelcome to the TIVA FreeRTOS Demo!\r\n");
    UARTprintf("\r\n\r\n FreeRTOS %s \r\n",
        tskKERNEL_VERSION_NUMBER);
    UARTprintf("\r\n Teclee ? para ver la ayuda \r\n");
    UARTprintf("> ");
    xQueueOverwrite (mailbox_modo_traza,0);
    xQueueOverwrite (mailbox_cambio_parametros,&variables);

    /* Loop forever */
    while (1)
    {

        /* Read data from the UART and process the command line */
        UARTgets(pcCmdBuf, sizeof(pcCmdBuf));
        if (strlen(pcCmdBuf) == 0)
        {
            UARTprintf("> ");
            continue;
        }

        //
        // Pass the line from the user to the command processor.  It will be
        // parsed and valid commands executed.
        //
        i32Status = CmdLineProcess(pcCmdBuf);

        //
        // Handle the case of bad command.
        //
        if(i32Status == CMDLINE_BAD_CMD)
        {
            UARTprintf("Comando erroneo!\r\n"); //No pongo acentos adrede

        }

        //
        // Handle the case of too many arguments.
        //
        else if(i32Status == CMDLINE_TOO_MANY_ARGS)
        {
            UARTprintf("El interprete de comandos no admite tantos parametros\r\n");    //El maximo, CMDLINE_MAX_ARGS, esta definido en cmdline.c
        }

        //
        // Otherwise the command was executed.  Print the error code if one was
        // returned.
        //
        else if(i32Status != 0)
        {
            UARTprintf("El comando devolvio el error %d\r\n",i32Status);
        }

        UARTprintf("> ");

    }
}


//
// Crea la tarea que gestiona los comandos (definida en el fichero commands.c)
//
BaseType_t initCommandLine(uint16_t stack_size,uint8_t prioriry )
{

    // Inicializa la UARTy la configura a 115.200 bps, 8-N-1 .
    //se usa para mandar y recibir mensajes y comandos por el puerto serie
    // Mediante un programa terminal como gtkterm, putty, cutecom, etc...
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //Esta funcion habilita la interrupcion de la UART y le da la prioridad adecuada si esta activado el soporte para FreeRTOS
    UARTStdioConfig(0,115200,SysCtlClockGet());
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);   //La UART tiene que seguir funcionando aunque el micro esta dormido

    return xTaskCreate(UARTCommandTask, "UartComm", stack_size,NULL,prioriry, NULL);
}

