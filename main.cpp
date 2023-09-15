/**
 * @brief Control de Alarma de Gas y Temperatura
 *
 * Este código implementa un sistema de alarma que monitorea la detección de gas y
 * la temperatura. El sistema se activa y desactiva mediante una secuencia de botones.
 * También se pueden realizar consultas a través de UART para verificar el estado de la alarma
 * y obtener lecturas de sensores. El código utiliza MBED para controlar los pines y la comunicación UART.
 *
 *
 * @note Asegúrese de que estén conectados los botones/switches a 3.3V y los pines D2, D3, D4, D5, D6 y D7.
 * @note Se utiliza una resistencia pull-down, por lo que los botones/switches deben conectarse a 3.3V.
 * Si se utilizara pull-up, se conectarían a GND.
 *
 * Ventajas de esta implementación:
 *
 * 1. El código es más legible y organizado al utilizar BusIn y BusOut para agrupar las
 *    entradas y salidas digitales.
 * 2. Facilita la expansión del código para agregar más entradas/salidas en el futuro
 *    sin modificar significativamente la lógica existente.
 * 3. Las líneas de printf proporcionan una visión clara del estado de las entradas y 
 *    salidas digitales en cada iteración, lo que facilita la depuración.
 *
 * Desventajas de esta implementación:
 *
 * - El código puede parecer un poco más largo debido a las declaraciones adicionales
 *   para configurar BusIn y BusOut, pero esta desventaja es mínima en comparación con
 *   las ventajas en términos de legibilidad y escalabilidad.
 */
//=====[Libraries]=============================================================

#include "mbed.h"
#include "arm_book_lib.h"
#include <string.h>

//=====[Defines]===============================================================

#define NUMBER_OF_KEYS                           4
#define BLINKING_TIME_GAS_ALARM               1000
#define BLINKING_TIME_OVER_TEMP_ALARM          500
#define BLINKING_TIME_GAS_AND_OVER_TEMP_ALARM  100
#define NUMBER_OF_AVG_SAMPLES                   100
#define OVER_TEMP_LEVEL                         50
#define TIME_INCREMENT_MS                       10

//=====[Declaration and initialization of public global objects]===============



BusIn buttons(BUTTON1, D2, D4, D5, D6, D7); // Botones Enter (BUTTON1), Test(D2), A (D4), B (D5), C (D6), D (D7)

DigitalIn mq2(PE_12);

BusOut leds(LED1, LED3, LED2); // Led de Alarma, Led de Incorrecto, Led de Sistema Bloqueado


DigitalInOut sirenPin(PE_10);

UnbufferedSerial uartUsb(USBTX, USBRX, 115200);

AnalogIn potentiometer(A0);
AnalogIn lm35(A1);

//=====[Declaration and initialization of public global variables]=============

bool alarmState    = OFF;
bool incorrectCode = false;
bool overTempDetector = OFF;

bool enterButtonState = buttons[0]; //ENTER
bool alarmTestButtonState = buttons[1];//D3
bool aButtonState = buttons[2];//D4
bool bButtonState = buttons[3];//D5
bool cButtonState = buttons[4];//D6
bool dButtonState = buttons[5];//D7

int numberOfIncorrectCodes = 0;
int buttonBeingCompared    = 0;
int codeSequence[NUMBER_OF_KEYS]   = { 1, 1, 0, 0 };
int buttonsPressed[NUMBER_OF_KEYS] = { 0, 0, 0, 0 };
int accumulatedTimeAlarm = 0;

bool gasDetectorState          = OFF;
bool overTempDetectorState     = OFF;

float potentiometerReading = 0.0;
float lm35ReadingsAverage  = 0.0;
float lm35ReadingsSum      = 0.0;
float lm35ReadingsArray[NUMBER_OF_AVG_SAMPLES];
float lm35TempC            = 0.0;

//=====[Declarations (prototypes) of public functions]=========================

void inputsInit();
void outputsInit();

void alarmActivationUpdate();
void alarmDeactivationUpdate();

void uartTask();
void availableCommands();
bool areEqual();
float celsiusToFahrenheit( float tempInCelsiusDegrees );
float analogReadingScaledWithTheLM35Formula( float analogReading );

//=====[Main function, the program entry point after power on or reset]========

int main()
{

    inputsInit();       //Inicializacion de pines de entrada
    outputsInit();      //Inicializacion de pines de salida
    while (true) {      //Loop principal

        enterButtonState = buttons[0]; //ENTER
        alarmTestButtonState = buttons[1];//D3
        aButtonState = buttons[2];//D4
        bButtonState = buttons[3];//D5
        cButtonState = buttons[4];//D6
        dButtonState = buttons[5];//D7
        alarmActivationUpdate();    //Actualizacion evento de activacion de alarma
        alarmDeactivationUpdate();  //Actualizacion evento de desactivacion de alarma
        uartTask();                 //Comunicacion por puerto serie
        printf("Enter Button: %d, Alarm Test Button: %d, A Button: %d, B Button: %d, C Button: %d, D Button: %d, Alarm LED: %d\n",
               enterButtonState, alarmTestButtonState, aButtonState, bButtonState, cButtonState, dButtonState, alarmState);
        //printf("Valor potenciometro %f \n",lm35.read());
        delay(TIME_INCREMENT_MS);   //Delay de 10ms
    }
}

//=====[Implementations of public functions]===================================

void inputsInit()
{
    buttons[1].mode(PullDown);
    buttons[2].mode(PullDown);
    buttons[3].mode(PullDown);
    buttons[4].mode(PullDown);
    buttons[5].mode(PullDown);
    sirenPin.mode(OpenDrain);
    sirenPin.input();
}

void outputsInit()
{
    leds[0] = OFF;
    leds[1] = OFF;
    leds[2] = OFF;
}

void alarmActivationUpdate()
{
    static int lm35SampleIndex = 0;
    int i = 0;

    lm35ReadingsArray[lm35SampleIndex] = lm35.read();
    lm35SampleIndex++;
    if ( lm35SampleIndex >= NUMBER_OF_AVG_SAMPLES) {
        lm35SampleIndex = 0;
    }
    
       lm35ReadingsSum = 0.0;
    for (i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        lm35ReadingsSum = lm35ReadingsSum + lm35ReadingsArray[i];
    }
    lm35ReadingsAverage = lm35ReadingsSum / NUMBER_OF_AVG_SAMPLES;
       lm35TempC = analogReadingScaledWithTheLM35Formula ( lm35ReadingsAverage );    
    
    if ( lm35TempC > OVER_TEMP_LEVEL ) {
        overTempDetector = ON;
    } else {
        overTempDetector = OFF;
    }

    if( !mq2) {
        gasDetectorState = ON;
        alarmState = ON;
    }
    if( overTempDetector ) {
        overTempDetectorState = ON;
        alarmState = ON;
    }
    if( alarmTestButtonState ) {             
        overTempDetectorState = ON;
        gasDetectorState = ON;
        alarmState = ON;
    }    
    if( alarmState ) { 
        accumulatedTimeAlarm = accumulatedTimeAlarm + TIME_INCREMENT_MS;
        sirenPin.output();                                     
        sirenPin = LOW;                                        
    
        if( gasDetectorState && overTempDetectorState ) {
            if( accumulatedTimeAlarm >= BLINKING_TIME_GAS_AND_OVER_TEMP_ALARM ) {
                accumulatedTimeAlarm = 0;
                leds[0] = !leds[0];
            }
        } else if( gasDetectorState ) {
            if( accumulatedTimeAlarm >= BLINKING_TIME_GAS_ALARM ) {
                accumulatedTimeAlarm = 0;
                leds[0] = !leds[0];
            }
        } else if ( overTempDetectorState ) {
            if( accumulatedTimeAlarm >= BLINKING_TIME_OVER_TEMP_ALARM  ) {
                accumulatedTimeAlarm = 0;
                leds[0] = !leds[0];
            }
        }
    } else{
        leds[0] = OFF;
        gasDetectorState = OFF;
        overTempDetectorState = OFF;
        sirenPin.input();                                  
    }
}

void alarmDeactivationUpdate()
{
    if ( numberOfIncorrectCodes < 5 ) {
        if ( aButtonState && bButtonState && cButtonState && dButtonState && !enterButtonState ) {
            leds[1] = OFF;
        }
        if ( enterButtonState && !leds[1] && alarmState ) {
            buttonsPressed[0] = aButtonState;
            buttonsPressed[1] = bButtonState;
            buttonsPressed[2] = cButtonState;
            buttonsPressed[3] = dButtonState;
            if ( areEqual() ) {
                alarmState = OFF;
                numberOfIncorrectCodes = 0;
            } else {
                leds[1] = ON;
                numberOfIncorrectCodes++;
            }
        }
    } else {
        leds[2] = ON;
    }
}

void uartTask()
{
    char receivedChar = '\0';
    char str[100];
    int stringLength;
    if( uartUsb.readable() ) {
        uartUsb.read( &receivedChar, 1 );
        switch (receivedChar) {
        case '1':
            if ( alarmState ) {
                uartUsb.write( "The alarm is activated\r\n", 24);
            } else {
                uartUsb.write( "The alarm is not activated\r\n", 28);
            }
            break;

        case '2':
            if ( !mq2 ) {
                uartUsb.write( "Gas is being detected\r\n", 22);
            } else {
                uartUsb.write( "Gas is not being detected\r\n", 27);
            }
            break;

        case '3':
            if ( overTempDetector ) {
                uartUsb.write( "Temperature is above the maximum level\r\n", 40);
            } else {
                uartUsb.write( "Temperature is below the maximum level\r\n", 40);
            }
            break;
            
        case '4':
            uartUsb.write( "Please enter the code sequence.\r\n", 33 );
            uartUsb.write( "First enter 'A', then 'B', then 'C', and ", 41 ); 
            uartUsb.write( "finally 'D' button\r\n", 20 );
            uartUsb.write( "In each case type 1 for pressed or 0 for ", 41 );
            uartUsb.write( "not pressed\r\n", 13 );
            uartUsb.write( "For example, for 'A' = pressed, ", 32 );
            uartUsb.write( "'B' = pressed, 'C' = not pressed, ", 34);
            uartUsb.write( "'D' = not pressed, enter '1', then '1', ", 40 );
            uartUsb.write( "then '0', and finally '0'\r\n\r\n", 29 );

            incorrectCode = false;

            for ( buttonBeingCompared = 0;
                  buttonBeingCompared < NUMBER_OF_KEYS;
                  buttonBeingCompared++) {

                uartUsb.read( &receivedChar, 1 );
                uartUsb.write( "*", 1 );

                if ( receivedChar == '1' ) {
                    if ( codeSequence[buttonBeingCompared] != 1 ) {
                        incorrectCode = true;
                    }
                } else if ( receivedChar == '0' ) {
                    if ( codeSequence[buttonBeingCompared] != 0 ) {
                        incorrectCode = true;
                    }
                } else {
                    incorrectCode = true;
                }
            }

            if ( incorrectCode == false ) {
                uartUsb.write( "\r\nThe code is correct\r\n\r\n", 25 );
                alarmState = OFF;
                leds[1] = OFF;
                numberOfIncorrectCodes = 0;
            } else {
                uartUsb.write( "\r\nThe code is incorrect\r\n\r\n", 27 );
                leds[1] = ON;
                numberOfIncorrectCodes++;
            }                
            break;

        case '5':
            uartUsb.write( "Please enter new code sequence\r\n", 32 );
            uartUsb.write( "First enter 'A', then 'B', then 'C', and ", 41 );
            uartUsb.write( "finally 'D' button\r\n", 20 );
            uartUsb.write( "In each case type 1 for pressed or 0 for not ", 45 );
            uartUsb.write( "pressed\r\n", 9 );
            uartUsb.write( "For example, for 'A' = pressed, 'B' = pressed,", 46 );
            uartUsb.write( " 'C' = not pressed,", 19 );
            uartUsb.write( "'D' = not pressed, enter '1', then '1', ", 40 );
            uartUsb.write( "then '0', and finally '0'\r\n\r\n", 29 );

            for ( buttonBeingCompared = 0; 
                  buttonBeingCompared < NUMBER_OF_KEYS; 
                  buttonBeingCompared++) {

                uartUsb.read( &receivedChar, 1 );
                uartUsb.write( "*", 1 );

                if ( receivedChar == '1' ) {
                    codeSequence[buttonBeingCompared] = 1;
                } else if ( receivedChar == '0' ) {
                    codeSequence[buttonBeingCompared] = 0;
                }
            }

            uartUsb.write( "\r\nNew code generated\r\n\r\n", 24 );
            break;
 
        case 'p':
        case 'P':
            potentiometerReading = potentiometer.read();
            sprintf ( str, "Potentiometer: %.2f\r\n", potentiometerReading );
            stringLength = strlen(str);
            uartUsb.write( str, stringLength );
            break;

        case 'c':
        case 'C':
            sprintf ( str, "Temperature: %.2f \xB0 C\r\n", lm35TempC );
            stringLength = strlen(str);
            uartUsb.write( str, stringLength );
            break;

        case 'f':
        case 'F':
            sprintf ( str, "Temperature: %.2f \xB0 F\r\n", 
            celsiusToFahrenheit( lm35TempC ) );
            stringLength = strlen(str);
            uartUsb.write( str, stringLength );
            break;

        default:
            availableCommands();
            break;

        }
    }
}

void availableCommands()
{
    uartUsb.write( "Available commands:\r\n", 21 );
    uartUsb.write( "Press '1' to get the alarm state\r\n", 34 );
    uartUsb.write( "Press '2' to get the gas detector state\r\n", 41 );
    uartUsb.write( "Press '3' to get the over temperature detector state\r\n", 54 );
    uartUsb.write( "Press '4' to enter the code sequence\r\n", 38 );
    uartUsb.write( "Press '5' to enter a new code\r\n", 31 );
    uartUsb.write( "Press 'P' or 'p' to get potentiometer reading\r\n", 47 );
    uartUsb.write( "Press 'f' or 'F' to get lm35 reading in Fahrenheit\r\n", 52 );
    uartUsb.write( "Press 'c' or 'C' to get lm35 reading in Celsius\r\n\r\n", 51 );
}

bool areEqual()
{
    int i;

    for (i = 0; i < NUMBER_OF_KEYS; i++) {
        if (codeSequence[i] != buttonsPressed[i]) {
            return false;
        }
    }

    return true;
}

float analogReadingScaledWithTheLM35Formula( float analogReading )
{
    return ( analogReading * 3.3 / 0.01 );
}

float celsiusToFahrenheit( float tempInCelsiusDegrees )
{
    return ( tempInCelsiusDegrees * 9.0 / 5.0 + 32.0 );
}
