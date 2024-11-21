// Incluir las bibliotecas de Arduino requeridas:
#include <OneWire.h>
#include <DallasTemperature.h>

// Definir a qué pin del Arduino está conectado el bus 1-Wire:
#define ONE_WIRE_BUS 4
#define PWM_PIN 3

// Crear una nueva instancia de la clase oneWire para comunicarse con cualquier dispositivo OneWire:
OneWire oneWire(ONE_WIRE_BUS);

// Definir variables de control PID
unsigned long lastTime;
double Input, Output, Setpoint; // Variable PID
double errSum, lastErr;
double kp, ki, kd;

void ComputePID()
{
    /*Cuánto tiempo ha pasado desde el último cálculo*/
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime);

    /*Calcular todas las variables de error de trabajo*/
    double error = Setpoint - Input;
    errSum += (error * timeChange);
    double dErr = (error - lastErr);  // / timeChange;

    /*Calcular salida PID*/
    Output = kp * error + ki * errSum + kd * dErr;

    /*Recordar algunas variables para la próxima vez*/
    lastErr = error;
    lastTime = now;
}

// Pasar la referencia oneWire a la biblioteca DallasTemperature:
DallasTemperature sensors(&oneWire);

void setup() {
    kp = 0.5;
    ki = 0.1;
    kd = 0.01;

    Setpoint = 28.0;
    Input = 0;
    //Output = 0;
    
    pinMode(PWM_PIN, OUTPUT);

    // Iniciar comunicación serial a una velocidad de 9600 baudios:
    Serial.begin(9600);
    // Iniciar la biblioteca:
    sensors.begin();
}

void loop() {
    // Enviar el comando para que todos los dispositivos en el bus realicen una conversión de temperatura:
    sensors.requestTemperatures();

    // Obtener la temperatura en grados Celsius para el índice del dispositivo:
    float tempC = sensors.getTempCByIndex(0); // el índice 0 se refiere al primer dispositivo

    //Input = constrain(tempC, 0 ,125);
    Input = tempC;
    ComputePID();

   // Output = constrain(Output, 0, 125);

    // Mapear la salida PID al rango PWM (0-255) e invertirlo
    //int pwmOutput = map(Output, 0, 255, 255, 0);
    //int pwmOutput = map(Output, -125, 125, 0, 255); //directo para que trabaje solo con el mofstet
    float coeficiente = Output / 10000;
    int pwmOutput = int(255 * coeficiente);

    // Limitar la salida PWM al rango válido
    //pwmOutput = constrain(pwmOutput, 100, 255);
    pwmOutput = constrain(pwmOutput, 0, 100);  // para trabajar solo con el mofset
    

    //pwmOutput = 0;

    // Escribir la salida PWM al pin
    analogWrite(PWM_PIN, pwmOutput);


    // Imprimir la temperatura en Celsius en el Monitor Serial:
    Serial.print("Temperatura: ");
    Serial.print(tempC);
    Serial.print(" \xC2\xB0"); // muestra el símbolo de grado
    Serial.print("C  |  ");


    Serial.print("Salida PID: ");
    Serial.print(Output);
    Serial.print(" | Salida PWM: ");
    Serial.println(pwmOutput);

    // Esperar 1 segundo
    delay(1000);
}