// Incluir las bibliotecas de Arduino requeridas:
#include <OneWire.h>
#include <DallasTemperature.h>

// Definir a qué pin del Arduino está conectado el bus 1-Wire:
#define ONE_WIRE_BUS 4

// Crear una nueva instancia de la clase oneWire para comunicarse con cualquier dispositivo OneWire:
OneWire oneWire(ONE_WIRE_BUS);

// Definir variables de control PID
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;

void Compute()
{
    /*Cuánto tiempo ha pasado desde el último cálculo*/
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime);

    /*Calcular todas las variables de error de trabajo*/
    double error = Setpoint - Input;
    errSum += (error * timeChange);
    double dErr = (error - lastErr) / timeChange;

    /*Calcular salida PID*/
    Output = kp * error + ki * errSum + kd * dErr;

    /*Recordar algunas variables para la próxima vez*/
    lastErr = error;
    lastTime = now;
}

// Pasar la referencia oneWire a la biblioteca DallasTemperature:
DallasTemperature sensors(&oneWire);

void setup() {
    kp = 1;
    ki = 0;
    kd = 1;

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
    // Obtener la temperatura en grados Fahrenheit para el índice del dispositivo:
    float tempF = sensors.getTempFByIndex(0);

    // Imprimir la temperatura en Celsius en el Monitor Serial:
    Serial.print("Temperatura: ");
    Serial.print(tempC);
    Serial.print(" \xC2\xB0"); // muestra el símbolo de grado
    Serial.print("C  |  ");

    // Imprimir la temperatura en Fahrenheit
    Serial.print(tempF);
    Serial.print(" \xC2\xB0"); // muestra el símbolo de grado
    Serial.println("F");

    Input = tempC;
    Compute();

    Serial.print(Output);

    // Esperar 1 segundo:
    delay(1000);
}