// Incluir las bibliotecas de Arduino requeridas:
#include <OneWire.h>           // Biblioteca para comunicación con dispositivos OneWire
#include <DallasTemperature>   // Biblioteca para manejar sensores de temperatura DS18B20

// Definir los pines utilizados:
#define ONE_WIRE_BUS 4         // Pin al que está conectado el bus 1-Wire
#define PWM_PIN 3              // Pin para la salida PWM

// Crear una instancia de OneWire para comunicarse con cualquier dispositivo OneWire:
OneWire oneWire(ONE_WIRE_BUS);

// Definir variables de control PID
unsigned long lastTime;        // Último tiempo de cálculo
double Input, Output, Setpoint;// Variables PID: entrada, salida y punto de referencia
double errSum, lastErr;        // Suma de errores y último error
double kp, ki, kd;             // Constantes proporcional, integral y derivativa

// Función para calcular la salida PID
void ComputePID()
{
    // Calcular el tiempo transcurrido desde el último cálculo (en segundos)
    unsigned long now = millis();
    
    // Explicación detallada de millis():
    // millis() es una función de Arduino que devuelve el número de milisegundos
    // transcurridos desde que el programa comenzó a ejecutarse.
    // - Retorna un valor de tipo unsigned long (0 a 4,294,967,295).
    // - Se desborda (vuelve a 0) después de aproximadamente 49.7 días.
    // - No se ve afectada por delay() o otras funciones que detienen la ejecución.
    // - Es útil para temporizaciones no bloqueantes y medición de intervalos.
    // - Tiene una resolución de 1 milisegundo en la mayoría de las placas Arduino.
    
    double timeChange = (double)((now - lastTime)/1000);
    
    // La resta (now - lastTime) da el tiempo transcurrido en milisegundos.
    // Se divide por 1000 para convertir a segundos.
    // El casting a (double) asegura una división de punto flotante precisa.

    // Calcular todas las variables de error
    double error = Setpoint - Input;  // Error actual
    errSum += (error * timeChange);   // Suma integral del error
    double dErr = (error - lastErr) / timeChange;  // Derivada del error

    // Calcular la salida PID
    Output = kp * error + ki * errSum + kd * dErr;

    // Guardar variables para el próximo cálculo
    lastErr = error;
    lastTime = now;
}

// Crear una instancia de DallasTemperature para manejar los sensores
DallasTemperature sensors(&oneWire);


// Explicación del patrón setup() y loop() en programas Arduino:
// Los programas Arduino se estructuran principalmente en dos funciones: setup() y loop().
//
// 1. setup():
//    - Se ejecuta una sola vez al inicio del programa.
//    - Se usa para inicializar variables, configurar modos de pines, iniciar comunicaciones, etc.
//    - Es el lugar ideal para realizar configuraciones que solo necesitan hacerse una vez.
//
// 2. loop():
//    - Se ejecuta repetidamente después de que setup() ha terminado.
//    - Contiene el código principal que se ejecutará continuamente.
//    - Es donde se realiza la lógica principal del programa, lectura de sensores, control de actuadores, etc.
//    - Se repite indefinidamente mientras el Arduino esté encendido.
//
// Este patrón permite:
// - Una clara separación entre la configuración inicial y la ejecución continua.
// - Facilita la creación de programas que respondan constantemente a cambios en el entorno.
// - Simula un sistema operativo simple, permitiendo multitarea cooperativa.
//
// Es importante recordar que:
// - El código en loop() debe ser lo más eficiente posible para evitar retrasos innecesarios.
// - Se pueden usar funciones adicionales llamadas desde setup() o loop() para organizar mejor el código.
// - Aunque loop() se repite continuamente, se pueden usar técnicas de temporización para controlar la frecuencia de ciertas acciones.


void setup() {
    // Configurar constantes PID
    kp = 0.5;  // Constante proporcional
    ki = 0.1;  // Constante integral
    kd = 0.01; // Constante derivativa

    // Inicializar variables PID
    Input = 0;
    Setpoint = 30.0;  // Temperatura objetivo

    // Configurar el pin PWM como salida
    pinMode(PWM_PIN, OUTPUT);

    // Iniciar comunicación serial a 9600 baudios
    Serial.begin(9600);

    // Explicación detallada de Serial.begin(9600):
    // Esta línea inicia la comunicación serial entre el Arduino y el ordenador.
    // - Serial: Es el objeto que maneja la comunicación serie en Arduino.
    // - begin(): Es un método que inicializa la comunicación serial.
    // - 9600: Es la velocidad de transmisión en baudios (bits por segundo).
    //
    // Detalles importantes:
    // 1. La velocidad de 9600 baudios es común y compatible con la mayoría de los ordenadores.
    // 2. Debe coincidir con la velocidad configurada en el Monitor Serial del IDE de Arduino.
    // 3. Velocidades más altas (ej. 115200) pueden usarse para transmisiones más rápidas.
    // 4. Esta línea es esencial para poder usar funciones como Serial.print() o Serial.println().
    // 5. La comunicación serial es útil para depuración y monitoreo del programa.
    // 6. No bloquea la ejecución del programa; la comunicación ocurre en segundo plano.
    // 7. En algunos Arduinos (ej. Leonardo), esta línea también inicia la conexión USB.
    
    // Inicializar la biblioteca de sensores
    sensors.begin();
}

void loop() {
    // Solicitar a todos los sensores que realicen una conversión de temperatura
    sensors.requestTemperatures();

    // Obtener la temperatura en grados Celsius del primer sensor
    float tempC = sensors.getTempCByIndex(0);

    // Asignar la temperatura leída como entrada del PID
    Input = tempC;

    // Calcular la salida PID
    ComputePID();

    // Convertir la salida PID a un valor PWM
    // Esta escala de 50 significa que queremos que cuando el
    // PID esté en 50, el PWM esté en su máximo (255).
    float coeficiente = Output / 50;
    int pwmOutput = int(255 * coeficiente);

    // Limitar la salida PWM al rango válido (0-100 para el MOSFET)
    pwmOutput = constrain(pwmOutput, 0, 100);

    // Escribir el valor PWM al pin de salida
    analogWrite(PWM_PIN, pwmOutput);

    // Imprimir la temperatura en Celsius en el Monitor Serial
    Serial.print("Temperatura: ");
    Serial.print(tempC);
    Serial.print(" \xC2\xB0"); // Símbolo de grado
    Serial.print("C  |  ");

    // Imprimir la salida PID y PWM
    Serial.print("Salida PID: ");
    Serial.print(Output);
    Serial.print(" | Salida PWM: ");
    Serial.println(pwmOutput);

    // Esperar 1 segundo antes de la siguiente iteración
    delay(1000);
}