## Vehículo de Conducción Autónoma

# Descripción

Un programa de arduino para controlar un vehículo autónomamente. Se usa la tarjeta de visión con inteligencia artificial HUSKYLENS para recolectar información del entorno y tomar decisiones dependiendo de dicha información. Se utiliza un puente H L298n para controlar un motor con encoder para el avance del vehículo. La dirección es controlada por un servomotor.

# Hardware

Se requiere la utilización de un arduino mega o similar; el programa usa 3 timers de 16 bits.

# Librerías

* [Arduino 2.3.2](https://www.arduino.cc/en/software)
* [HUSKYLENS Arduino API](https://github.com/HuskyLens/HUSKYLENSArduino) - Una API de comunicación y control de la tarjeta HUSKYLENS con el lenguaje arduino.
* [PIDLoop](https://github.com/charmedlabs/pixy2/tree/master/src/host/arduino/libraries/Pixy2) - Librería para cálculo de controlador PID.