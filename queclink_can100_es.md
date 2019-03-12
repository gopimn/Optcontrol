# eTrans Queclink CAN100
## Introducción
El accesorio CAN100 esta diseñado para obtener información logística de vehiculos y usarla para sistemas de monitoreo. 
Tiene una salida RS-232 que no depende de ningún fabricante o modelo. Funciona con el bus CAN y J1708.
Algunas variables que decodifica son:

* Distancia recorrida del vehículo.
* Nivel de combustibe.
* Consumo de combustible.
* Velocidad del motor.
* Temperaturas del motor.
* Estado de las puertas, cerrojos e indicadores del panel del vehículo.
* Identificación del conductor basado en tacógrafo digital.

## Especificación técnica 

|Item|Valor|
|:---|---:|
|tamaño|69x49.5x18 mm|
|Voltaje de alimentación|7 to 32 Vdc|
|Corriente de alimentación|11 mA|
|Corriente en modo sleep|max 1mA|
|Temperatura de operación|-40 to 80 °C|

El dispositivo viene con el dispositivo CAN100, un conector de 4 pines para la alimentación y conexión serial, un conector 
de 6 pines para el bus can, un conector de 8 pines para el protocolo J1708 y opcionalmente un cable 
CAN click (para conexión sin contacto al bus).
## Pinout
![pinout img](https://github.com/gopimn/gopimn_style/blob/img/eTrans_can100_pinout.png)
El modo actua del dispositivo esta indicato por el LED en la carcaza, el botón en frente del panel sirve para sincronizar
el dispositivo y para despertarlo. 

* CANH(S2-1) y CANL(S2-4) son las líneas del bus CAN.
* J1708 A(S3-4) y B(S3-8) son las líneas CAN para el protocolo J1708.
* SUPPLY POWER(S1-3) y GROUND(S1-4) son las conexiones a la fuente de alimentación.
* TX(S1-1) y RX(S1-2) son las conexiones del puerto serial. 
* _Ignition output_ (S2-2) lee desde el CAN cuando la ignición del vehículo está prendida.  Logica positiva con max 100mA.
* _"Active" output_ (S2-3), salida con lógica negativa que indica el estado de los buses de datos del vehículo. 
  Se amarra a GND cuando hay datos presentes, y entra en un estado de alta impedancia cuando el vehículo
  está en modo de bajo consumo. Max 100mA.
* _Output 3_(S3-3) es una salida configurable con lógica negativa con una corriente máxima de 100 mA. 
* _Inputs_ 1(S2-5),2(S2-6) y 3(S3-7) pueden ser configuradas como entradas análogas (ADC 12 bit) o digitales.

## Instalación

Cuando se conecta la alimentación, el LED titilará verde una vez por segundo si hay información en el bus, 
o titilará una vez cada 4 segundos si el vehículo esta en modo bajo consumo o el CAN100 no esta correctamente conectado.

Si el vehículo entra a modo bajo consumo, el dispositivo CAN100 entra en modo sleep. El LED se apaga cuando 
esto sucede.

Si el dispositivo recibe información de cualquiera de los buses, regresa a la operación normal y el pin
"active" se amarra a tierra (salida 0V). El modo sleep también se puede finalizar presionando el botón del CAN100.

**Si se activa la ignición del vehículo, y el LED titila cada 4 segundos, el dispositivo no esta correctamente conectado**

## Sincronización

Los datos en el bus can pueden variar entre modelos y fabricantes significativamente. El CAN100 puede reconocer cada tipo 
de bus CAN y ajustarse a él automáticamente.

Si un CAN100 que no ha sido configurado se conecta por primera vez, se sincroniza luesgo de recibir energía. Se debe
asegurar la conexión de la ignición.

Si se quiere resincronizar el dispositovo:
1. Conectar la alimentación, el LED brillará rojo.
2. Presionar el botón en el panel frontal (mantenerlo presionado al conectar la fuente).
3. Después de pocos segundos, el LED brillará verde, lo que indica que hay que soltar el botón. Luego de iniciar el
   dispositivo, el LED titilará rojo. Luego de unos segundos la sincronización esta lista:
   * Si el LED brilla verde, el vehículo está sincronizado correctamente. 
     Apagar y luego de 5 segundos prender el dispositivo .
   * Si el LED brilla alternadamente rojo y verde significa que la conexión al bus está mal hecha. 
     Asegúrese de que los cables del CAN no estan invertidos y que la ignición esta activada. Si se sigue con el 
     problema, el dispositivo no esta conectado al bus CAN.
   * Si el LED brilla rojo, las conexiones al CAN esta correctas pero el vehículo no está soportado.

La sincronización con el bus CAN se puede realizar también por el puerto serial.

### Firmware 

Antes de instalar, hay que asegurarse de que el último _firmware_ (mas actual) está instalado en el dispositivo. El _firmware_ del
CAN100 puede ser actualizado a través del puerto serial.

## Ejemplo GV300

El Queclink GV300 puede comunicarse con el CAN100 por el protocolo RS-232. El dispositivo para esta conexión 
es el CAN100_STD (para conexión con el queclink GV65 se usa el CAN100_INV). La siguiente tabla muestra las conexiones:
<table>
   <tr>
    <td colspan="2" align="center">GV300</td>
    <td></td>
    <td colspan="2" align="center">CAN100_STD</td>
  </tr>
  <tr>
    <td>Pin</td>
    <td>Nombre</td>
    <td align="center">Conexión</td>
    <td>Nombre</td>
    <td>Pin</td>
  </tr>
  <tr>
    <td>4</td>
    <td>RXD</td>
    <td align="center"><----></td>
    <td align="right">TX</td>
    <td align="right">S1-1</td>
  </tr>
  <tr>
    <td>5</td>
    <td>TXD</td>
    <td align="center"><----></td>
    <td align="right">RX</td>
    <td align="right">S1-2</td>
  </tr>
  <tr>
    <td>11</td>
    <td>Power</td>
    <td align="center"><----></td>
    <td align="right">Power supply</td>
    <td align="right">S1-3</td>
  </tr>
  <tr>
    <td>6</td>
    <td>Ground</td>
    <td align="center"><----></td>
    <td align="right">Ground</td>
    <td align="right">S1-4</td>
  </tr>
</table>

Es importante recordar que de esta manera se está alimentando el CAN100_STD desde el GV300. Si se quiere
alimentar de una fuente externa, no conectar la alimentación (sí GND).

## Referecias
La referencia principal es [CAN100 User Guide V1.01](https://drive.google.com/open?id=1tS-P5NAi1Ux6r_UjPbvylLiTXeWuQ86A).

Para una manera detallada de como sincronizar y verificar el dispositivo, leer el documento [CAN100 Synchro and Verifying V2.1](https://drive.google.com/open?id=1VqohfdTpn7xUJhko-kuwn3CuAHL6UF-b).

[CAN100 supported car models V2.7](https://drive.google.com/open?id=1V3fXr-EIT2Gz8c4_OnbVT3Jj3m5fjY98) tiene los modelos
de vehículos soportados y sus parámetros (pesados y ligeros).

[CAN100 supported machines 2015-03-20](https://drive.google.com/open?id=1wKen_dIr94m4pK54fqXjam4E7VBp2CAn) tiene los modelos
de maquinaria soportados.
