#include "TimeoutSerial.h"

class imagenex_echosounder
{
public:

  class config
  {
  public:

    config()
    {
      min_range = 0.5;
      max_range = 5.0;
      gain_db = 6;
      absorption_db = 0.2;
      pulse_length_us = 100;
      delay_ms = 0;
      data_points = 250;
      timeout_serial = 1;
      timer_duration = 0.25;

      check();
    }

    void check()
    {
      // sanity checks according to the technical specifications included in the datasheet.

      if (max_range < 5.0)
        max_range = 5.0;
      else if (max_range < 10.0)
        max_range = 10.0;
      else if (max_range < 20.0)
        max_range = 20.0;
      else if (max_range < 30.0)
        max_range = 30.0;
      else if (max_range < 40.0)
        max_range = 40.0;
      else
        max_range = 50.0;

      if (min_range < 0)
        min_range = 0;
      else if (min_range > 25.0)
        min_range = 25.0; // B: ¿Por que el mínimo es de 25.0 m?
      //eco sounder kit manual, pg 23: Byte 15 Profile Minimum Range Minimum range for profile point digitization
      //0 – 250  0 to 25 meters in 0.1 meter increments
      // Byte 15 = min range in meters / 10

      if (gain_db > 40)
        gain_db = 40;
      else if (gain_db < 0)
        gain_db = 0; //Start Gain: 0 to 40dB in 1 dB increments, see data sheet

      // # pulse_length_us --> Byte 14 Pulse Length, Length of acoustic transmit pulse. 1-255. 1 to 255 micro sec in 1 micro sec increments

      if (pulse_length_us > 255)
        pulse_length_us = 255;
      else if (pulse_length_us < 1)
        pulse_length_us = 1;

      if (delay_ms / 2 == 253)
        delay_ms = 508; // B: Ajuste para que el delay de 253 sea el del 254.
                        // Datasheet: The echo sounder can be commanded to pause (from 0 to 510 msec) before sending its return data to allow the commanding program
                        // enough time to setup for serial reception of the return data. 0 to 255 in 2 msec increments Byte 24 = delay_in_milliseconds/2 Do not use a value of 253!
    }

  public:
    double min_range; //Distancia mínima la cual la ecosonda digitaliza la señal analógica recibida.
                      // Digitalizar la señal sirve para obtener un perfil del fondo marino y poder cuantificarlo.
    double max_range;
    int gain_db; //Amplifica la señal obtenida pero también el ruido (También llamado sensibilidad)
    double absorption_db;
    int pulse_length_us; //Longitud de las ondas que envía la ecosonda. Mayores longitudes son para fondos profundos.
    int delay_ms;        //delay_ms para enviar los datos recogidos por la ecosonda por el puerto serie.
    int data_points;
    int timeout_serial;
    double timer_duration;
  };

  imagenex_echosounder()
  {
  }

  void initialize()
  {
    serial.open(115200);                                                // B: Se abre un puerto serial y se le asigna una velocidad. ¿Que puerto se abre? ¿COM1?
    serial.setTimeout(boost::posix_time::seconds(conf.timeout_serial)); // B: No lo entiendo... ¿Es un delay?
  }

  void request()
  {
    set_buffer();

    serial.write(reinterpret_cast<char *>(buffer_tx), sizeof(buffer_tx)); // B: Escribe el contenido en caracteres y el tamaño del Buffer Tx en el serial
  }

  bool get(float &range)
  {
    // B: ¿Mejora la lectura humana de los datos y los metemos en los Buffers rx?
    // B: ¿Por que escribe y lee automáticamente si despues los datos se envian mediante un mensaje?

    // enviar solo si se ha recibido la respuesta del anterior. fbf 30/06/2020

    try
    {
      serial.read(reinterpret_cast<char *>(buffer_rx), sizeof(buffer_rx)); // B: Lee el contenido en caracteres y el tamaño del Buffer Rx en el serial ¿Por que? ¿Para que lo pueda leer una persona bien?
    }
    catch (...)
    {
    }

    range = 0.01 * float(((buffer_rx[9] & 0x7F) << 7) | (buffer_rx[8] & 0x7F)); //B: ¿Por que se lee la posición 9 del buffer si siempre es 0?

    return range > 0.00;
  }

private:

  void set_buffer()
  {
    // B: una vez que los datos son pre-tratados los metemos en los buffers

    buffer_tx[0] = 0xFE;           //Switch Data Header (1st Byte)
    buffer_tx[1] = 0x44;           //Switch Data Header (2nd Byte)
    buffer_tx[2] = 0x11;           //Head ID
    buffer_tx[3] = conf.max_range; //Range: 5,10,20,30,40,50 in meters
    buffer_tx[4] = 0;
    buffer_tx[5] = 0;
    buffer_tx[6] = 0x43; //Master/Slave: Slave mode only (0x43)
    buffer_tx[7] = 0;
    buffer_tx[8] = conf.gain_db; //Start Gain: 0 to 40dB in 1 dB increments, ecosound kit manual ,pg 23
    buffer_tx[9] = 0;
    buffer_tx[10] = conf.absorption_db * 100; //Absorption: 20 = 0.2dB/m for 675kHz
    buffer_tx[11] = 0;
    buffer_tx[12] = 0;
    buffer_tx[13] = 0;
    buffer_tx[14] = conf.pulse_length_us; //Pulse Length: 100 microseconds
    buffer_tx[15] = conf.min_range * 10;  //Minimun Range: 0-25m in 0.1 increments
    buffer_tx[16] = 0;
    buffer_tx[17] = 0;
    buffer_tx[18] = 0;                     //External Trigger Control
    buffer_tx[19] = conf.data_points / 10; //Data Points: 25=250 points 'IMX'
    buffer_tx[20] = 0;
    buffer_tx[21] = 0;
    buffer_tx[22] = 0; //Profile: 0=OFF, 1=IPX output
    buffer_tx[23] = 0;
    buffer_tx[24] = conf.delay_ms / 2; //Switch Delay: (delay in milliseconds)/2
    buffer_tx[25] = 0;                 //Frequency: 0=675kHz
    buffer_tx[26] = 0xFD;              //Termination Byte - always 0xFD
  }

  // B: ¿Se llama a este método cada vez que el timer de ROS llega al final y ocurre un "TimerEvent&"?

  TimeoutSerial serial;

  unsigned char buffer_rx[265];
  unsigned char buffer_tx[27];

protected:
  config conf;
};
