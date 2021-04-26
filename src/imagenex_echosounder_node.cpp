#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "TimeoutSerial.h"

class imagenex_echosounder {
 public:
 
  imagenex_echosounder(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
      : nh_(nh), nhp_(nhp), seq_(0)  {
	// B: ¿Parametros iniciales?
    // nhp_.param("min_range", min_range, 0.5);
    // nhp_.param("max_range", max_range, 5.0);
    // nhp_.param("gain_db", gain_db, 6);
    // nhp_.param("pulse_length_us", pulse_length_us, 100);
    // nhp_.param("delay_ms", delay_ms, 0);

    // Get configuration from rosparam server
  getConfig();
  ROS_INFO("max_range: %f", max_range);
	ROS_INFO("min_range: %f", min_range);
	ROS_INFO("gain_db: %i", gain_db);
	ROS_INFO("pulse_length_us: %i", pulse_length_us);
	ROS_INFO("delay_ms: %i", delay_ms);
	ROS_INFO("data_points: %i", data_points);
  ROS_INFO("timeout_serial: %i", timeout_serial);
  ROS_INFO("timer_duration: %f", timer_duration);

  range_pub_ = nhp_.advertise<sensor_msgs::Range>("/imagenex_echosounder/range", 1); // B: ¿Anuncia un topic/tema de un mensaje? Es usado a la hora de crear el mensaje. Supongo que crea el topic del mensaje.
    
  serial.open(115200); // B: Se abre un puerto serial y se le asigna una velocidad. ¿Que puerto se abre? ¿COM1?
	serial.setTimeout(boost::posix_time::seconds(timeout_serial)); // B: No lo entiendo... ¿Es un delay? 

  timer_ = nh_.createTimer(ros::Duration(timer_duration), // B: CreateTimer crea un temporizador. El temporizador usado es el propio de ROS. El temporizador dura 1 s.
                             &imagenex_echosounder::timerCallback, // B: Se llama al método privado timerCallback.
                             this);
  	}

 private:
  // B: ¿Se llama a este método cada vez que el timer de ROS llega al final y ocurre un "TimerEvent&"?
  
  void timerCallback(const ros::TimerEvent&) {
	 // B: Posible Breakpoint... Pasaremos por aquí cada segundo...
	// B: Pre-tratamiento de los datos
	// B: ¿Por que se hacen estos redondeos tan brutales? 
	// if(max_range < 7.5) max_range = 5.0;
 //    else if (max_range < 15.0) max_range = 10.0;
 //    else if (max_range < 25.0) max_range = 20.0;
 //    else if (max_range < 35.0) max_range = 30.0;
 //    else if (max_range < 45.0) max_range = 40.0;
 //    else max_range = 50.0;
            
    // sanity checks according to the technical specifications included in the datasheet. 

    if(min_range < 0) min_range = 0;
    else if (min_range > 25.0) min_range = 25.0; // B: ¿Por que el mínimo es de 25.0 m? 
    //eco sounder kit manual, pg 23: Byte 15 Profile Minimum Range Minimum range for profile point digitization
    //0 – 250  0 to 25 meters in 0.1 meter increments
    // Byte 15 = min range in meters / 10
            
    if(gain_db > 40) gain_db = 40;
    else if(gain_db < 0) gain_db = 0; //Start Gain: 0 to 40dB in 1 dB increments, see data sheet
    
	// # pulse_length_us --> Byte 14 Pulse Length, Length of acoustic transmit pulse. 1-255. 1 to 255 micro sec in 1 micro sec increments

    if (pulse_length_us >255) pulse_length_us = 255;
    //else if (pulse_length_us == 253) pulse_length_us = 254;
	  else if (pulse_length_us < 1) pulse_length_us = 1;
            
    if(delay_ms/2 == 253) delay_ms = 508; // B: Ajuste para que el delay de 253 sea el del 254.
    //Datasheet: The echo sounder can be commanded to pause (from 0 to 510 msec) before sending its return data to allow the commanding program
	// enough time to setup for serial reception of the return data. 0 to 255 in 2 msec increments Byte 24 = delay_in_milliseconds/2 Do not use a value of 253!
    
	// B: una vez que los datos son pre-tratados los metemos en los buffers
	
	  buffer_tx[0] = 0xFE;		        //Switch Data Header (1st Byte)
	  buffer_tx[1] = 0x44;			    //Switch Data Header (2nd Byte)
    buffer_tx[2] = 0x11;	    		//Head ID
	  buffer_tx[3] = max_range;		//Range: 5,10,20,30,40,50 in meters
	  buffer_tx[4] = 0;
	  buffer_tx[5] = 0;
	  buffer_tx[6] = 0x43;				//Master/Slave: Slave mode only (0x43)
	  buffer_tx[7] = 0;
	  buffer_tx[8] = gain_db;			//Start Gain: 0 to 40dB in 1 dB increments, ecosound kit manual ,pg 23
	  buffer_tx[9] = 0;
	  buffer_tx[10] = 20;					//Absorption: 20 = 0.2dB/m for 675kHz
	  buffer_tx[11] = 0;
	  buffer_tx[12] = 0;
	  buffer_tx[13] = 0;
	  buffer_tx[14] = pulse_length_us;			//Pulse Length: 100 microseconds
	  buffer_tx[15] = min_range*10; //Minimun Range: 0-25m in 0.1 increments
	  buffer_tx[16] = 0;
	  buffer_tx[17] = 0;
	  buffer_tx[18] = 0;					//External Trigger Control
	  buffer_tx[19] = data_points;		//Data Points: 25=250 points 'IMX'
	  buffer_tx[20] = 0;
	  buffer_tx[21] = 0;
	  buffer_tx[22] = 0;					//Profile: 0=OFF, 1=IPX output
	  buffer_tx[23] = 0;
	  buffer_tx[24] = delay_ms/2;			//Switch Delay: (delay in milliseconds)/2
	  buffer_tx[25] = 0;					//Frequency: 0=675kHz
	  buffer_tx[26] = 0xFD;				//Termination Byte - always 0xFD
	  return_data_received=false;
	
	 // B: ¿Mejora la lectura humana de los datos y los metemos en los Buffers rx?
    	// B: ¿Por que escribe y lee automáticamente si despues los datos se envian mediante un mensaje? 

    // enviar solo si se ha recibido la respuesta del anterior. fbf 30/06/2020
    
  
    serial.write(reinterpret_cast<char*>(buffer_tx), sizeof(buffer_tx)); // B: Escribe el contenido en caracteres y el tamaño del Buffer Tx en el serial 
	  try {
	 	 serial.read(reinterpret_cast<char*>(buffer_rx), sizeof(buffer_rx)); // B: Lee el contenido en caracteres y el tamaño del Buffer Rx en el serial ¿Por que? ¿Para que lo pueda leer una persona bien?
	 	} catch (...){
	 	}

    range = 0.01 * float(((buffer_rx[9] & 0x7F) << 7) | (buffer_rx[8] & 0x7F)); //B: ¿Por que se lee la posición 9 del buffer si siempre es 0?
      
    if (range>0.00){
        return_data_received=true;

        // B: Se crea la trama/mensaje para enviar la información.
        sensor_msgs::Range msg; // B: ¿Se llama al método Range msg para obtener el topic?
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "imagenex_echosounder";
        msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
        msg.field_of_view = 0.1745329252; //10 grados
        msg.min_range = min_range;
        msg.max_range = max_range;
        msg.range = range;
        ROS_INFO("max_range: %f", msg.max_range);
        ROS_INFO("min_range: %f", msg.min_range);
        ROS_INFO("range: %f", msg.range);
        range_pub_.publish(msg);
    }
    else{
      ROS_INFO("Range: %f. No range published. Either return serial data not received or out of maximum range. ", range);
    }
	 

	
    

  }
  
  
  // B: Declaraciones de variables, buffers, etc.
  
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher range_pub_;
  ros::Timer timer_;
  int64_t seq_;
  
  TimeoutSerial serial;

  unsigned char buffer_rx[265];
  unsigned char buffer_tx[27];
  double range;
  double min_range;//Distancia mínima la cual la ecosonda digitaliza la señal analógica recibida.
			// Digitalizar la señal sirve para obtener un perfil del fondo marino y poder cuantificarlo.
  double max_range;
  int gain_db; //Amplifica la señal obtenida pero también el ruido (También llamado sensibilidad)
  double absorcion;
  int pulse_length_us; //Longitud de las ondas que envía la ecosonda. Mayores longitudes son para fondos profundos.
  int delay_ms; //delay_ms para enviar los datos recogidos por la ecosonda por el puerto serie.
  int data_points;
  int timeout_serial; 
  double timer_duration; 
  bool return_data_received;

  void getConfig() {
    bool valid_config = true;

    valid_config = valid_config && ros::param::getCached("~max_range", max_range);
    valid_config = valid_config && ros::param::getCached("~min_range", min_range);
    valid_config = valid_config && ros::param::getCached("~gain_db", gain_db);
    valid_config = valid_config && ros::param::getCached("~pulse_length_us", pulse_length_us);
    valid_config = valid_config && ros::param::getCached("~delay_ms", delay_ms);
    valid_config = valid_config && ros::param::getCached("~data_points", data_points);
    valid_config = valid_config && ros::param::getCached("~timeout_serial", timeout_serial);
    valid_config = valid_config && ros::param::getCached("~timer_duration", timer_duration);
    // Shutdown if not valid
    if (!valid_config) {
        ROS_FATAL_STREAM("Shutdown due to invalid config parameters!");
        ros::shutdown();
    	}
	}
};






int main(int argc, char** argv){
  ros::init(argc, argv, "imagenex_echosounder"); //B: Inicializa el nodo de ros 
  ros::NodeHandle nh; //B: Arranca el nodo publico de ros
  ros::NodeHandle nhp("~"); //B: Arranca un nodo privado. ¿El nodo privado es copia del primero?
  imagenex_echosounder ec(nh, nhp);
  ros::spin();
  return 0;
};
