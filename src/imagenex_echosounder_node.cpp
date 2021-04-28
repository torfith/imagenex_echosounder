#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "TimeoutSerial.h"

#include "imagenex_echosounder.h"
class imagenex_echosounder_ros : public imagenex_echosounder
{
public:

  imagenex_echosounder_ros(const ros::NodeHandle &nh, const ros::NodeHandle &nhp)
      : nh_(nh), nhp_(nhp), seq_(0)
  {
    // B: ¿Parametros iniciales?
    // nhp_.param("min_range", min_range, 0.5);
    // nhp_.param("max_range", max_range, 5.0);
    // nhp_.param("gain_db", gain_db, 6);
    // nhp_.param("pulse_length_us", pulse_length_us, 100);
    // nhp_.param("delay_ms", delay_ms, 0);

    // Get configuration from rosparam server

    get_config();

    initialize();

    range_pub_ = nhp_.advertise<sensor_msgs::Range>("/imagenex_echosounder/range", 1); // B: ¿Anuncia un topic/tema de un mensaje? Es usado a la hora de crear el mensaje. Supongo que crea el topic del mensaje.

    timer_ = nh_.createTimer(ros::Duration(conf.timer_duration),        // B: CreateTimer crea un temporizador. El temporizador usado es el propio de ROS. El temporizador dura 1 s.
                             &imagenex_echosounder_ros::timerCallback, // B: Se llama al método privado timerCallback.
                             this);
  }

private:

  // B: ¿Se llama a este método cada vez que el timer de ROS llega al final y ocurre un "TimerEvent&"?

  void get_config()
  {
    bool valid_config = true;

    valid_config = valid_config && ros::param::getCached("~max_range", conf.max_range);
    valid_config = valid_config && ros::param::getCached("~min_range", conf.min_range);
    valid_config = valid_config && ros::param::getCached("~gain_db", conf.gain_db);
    valid_config = valid_config && ros::param::getCached("~pulse_length_us", conf.pulse_length_us);
    valid_config = valid_config && ros::param::getCached("~delay_ms", conf.delay_ms);
    valid_config = valid_config && ros::param::getCached("~data_points", conf.data_points);
    valid_config = valid_config && ros::param::getCached("~timeout_serial", conf.timeout_serial);
    valid_config = valid_config && ros::param::getCached("~timer_duration", conf.timer_duration);

    conf.check();

    ROS_INFO("max_range: %f", conf.max_range);
    ROS_INFO("min_range: %f", conf.min_range);
    ROS_INFO("gain_db: %i", conf.gain_db);
    ROS_INFO("pulse_length_us: %i", conf.pulse_length_us);
    ROS_INFO("delay_ms: %i", conf.delay_ms);
    ROS_INFO("data_points: %i", conf.data_points);
    ROS_INFO("timeout_serial: %i", conf.timeout_serial);
    ROS_INFO("timer_duration: %f", conf.timer_duration);

    // Shutdown if not valid
    if (!valid_config)
    {
      ROS_FATAL_STREAM("Shutdown due to invalid config parameters!");
      ros::shutdown();
    }
  }

  void timerCallback(const ros::TimerEvent &)
  {
    float range = 0.00;

    conf.check();

    request();

    if (get(range))
    {
      // B: Se crea la trama/mensaje para enviar la información.
      sensor_msgs::Range msg; // B: ¿Se llama al método Range msg para obtener el topic?
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "imagenex_echosounder";
      msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
      msg.field_of_view = 0.1745329252; //10 grados
      msg.min_range = conf.min_range;
      msg.max_range = conf.max_range;
      msg.range = range;
      ROS_INFO("max_range: %f", msg.max_range);
      ROS_INFO("min_range: %f", msg.min_range);
      ROS_INFO("range: %f", msg.range);
      range_pub_.publish(msg);
    }
    else
    {
      ROS_INFO("Range: %f. No range published. Either return serial data not received or out of maximum range. ", range);
    }
  }

  // B: Declaraciones de variables, buffers, etc.

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher range_pub_;
  ros::Timer timer_;
  int64_t seq_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imagenex_echosounder"); //B: Inicializa el nodo de ros
  ros::NodeHandle nh;                            //B: Arranca el nodo publico de ros
  ros::NodeHandle nhp("~");                      //B: Arranca un nodo privado. ¿El nodo privado es copia del primero?
  imagenex_echosounder_ros ec(nh, nhp);
  ros::spin();
  return 0;
};
