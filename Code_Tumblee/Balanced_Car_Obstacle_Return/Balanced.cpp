#include "Balanced.h"
#include "Wire.h"
#include "Motor.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "KalmanFilter.h"

MPU6050 mpu;
Mpu6050 Mpu6050;
Balanced Balanced;
KalmanFilter kalmanfilter;
Motor Motor;

MotionMode motion_mode = MODE_START;
char balance_angle_min = -22;
char balance_angle_max = 22;
volatile float debug_angle = 0.0; //zum debuggen
double angle_setpoint = 0.0; 

void Timer2::init(int time)
{
  MsTimer2::set(time,interrupt);
  MsTimer2::start();
}

static void Timer2::interrupt()
{ 
  sei();//enable the global interrupt
  Balanced.Get_EncoderSpeed();
  Mpu6050.DataProcessing();
  Balanced.PD_VerticalRing();//Serial.println(Balanced.setting_turn_speed);
  Balanced.interrupt_cnt++;
  if(Balanced.interrupt_cnt > 8)
  {
    Balanced.interrupt_cnt=0;
    Balanced.PI_SpeedRing();
    Balanced.PI_SteeringRing();
   }
  Balanced.Total_Control();
}

Balanced::Balanced()
{
  //kp_balance = 55, kd_balance = 0.75;     // Original
  kp_balance = 55, kd_balance = 0.75;
  //kp_speed = 10, ki_speed = 0.26;         // Original
  kp_speed = 10, ki_speed = 0.26;
  //kp_turn = 2.5, kd_turn = 0.5;           // Original
  kp_turn = 2.5, kd_turn = 0.5;
}

void Balanced::Total_Control()
{
  pwm_left = balance_control_output - speed_control_output - rotation_control_output;//Superposition of Vertical Velocity Steering Ring
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;//Superposition of Vertical Velocity Steering Ring
  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  double angle_rel = kalmanfilter.angle - angle_setpoint;

  if (motion_mode && motion_mode != MODE_STOP &&
    (angle_rel < balance_angle_min || angle_rel > balance_angle_max)) {
    motion_mode = MODE_STOP;
    motion_mode = MODE_STOP;
    car_speed_integeral = 0;
    setting_car_speed = 0;
    setting_turn_speed = 0;
    pwm_left = 0;
    pwm_right = 0;
    Motor.Stop();
    return; // ganz wichtig: kein Motor-PWM mehr ausgeben
  }

  // Wenn STOP: Motor aus und raus
  if (motion_mode == MODE_STOP)
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    setting_turn_speed = 0;
    pwm_left = 0;
    pwm_right = 0;
    Motor.Stop();
    return;
  }
  
  (pwm_left < 0) ?  (Motor.Control(AIN1,1,PWMA_LEFT,-pwm_left)):
                    (Motor.Control(AIN1,0,PWMA_LEFT,pwm_left));
  
  (pwm_right < 0) ? (Motor.Control(BIN1,1,PWMB_RIGHT,-pwm_right)): 
                    (Motor.Control(BIN1,0,PWMB_RIGHT,pwm_right));

}

void Balanced::Get_EncoderSpeed()
{
  encoder_left_pulse_num_speed += pwm_left < 0 ? -Motor::encoder_count_left_a : 
                                                  Motor::encoder_count_left_a;
  encoder_right_pulse_num_speed += pwm_right < 0 ? -Motor::encoder_count_right_a : 
                                                    Motor::encoder_count_right_a;
  Motor::encoder_count_left_a=0;
  Motor::encoder_count_right_a=0;
}

void Balanced::Motion_Control(Direction direction)
{
  switch(direction)
  {
    case STOP:
                  Stop();break;
    case FORWARD:
                  Forward(4);break;
    case BACK:
                  Back(4);break;
    case LEFT:
                  Left(5);break;
    case RIGHT:
                  Right(5);break;
    default:      
                  Stop();break;
  }
}

void Balanced::Stop()
{
  setting_car_speed = 0;
  setting_turn_speed = 0;
}

void Balanced::Forward(int speed)
{
  setting_car_speed = speed;
  setting_turn_speed = 0;
}

void Balanced::Back(int speed)
{
  setting_car_speed = -speed;
  setting_turn_speed = 0;
}

void Balanced::Left(int speed)
{
  setting_car_speed = 0;
  setting_turn_speed = speed;
}

void Balanced::Right(int speed)
{
  setting_car_speed = 0;
  setting_turn_speed = -speed;
}

void Balanced::PI_SpeedRing()
{
   double car_speed=(encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
   encoder_left_pulse_num_speed = 0;
   encoder_right_pulse_num_speed = 0;
   speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
   speed_filter_old = speed_filter;
   car_speed_integeral += speed_filter;
   car_speed_integeral += -setting_car_speed; 
   car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);

   speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
}

void Balanced::PD_VerticalRing()
{
  // Fehler gegen Sollwinkel
  double angle_error = kalmanfilter.angle - angle_setpoint;

  // Gyro-Offset weiter nutzen
  balance_control_output =
      kp_balance * (angle_error) +
      kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);
}


void Balanced::PI_SteeringRing()
{  
   rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;////control with Z-axis gyroscope
}


void Mpu6050::init()
{
   Wire.begin();         
   mpu.initialize();    
 }

Mpu6050::Mpu6050()
{
    dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
}

void Mpu6050::DataProcessing()
{  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);// Data acquisition of MPU6050 gyroscope and accelerometer
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);// Obtaining Angle by Kalman Filter
}

void Balanced::CalibrateZero(int samples)
{
  double gx_sum = 0;

  for (int i = 0; i < samples; i++) {
    Mpu6050.DataProcessing();
    gx_sum += kalmanfilter.Gyro_x;
    delay(5);
  }

  angular_velocity_zero = gx_sum / samples;
  angle_zero = 0; // <- wichtig: nicht mehr “Schräg-Null” als Ziel verwenden
}

void Balanced::CurveRight(int forwardSpeed, int turnSpeed)
{
  setting_car_speed = forwardSpeed;
  setting_turn_speed = -turnSpeed;
}

