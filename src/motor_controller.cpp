#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <pigpio/pigpiod_if.h>
#include <JetsonGPIO/JetsonGPIO.h>
#include <iostream>
#include <math.h>

#define track_width 1.005 // wheel_base 1.4
#define wheel_diameter 0.30 
#define max_wheel_RPM 100

class motor_controller
{
public:

    motor_controller ()
    {
        max_linear_velocity = ((M_PI * wheel_diameter) * max_wheel_RPM) / 60;
    }
    int pwm_start = 0;
    float right_dutyCyclePercentage, left_dutyCyclePercentage;
    int direction;
    void vel_callback(const geometry_msgs::Twist::ConstPtr &Twist);

private:
    float linear_vel, angular_vel, right_vel, left_vel, right_rpm, left_rpm, maxAngularVel, max_linear_velocity;
    void wheelVelocity();
    void setRPM();
    void settingDutyCycle();
    void setDirection();
    void findMaxAngular();
};

void motor_controller::vel_callback(const geometry_msgs::Twist::ConstPtr &Twist)
{
    pwm_start = 1;
    linear_vel = Twist->linear.x;
    angular_vel = Twist->angular.z;

    if (linear_vel == 0 && angular_vel == 0)
        pwm_start = 0;

    if (linear_vel > max_linear_velocity)
        linear_vel = max_linear_velocity;
    else if (linear_vel < -max_linear_velocity)
        linear_vel = -max_linear_velocity;

    findMaxAngular();

    if (angular_vel > maxAngularVel)
        angular_vel = maxAngularVel;
    else if (angular_vel < -maxAngularVel)
        angular_vel = -maxAngularVel;

    wheelVelocity();
}

void motor_controller::findMaxAngular()
{
    maxAngularVel = (max_linear_velocity - linear_vel) * 2 / track_width;
}

void motor_controller::wheelVelocity()
{
    right_vel = abs(linear_vel + angular_vel * track_width / 2);
    left_vel = abs(linear_vel - angular_vel * track_width / 2);
    setRPM();
    setDirection();
}

void motor_controller::setDirection()
{
    // direction == 1 => forward
    // direction == 2 => backward
    // direction == 3 => hard left
    // direction == 4 => hard right

    if ((linear_vel + angular_vel * track_width / 2) > 0 && (linear_vel - angular_vel * track_width / 2) > 0)
        direction = 1;
    if ((linear_vel + angular_vel * track_width / 2) < 0 && (linear_vel - angular_vel * track_width / 2) < 0)
        direction = 2;
    if ((linear_vel + angular_vel * track_width / 2) < 0 && (linear_vel - angular_vel * track_width / 2) > 0)
        direction = 3;
    if ((linear_vel + angular_vel * track_width / 2) > 0 && (linear_vel - angular_vel * track_width / 2) < 0)
        direction = 4;
}

void motor_controller::setRPM()
{
    right_rpm = right_vel * 60 / (wheel_diameter * M_PI);
    left_rpm = left_vel * 60 / (wheel_diameter * M_PI);

    settingDutyCycle();
}

void motor_controller::settingDutyCycle()
{
    right_dutyCyclePercentage = right_rpm / max_wheel_RPM;
    left_dutyCyclePercentage = left_rpm / max_wheel_RPM;

    right_dutyCyclePercentage = right_dutyCyclePercentage * 100;
    left_dutyCyclePercentage = left_dutyCyclePercentage * 100;

    if (right_dutyCyclePercentage > 100)
        right_dutyCyclePercentage = 99;
    if (left_dutyCyclePercentage > 100)
        left_dutyCyclePercentage = 99;
}

int main(int argc, char **argv)
{
    motor_controller motor_control;

    ros::init(argc, argv, "motorControllerNode");
    ros::NodeHandle n;
    ros::Rate rate(60);

    // Pigpio 
    
    if (gpioInitialise() < 0)
    {
        fprintf(stderr, "pigpio initialisation failed\n");
        return 1;
    }

    // Jetson GPIO 

    // GPIO::setmode(GPIO::BCM);

    int Ena, Enb, Dira, Dirb;

    Ena = 12;
    Enb = 13;

    Dira = 6;
    Dirb = 19;

    gpioSetMode(Ena, PI_OUTPUT);
    gpioSetMode(Enb, PI_OUTPUT);

    gpioSetMode(Dira, PI_OUTPUT);
    gpioSetMode(Dirb, PI_OUTPUT);

    gpioSetPWMrange(Ena, 100);
    gpioSetPWMrange(Enb, 100);

    // L298 Motor Driver

    // Ena = 12;
    // ln1 = 19;
    // ln2 = 26;
    // ln3 = 16;
    // ln4 = 20;
    // Enb = 13;

    // GPIO::setup(Ena, GPIO::OUT, GPIO::LOW);
    // GPIO::setup(ln1, GPIO::OUT);
    // GPIO::setup(ln2, GPIO::OUT);
    // GPIO::setup(ln3, GPIO::OUT);
    // GPIO::setup(ln4, GPIO::OUT);
    // GPIO::setup(Enb, GPIO::OUT, GPIO::LOW);

    // GPIO::PWM pwm_r(Ena, 254);
    // GPIO::PWM pwm_l(Enb, 254);

    // gpioSetMode(ln3, PI_OUTPUT);
    // gpioSetMode(ln4, PI_OUTPUT);

    gpioPWM(Enb, 0);
    gpioPWM(Ena, 0);
    gpioPWM(Dira, 0);
    gpioPWM(Dirb, 0);
    // Initiallizing subscribers

    ros::Subscriber subCMD_VEL = n.subscribe("/cmd_vel", 1, &motor_controller::vel_callback, &motor_control);

    ros::Duration(0.1).sleep();

    while (ros::ok())
    {
        ros::spinOnce();
        if (motor_control.pwm_start == 1)
        {
            if (motor_control.direction == 1)
            {
                // Jetson GPIO

                // pwm_l.ChangeDutyCycle(motor_control.left_dutyCyclePercentage);
                // pwm_r.ChangeDutyCycle(motor_control.right_dutyCyclePercentage);

                // Pi GPIO

                gpioPWM(Enb, motor_control.left_dutyCyclePercentage);
                gpioPWM(Ena, motor_control.right_dutyCyclePercentage);

                std::cout << "Left Duty Cycle: " << motor_control.left_dutyCyclePercentage << "% ";
                std::cout << "Right Duty Cycle: " << motor_control.right_dutyCyclePercentage << "% ";
                std::cout << "Direction: " << motor_control.direction << std::endl;

                // Jetson GPIO

                // GPIO::output(ln1, GPIO::HIGH);
                // GPIO::output(ln2, GPIO::LOW);
                // GPIO::output(ln3, GPIO::HIGH);
                // GPIO::output(ln4, GPIO::LOW);


                // pigpio

                gpioWrite(Dira, 0);
                gpioWrite(Dirb, 0);

            }
            else if (motor_control.direction == 2)
            {
                // Jetson GPIO

                // pwm_l.ChangeDutyCycle(motor_control.left_dutyCyclePercentage);
                // pwm_r.ChangeDutyCycle(motor_control.right_dutyCyclePercentage);

                // Pi GPIO

                gpioPWM(Enb, motor_control.left_dutyCyclePercentage);
                gpioPWM(Ena, motor_control.right_dutyCyclePercentage);

                std::cout << "Left Duty Cycle: " << motor_control.left_dutyCyclePercentage << "% ";
                std::cout << "Right Duty Cycle: " << motor_control.right_dutyCyclePercentage << "% ";
                std::cout << "Direction: " << motor_control.direction << std::endl;
                
                // Jetson GPIO

                // GPIO::output(ln1, GPIO::LOW);
                // GPIO::output(ln2, GPIO::HIGH);
                // GPIO::output(ln3, GPIO::LOW);
                // GPIO::output(ln4, GPIO::HIGH);

                // pigpio 

                gpioWrite(Dira, 1);
                gpioWrite(Dirb, 1);
            }
            else if (motor_control.direction == 3)
            {
                // Jetson GPIO

                // pwm_l.ChangeDutyCycle(motor_control.left_dutyCyclePercentage);
                // pwm_r.ChangeDutyCycle(motor_control.right_dutyCyclePercentage);

                // Pi GPIO

                gpioPWM(Enb, motor_control.left_dutyCyclePercentage);
                gpioPWM(Ena, motor_control.right_dutyCyclePercentage);

                std::cout << "Left Duty Cycle: " << motor_control.left_dutyCyclePercentage << "% ";
                std::cout << "Right Duty Cycle: " << motor_control.right_dutyCyclePercentage << "% ";
                std::cout << "Direction: " << motor_control.direction << std::endl;
                
                // Jetson GPIO

                // GPIO::output(ln1, GPIO::HIGH);
                // GPIO::output(ln2, GPIO::LOW);
                // GPIO::output(ln3, GPIO::LOW);
                // GPIO::output(ln4, GPIO::HIGH);

                // Pigpio
                
                gpioWrite(Dira, 1);
                gpioWrite(Dirb, 0);
            }
            else if (motor_control.direction == 4)
            {
                // Jetson GPIO

                // pwm_l.ChangeDutyCycle(motor_control.left_dutyCyclePercentage);
                // pwm_r.ChangeDutyCycle(motor_control.right_dutyCyclePercentage);

                // Pi GPIO

                gpioPWM(Enb, motor_control.left_dutyCyclePercentage);
                gpioPWM(Ena, motor_control.right_dutyCyclePercentage);

                std::cout << "Left Duty Cycle: " << motor_control.left_dutyCyclePercentage << "% ";
                std::cout << "Right Duty Cycle: " << motor_control.right_dutyCyclePercentage << "% ";
                std::cout << "Direction: " << motor_control.direction << std::endl;
                
                // Jetson GPIO

                // GPIO::output(ln1, GPIO::LOW);
                // GPIO::output(ln2, GPIO::HIGH);
                // GPIO::output(ln3, GPIO::HIGH);
                // GPIO::output(ln4, GPIO::LOW);

                // Pigpio
                
                gpioWrite(Dira, 0);
                gpioWrite(Dirb, 1);
            }
        }
        else if (motor_control.pwm_start == 0)
        {
            std::cout << "Right Duty Cycle: 0% Left Duty Cycle: 0%" << std::endl;

            // Jetson GPIO

            // pwm_l.ChangeDutyCycle(0);
            // pwm_r.ChangeDutyCycle(0);

            // Pigpio

            gpioPWM(Enb, 0);
            gpioPWM(Ena, 0);
        }
    }
    gpioTerminate();	
    // GPIO::cleanup();
}
