#include <Arduino.h>
#include <SimpleFOC.h>

// measured KV for GM3506 is ~ 128 rpm/V
/*
20241001 - B phase is not measuring any current during calibration - try swapping ADC pins?
*/

//  monitoring
#define _MON_TARGET 0b1000000 // monitor target value
#define _MON_VOLT_Q 0b0100000 // monitor voltage q value
#define _MON_VOLT_D 0b0010000 // monitor voltage d value
#define _MON_CURR_Q 0b0001000 // monitor current q value - if measured
#define _MON_CURR_D 0b0000100 // monitor current d value - if measured
#define _MON_VEL 0b0000010    // monitor velocity value
#define _MON_ANGLE 0b0000001  // monitor angle value

/* MODE SELECTION - ONLY CHOOSE ONE*/
// #define voltage_control
//  #define voltage_control_resistance
//  #define voltage_control_kv
#define dc_current_control
//  #define foc_current_control

// create as5048a magnetic encoder object
MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);

// create object for simplefoc shield v2.0.4
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 9, 6, 8);

// create motor object for gimbal motor
#ifdef voltage_control
BLDCMotor motor = BLDCMotor(11);
#endif

#ifdef voltage_control_resistance
BLDCMotor motor = BLDCMotor(11, 5.50f / 2.0);
#endif

#if defined(voltage_control_kv) || defined(dc_current_control)
BLDCMotor motor = BLDCMotor(11, 5.50f / 2.0, 150);
#endif

#if defined(dc_current_control) || defined(foc_current_control)
// current sensing
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, A0, A3, _NC);
#endif

// instantiate the command
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.motor(&motor, cmd); }

// voltage limit to protect motor
uint8_t phase_voltage_limit = 3;

void setup()
{
    // monitoring port
    Serial.begin(115200);
    motor.useMonitoring(Serial);
    motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_CURR_Q;
    motor.monitor_downsample = 10000; // default 10

    // initialise magnetic sensor hardwareM5
    sensor.init();

    // setup driver
    driver.voltage_power_supply = 18;
    driver.voltage_limit = phase_voltage_limit;
    driver.init();

// setup controller
#ifdef voltage_control
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::torque;

#endif

    motor.voltage_limit = phase_voltage_limit;
    motor.voltage_sensor_align = 6.0f;

    // linking components
    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);

// current sensor setup
#ifdef dc_current_control
    current_sense.linkDriver(&driver);
    // current_sense.init();
    // motor.linkCurrentSense(&current_sense);
    motor.torque_controller = TorqueControlType::foc_current;
    motor.controller = MotionControlType::torque;

    // PID parameters - default
    motor.PID_current_q.P = 0.5; // 3    - Arduino UNO/MEGA
    motor.PID_current_q.I = 1;   // 300  - Arduino UNO/MEGA
    motor.PID_current_q.D = 0;
    motor.PID_current_q.limit = motor.voltage_limit;
    motor.PID_current_q.output_ramp = 100; // 1000 - Arduino UNO/MEGA
    motor.LPF_current_q.Tf = 0.01;
    motor.LPF_current_d.Tf = 0.01;
#endif

    // current sense init and linking

    // init
    motor.init();

    // init current sense
    current_sense.init();
    motor.linkCurrentSense(&current_sense);

    Serial.println(current_sense.alignBLDCDriver(8.0f, &driver, 1));

    // init FOC
    motor.initFOC();

    // add target command M
    command.add('M', doTarget, "motor");

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target current using serial terminal:"));
    _delay(1000);
}

void loop()
{
    motor.loopFOC();
    motor.move();
    command.run();
    motor.monitor();
}
