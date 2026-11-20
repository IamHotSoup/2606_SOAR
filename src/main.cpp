#include "main.h"
#include "fmt/core.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/pid.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <atomic>
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
 double gear_ratio = 36.0/60.0;
//motor group definition
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup right_mg({10, 19},pros::MotorGearset::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup left_mg({-17,- 18},pros::MotorGearset::blue);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::MotorGroup double_reverse_bar({15,-14}, pros::MotorGearset::green); // Creates a motor with port 7 and reversed
pros::MotorGroup intake({-13}, pros::MotorGearset::blue); // Creates a motor with port 8 and reversed
pros::Motor claw(16, pros::MotorGearset::green); // Creates a motor with port 9 and reversed
pros::Motor counter_roller(-3, pros::MotorGearset::green); // Creates a motor with port 9 and reversed
pros::Motor claw_intake(-6, pros::MotorGearset::green); // Creates a motor with port 9 and reversed

pros::Rotation claw_rotation_sensor(5); // Creates a rotation sensor on port 12

lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              14, // 14 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600*gear_ratio, // drivetrain rpm is 600
                              2 // horizontal drift is 2 (for now)
);

lemlib::TrackingWheel left_tracking(
    &left_mg,
    lemlib::Omniwheel::NEW_275,
    -7,
    600*gear_ratio
);



lemlib::TrackingWheel right_tracking(
    &right_mg,
    lemlib::Omniwheel::NEW_275,
    7,
    600*gear_ratio
);

lemlib::OdomSensors sensors(&left_tracking, // vertical tracking wheel 1, set to null
                            &right_tracking, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr // inertial sensor
);

lemlib::ControllerSettings lateral_controller(
    6.5, 
	0, 
	3, 
	3, //anti windup
	.7, //small error range, in inches 
	100, //small error range timeout, in ms
	3, //large error range, in inches
	5000, //large error range timeout, in ms
	5  //maximum allecleration (slew)
);

lemlib::ControllerSettings angular_controller(
    5,
	0, 
	3.5, 
	3, //anti windup
	1, //small error range, in inches 
	100, //small error range timeout, in ms
	3, //large error range, in inches
	250, //large error range timeout, in ms
	5  //maximum allecleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

lemlib::PID claw_pid(
		30, // kP
        0.01, // kI
        7, // kD
        5, // integral anti windup range
        false);


std::atomic<double> controller_input{0.0};
std::atomic<double> claw_target{0.0};
std::atomic<bool> claw_pid_enabled{false};
std::atomic<bool> claw_new_target{false};


void claw_task_move(void* param) {
	claw_pid.reset();
	while (true) {
		
		if (claw_new_target.load() == true) {
    		claw_pid.reset();
    		claw_new_target.store(false);
		}
		
		if (claw_pid_enabled.load() == true) {
			double claw_position = claw_rotation_sensor.get_position()/100.0;

			
			double target = claw_target.load();
			double error = target - claw_position;
			double output = claw_pid.update(error);
			output = std::clamp(output, -12000.0, 12000.0);
			claw.move_voltage(output); 
	
	 		if (std::abs(error) < 5.0) {
            	claw_pid_enabled.store(false);
				claw.move_voltage(0);
				}
			

			pros::delay(20);
		}
		else if (claw_pid_enabled.load() == false) {
			claw_pid.reset();
			claw.move_voltage(controller_input.load());
			pros::delay(20);
		}
	}
	claw.move_voltage(0);
}





void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	double_reverse_bar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	pros::lcd::initialize();
	
	pros::lcd::register_btn1_cb(on_center_button);
	
    chassis.calibrate();
	//pros::Task claw_task(claw_move_pid, nullptr);*/
   
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	chassis.setPose(0, 0, 0);
	

}
int a(){
	chassis.moveToPose(0,-17.2,0,100000);
	pros::delay(500);
	chassis.turnToHeading(34.9,100000);
	pros::delay(500);
	chassis.moveToPose(-2.2,-22.9,34.9,100000);
	return 0;
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
 double target_claw_up = 563.55;
 double target_claw_down = 211.64;
 void opcontrol() {
	chassis.setPose(0, 0, 0);
	pros::Task claw_task(claw_task_move);
	while (true) {
		
		
	
		lemlib::Pose pose = chassis.getPose();

		pros::lcd::print(0, "X: %.1f Y: %.1f", pose.x, pose.y);
		pros::lcd::print(1, "Heading: %.1f", pose.theta);
		pros::lcd::print(2, "Claw: %.1f",
		claw_rotation_sensor.get_position() / 100.0);
		
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_LEFT_X); 
			
		if (master.get_analog(ANALOG_RIGHT_Y) > 0) {
			double_reverse_bar.move_voltage(master.get_analog(ANALOG_RIGHT_Y) * (12000.0 / 127.0));
			
		}
		else if (master.get_analog(ANALOG_RIGHT_Y) < 0) {
			double_reverse_bar.move_voltage(master.get_analog(ANALOG_RIGHT_Y) * (12000.0 / 127.0));
			
		}
		else {
			double_reverse_bar.brake();
		}

		left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);                     // Sets right motor voltage
        
		if (master.get_digital(DIGITAL_L1) == 1) {
			intake.move_voltage(12000.0);
			claw_intake.move_voltage(12000.0);
		}
		else if (master.get_digital(DIGITAL_L2) == 1) {
			intake.move_voltage(-12000.0);
			claw_intake.move_voltage(-12000.0);
		}
		else {
			intake.brake();
			claw_intake.brake();
		}

		if (master.get_digital_new_press(DIGITAL_R1) == 1) {
			claw_target.store(target_claw_up);
			
			claw_new_target.store(true);
			claw_pid_enabled.store(true);
		}
		else if (master.get_digital_new_press(DIGITAL_R2) == 1) {
			claw_target.store(target_claw_down);
			
			claw_new_target.store(true);
			claw_pid_enabled.store(true);
		}

		if (master.get_digital(DIGITAL_B) == 1) {
			claw_pid_enabled.store(false);
			controller_input.store(12000.0);
		}
		else if (master.get_digital(DIGITAL_X) == 1) {
			claw_pid_enabled.store(false);
			controller_input.store(-12000.0);
		}
		else if (claw_pid_enabled.load() != true) {
			claw.brake();
		}
		

		pros::delay(20);                               // Run for 20 ms then update
	}
}

