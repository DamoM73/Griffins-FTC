package org.firstinspires.ftc.compcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class Exhibition_Maze extends LinearOpMode {
    // Create objects for this robot
    private Blinker expansion_Hub_2;
    //private Gyroscope imu;
    private DcMotor motor_front_right;
    private DcMotor motor_back_right;
    private DcMotor motor_front_left;
    private DcMotor motor_back_left;
    //private DcMotorEx arm_motor;


    public float motor1_power(float jY, float jX, float rX) {
        return ((jX + jY)/2 - rX /2);
    }

    public float motor2_power(float jY, float jX, float rX) {
        return ((jY - jX)/2 + rX /2);
    }

    public float motor3_power(float jY, float jX, float rX) {
        return ((jY - jX)/2 - rX /2);
    }

    public float motor4_power(float jY, float jX, float rX) {
        return ((jX + jY)/2 + rX /2);
    }
    
    //public void arm_down() {
    
    //}

    @Override
    public void runOpMode() {
        /*
        SETUP
        Initialises all the required variables and objects and initialises them
        ready for the start();
        */

        // initialise objects for expansion hub components
        //expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Control Hub");
        //imu = hardwareMap.get(Gyroscope.class, "imu");

        // initialise object for the dc motor
        motor_front_right = hardwareMap.get(DcMotor.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotor.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotor.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotor.class, "motor_back_left");
        //arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        //arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // initialise the direction of the motor
        motor_front_right.setDirection(DcMotor.Direction.FORWARD);
        motor_front_left.setDirection(DcMotor.Direction.REVERSE);
        motor_back_right.setDirection(DcMotor.Direction.FORWARD);
        motor_back_left.setDirection(DcMotor.Direction.REVERSE);


        // set up telemetry to disply on driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
            RUNNING
            This section loops endlessly until the program has been stopped
            */

            // sets the test_motor to test_motor_power
            motor_front_right.setPower(motor2_power(gamepad1.right_stick_y*-1, gamepad1.right_stick_x, -gamepad1.left_stick_x));
            motor_back_right.setPower(motor4_power(gamepad1.right_stick_y*-1, gamepad1.right_stick_x, -gamepad1.left_stick_x));
            motor_front_left.setPower(motor1_power(gamepad1.right_stick_y*-1, gamepad1.right_stick_x, -gamepad1.left_stick_x));
            motor_back_left.setPower(motor3_power(gamepad1.right_stick_y*-1, gamepad1.right_stick_x, -gamepad1.left_stick_x));
            
            /*if(gamepad1.a == true) {
                arm_motor.setTargetPosition(0);
                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_motor.setVelocity(50);
            }
            
            if(gamepad1.b == true) {
                arm_motor.setTargetPosition(-38);
                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_motor.setVelocity(200);
            }*/
            
            //telemetry.addData("Encoder Value", arm_motor.getCurrentPosition());
            // display current motor power to the driver station
            telemetry.update();
        }
    }
}
