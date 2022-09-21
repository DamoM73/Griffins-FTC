package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {
    private DcMotor Motor;
    //Convert from the counts per revolution of the encoder to counts per inch

    // Create objects for this robot
    private Blinker expansion_Hub_2;
    //private Gyroscope imu;
    private DcMotor motor_front_right;
    private DcMotor motor_back_right;
    private DcMotor motor_front_left;
    private DcMotor motor_back_left;
    private DcMotorEx arm_motor;
    private DcMotorEx input_output_motor;
    private DcMotor DDS_motor;
    private ColorSensor colour;
    private DistanceSensor distance;

    // convert count per revolution to counts per cm 
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = ((1+(46/11))*(1+(46/11)));
    static final double WHEEL_CIRCUMFERENCE_MM = 100 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM_STRAIGHT = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM/1.2682;
    static final double DRIVE_COUNTS_PER_CM_STRAIGHT = DRIVE_COUNTS_PER_MM_STRAIGHT * 10;
    static final double DRIVE_COUNTS_PER_MM_SIDEWAYS = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_CM_SIDEWAYS = DRIVE_COUNTS_PER_MM_SIDEWAYS * 10;
    static final double DRIVE_COUNTS_PER_DEGREE = 20;


    
    public void motorFwdTargetPositions (float cmDistance, double speed) {
        motor_front_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_front_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        // set target positions when travelling forward (all +)
        int motor1Target = (int)motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT);
        int motor2Target = (int)motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT);
        int motor3Target = (int)motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT);
        int motor4Target = (int)motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT);

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        
        
        // sets power of motors
        motor_front_right.setPower(speed);
        motor_front_left.setPower(speed);
        motor_back_right.setPower(speed);
        motor_back_left.setPower(speed);
        
        motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //Waits until motors finished
        while (motor_front_left.isBusy() || motor_front_right.isBusy() || motor_back_left.isBusy() || motor_back_right.isBusy()){
            idle();
        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }
    
    public void motorBwdTargetPositions (float cmDistance, double speed) {
        motor_front_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_front_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        // set target positions when travelling backward
        int motor1Target = (int)(-1 * (motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT)));
        int motor2Target = (int)(-1 * (motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT)));
        int motor3Target = (int)(-1 * (motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT)));
        int motor4Target = (int)(-1 * (motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT)));

        // set motors to drive to position
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        
        
        // sets power of motors
        motor_front_right.setPower(-speed);
        motor_front_left.setPower(-speed);
        motor_back_right.setPower(-speed);
        motor_back_left.setPower(-speed);
        
        motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //Waits until motors finished
        while (motor_front_left.isBusy() || motor_front_right.isBusy() || motor_back_left.isBusy() || motor_back_right.isBusy()){
            telemetry.addData("Front Right", motor_front_right.isBusy());
            telemetry.addData("Front Left", motor_front_left.isBusy());
            telemetry.addData("Back Right", motor_back_right.isBusy());
            telemetry.addData("Back Left", motor_back_left.isBusy());
            telemetry.update();
        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }
    
    public void motorRgtTargetPositions (float cmDistance, double speed) {
        motor_front_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_front_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        // set target positions when driving right (fr -, bl -)
        int motor1Target = (int)(-1 * (motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS)));
        int motor2Target = (int)(motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS));
        int motor3Target = (int)(motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS));
        int motor4Target = (int)(-1 * (motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS)));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        
        // sets power of motors
        motor_front_right.setPower(-speed);
        motor_front_left.setPower(speed);
        motor_back_right.setPower(speed);
        motor_back_left.setPower(-speed);
                
        motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        //Waits until motors finished
        while (motor_front_left.isBusy() || motor_front_right.isBusy() || motor_back_left.isBusy() || motor_back_right.isBusy()){
            telemetry.addData("Front Right", motor_front_right.getCurrentPosition());
            telemetry.addData("Front Left", motor_front_left.getCurrentPosition());
            telemetry.addData("Back Right", motor_back_right.getCurrentPosition());
            telemetry.addData("Back Left", motor_back_left.getCurrentPosition());
            telemetry.update();
        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }
    
    public void motorLftTargetPositions (float cmDistance, double speed) {
        motor_front_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_front_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        
        // set target positions when driving left (fl -, br -)
        int motor1Target = (int)(motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS));
        int motor2Target = (int)(-1 * (motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS)));
        int motor3Target = (int)(-1 * (motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS)));
        int motor4Target = (int)(motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        
        
        // sets power of motors
        motor_front_right.setPower(speed);
        motor_front_left.setPower(-speed);
        motor_back_right.setPower(-speed);
        motor_back_left.setPower(speed);

        
        motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        //Waits until motors finished
        while (motor_front_left.isBusy() || motor_front_right.isBusy() || motor_back_left.isBusy() || motor_back_right.isBusy()){
            telemetry.addData("Front Right", motor_front_right.getCurrentPosition());
            telemetry.addData("Front Left", motor_front_left.getCurrentPosition());
            telemetry.addData("Back Right", motor_back_right.getCurrentPosition());
            telemetry.addData("Back Left", motor_back_left.getCurrentPosition());
            telemetry.update();
        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }
    
    /**
    public void motorFwdRgtTargetPositions (float cmDistance, double speed) {
        
        // set target positions when driving diagonally right and forward (fr 0, bl 0)
        int motor1Target = (int)motor_front_right.getCurrentPosition();
        int motor2Target = (int)(motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor3Target = (int)(motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor4Target = (int)motor_back_left.getCurrentPosition();


        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        
        
        // sets power of motors
        
        motor_back_right.setPower(speed);
        motor_front_left.setPower(speed);
        runtime.reset();
        
        //Caluculates time required
        time = CM_PER_SECOND_PER_POWER * cmDistance * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        
    }

    public void motorFwdLftTargetPositions (float cmDistance, double speed) {
        
        // set target positions when driving diagonally left forward (fl 0, br 0)
        int motor1Target = (int)(motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor2Target = (int)motor_front_left.getCurrentPosition();
        int motor3Target = (int)motor_back_right.getCurrentPosition();
        int motor4Target = (int)(motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        
        
        // sets power of motors
        
        motor_front_right.setPower(speed);
        motor_back_left.setPower(speed);
        runtime.reset();
        
        //Caluculates time required
        time = CM_PER_SECOND_PER_POWER * cmDistance * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_left.setPower(0);
    }

    public void motorBwdRgtTargetPositions (float cmDistance, double speed) {
        
        // set target positions when driving diagonally right backward (fr -1, bl -1, fl 0, br 0)
        int motor1Target = (int)(-1 * (motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor2Target = (int)motor_front_left.getCurrentPosition();
        int motor3Target = (int)motor_back_right.getCurrentPosition();
        int motor4Target = (int)(-1 * (motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        
        
        // sets power of motors
        
        motor_front_right.setPower(-speed);
        motor_back_left.setPower(-speed);
        runtime.reset();
        
        //Caluculates time required
        time = CM_PER_SECOND_PER_POWER * cmDistance * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_left.setPower(0);
    }

    public void motorBwdLftTargetPositions (float cmDistance, double speed) {
        
        // set target positions when driving diagonally left backward (fr 0, bl 0, fl -1, br -1)
        int motor1Target = (int)motor_front_right.getCurrentPosition();
        int motor2Target = (int)(-1 * (motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor3Target = (int)(-1 * (motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor4Target = (int)motor_back_left.getCurrentPosition();

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        
        
        // sets power of motors
        
        motor_back_right.setPower(-speed);
        motor_front_left.setPower(-speed);
        runtime.reset();
        
        //Caluculates time required
        time = CM_PER_SECOND_PER_POWER * cmDistance * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        
    }
    **/
    public void turnMotors(int angle, double speed) {
        motor_front_right.setTargetPosition((int)motor_front_right.getCurrentPosition()+(int)(angle*DRIVE_COUNTS_PER_DEGREE));
        motor_front_left.setTargetPosition((int)motor_front_left.getCurrentPosition()+(int)(-1*angle*DRIVE_COUNTS_PER_DEGREE));
        motor_back_right.setTargetPosition((int)motor_back_right.getCurrentPosition()+(int)(angle*DRIVE_COUNTS_PER_DEGREE));
        motor_back_left.setTargetPosition((int)motor_back_left.getCurrentPosition()+(int)(-1*angle*DRIVE_COUNTS_PER_DEGREE));
        
        // sets power of motors
        motor_front_right.setPower(speed);
        motor_back_right.setPower(-speed);
        motor_front_left.setPower(speed);
        motor_back_left.setPower(-speed);
    
        
        motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        //Waits until motors finished
        while (motor_front_left.isBusy() || motor_front_right.isBusy() || motor_back_left.isBusy() || motor_back_right.isBusy()){
            telemetry.addData("Front Right", motor_front_right.getCurrentPosition());
            telemetry.addData("Front Left", motor_front_left.getCurrentPosition());
            telemetry.addData("Back Right", motor_back_right.getCurrentPosition());
            telemetry.addData("Back Left", motor_back_left.getCurrentPosition());
            telemetry.update();
        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }
    

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
        motor_front_right = hardwareMap.get(DcMotorEx.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotorEx.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotorEx.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotorEx.class, "motor_back_left");

        // initialise sensors
        //colour = hardwareMap.get(ColorSensor.class, "colour");
        //distance = hardwareMap.get(DistanceSensor.class, "colour");

        // initialise the directions of the motors
        motor_front_right.setDirection(DcMotor.Direction.FORWARD);
        motor_front_left.setDirection(DcMotor.Direction.REVERSE);
        motor_back_right.setDirection(DcMotor.Direction.FORWARD);
        motor_back_left.setDirection(DcMotor.Direction.REVERSE);

        motor_front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // set up telemetry to disply on driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run during autonomous
        if (opModeIsActive()) {
            //Move forwards
            turnMotors(360,0.3);
            
            sleep(10000);
            motorLftTargetPositions(40,0.3);
            sleep(10000);
            motorLftTargetPositions(60,0.3);
            sleep(10000);
            motorLftTargetPositions(80,0.1);
            sleep(10000);
            motorLftTargetPositions(100,0.1);
        }
    }
}