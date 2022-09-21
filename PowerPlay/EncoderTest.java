package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gyroscope;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "Encoder_Test")
public class Encoder_Test extends LinearOpMode {
    private DcMotor Motor;
    //Convert from the counts per revolution of the encoder to counts per inch

    // Create objects for this robot
    private Blinker expansion_Hub_2;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    
    private DcMotor motor_front_right;
    private DcMotor motor_back_right;
    private DcMotor motor_front_left;
    private DcMotor motor_back_left;

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
            sleep(10);
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

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }
    
    private double absolute(double number){
        if (number>0){return number;}
        else {return number*-1;}
    }
    
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;
        
        // restart imu movement tracking.
        double target = degrees + getAngle();
        double closeToNinety = target/90;
        target = Math.round(closeToNinety)*90;
        
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        motor_front_right.setPower(rightPower);
        motor_back_right.setPower(rightPower);
        motor_front_left.setPower(leftPower);
        motor_back_left.setPower(leftPower);
        
        motor_front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // rotate until turn is completed.
        double gap = absolute(getAngle() - target);
        telemetry.addData("Gyroscope",getAngle());
        telemetry.addData("Gap",gap);
        telemetry.update();
        sleep(1000);
        while (gap>1 && opModeIsActive()) {
            gap = absolute(getAngle() - target);
            if (gap > 360){gap-=360;}
            
            if (gap <=15){
                double scaleFactor = gap/15;
                if (scaleFactor*power<0.35){
                    scaleFactor=0.35/power;
                }
                
                if (degrees < 0) {   // turn right.
                    leftPower = power*scaleFactor;
                    rightPower = -power*scaleFactor;
                }
                else if (degrees > 0) {   // turn left.
                    leftPower = -power*scaleFactor;
                    rightPower = power*scaleFactor;
                }
                
                motor_front_right.setPower(rightPower);
                motor_back_right.setPower(rightPower);
                motor_front_left.setPower(leftPower);
                motor_back_left.setPower(leftPower);
            }
        }

        // turn the motors off.
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);

        // wait for rotation to stop.
        telemetry.addData("Finished","Finished");
        telemetry.update();
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
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        
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
            for (int i=1;i<9;i++){
                rotate(90,0.4);
                
            }
        }
    }
}