package org.firstinspires.ftc.compcode.PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

@Autonomous(name = "BasicAutonomousTesting")
public class BasicAutonomousTesting extends LinearOpMode {
    public DcMotor Motor;
    // Create objects for this robot
    public Blinker expansion_Hub_2;
    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    
    public DcMotor motor_front_right;
    public DcMotor motor_back_right;
    public DcMotor motor_front_left;
    public DcMotor motor_back_left;

    private DcMotorEx arm_motor;
    private DcMotorEx input_output_motor;
    private DcMotor DDS_motor;
    private ColorSensor colour;
    private DistanceSensor distance;

    
    
    public void addTelemetry(String name, double value) {
        telemetry.addData(name,value);
    }

    public void updateTelemetry() {
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
        while (!isStopRequested() && !imu.isGyroCalibrated()){
            try {
                Thread.sleep(7000);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }
            
        
        // initialise object for the dc motor
        motor_front_right = hardwareMap.get(DcMotorEx.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotorEx.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotorEx.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotorEx.class, "motor_back_left");

        // initialise the directions of the motors
        motor_front_right.setDirection(DcMotor.Direction.FORWARD);
        motor_front_left.setDirection(DcMotor.Direction.REVERSE);
        motor_back_right.setDirection(DcMotor.Direction.FORWARD);
        motor_back_left.setDirection(DcMotor.Direction.REVERSE);

        
        // initialise sensors
        //colour = hardwareMap.get(ColorSensor.class, "colour");
        //distance = hardwareMap.get(DistanceSensor.class, "colour");
        
        Motion driveTrain = new Motion(motor_front_right,motor_back_left,motor_front_left,motor_back_right,imu);
        
        // set up telemetry to disply on driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run during autonomous
        if (opModeIsActive()) {
            //Move forwards
            driveTrain.motorFwdTargetPositions(100,0.5);
        }
    }
}

class Motion {
    public Blinker expansion_Hub_2;
    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    
    public DcMotor motor_front_right;
    public DcMotor motor_back_right;
    public DcMotor motor_front_left;
    public DcMotor motor_back_left;
    
    // convert count per revolution to counts per cm 
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = ((1+(46/11))*(1+(46/11)));
    static final double WHEEL_CIRCUMFERENCE_MM = 100 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM_STRAIGHT = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM/1.2682;
    static final double DRIVE_COUNTS_PER_CM_STRAIGHT = DRIVE_COUNTS_PER_MM_STRAIGHT * 10;
    static final double DRIVE_COUNTS_PER_MM_SIDEWAYS = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_CM_SIDEWAYS = DRIVE_COUNTS_PER_MM_SIDEWAYS * 10;

    private float motor1_power(float jY, float jX, float rX) {
        return ((jX + jY)/2 - rX /2);
    }

    private float motor2_power(float jY, float jX, float rX) {
        return ((jY - jX)/2 + rX /2);
    }

    private float motor3_power(float jY, float jX, float rX) {
        return ((jY - jX)/2 - rX /2);
    }

    private float motor4_power(float jY, float jX, float rX) {
        return ((jX + jY)/2 + rX /2);
    }
    
    Motion (DcMotor motor_front_rightN,DcMotor motor_back_leftN, DcMotor motor_front_leftN, DcMotor motor_back_rightN,BNO055IMU imuN) {
        this.motor_front_right = motor_front_rightN;
        this.motor_back_left = motor_back_leftN;
        this.motor_front_left = motor_front_leftN;
        this.motor_back_right = motor_back_rightN;
        this.imu = imuN;
        
        
        this.motor_front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    
    public void JoystickMoving (float left_x,float right_x,float right_y) {
        motor_front_right.setPower(motor2_power(right_y*-1, right_x, -left_x));
        motor_back_right.setPower(motor4_power(right_y*-1, right_x, -left_x));
        motor_front_left.setPower(motor1_power(right_y*-1, right_x, -left_x));;
        motor_back_left.setPower(motor3_power(right_y*-1, right_x, -left_x));
    }
    
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
            try {
                Thread.sleep(7000);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
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
            try {
                Thread.sleep(7000);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
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
            try {
                Thread.sleep(7000);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
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
            try {
                Thread.sleep(7000);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }
    
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

        while (gap>1) {
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
    }
}