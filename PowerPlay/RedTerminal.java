package org.firstinspires.ftc.compcode.PowerPlay;
// Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
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

@Autonomous(name = "Red Terminal")
public class RedTerminal extends LinearOpMode {
    public int square = 60;
    
    public DcMotor Motor;
    // Create objects for this robot
    public Blinker expansion_Hub_2;
    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    
    public DcMotor motor_front_right;
    public DcMotor motor_back_right;
    public DcMotor motor_front_left;
    public DcMotor motor_back_left;
    public DcMotor left_lift_motor;
    public DcMotor right_lift_motor;

    public Servo intake_motor;

    private ColorSensor colour;
    private DistanceSensor distance;

    public void addTelemetry(String name, double value) {
        // Adds data to telemetry on driver hub
        telemetry.addData(name,value);
    }

    public void updateTelemetry() {
        // Updates telemetry to display
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        
        //SETUP
        //Initialises all the required variables and objects and initialises them
        //ready for the start();
        Motion driveTrain;
        Lift lift;
        Intake intake;
        
        // initialise object for the dc motor
        motor_front_right = hardwareMap.get(DcMotorEx.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotorEx.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotorEx.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotorEx.class, "motor_back_left");
        intake_motor = hardwareMap.get(Servo.class, "intake_servo");

        left_lift_motor = hardwareMap.get(DcMotorEx.class, "left_lift_motor");
        right_lift_motor = hardwareMap.get(DcMotorEx.class, "right_lift_motor");

        // initialise the directions of the motors
        motor_front_right.setDirection(DcMotor.Direction.FORWARD);
        motor_front_left.setDirection(DcMotor.Direction.REVERSE);
        motor_back_right.setDirection(DcMotor.Direction.FORWARD);
        motor_back_left.setDirection(DcMotor.Direction.REVERSE);

        lift = new Lift(left_lift_motor,right_lift_motor);
        intake = new Intake(intake_motor);

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
        
        // Intiialise drive chain
        driveTrain = new Motion(motor_front_right,motor_back_left,motor_front_left,motor_back_right,imu);
        
        // Pick up preload cone
        intake.pickUpConeAuto();
        
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()){
            try {
                Thread.sleep(100);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }
            
        
        // set up telemetry to disply on driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run during autonomous
        if (opModeIsActive()) {
            // Move forwards to sensor
            lift.MoveToPosition(200);
            driveTrain.motorFwdTargetPositions(50,0.3);
            
            // Pretend to read sensor
            
            int position = 2;
            
            /**
            // Code to analyse sensor reading if we had sensor
            if (green > red) {
                if (green > blue) {
                    // green > blue OR red
                    position = 1;
                }
                else {
                    // blue > green > red
                    position = 3;
                }
            } else {
                // red > green
                if (blue > red) {
                    // blue>red>green
                    position = 3;
                } else {
                    // red > (blue or green)
                    position = 2;
                }
            }
            **/
            
            telemetry.addData("Position",position);
            telemetry.update();
            
            // Move to high pole
            driveTrain.motorFwdTargetPositions(33,0.4);
            lift.MoveToPosition(390);
            telemetry.addData("Moving","turn");
            telemetry.update();
            driveTrain.rotate(-90,0.7);
            telemetry.addData("Moving","forward");
            telemetry.update();
            
            try {
                Thread.sleep(4000);
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }

            // Put down cone
            intake.putDownConeAuto();
            telemetry.addData("Moving","backward");
            telemetry.update();
            
            // Move to correct position
            if (position == 1) {
                // If 1 (leftmost)
                telemetry.addData("Status","Cone dropped");
                driveTrain.motorRgtTargetPositions(square/2,0.5);
                driveTrain.motorBwdTargetPositions(square,0.5);
            } else if (position == 2) {
                // If 2 (middle)
                // Already in the right place
            } else {
                // If 3 (rightmost)
                driveTrain.motorLftTargetPositions(square/2,0.5);
                driveTrain.motorFwdTargetPositions(square,0.5);
            }
        }
    }
}
