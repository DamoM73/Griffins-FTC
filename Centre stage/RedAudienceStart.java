package org.firstinspires.ftc.compcode.CentreStage;
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

@Autonomous(name = "Red Audience")
public class RedAudienceStart extends LinearOpMode {
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
    private DcMotor liftRotateMotor;
    private DcMotor liftExtendMotor;
    private Servo wristServo;

    public Servo leftIntakeServo;
    public Servo rightIntakeServo;

    private ColorSensor colour;
    private DistanceSensor distance;

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
        liftRotateMotor = hardwareMap.get(DcMotorEx.class, "lift_rotate_motor");
        liftExtendMotor = hardwareMap.get(DcMotorEx.class, "right_extend_motor");
        wristServo = hardwareMap.get(Servo.class, "intake_servo");
        leftIntakeServo = hardwareMap.get(Servo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(Servo.class, "right_intake_servo");

        lift = new Lift(liftRotateMotor, liftExtendMotor, wristServo);
        intake = new Intake(leftIntakeServo, rightIntakeServo);

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
            driveTrain.motorFwdTargetPositions(60,0.5);
            centralPixelDistance = distance.getDistance(DistanceUnit.CM);
            if (centralPixelDistance > 20) {
                // Pixel in centre
                driveTrain.motorRgtTargetPositions(10,0.5);
                driveTrain.motorFwdTargetPositions(20,0.2);
                lift.pickUpPosition();
                intake.outakeAutoLeft();
                driveTrain.rotate(180);
                lift.compact();
                driveTrain.motorFwdTargetPositions(60,0.4);
                driveTrain.rotate(90);
                driveTrain.motorFwdTargetPositions(180,0.4);
                driveTrain.rotate(-90);
                lift.moveToBasePosition();
                driveTrain.motorFwdTargetPositions(60,0.4);
                driveTrain.rotate(-90);
                driveTrain.motorFwdTargetPositions(20,0.2);
                intake.outakeAutoRight();
            }
            else {
                driveTrain.rotate(90);
                driveTrain.motorRgtTargetPositions(15,0.1);
                leftPixelDistance = distance.getDistance(DistanceUnit.CM);
                if (leftPixelDistance > 20) {
                    driveTrain.motorFwdTargetPositions(20,0.2);
                    lift.pickUpPosition();
                    intake.outakeAutoLeft();
                    driveTrain.motorBwdTargetPositions(20,0.1)
                    driveTrain.rotate(90);
                    lift.compact();
                    driveTrain.motorFwdTargetPositions(60,0.4);
                    driveTrain.rotate(90);
                    driveTrain.motorFwdTargetPositions(180,0.4);
                    driveTrain.rotate(-90);
                    lift.moveToBasePosition();
                    driveTrain.motorFwdTargetPositions(70,0.4);
                    driveTrain.rotate(-90);
                    driveTrain.motorFwdTargetPositions(20,0.2);
                    intake.outakeAutoRight();
                }
                else {
                    driveTrain.rotate(180);
                    driveTrain.motorFwdTargetPositions(20,0.2);
                    lift.pickUpPosition();
                    intake.outakeAutoLeft();
                    driveTrain.motorBwdTargetPositions(20,0.1)
                    driveTrain.rotate(-90);
                    lift.compact();
                    driveTrain.motorFwdTargetPositions(60,0.4);
                    driveTrain.rotate(90);
                    driveTrain.motorFwdTargetPositions(180,0.4);
                    driveTrain.rotate(-90);
                    lift.moveToBasePosition();
                    driveTrain.motorFwdTargetPositions(50,0.4);
                    driveTrain.rotate(-90);
                    driveTrain.motorFwdTargetPositions(20,0.2);
                    intake.outakeAutoRight();
                }
            }
        }
    }
}
