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

@Autonomous(name = "Blue Backdrop")
public class BlueBackdropStart extends LinearOpMode {
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
        IntakeOuttake intake;
        
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
        intake = new IntakeOuttake(leftIntakeServo, rightIntakeServo);

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
            double centralPixelDistance = distance.getDistance(DistanceUnit.CM);
            if (centralPixelDistance > 20){
                //Pixel in centre
                driveTrain.motorLftTargetPositions(10,0.5);
                driveTrain.motorFwdTargetPositions(20,0.2); //May need to be changed
                lift.pickUpPosition();
                intake.outakeLeftAuto();
                driveTrain.rotate(-90,0.5);
                lift.compact();
                driveTrain.motorFwdTargetPositions(120,0.5);
                intake.outakeRightAuto();
                driveTrain.motorLftTargetPositions(70,0.5);
            }
            else{
                driveTrain.rotate(90,0.5);
                driveTrain.motorLftTargetPositions(15,0.5);
                double leftPixelDistance = distance.getDistance(DistanceUnit.CM);
                if(leftPixelDistance < 20){
                    driveTrain.motorFwdTargetPositions(20,0.2); //May need to be changed
                    lift.pickUpPosition();
                    intake.outakeLeftAuto();
                    driveTrain.motorBwdTargetPositions(20,0.5);
                    driveTrain.rotate(180,0.5);
                    lift.compact();
                    driveTrain.motorFwdTargetPositions(120,0.5);
                    intake.outakeRightAuto();
                    driveTrain.motorLftTargetPositions(60,0.5);
                }
                else{
                    driveTrain.rotate(180,0.5);
                    driveTrain.motorFwdTargetPositions(30,0.5);
                    lift.pickUpPosition();
                    intake.outakeLeftAuto();
                    driveTrain.motorBwdTargetPositions(20,0.5);
                    lift.compact();
                    driveTrain.motorLftTargetPositions(60,0.5);
                    driveTrain.motorFwdTargetPositions(60,0.5);
                    driveTrain.motorRgtTargetPositions(60,0.5);
                    intake.outakeRightAuto();
                    driveTrain.motorLftTargetPositions(50,0.5);
                }
            }
        }
    }
}