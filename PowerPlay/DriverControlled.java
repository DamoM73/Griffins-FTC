package org.firstinspires.ftc.compcode.PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@TeleOp

public class DriverControlled extends OpMode {
    /* Declare OpMode members. */
    public DcMotor Motor;
    // Create expansion hub for this robot
    public Blinker expansion_Hub_2;
    
    // Initialise variables for gyroscope
    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    
    // Initialise motors
    public DcMotor motor_front_right;
    public DcMotor motor_back_right;
    public DcMotor motor_front_left;
    public DcMotor motor_back_left;
    public DcMotor left_lift_motor;
    public DcMotor right_lift_motor;

    public Servo intake_motor;
    /**
    private ColorSensor colour;
    private DistanceSensor distance;
    **/
    
    // Initialise interfaces with other modules
    Motion drivetrain;
    Lift lift;
    Intake intake;

    @Override
    public void init() {
        // Create expansion hub
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Control Hub");
        
        // Create gyroscope
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        // Calibrate Gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // Wait until gyroscope calibrated
        while (!imu.isGyroCalibrated()){
            try {
                Thread.sleep(100);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }
            
        
        // initialise motors
        motor_front_right = hardwareMap.get(DcMotorEx.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotorEx.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotorEx.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotorEx.class, "motor_back_left");
        
        
        left_lift_motor = hardwareMap.get(DcMotorEx.class, "left_lift_motor");
        right_lift_motor = hardwareMap.get(DcMotorEx.class, "right_lift_motor");
        intake_motor = hardwareMap.get(Servo.class, "intake_servo");
        
        // initialise the directions of the motors
        motor_front_right.setDirection(DcMotor.Direction.FORWARD);
        motor_front_left.setDirection(DcMotor.Direction.REVERSE);
        motor_back_right.setDirection(DcMotor.Direction.FORWARD);
        motor_back_left.setDirection(DcMotor.Direction.REVERSE);


        
        // initialise sensors
        /**
        colour = hardwareMap.get(ColorSensor.class, "colour");
        distance = hardwareMap.get(DistanceSensor.class, "colour");
        **/

        // Create module references
        drivetrain = new Motion(motor_front_right,motor_back_left,motor_front_left,motor_back_right,imu);
        
        lift = new Lift(left_lift_motor,right_lift_motor);
        intake = new Intake(intake_motor);
        

        // set up telemetry to disply on driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.addData("Have Fun","Drivers");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Movement
        // Standard Mechannum
        drivetrain.JoystickMoving(gamepad1.right_stick_y,gamepad1.right_stick_x,gamepad1.left_stick_x);
        
        // 90 degree turns
        if (this.gamepad1.dpad_left) {
            drivetrain.rotateAuto(90,0.4);
        }
        else if (this.gamepad1.dpad_right) {
            drivetrain.rotateAuto(-90,0.4);
        }
        
        // Lift
        lift.SetMoveSpeed(gamepad2.left_stick_y);
        if (gamepad2.dpad_up) {
            lift.MoveInPositionList(1);
        } else if (gamepad2.dpad_down) {
            lift.MoveInPositionList(-1);
        }

        
        // Intake
        if (gamepad2.b) {
            intake.pickUpCone();
        }
        if (gamepad2.x) {
            intake.putDownCone();
        }
        if (gamepad2.a) {
            intake.stopRuning();
        
        }
        
    }
}
