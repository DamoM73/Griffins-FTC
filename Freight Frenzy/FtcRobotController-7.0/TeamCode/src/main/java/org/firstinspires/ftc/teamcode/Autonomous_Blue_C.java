package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Autonomous_Red_C extends LinearOpMode {
    //Object defintions
    private Blinker Control_Hub;
    private Blinker Expansion_Hub;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotor armlift;
    private DcMotor intake;
    private Servo armextend;
    private Servo claw;
    private DcMotor DDS;
    private ColorSensor colour;
    private DistanceSensor distance;

    private ElapsedTime runtime = new ElapsedTime();
    
     // convert count per revolution to counts per cm 
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 19.2;
    static final double WHEEL_CIRCUMFERENCE_MM = 10 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_CM = DRIVE_COUNTS_PER_MM * 10;
    static final double TIME_AT_HALF_POWER_S = 4.73;
    static final double VELOCITY_AT_HALF_POWER_CMS = (300 / TIME_AT_HALF_POWER_S);
    //functions
    // Creates a function to move the robot forward a set distance
    public void fwdDrive(double distance) {
        // determines the ideal time for any distance (cm)
        double time = (distance / VELOCITY_AT_HALF_POWER_CMS);
        motor1.setPower(0.5);
        motor2.setPower(0.5);
        motor3.setPower(0.5);
        motor4.setPower(0.5);
        telemetry.addData("time", time);
        
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
        }
        
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    
    // Creates a function to move the robot backward a set distance
    public void bwdDrive(double distance) {
        // determines the ideal time for any distance (cm)
        double time = (distance / VELOCITY_AT_HALF_POWER_CMS);
        motor1.setPower(-0.5);
        motor2.setPower(-0.5);
        motor3.setPower(-0.5);
        motor4.setPower(-0.5);
        telemetry.addData("time", time);
        
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
        }
        
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    
    // Creates a function to move the robot left a set distance
    public void ltDrive(double distance) {
        // determines the ideal time for any distance (cm)
        double time = (distance / VELOCITY_AT_HALF_POWER_CMS);
        motor1.setPower(-0.5);
        motor2.setPower(0.5);
        motor3.setPower(0.5);
        motor4.setPower(-0.5);
        telemetry.addData("time", time);
        
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
        }
        
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    
    // Creates a function to move the robot right a set distance
    public void rtDrive(double distance) {
        // determines the ideal time for any distance (cm)
        double time = (distance / VELOCITY_AT_HALF_POWER_CMS);
        motor1.setPower(0.5);
        motor2.setPower(-0.5);
        motor3.setPower(-0.5);
        motor4.setPower(0.5);
        telemetry.addData("time", time);
        
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
        }
        
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    
    public void ltTurn(double degree) {
        //determines ideal time for any degree
        double time = (degree/360 * 4.2);
        motor1.setPower(-0.5);
        motor2.setPower(0.5);
        motor3.setPower(-0.5);
        motor4.setPower(0.5);
        
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
        }
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    
    public void rtTurn(double degree) {
        //determines ideal time for any degree
        double time = (degree/360 * 4.2);
        motor1.setPower(0.5);
        motor2.setPower(-0.5);
        motor3.setPower(0.5);
        motor4.setPower(-0.5);
        
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
        }
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    
    
     public double extensionPos = 1;
    
    @Override
    public void runOpMode() {
        //initialise hubs
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        Expansion_Hub = hardwareMap.get(Blinker.class, "Expansion Hub");
        //initialise motors
        motor1 = hardwareMap.get(DcMotor.class, "motor_front_left");
        motor2 = hardwareMap.get(DcMotor.class, "motor_front_right");
        motor3 = hardwareMap.get(DcMotor.class, "motor_back_left");
        motor4 = hardwareMap.get(DcMotor.class, "motor_back_right");
        armlift = hardwareMap.get(DcMotor.class, "motor_liftArm");
        intake = hardwareMap.get(DcMotor.class, "motor_spinny");
        DDS = hardwareMap.get(DcMotor.class, "motor_DDS");
        //initialse servos
        armextend = hardwareMap.get(Servo.class, "Servo_ArmExtension");
        claw = hardwareMap.get(Servo.class, "ServoClaw");
        //initialise sensors
        colour = hardwareMap.get(ColorSensor.class, "rightColour");
        distance = hardwareMap.get(DistanceSensor.class, "rightColour");
        
        //initialise the motor direction
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);
        
        //set mode
        armlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialised");    
        telemetry.update();
        
        waitForStart();
        
        //lift arm to move robot
        //move extension to lift arm 
        armextend.setPosition(0.8);
        armlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armlift.setTargetPosition(172);
        armlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armlift.setPower(1);
        sleep(500);
        
        // Move forward and sense barcode
        fwdDrive(44.0);
        sleep(500);
        int j = 0;
        int level = 3;
        for (int i = 0; i < 3; i++) {
            if (distance.getDistance(DistanceUnit.CM) < 20) {
                level = 3 - i;
                telemetry.addData("i;", i);
                j = j +25;
            }
            sleep(1000);
            ltDrive(42);
            telemetry.addData("i;", i);
            telemetry.update();
            //error check - speed
            if (i == 3) {
                j = 99;
            }
            sleep(500);
        }
        
        //check level detected
        telemetry.addData("Level:", level);
        telemetry.addData("j:", j); 
        telemetry.update();
        
        // deposit preload box (default to bottom level)
        fwdDrive(60);
        
        // sets arm to height of first level
        if (level == 1) {
            armlift.setTargetPosition(172);
        }
        // sets arm to height of second level
        else if (level == 2) {
            armlift.setTargetPosition(342);
        }
        // sets arm to height of third level
        else if (level == 3) {
            armlift.setTargetPosition(494);
        }
        
        armlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armlift.setPower(1);
        
        intake.setPower(0.5);
        sleep(2000);
        intake.setPower(0);
        armlift.setTargetPosition(172);
        armlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //Move to and spin the turntable
        bwdDrive(100);
        ltTurn(180);
        bwdDrive(148);
        //sets power of the Duck Distribution System to calculated value for ideal velocity
        DDS.setPower(0.5);
        sleep(2000);
        DDS.setPower(0);
        
        // move into storage area 
        rtDrive(60);
        bwdDrive(5);
        
    }
}
