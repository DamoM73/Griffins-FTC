package org.firstinspires.ftc.compcode.PowerPlay;

// Imports
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


// Class for Driver Controlled
@TeleOp(name = "Driver Controlled")
public class DriverControlled extends OpMode {
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
    
    // Initialise interfaces with other modules
    Motion drivetrain;
    Lift lift;
    Intake intake;

    @Override
    public void init() {
        // Create expansion hub
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Control Hub");
        
        motor_front_right = hardwareMap.get(DcMotorEx.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotorEx.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotorEx.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotorEx.class, "motor_back_left");
        liftRotateMotor = hardwareMap.get(DcMotorEx.class, "lift_rotate_motor");
        liftExtendMotor = hardwareMap.get(DcMotorEx.class, "right_extend_motor");
        wristServo = hardwareMap.get(Servo.class, "intake_servo");
        leftIntakeServo = hardwareMap.get(Servo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(Servo.class, "right_intake_servo");

        // Create module references
        lift = new Lift(liftRotateMotor, liftExtendMotor, wristServo);
        intake = new Intake(leftIntakeServo, rightIntakeServo);
        drivetrain = new Motion(motor_front_right,motor_back_left,motor_front_left,motor_back_right,imu);
        

        // set up telemetry to disply on driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // Fun message to drivers
        telemetry.addData("Have Fun","Drivers");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Main loop
        // Movement
        // Standard Mechannum
        drivetrain.JoystickMoving(gamepad1.left_stick_x, gamepad1.right_stick_x,gamepad1.right_stick_y);
        
        // Move arm
        lift.rotateArm(gamepad2.r_stick_y);
        lift.extendArm(gamepad2.left_trigger-gamepad2.right_trigger);

        // wrist
        if (gamepad2.right_bumper) {
            lift.rotateWrist(0.4);
        }
        else if (gamepad2.left_bumper) {
            lift.rotateWrist(-0.4);
        } else {
            lift.rotateWrist(0);
        }

        if (gamepad2.x){
            intake.outakeLeft()
        }
        else if (gamepad2.b){
            intake.outakeRight()
        }
        else if (gamepad2.a) {
            intake.intake()
        } 
        else {
            intake.stopIntakeOuttake()
        }
        
        telemetry.update();
    }
}
