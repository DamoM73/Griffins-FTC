package org.firstinspires.ftc.compcode.STEM_Night;
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

@TeleOp(name = "CHOOSE ME")
public class DriveClass extends OpMode {
    // Create objects for this robot
    public Blinker expansion_Hub_2;
    public BNO055IMU imu;
    
    public DcMotor motor_front_right;
    public DcMotor motor_back_right;
    public DcMotor motor_front_left;
    public DcMotor motor_back_left;
    public DcMotor lift_motor;
    public Motion driveTrain;

    public void addTelemetry(String name, double value) {
        // Adds data to telemetry on driver hub
        telemetry.addData(name,value);
    }

    public void updateTelemetry() {
        // Updates telemetry to display
        telemetry.update();
    }

    @Override
    public void init() {
        
        //SETUP
        //Initialises all the required variables and objects and initialises them
        //ready for the start();
        
        // initialise object for the dc motor
        motor_front_right = hardwareMap.get(DcMotor.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotor.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotor.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotor.class, "motor_back_left");

        lift_motor = hardwareMap.get(DcMotor.class, "lift_motor");
        
        // initialise the directions of the motors
        motor_front_right.setDirection(DcMotor.Direction.REVERSE);
        motor_front_left.setDirection(DcMotor.Direction.REVERSE);
        motor_back_right.setDirection(DcMotor.Direction.FORWARD);
        motor_back_left.setDirection(DcMotor.Direction.FORWARD);


        // initialise objects for expansion hub components
        //expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Control Hub");

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        
        // Intiialise drive chain
        driveTrain = new Motion(motor_front_right,motor_back_left,motor_front_left,motor_back_right,imu);
        
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
        telemetry.addData("Have Fun","STEM Enthusiasts");
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
        driveTrain.JoystickMoving(gamepad1.left_stick_x, gamepad1.right_stick_x,gamepad1.right_stick_y);
        
        if(gamepad1.a == true) {
                lift_motor.setTargetPosition(0);
                lift_motor.setPower(0.4);
            }
            
            if(gamepad1.b == true) {
                lift_motor.setTargetPosition(-38);
                lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_motor.setPower(-1);
            }
            
        telemetry.addData("Encoder Value", lift_motor.getCurrentPosition());
    }
}
