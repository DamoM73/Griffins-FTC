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
    // Create objects for this robot
    public Blinker expansion_Hub_2;

    public DcMotor left_lift_motor;
    public DcMotor right_lift_motor;

    public Servo intake_motor;
    /**
    private ColorSensor colour;
    private DistanceSensor distance;
    **/
    Lift lift;
    Intake intake;

    @Override
    public void init() {
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Control Hub");

        left_lift_motor = hardwareMap.get(DcMotorEx.class, "left_lift_motor");
        right_lift_motor = hardwareMap.get(DcMotorEx.class, "right_lift_motor");
        intake_motor = hardwareMap.get(Servo.class, "intake_servo");
        

        Lift lift = new Lift(left_lift_motor,right_lift_motor);
        Intake intake = new Intake(intake_motor);
        

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
        // Lift
        lift.SetMoveSpeed(gamepad2.left_stick_x);
        if (gamepad2.dpad_up) {
            lift.MoveInPositionList(1);
        } else if (gamepad2.dpad_down) {
            lift.MoveInPositionList(-1);
        }

        telemetry.addData("LiftPosition",lift.currentPosition);

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
