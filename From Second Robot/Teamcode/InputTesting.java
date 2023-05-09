
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name = "InputTesting")
public class InputTesting extends LinearOpMode {
    
    public Servo intake_motor;
    
    @Override
    public void runOpMode() {
        intake_motor = hardwareMap.get(Servo.class, "intake_servo");
        Intake intake = new Intake(intake_motor);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        intake.pickUpConeAuto();
        for (int i = 0;i<50;i++) {
            try {
                Thread.sleep(100);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }
        intake.putDownConeAuto();
    }
}
