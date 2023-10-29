package org.firstinspires.ftc.compcode.CentreStage;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class IntakeOuttake {

    private Blinker Control_Hub;
    private Servo rightIntakeOuttake;
    private Servo leftIntakeOuttake;

    IntakeOuttake (Servo rightIntakeOuttake, Servo leftIntakeOuttake) {
        this.rightIntakeOuttake = rightIntakeOuttake;
        this.leftIntakeOuttake = leftIntakeOuttake;
    }  

    public void intakeAuto(){
        rightIntakeOuttake.setPosition(1);
        leftIntakeOuttake.setPosition(1);
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        rightIntakeOuttake.setPosition(0);
        leftIntakeOuttake.setPosition(0);
    }

    public void outakeLeftAuto(){
        leftIntakeOuttake.setPosition(-1);
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        leftIntakeOuttake.setPosition(0);
    }

    public void outakeRightAuto(){
        rightIntakeOuttake.setPosition(-1);
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        rightIntakeOuttake.setPosition(0);
    }
    
    public void outakeLeft(){
        leftIntakeOuttake.setPosition(0);
    }

    public void outakeRight(){
        rightIntakeOuttake.setPosition(1);
    }

    public void intake(){
        rightIntakeOuttake.setPosition(0);
        leftIntakeOuttake.setPosition(1);
    }

    public void stopIntakeOuttake(){
        rightIntakeOuttake.setPosition(0.7);
        leftIntakeOuttake.setPosition(0.6);
    }
}