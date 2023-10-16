package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import Thread

public class IntakeOuttake {

    private Blinker Control_Hub;
    private Servo rightIntakeOuttake;
    private Servo leftIntakeOuttake;

    IntakeOuttake (Servo rightIntakeOuttake, Servo leftIntakeOuttake) {
        this.rightIntakeOuttake = rightIntakeOuttake;
        this.leftIntakeOuttake = leftIntakeOuttake;
    }  

    public void intake(){
        rightIntakeOuttake.setPosition(1);
        leftIntakeOuttake.setPosition(1);
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

    public void OutakeLeftAuto(){
        leftIntakeOuttake.setPosition(-1);
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        leftIntakeOuttake.setPosition(0);
    }

    public void OutakeRightAuto(){
        rightIntakeOuttake.setPosition(-1);
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        rightIntakeOuttake.setPosition(0);
    }

    public void OutakeLeft(){
        leftIntakeOuttake.setPosition(-1);
    }

    public void OutakeRight(){
        rightIntakeOuttake.setPosition(-1);
    }

    public void StopIntakeOuttake(){
        rightIntakeOuttake.setPosition(0);
        leftIntakeOuttake.setPosition(0);
    }
}