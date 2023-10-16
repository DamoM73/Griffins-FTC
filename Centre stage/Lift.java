package org.firstinspires.ftc.compcode.CentreStage;
// Imports
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;

// Class for control of vertical lift
public class Lift {
    // Create variables
    public Blinker expansion_Hub_2;

    private DcMotor liftRotateMotor;
    private DcMotor liftExtendMotor;
    private Servo wristServo;

    Lift (DcMotor liftRotateMotor, DcMotor liftExtendMotor, Servo wristServo) {
        // Create lift object with all powers
        this.liftRotateMotor = liftRotateMotor;
        this.liftExtendMotor = liftExtendMotor;
        this.wristServo = wristServo;

        liftRotateMotor.setDirection(DcMotor.Direction.REVERSE);
        liftRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftExtendMotor.setDirection(DcMotor.Direction.REVERSE);
        liftExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    public void extendArm(double speed){
        // Positive for up, negative for down
        if (-0.1 < speed && speed < 0.1) {
            liftExtendMotor.setPower(0);
        }
        else {
            liftExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftExtendMotor.setPower(speed*0.5);
        }
    }

    public void rotateArm(double speed) {
        if (-0.1 < speed && speed < 0.1) {
            liftRotateMotor.setPower(0);
        }
        else {
            liftRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRotateMotor.setPower(speed);
        }
    }

    public void rotateWrist(double speed) {
        if (-0.1 < speed && speed < 0.1) {
            wristServo.setPower(0);
        }
        else {
            wristServo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wristServo.setPower(speed);
        }
    }

    public void pickUpPosition() {
        /**Move to pick up Position */
        wristServo.setPosition(350);
        liftExtendMotor.setTargetPosition(0);
        liftRotateMotor.setTargetPosition(-200);
        liftRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotor.setPower(1);
        liftRotateMotor.setPower(1);
        try {
            Thread.sleep(1000);
        }
        catch(InterruptedException ex){
            ex.printStackTrace();
        }
    }

    public void moveToBasePosition(){
        /**Move to position to place on bottom position */
        wristServo.setPosition(100);
        liftExtendMotor.setTargetPosition(0);
        liftRotateMotor.setTargetPosition(-50);
        liftRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotor.setPower(1);
        liftRotateMotor.setPower(1);
        try {
            Thread.sleep(1000);
        }
        catch(InterruptedException ex){
            ex.printStackTrace();
        }
    }

    public void compact() {
        /**Move to compacted position */
        wristServo.setPosition(360);
        liftExtendMotor.setTargetPosition(0);
        liftRotateMotor.setTargetPosition(0);
        liftRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotor.setPower(1);
        liftRotateMotor.setPower(1);
        try {
            Thread.sleep(1000);
        }
        catch(InterruptedException ex){
            ex.printStackTrace();
        }
    }
}