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
        private float wristPosition;
        private double armExtendModifier = 0.5;
        private double wristSpeedModifier = 0.05;
        private double wristPickupAngle = 0.5;
        private int liftPickupExtend = 0;
        private int armPickupAngle = -200;

        private float wristBaseAngle = 1;
        private int liftBaseExtend = 0;
        private int armBaseAngle = -50;

        private float wristCompactAngle = 0;
        private int liftCompactExtend = 0;
        private int armCompactAngle = 0;

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
                liftExtendMotor.setPower(speed*armExtendModifier*0.25);
            }
        }

        public void rotateArm(double speed) {
            if (-0.1 < speed && speed < 0.1) {
                liftRotateMotor.setPower(0);
            }
            else {
                liftRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftRotateMotor.setPower(speed*0.25);
            }
        }

        public void rotateWrist(double speed) {
        
            ///????? IDK, it is a servo which means you cnat use set power and stuff so i am seeing if this works, it gets the position and then modifys it so it can rotate or maybe it is a contin
            //wristServo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //wristServo.setPower(speed);

            wristServo.setPosition(wristServo.getPosition() + speed*wristSpeedModifier);
        }

        public void pickUpPosition() {
            /**Move to pick up Position */
            wristServo.setPosition(wristPickupAngle);
            liftExtendMotor.setTargetPosition(liftPickupExtend);
            liftRotateMotor.setTargetPosition(armPickupAngle);
            liftRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftExtendMotor.setPower(0.25);
            liftRotateMotor.setPower(0.25);
            try {
                Thread.sleep(1000);
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }

        public void moveToBasePosition(){
            /**Move to position to place on bottom position */
            wristServo.setPosition(wristBaseAngle);
            liftExtendMotor.setTargetPosition(liftBaseExtend);
            liftRotateMotor.setTargetPosition(armBaseAngle);
            liftRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftExtendMotor.setPower(0.25);
            liftRotateMotor.setPower(0.25);
            try {
                Thread.sleep(1000);
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }

        public void compact() {
            /**Move to compacted position */
            wristServo.setPosition(wristCompactAngle);
            liftExtendMotor.setTargetPosition(liftCompactExtend);
            liftRotateMotor.setTargetPosition(armCompactAngle);
            liftRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftExtendMotor.setPower(0.25);
            liftRotateMotor.setPower(0.25);
            try {
                Thread.sleep(1000);
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }
    }