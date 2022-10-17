package org.firstinspires.ftc.compcode.PowerPlay;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    // Initialise Objects
    public Blinker expansion_Hub_2;
    private Servo intake_motor;

    // Initialise variables
    public Boolean activated = true;
    
    Intake (Servo intake_motor) {
        // Creator class to assign motor
        this.intake_motor = intake_motor;
    }

    public Boolean hasCone(){
        /**
        Returns whether the intake is holding a cone or is vacant
        Used for autonomous.
        **/
        return activated;
    }

    public void pickUpCone(){
        /**
        Starts the motor running to pick up a cone
        **/
        intake_motor.setPosition(-0.5);
        activated = true;
    }

    public void putDownCone(){
        /**
        Starts the motor running to drop a cone
        **/
        intake_motor.setPosition(1.5);
        activated = false;
    }

    public void stopRuning() {
        /**
        Stops the motor to maintain cone in intake, or to avoid running.
        **/
        intake_motor.setPosition(0.5);
    }
    public void pickUpConeAuto() {
        /**
        Starts the motor running to pick up a cone and stops when the cone is secured.
        **/
        intake_motor.setPosition(-0.5);
        for (int i = 0;i<15;i++) {
            try {
                Thread.sleep(100);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }
        
        intake_motor.setPosition(0.5);
    }

    public void putDownConeAuto() {
        /**
        Stops the motor running to drop a cone and stops when the cone is dropped.
        **/
        intake_motor.setPosition(1.5);
        for (int j = 0;j<10;j++) {
            try {
                Thread.sleep(100);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }
        
        intake_motor.setPosition(0.5);
    }
}

