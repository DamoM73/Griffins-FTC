package org.firstinspires.ftc.compcode.PowerPlay;
// Imports
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    // Create objects
    public Blinker expansion_Hub_2;

    private Servo intake_motor;

    public Boolean activated = true;
    
    Intake (Servo intake_motor) {
        // Create intake
        this.intake_motor = intake_motor;
    }

    public Boolean hasCone(){
        return activated;
    }

    public void pickUpCone(){
        // Pick Up Cone with motor
        intake_motor.setPosition(-0.5);
        activated = true;
    }

    public void putDownCone(){
        // Put down cone with motor
        intake_motor.setPosition(1.5);
        activated = false;
    }

    public void stopRuning() {
        // Stops the servo from turning 
        intake_motor.setPosition(0.5);
    }
    public void pickUpConeAuto() {
        // Start servo motor
        intake_motor.setPosition(-0.5);
        // Wait for cone to be picked up
        for (int i = 0;i<15;i++) {
            try {
                Thread.sleep(100);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }
        // Stop motor
        intake_motor.setPosition(0.5);
    }

    public void putDownConeAuto() {
        // Start motor
        intake_motor.setPosition(1.5);
        // Wait for cone to be put down
        for (int j = 0;j<30;j++) {
            try {
                Thread.sleep(100);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }
        // Stop motor
        intake_motor.setPosition(0.5);
    }
}

