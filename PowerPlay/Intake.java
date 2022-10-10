package org.firstinspires.ftc.compcode.PowerPlay;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public Blinker expansion_Hub_2;

    private Servo intake_motor;

    public Boolean activated = true;
    
    Intake (Servo intake_motor) {
        this.intake_motor = intake_motor;
    }

    public Boolean hasCone(){
        return activated;
    }

    public void pickUpCone(){
        intake_motor.setPosition(-0.5);
        activated = true;
    }

    public void putDownCone(){
        intake_motor.setPosition(1.5);
        activated = false;
    }

    public void stopRuning() {
        intake_motor.setPosition(0.5);
    }
    public void pickUpConeAuto() {
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

