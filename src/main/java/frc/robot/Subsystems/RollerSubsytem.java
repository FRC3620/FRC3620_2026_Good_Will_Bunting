package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsytem extends SubsystemBase{
    public RollerSubsytem(){
        rollers= new TalonFX(57);
    }
    TalonFX rollers;
    public void rollersOn(){
        rollers.set(0.5);

    }

    public Command rollersOff(){
        
                return runOnce(()->{
                    rollers.set(0);
                });
    }
    public void rollersBackwards(){
        rollers.set(-0.5);
    }
}
