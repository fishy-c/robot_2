package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Wrist extends SubsystemBase{
    private final WristIO wristIO;
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    public Wrist(WristIO wristIO) {
        this.wristIO = wristIO;
      }

    public void setSetPoint(double setPoint){
        wristIO.setSetPoint(setPoint);
    }
    public void setOutput(double output){
        wristIO.setOutput(output);
    }

    public void moveWrist(){
        wristIO.moveWrist();
    }

    public void homing(){
        wristIO.homing();
    }
 
    public void wristConfiguration(){
        wristIO.wristConfiguration();
    }


    @Override
    public void periodic(){
        wristIO.updateInputs(inputs);
        wristIO.updateTunableNumbers();
        wristIO.smartdashboard();
        Logger.getInstance().processInputs("Wrist", inputs);
    }
}
