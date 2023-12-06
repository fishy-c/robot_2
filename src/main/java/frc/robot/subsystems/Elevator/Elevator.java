package frc.robot.subsystems.Elevator;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
      }

      public void setHeight(double setPoint){
        elevatorIO.setHeight(setPoint);
    }

    public void setOutput(double output){
        elevatorIO.setOutput(output);
    }
    
    public void setElevator(){
        elevatorIO.setElevator();
    }

    public void homing(){
        elevatorIO.homing();
    }
 
    public void elevatorConfiguration(){
        elevatorIO.elevatorConfiguration();
    }
    
    @Override
    public void periodic(){
        elevatorIO.updateInputs(inputs);
        elevatorIO.updateTunableNumbers(); 
        elevatorIO.smartdashboard();
        Logger.getInstance().processInputs("Elevator", inputs);
    }
}
