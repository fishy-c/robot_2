package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase{
    private final IntakeIO intakeIO;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
      }
    
    public void intakeSpin(double volts){
        intakeIO.setVoltage(volts);
    }

    public void setIntakeOutake(boolean intakeOutake){
        intakeIO.setIntakeOutake(intakeOutake);
    }

    @Override
    public void periodic(){
        intakeIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);
    }
}
