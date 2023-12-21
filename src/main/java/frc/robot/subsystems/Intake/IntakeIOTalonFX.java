package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IntakeIOTalonFX implements IntakeIO{
    private final TalonFX intake;
    private boolean intakeOutake;
    private VoltageOut intakeRequest;

    public IntakeIOTalonFX(int motorID){
        intake= new TalonFX(motorID);
        var intakeConfigs = new TalonFXConfiguration();
        intakeConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeRequest = new VoltageOut(0);

        intake.getConfigurator().apply(intakeConfigs);
    }

    public void updateInputs(IntakeIOInputs intakeInputs){
        intakeInputs.appliedVolts = intakeRequest.Output;
        intakeInputs.intakeOutake = this.intakeOutake;
    }

    public void setVoltage(double output){
        intake.setControl(intakeRequest.withOutput(output));
    }

    public void setIntakeOutake(boolean intakeOutake){
        this.intakeOutake = intakeOutake;
    }
    
}
