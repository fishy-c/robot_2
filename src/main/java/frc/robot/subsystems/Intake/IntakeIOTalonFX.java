package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IntakeIOTalonFX implements IntakeIO{
    private final TalonFX intake;
    private boolean intakeOutake;

    public IntakeIOTalonFX(int motorID){
        intake= new TalonFX(motorID);
        var intakeConfigs = new TalonFXConfiguration();
        intakeConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        intake.getConfigurator().apply(intakeConfigs);
    }

    public void updateInputs(IntakeIOInputs intakeInputs){
        intakeInputs.appliedVolts = 
            intake.getSupplyVoltage().getValue();
        intakeInputs.intakeOutake = this.intakeOutake;
    }

    public void setVoltage(double output){
        System.out.printf("*****RUNNING THE INTAKE****");
        var intakeRequest = new VoltageOut(0);
        intake.setControl(intakeRequest.withOutput(output));
    }

    public void setIntakeOutake(boolean intakeOutake){
        this.intakeOutake = intakeOutake;
    }
    
}
