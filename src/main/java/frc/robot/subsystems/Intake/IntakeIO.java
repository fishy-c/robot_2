package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {
    public static class IntakeIOInputs implements LoggableInputs{
   
        public double appliedVolts = 0.0;
        public boolean intakeOutake = true; //intake = true; outake= false

        public void toLog(LogTable table) {
            table.put("AppliedVolts", appliedVolts);
            table.put("IntakeOutake", intakeOutake);
        }

        public void fromLog(LogTable table){
            appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
            intakeOutake = table.getBoolean("IntakeOutake", intakeOutake);
        }
    }


public default void updateInputs(IntakeIOInputs inputs){}

public default void setVoltage(double volts){}

public default void setIntakeOutake(boolean intake){}

    
}
