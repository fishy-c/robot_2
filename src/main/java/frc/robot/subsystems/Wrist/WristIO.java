package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs{
        public double appliedVolts = 0.0;
        public double setPoint = 0.0;
        public double wristPos = 0.0;
    }


public default void updateInputs(WristIOInputs inputs){}

public default void updateTunableNumbers(){}

public default void setSetPoint(double setPoint){}

public default void setOutput(double output){}

public default void moveWrist(){}

public default void homing(){}

public default void smartdashboard(){}

public default void wristConfiguration(){}

    
}
