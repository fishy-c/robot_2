
package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public double appliedVolts = 0.0;
        public double setPoint = 0;
        public double elevatorPos = 0;
        public double drawnCurrent = 0;
        public double leftTemperature = 0;
        public double rightTemperature = 0;
    }


public default void updateInputs(ElevatorIOInputs inputs){}

public default void updateTunableNumbers(){}

public default void setHeight(double setPoint){}

public default void testOutput(){}

public default void setOutput(double output){}

public default void setElevator(){}

public default void homing(){}

public default void smartdashboard(){}

public default void elevatorConfiguration(){}
}
