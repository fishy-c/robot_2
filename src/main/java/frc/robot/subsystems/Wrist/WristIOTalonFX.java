package frc.robot.subsystems.Wrist;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.lib.math.Conversions;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commons.LoggedTunableNumber;

import frc.robot.Constants;


public class WristIOTalonFX implements WristIO{
    private final TalonFX wrist;
    private final TalonFXConfigurator wristConfigurator;
    private final TalonFXConfiguration wristConfigs;
    MotionMagicVoltage motionMagicRequest;
    DutyCycleOut dutyCycleRequest;
    double startTime;
    double setPoint;

    LoggedTunableNumber kP = new LoggedTunableNumber("Wrist/kP", 0);
    LoggedTunableNumber kD = new LoggedTunableNumber("Wrist/kD", 0);
    LoggedTunableNumber kS = new LoggedTunableNumber("Wrist/kS", 0);
    LoggedTunableNumber kV = new LoggedTunableNumber("Wrist/kV", 0);
    LoggedTunableNumber kG = new LoggedTunableNumber("Wrist/kG", 0);
    LoggedTunableNumber kMotionCruiseVelocity = new LoggedTunableNumber( "Wrist/kMotionCruiseVelocity",3.0);
    LoggedTunableNumber kMotionAcceleration = new LoggedTunableNumber( "Wrist/kMotionAcceleration",3.0);
    LoggedTunableNumber kMotionJerk = new LoggedTunableNumber("Wrist/kMotionJerk",10000);

    public WristIOTalonFX(int motorID){
        wrist = new TalonFX(motorID);
        wristConfigurator = wrist.getConfigurator();
        wristConfigs = new TalonFXConfiguration();
        motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
        dutyCycleRequest = new DutyCycleOut(0);
        setPoint = Constants.Wrist.minRangeInDegrees; //TODO: change to max range after

    }

    public void updateInputs(WristIOInputs wristInputs){
        wristInputs.appliedVolts = wrist.getSupplyVoltage().getValue();
        wristInputs.setPoint = setPoint;
        wristInputs.wristPos = getWristPos();
    }

    public void updateTunableNumbers() {
        if (
          kD.hasChanged(kD.hashCode()) ||
          kG.hasChanged(kG.hashCode()) ||
          kS.hasChanged(kS.hashCode()) ||
          kP.hasChanged(kP.hashCode()) ||
          kMotionAcceleration.hasChanged(kMotionAcceleration.hashCode()) ||
          kMotionCruiseVelocity.hasChanged(kMotionCruiseVelocity.hashCode()) ||
          kV.hasChanged(kV.hashCode())
        ) {
          wristConfiguration();
        }
      }

    public void setSetPoint(double setPoint){
        this.setPoint = setPoint;
    }
    
    public void setOutput(double output){
        wrist.setControl(dutyCycleRequest.withOutput(output));
    }

    public void moveWrist(){
        wrist.setControl(motionMagicRequest.withPosition(Conversions.DegreesToRotations(setPoint, Constants.Wrist.gearRatio)));
    }

    
    public void homing(){
        startTime = Timer.getFPGATimestamp();
        wrist.setControl(dutyCycleRequest.withOutput(Constants.Wrist.homingOutput));
        if(((Timer.getFPGATimestamp()-startTime) < 2) && wrist.getRotorVelocity().getValue() < 0.1){
            wrist.setControl(dutyCycleRequest.withOutput(0));
            wristConfigs.Feedback.FeedbackRotorOffset = Constants.Wrist.rotorOffset1;
            wristConfigurator.apply(wristConfigs);
        }
    }

    public double getWristPos(){
        return Conversions.RotationsToDegrees(wrist.getRotorPosition().getValue(), Constants.Wrist.gearRatio);
    }

    public void wristConfiguration(){
        var motorOuputConfigs = wristConfigs.MotorOutput;
        motorOuputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOuputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        
        var slot0Configs = wristConfigs.Slot0;
        slot0Configs.kP = kP.get();
        slot0Configs.kI = 0.0;
        slot0Configs.kD = kD.get();
        slot0Configs.kS = kS.get();
        slot0Configs.kV = kV.get();
        slot0Configs.kG = kG.get();
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    
        var motionMagicConfigs = wristConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = kMotionCruiseVelocity.get();
        motionMagicConfigs.MotionMagicAcceleration = kMotionAcceleration.get();
        motionMagicConfigs.MotionMagicJerk = kMotionJerk.get();
        
        var feedbackConfigs = wristConfigs.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        feedbackConfigs.FeedbackRotorOffset = Constants.Wrist.minRangeInRotations; // TODO: set to 0 at bottom and find the angle at the top -> set the angle to maxrange

        wristConfigurator.apply(wristConfigs);
    }

}
