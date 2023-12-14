package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
//import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.commons.LoggedTunableNumber;

public class ElevatorIOTalonFX implements ElevatorIO{
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFXConfiguration leftMotorConfigs;
    private final TalonFXConfiguration rightMotorConfigs;
    private final TalonFXConfigurator leftMotorConfigurator;
    private final TalonFXConfigurator rightMotorConfigurator;
    private double startTime;
    private MotionMagicVoltage motionMagicRequest; 
    //private DutyCycleOut dutyCycleRequest;
    private VoltageOut voltageOutRequest;
    double setPoint;

    LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0);
    LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);
    LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 1.95);
    LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0);
    LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.01);
    LoggedTunableNumber kMotionCruiseVelocity = new LoggedTunableNumber( "Elevator/kMotionCruiseVelocity",3.0);
    LoggedTunableNumber kMotionAcceleration = new LoggedTunableNumber( "Elevator/kMotionAcceleration",3.0);
    LoggedTunableNumber kMotionJerk = new LoggedTunableNumber("Elevator/kMotionJerk",10000);
    LoggedTunableNumber elevatorVolts = new LoggedTunableNumber("Elevator/ElevatorVolts", 2);



    public ElevatorIOTalonFX(int leftMotorID, int rightMotorID){
        leftMotor = new TalonFX(leftMotorID);
        rightMotor = new TalonFX(rightMotorID);
        leftMotorConfigurator = leftMotor.getConfigurator();
        rightMotorConfigurator = rightMotor.getConfigurator();
        leftMotorConfigs = new TalonFXConfiguration();
        rightMotorConfigs = new TalonFXConfiguration();
        //dutyCycleRequest = new DutyCycleOut(0);
        voltageOutRequest = new VoltageOut(0);
        motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
        startTime = 0;
        setPoint = 0;  

    }
    
    public void updateInputs(ElevatorIOInputs elevatorInputs){
        elevatorInputs.appliedVolts = leftMotor.getSupplyVoltage().getValue();
        elevatorInputs.setPoint = setPoint;
        elevatorInputs.elevatorPos = getElevatorHeight();
        elevatorInputs.drawnCurrent = leftMotor.getSupplyCurrent().getValue();
        elevatorInputs.leftTemperature = leftMotor.getDeviceTemp().getValue();
        elevatorInputs.rightTemperature = rightMotor.getDeviceTemp().getValue();
        
    }

    public void updateTunableNumbers() {
        if (
          kD.hasChanged(kD.hashCode()) ||
          kG.hasChanged(kG.hashCode()) ||
          kS.hasChanged(kS.hashCode()) ||
          kP.hasChanged(kP.hashCode()) ||
          kMotionAcceleration.hasChanged(kMotionAcceleration.hashCode()) ||
          kMotionCruiseVelocity.hasChanged(kMotionCruiseVelocity.hashCode()) ||
          kV.hasChanged(kV.hashCode())||
          elevatorVolts.hasChanged(elevatorVolts.hashCode())
        ) {
          elevatorConfiguration();
        }
      }

    public void setHeight(double desiredHeight){
        setPoint = desiredHeight;
    }

    public void testOutput(){
        //leftMotor.setControl(dutyCycleRequest.withOutput(output));
        leftMotor.setControl(voltageOutRequest.withOutput(elevatorVolts.get()));
    }

    public void setOutput(double output){
        //leftMotor.setControl(dutyCycleRequest.withOutput(output));
        leftMotor.setControl(voltageOutRequest.withOutput(output));
    }

    public void setElevator(){
        //PositionVoltage positionRequest= new PositionVoltage(0);
        leftMotor.setControl(motionMagicRequest.withPosition(Conversions.metersToRotations(setPoint, Constants.Elevator.wheelCircumference, Constants.Elevator.gearRatio)).withFeedForward(kG.get())); 
        //leftMotor.setControl(positionRequest.withPosition(setPoint).withFeedForward(kG.get()));
    }
    
    /*public void homing(){
        startTime = Timer.getFPGATimestamp();
        leftMotor.setControl(dutyCycleRequest.withOutput(Constants.Elevator.homingOutput));
        if(((Timer.getFPGATimestamp()-startTime) < 2) && leftMotor.getRotorVelocity().getValue() < 0.1){
            leftMotor.setControl(dutyCycleRequest.withOutput(0));
            //leftMotorConfigs.Feedback.FeedbackRotorOffset = Constants.Elevator.minHeightInRotations;
            leftMotor.setRotorPosition(Constants.Elevator.minHeightInRotations);
            leftMotorConfigurator.apply(leftMotorConfigs);
        }
    }*/

    public double getElevatorHeight(){
        return Conversions.RotationsToMeters(leftMotor.getRotorPosition().getValue(), Constants.Elevator.wheelCircumference, Constants.Elevator.gearRatio);
    }

    public double getElevatorHeightRotations(){
        return leftMotor.getRotorPosition().getValue();
    }

    public void smartdashboard(){
        SmartDashboard.putNumber("Elevator Position", getElevatorHeight());
        SmartDashboard.putNumber("Elevator Position Rotor", getElevatorHeightRotations());
    }

    public void elevatorConfiguration(){
        var leftMotorOuputConfigs = leftMotorConfigs.MotorOutput;
        var rightMotorOutputConfigs = rightMotorConfigs.MotorOutput;
        leftMotorOuputConfigs.NeutralMode = NeutralModeValue.Brake;
        leftMotorOuputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        
        var slot0Configs = leftMotorConfigs.Slot0;
        slot0Configs.kP = kP.get();
        slot0Configs.kI = 0.0;
        slot0Configs.kD = kD.get();
        slot0Configs.kS = kS.get();
        slot0Configs.kV = kV.get();
        //slot0Configs.kG = kG.get();
        //slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        
        var motionMagicConfigs = leftMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = kMotionCruiseVelocity.get();
        motionMagicConfigs.MotionMagicAcceleration = kMotionAcceleration.get();
        motionMagicConfigs.MotionMagicJerk = kMotionJerk.get();

        var feedbackConfigs = leftMotorConfigs.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        //feedbackConfigs.FeedbackRotorOffset = Constants.Elevator.minHeightInRotations; //TODO: double check max height -> delete this
        leftMotor.setRotorPosition(Constants.Elevator.minHeightInRotations);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
        leftMotorConfigurator.apply(leftMotorConfigs);
        rightMotorConfigurator.apply(rightMotorConfigs);
        
    }    

}


    

