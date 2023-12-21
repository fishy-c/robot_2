package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.commons.LoggedTunableNumber;
import frc.lib.math.Conversions;

public class ModuleIOTalonFX implements ModuleIO{
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder angleEncoder;
    private final Rotation2d CANcoderOffset;
    private final InvertedValue driveInvert;
    private final InvertedValue steerInvert;
    private final SensorDirectionValue CANcoderInvert;
    private TalonFXConfiguration driveConfigs;
    private TalonFXConfiguration steerConfigs;
    private CANcoderConfiguration angleEncoderConfigs;
    private CANcoderConfigurator angleEncoderConfigurator;
    private TalonFXConfigurator driveConfigurator;
    private TalonFXConfigurator steerConfigurator;
    private DutyCycleOut driveRequest;
    private PositionDutyCycle steerRequest;
    private VelocityVoltage velocityVoltageRequest;

    LoggedTunableNumber drivekP = new LoggedTunableNumber("Drive/kP", Constants.Swerve.drivekP);
    LoggedTunableNumber drivekD = new LoggedTunableNumber("Drive/kD", 0);
    LoggedTunableNumber drivekS = new LoggedTunableNumber("Drive/kS", 0);
    LoggedTunableNumber drivekV = new LoggedTunableNumber("Drive/kV", 0);

    LoggedTunableNumber steerkP = new LoggedTunableNumber("Steer/kP", Constants.Swerve.anglekP);
    LoggedTunableNumber steerkD = new LoggedTunableNumber("Steer/kD", 0);
    LoggedTunableNumber steerkS = new LoggedTunableNumber("Steer/kS", 0);
    LoggedTunableNumber steerkV = new LoggedTunableNumber("Steer/kV", 0);


    public ModuleIOTalonFX(int driveID, int steerID, int CANcoderID, Rotation2d CANcoderOffset, InvertedValue driveInvert, InvertedValue steerInvert, SensorDirectionValue CANcoderInvert, String swerveModuleName){
        driveMotor = new TalonFX(driveID);
        steerMotor = new TalonFX(steerID);
        angleEncoder = new CANcoder(CANcoderID);
        this.CANcoderOffset = CANcoderOffset;
        this.driveInvert = driveInvert;
        this.steerInvert = steerInvert;
        this.CANcoderInvert = CANcoderInvert;
        driveConfigs = new TalonFXConfiguration();
        steerConfigs = new TalonFXConfiguration();
        angleEncoderConfigs = new CANcoderConfiguration();
        driveConfigurator = driveMotor.getConfigurator();
        steerConfigurator = steerMotor.getConfigurator();
        angleEncoderConfigurator = angleEncoder.getConfigurator();
        driveRequest = new DutyCycleOut(0);
        steerRequest = new PositionDutyCycle(0);
        velocityVoltageRequest = new VelocityVoltage(0);


        var driveMotorOutputConfigs = driveConfigs.MotorOutput;
        driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        driveMotorOutputConfigs.Inverted = driveInvert; 

        var driveFeedbackConfigs = driveConfigs.Feedback;
        driveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveMotor.setRotorPosition(0);

        var driveCurrentLimitConfigs = driveConfigs.CurrentLimits;
        driveCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        driveCurrentLimitConfigs.StatorCurrentLimit = 120;

        var driveSlot0Configs = driveConfigs.Slot0;
        driveSlot0Configs.kP = drivekP.get();
        driveSlot0Configs.kI = 0.0;
        driveSlot0Configs.kD = drivekD.get();
        driveSlot0Configs.kS = drivekS.get();
        driveSlot0Configs.kV = drivekV.get();

        //STEER

        var steerMotorOutputConfigs = steerConfigs.MotorOutput;
        steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        steerMotorOutputConfigs.Inverted = steerInvert;

        var steerFeedbackConfigs = steerConfigs.Feedback;
        steerFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        var steerSlot0Configs = steerConfigs.Slot0;
        steerSlot0Configs.kP = steerkP.get();
        steerSlot0Configs.kI = 0.0;
        steerSlot0Configs.kD = steerkD.get();
        steerSlot0Configs.kS = steerkS.get();
        steerSlot0Configs.kV = steerkV.get();

        var steerCurrentLimitConfigs = steerConfigs.CurrentLimits;
        steerCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        steerCurrentLimitConfigs.StatorCurrentLimit = 50;

        // CANcoder
        var magnetSensorConfigs = angleEncoderConfigs.MagnetSensor;
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.MagnetOffset = 0;
        magnetSensorConfigs.SensorDirection = CANcoderInvert;
        
        
        driveConfigurator.apply(driveConfigs);
        steerConfigurator.apply(steerConfigs);
        angleEncoderConfigurator.apply(angleEncoderConfigs);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs){
        inputs.driveVelocityMetersPerSec = Conversions.RPMtoMPS(driveMotor.getRotorVelocity().getValue(), Constants.Swerve.wheelCircumferenceInMeters, Constants.Swerve.driveGearRatio);
        inputs.driveAppliedVolts = driveMotor.getDutyCycle().getValue() * driveMotor.getSupplyVoltage().getValue();
        inputs.driveCurrentAmps = driveMotor.getStatorCurrent().getValue();
        inputs.driveTempCelcius = driveMotor.getDeviceTemp().getValue();
        inputs.driveDistanceMeters = Conversions.RotationsToMeters(driveMotor.getRotorPosition().getValue(), Constants.Swerve.wheelCircumferenceInMeters, Constants.Swerve.driveGearRatio);
        inputs.driveOutputPercent = driveMotor.get();
        inputs.rawDriveRPM = driveMotor.getRotorVelocity().getValue();

        inputs.moduleAngleRads = Units.degreesToRadians(Conversions.RotationsToDegrees(steerMotor.getPosition().getValue(), Constants.Swerve.angleGearRatio));
        inputs.moduleAngleDegs = Conversions.RotationsToDegrees(steerMotor.getPosition().getValue(), Constants.Swerve.angleGearRatio);
        inputs.rawAbsolutePositionRotations = angleEncoder.getAbsolutePosition().getValue();
        inputs.absolutePositionRadians = angleEncoder.getAbsolutePosition().getValue() * 2 * Math.PI;
        inputs.absolutePositionDegrees = angleEncoder.getAbsolutePosition().getValue() * 360;
        inputs.turnAppliedVolts = steerMotor.getDutyCycle().getValue() * steerMotor.getSupplyVoltage().getValue();
        inputs.turnCurrentAmps = steerMotor.getStatorCurrent().getValue();
        inputs.turnTempCelcius = steerMotor.getDeviceTemp().getValue();
    }

    @Override
    public void updateTunableNumbers(){
        if (
        drivekD.hasChanged(drivekD.hashCode()) ||
        drivekS.hasChanged(drivekS.hashCode()) ||
        drivekP.hasChanged(drivekP.hashCode()) ||
        drivekV.hasChanged(drivekV.hashCode())
        ) {
        var driveSlot0Configs = new Slot0Configs();
        driveSlot0Configs.kP = drivekP.get();
        driveSlot0Configs.kI = 0.0;
        driveSlot0Configs.kD = drivekD.get();
        driveSlot0Configs.kS = drivekS.get();
        driveSlot0Configs.kV = drivekV.get();

        driveConfigurator.apply(driveSlot0Configs);
        }

        if (
        steerkD.hasChanged(steerkD.hashCode()) ||
        steerkS.hasChanged(steerkS.hashCode()) ||
        steerkP.hasChanged(steerkP.hashCode()) ||
        steerkV.hasChanged(steerkV.hashCode())
        ){
        var steerSlot0Configs = new Slot0Configs();
        steerSlot0Configs.kP = steerkP.get();
        steerSlot0Configs.kI = 0.0;
        steerSlot0Configs.kD = steerkD.get();
        steerSlot0Configs.kS = steerkS.get();
        steerSlot0Configs.kV = steerkV.get();

        steerConfigurator.apply(steerSlot0Configs);
        }
    }

    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState optimizedDesiredStates = SwerveModuleState.optimize(desiredState, getAngle());
        double drivePercentOut = optimizedDesiredStates.speedMetersPerSecond / (Constants.Swerve.maxSpeed);

        double angleDeg = optimizedDesiredStates.angle.getDegrees();
        setDrivePercent(drivePercentOut);
        setTurnAngle(angleDeg);
    }

    public void setDriveVelocity(double velocityMetersPerSecond, boolean auto) {
        velocityVoltageRequest.Velocity = Conversions.MPStoRPM(velocityMetersPerSecond, Constants.Swerve.wheelCircumferenceInMeters, Constants.Swerve.driveGearRatio);
        driveMotor.setControl(velocityVoltageRequest);
    }

    @Override
    public void setDrivePercent(double percent) {
        driveRequest.Output = percent;
        driveMotor.setControl(driveRequest);
    }

    @Override
    public void setTurnAngle(double angleDeg) {
        steerRequest.Position = Conversions.DegreesToRotations(angleDeg, Constants.Swerve.angleGearRatio);
        steerMotor.setControl(steerRequest.withSlot(0));
    }

    @Override
    public void resetToAbsolute() {
        double absolutePositionDegrees_1 = Units.radiansToDegrees((angleEncoder.getAbsolutePosition().getValue() * 2 * Math.PI) - CANcoderOffset.getRadians());
        double absolutePositionRotations_1 = Conversions.DegreesToRotations(absolutePositionDegrees_1, Constants.Swerve.angleGearRatio);
        steerMotor.setRotorPosition(absolutePositionRotations_1);

    }

    @Override
    public void setDriveBrakeMode(boolean enable){
        var driveMotorOutputConfigs = new MotorOutputConfigs();
        if (enable){
            
            driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
            driveMotorOutputConfigs.Inverted = driveInvert;
        }
        else{
            driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
            driveMotorOutputConfigs.Inverted = driveInvert;
        }
        driveConfigurator.apply(driveMotorOutputConfigs);
    }

    @Override
    public void setTurnBrakeMode(boolean enable){
        var steerMotorOutputConfigs = new MotorOutputConfigs();
        if (enable){
            steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
            steerMotorOutputConfigs.Inverted = steerInvert;
        }
        else{
            steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
            steerMotorOutputConfigs.Inverted = steerInvert;
        }
        steerConfigurator.apply(steerMotorOutputConfigs);
    }

    public Rotation2d getAngle(){
        return new Rotation2d(Units.degreesToRadians(Conversions.RotationsToDegrees(steerMotor.getPosition().getValue(), Constants.Swerve.angleGearRatio)));
    }

}
