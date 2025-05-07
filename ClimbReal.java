package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.TurboLogger;

public class ClimbReal extends Climb {
    private TalonFX motor;

    private MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withSlot(0);
    private DutyCycleOut percentControl = new DutyCycleOut(0);
    private VoltageOut voltageControl = new VoltageOut(0);

    public ClimbReal(int motorId) {
        motor = new TalonFX(motorId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = ClimbConstants.currentLimit.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = ClimbConstants.gearRatio;
        config.Slot0.kP = TurboLogger.get("Climb_kP", ClimbConstants.kPDefault);
        config.Slot0.kI = TurboLogger.get("Climb_kI", ClimbConstants.kIDefault);
        config.Slot0.kD = TurboLogger.get("Climb_kD", ClimbConstants.kDDefault);
        config.MotionMagic.MotionMagicAcceleration =
                ClimbConstants.maxAcceleration.in(RadiansPerSecondPerSecond);
        config.MotionMagic.MotionMagicCruiseVelocity =
                ClimbConstants.maxVelocity.in(RadiansPerSecond);

        motor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        Slot0Configs pidConfig = new Slot0Configs();
        if (TurboLogger.hasChanged("Climb_kP")) pidConfig.kP = TurboLogger.get("Climb_kP", ClimbConstants.kPDefault);
        if (TurboLogger.hasChanged("Climb_kI")) pidConfig.kI = TurboLogger.get("Climb_kI", ClimbConstants.kIDefault);
        if (TurboLogger.hasChanged("Climb_kD")) pidConfig.kD = TurboLogger.get("Climb_kD", ClimbConstants.kDDefault);
        if (pidConfig.serialize().equals(new Slot0Configs().serialize())) motor.getConfigurator().apply(pidConfig);

        TurboLogger.log("/Climb/Acceleration", getAcceleration().in(RadiansPerSecondPerSecond));
        TurboLogger.log("/Climb/Current", getCurrent().in(Amps));
        TurboLogger.log("/Climb/Percent/Actual", getPercent());
        TurboLogger.log("/Climb/Position/Actual", getPosition().in(Radians));
        TurboLogger.log("/Climb/Temperature", getTemperature().in(Celsius));
        TurboLogger.log("/Climb/Velocity", getVelocity().in(RadiansPerSecond));
        TurboLogger.log("/Climb/Voltage/Actual", getVoltage().in(Volts));
    }

    @Override
    public void setAngle(Angle angle) {
        TurboLogger.log("/Climb/Position/Setpoint", angle.in(Radians));

        // Clamping the input angle
        if (getPosition().gt(ClimbConstants.maxAngle)) angle = ClimbConstants.maxAngle;
        if (getPosition().lt(ClimbConstants.minAngle)) angle = ClimbConstants.minAngle;

        motor.setControl(positionControl.withPosition(angle));
    }

    @Override
    public void setPercent(double percent) {
        TurboLogger.log("/Climb/Percent/Setpoint", percent);

        // Stopping the motor if at the max or min angles.
        if (getPosition().gte(ClimbConstants.maxAngle)
                || getPosition().lte(ClimbConstants.minAngle)) percent = 0;

        motor.setControl(percentControl.withOutput(percent));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        TurboLogger.log("/Climb/Voltage/Setpoint", voltage.in(Volts));

        // Stopping the motor if at the max or min angles.
        if (getPosition().gte(ClimbConstants.maxAngle)
                || getPosition().lte(ClimbConstants.minAngle)) voltage = Volts.zero();

        motor.setControl(voltageControl.withOutput(voltage));
    }

    public AngularAcceleration getAcceleration() {
        return motor.getAcceleration().getValue();
    }

    public Current getCurrent() {
        return motor.getStatorCurrent().getValue();
    }

    public Angle getPosition() {
        return motor.getPosition().getValue();
    }

    public double getPercent() {
        return motor.get();
    }

    public Temperature getTemperature() {
        return motor.getDeviceTemp().getValue();
    }

    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }

    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }
}
