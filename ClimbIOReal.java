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
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.AdjustableValues;

public class ClimbIOReal implements ClimbIO {
    private TalonFX motor;

    private MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withSlot(0);
    private DutyCycleOut percentControl = new DutyCycleOut(0);
    private VoltageOut voltageControl = new VoltageOut(0);

    public ClimbIOReal(int motorId) {
        motor = new TalonFX(motorId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = ClimbConstants.currentLimit.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = ClimbConstants.gearRatio;
        config.Slot0.kP = AdjustableValues.getNumber("Climb_kP");
        config.Slot0.kI = AdjustableValues.getNumber("Climb_kI");
        config.Slot0.kD = AdjustableValues.getNumber("Climb_kD");
        config.MotionMagic.MotionMagicAcceleration = ClimbConstants.maxAcceleration.in(RadiansPerSecondPerSecond);
        config.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.maxVelocity.in(RadiansPerSecond);

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        Slot0Configs pidConfig = new Slot0Configs();
        if (AdjustableValues.hasChanged("Climb_kP")) pidConfig.kP = AdjustableValues.getNumber("Climb_kP");
        if (AdjustableValues.hasChanged("Climb_kI")) pidConfig.kI = AdjustableValues.getNumber("Climb_kI");
        if (AdjustableValues.hasChanged("Climb_kD")) pidConfig.kD = AdjustableValues.getNumber("Climb_kD");
        if (pidConfig.serialize().equals(new Slot0Configs().serialize())) motor.getConfigurator().apply(pidConfig);

        inputs.acceleration = motor.getAcceleration().getValue();
        inputs.current = motor.getStatorCurrent().getValue();
        inputs.percent = motor.getDutyCycle().getValue();
        inputs.position = motor.getPosition().getValue();
        inputs.temperature = motor.getDeviceTemp().getValue();
        inputs.velocity = motor.getVelocity().getValue();
        inputs.voltage = motor.getMotorVoltage().getValue();
    }

    @Override
    public void setAngle(Angle angle) {
        motor.setControl(positionControl.withPosition(angle));
    }

    @Override
    public void setPercent(double percent) {
        motor.setControl(percentControl.withOutput(percent));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setControl(voltageControl.withOutput(voltage));
    }
}