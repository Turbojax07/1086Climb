package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.AdjustableValues;
import frc.robot.Constants.ClimbConstants;

import org.littletonrobotics.junction.Logger;

public class ClimbIOReal implements ClimbIO {
    private TalonFX motor;

    private ClimbIOInputsAutoLogged inputs;

    public ClimbIOReal(int motorId) {
        motor = new TalonFX(motorId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = ClimbConstants.currentLimit.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Slot0.kP = AdjustableValues.getNumber("Climb_kP");
        config.Slot0.kI = AdjustableValues.getNumber("Climb_kI");
        config.Slot0.kD = AdjustableValues.getNumber("Climb_kD");
        config.Feedback.SensorToMechanismRatio = ClimbConstants.gearRatio;

        motor.getConfigurator().apply(config);

        inputs = new ClimbIOInputsAutoLogged();
    }

    public void updateInputs() {
        if (AdjustableValues.hasChanged("Climb_kP")) {
            Slot0Configs config = new Slot0Configs();
            config.kP = AdjustableValues.getNumber("Climb_kP");

            motor.getConfigurator().apply(config);
        }

        if (AdjustableValues.hasChanged("Climb_kI")) {
            Slot0Configs config = new Slot0Configs();
            config.kI = AdjustableValues.getNumber("Climb_kI");

            motor.getConfigurator().apply(config);
        }

        if (AdjustableValues.hasChanged("Climb_kD")) {
            Slot0Configs config = new Slot0Configs();
            config.kD = AdjustableValues.getNumber("Climb_kD");

            motor.getConfigurator().apply(config);
        }

        inputs.angle = getAngle();
        inputs.velocity = getVelocity();
        inputs.acceleration = getAcceleration();
        inputs.voltage = getVoltage();
        inputs.current = getCurrent();
        inputs.temperature = getTemperature();

        Logger.processInputs("/RealOutputs/Subsystems/Climb/ClimbIOReal", inputs);
    }

    public void setAngle(Angle angle) {
        motor.setControl(new PositionVoltage(angle));
    }

    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }

    public AngularAcceleration getAcceleration() {
        return motor.getAcceleration().getValue();
    }

    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    public Current getCurrent() {
        return motor.getStatorCurrent().getValue();
    }

    public Temperature getTemperature() {
        return motor.getDeviceTemp().getValue();
    }
}