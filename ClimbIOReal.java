package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
        config.Feedback.SensorToMechanismRatio = ClimbConstants.gearRatio;

        motor.getConfigurator().apply(config);

        inputs = new ClimbIOInputsAutoLogged();
    }

    @Override
    public void updateInputs() {
        // Stopping the motor if at the extremes.
        if (getAngle().gt(ClimbConstants.maxAngle) || getAngle().lt(ClimbConstants.minAngle)) {
            motor.setControl(new VoltageOut(Volts.zero()));
        }

        inputs.angle = getAngle();
        inputs.velocity = getVelocity();
        inputs.acceleration = getAcceleration();
        inputs.voltage = getVoltage();
        inputs.current = getCurrent();
        inputs.temperature = getTemperature();

        Logger.processInputs("/RealOutputs/Climb/ClimbIOReal", inputs);
    }

    @Override
    public void setVolts(Voltage volts) {
        // Stopping the motor if at the extremes.
        if (getAngle().gt(ClimbConstants.maxAngle) || getAngle().lt(ClimbConstants.minAngle)) {
            volts = Volts.zero();
        }

        motor.setControl(new VoltageOut(volts));
    }

    @Override
    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }

    @Override
    public AngularAcceleration getAcceleration() {
        return motor.getAcceleration().getValue();
    }

    @Override
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    @Override
    public Current getCurrent() {
        return motor.getStatorCurrent().getValue();
    }

    @Override
    public Temperature getTemperature() {
        return motor.getDeviceTemp().getValue();
    }
}