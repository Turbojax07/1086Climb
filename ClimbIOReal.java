package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.AdjustableValues;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimbIOReal implements ClimbIO {
    private SparkMax motor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController controller;

    private ClimbIOInputsAutoLogged inputs;

    public ClimbIOReal(int motorId) {
        motor = new SparkMax(motorId, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        config.smartCurrentLimit((int) Constants.ClimbConstants.currentLimit.in(Amps));
        config.closedLoop.p(AdjustableValues.getNumber("Climb_kP"));
        config.closedLoop.i(AdjustableValues.getNumber("Climb_kI"));
        config.closedLoop.d(AdjustableValues.getNumber("Climb_kD"));
        config.encoder.positionConversionFactor(Constants.ClimbConstants.positionConversionFactor);
        config.encoder.velocityConversionFactor(Constants.ClimbConstants.velocityConversionFactor);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        inputs = new ClimbIOInputsAutoLogged();
    }

    public void updateInputs() {
        if (AdjustableValues.hasChanged("Climb_kP")) {
            SparkMaxConfig pidConfig = new SparkMaxConfig();
            pidConfig.closedLoop.p(AdjustableValues.getNumber("Climb_kP"));

            motor.configure(pidConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        if (AdjustableValues.hasChanged("Climb_kI")) {
            SparkMaxConfig pidConfig = new SparkMaxConfig();
            pidConfig.closedLoop.i(AdjustableValues.getNumber("Climb_kI"));

            motor.configure(pidConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        if (AdjustableValues.hasChanged("Climb_kD")) {
            SparkMaxConfig pidConfig = new SparkMaxConfig();
            pidConfig.closedLoop.d(AdjustableValues.getNumber("Climb_kD"));

            motor.configure(pidConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
        controller.setReference(angle.in(Radians), ControlType.kPosition);
    }

    public Angle getAngle() {
        return Radians.of(encoder.getPosition());
    }

    public AngularVelocity getVelocity() {
        return RadiansPerSecond.of(encoder.getVelocity());
    }

    public AngularAcceleration getAcceleration() {
        return RadiansPerSecondPerSecond.of(encoder.getVelocity() / 0.02);
    }

    public Voltage getVoltage() {
        return Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    }

    public Current getCurrent() {
        return Amps.of(motor.getOutputCurrent());
    }

    public Temperature getTemperature() {
        return Celsius.of(motor.getMotorTemperature());
    }

}