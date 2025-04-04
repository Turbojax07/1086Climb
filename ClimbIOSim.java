package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class ClimbIOSim implements ClimbIO {
    private DCMotorSim motor;
    private ClimbIOInputsAutoLogged inputs;

    public ClimbIOSim() {
        motor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getNEO(1),
                        ClimbConstants.moi.magnitude(),
                        ClimbConstants.gearRatio),
                DCMotor.getNEO(1));

        inputs = new ClimbIOInputsAutoLogged();
    }

    @Override
    public void updateInputs() {
        motor.update(0.02);

        inputs.angle = getAngle();
        inputs.velocity = getVelocity();
        inputs.acceleration = getAcceleration();

        inputs.voltage = getVoltage();
        inputs.current = getCurrent();
        inputs.temperature = getTemperature();

        Logger.processInputs("/RealOutputs/Subsystems/Climb/ClimbIOSim", inputs);
    }

    @Override
    public void setVolts(Voltage volts) {
        motor.setInputVoltage(volts.in(Volts));
    }

    @Override
    public Angle getAngle() {
        return motor.getAngularPosition();
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getAngularVelocity();
    }

    @Override
    public AngularAcceleration getAcceleration() {
        return motor.getAngularAcceleration();
    }

    @Override
    public Voltage getVoltage() {
        return Volts.of(motor.getInputVoltage());
    }

    @Override
    public Current getCurrent() {
        return Amps.of(motor.getCurrentDrawAmps());
    }

    @Override
    public Temperature getTemperature() {
        return Celsius.zero();
    }
}