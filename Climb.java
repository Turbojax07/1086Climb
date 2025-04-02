package frc.robot.subsystems.climb;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private ClimbIO climbIO;

    public Climb(ClimbIO climbIO) {
        this.climbIO = climbIO;
    }

    @Override
    public void periodic() {
        climbIO.updateInputs();
    }

    public void setAngle(Angle angle) {
        climbIO.setAngle(angle);
    }

    public Angle getAngle() {
        return climbIO.getAngle();
    }

    public AngularVelocity getVelocity() {
        return climbIO.getVelocity();
    }

    public AngularAcceleration getAcceleration() {
        return climbIO.getAcceleration();
    }

    public Voltage getVoltage() {
        return climbIO.getVoltage();
    }

    public Current getCurrent() {
        return climbIO.getCurrent();
    }

    public Temperature getTemperature() {
        return climbIO.getTemperature();
    }

    public void setVolts(Voltage volts) {
        climbIO.setVolts(volts);
    }
}