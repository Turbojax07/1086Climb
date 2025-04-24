
package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    protected Climb() {}

    public void setAngle(Angle angle) {}
    public void setPercent(double percent) {}
    public void setVoltage(Voltage voltage) {}

    public AngularAcceleration getAcceleration() {
        return RadiansPerSecondPerSecond.zero();
    }

    public Current getCurrent() {
        return Amps.zero();
    }

    public Angle getPosition() {
        return Radians.zero();
    }

    public double getPercent() {
        return 0;
    }

    public Temperature getTemperature() {
        return Celsius.zero();
    }

    public AngularVelocity getVelocity() {
        return RadiansPerSecond.zero();
    }

    public Voltage getVoltage() {
        return Volts.zero();
    }
}
