package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public class ClimbIOInputs {
        Angle position = Radians.zero();
        AngularVelocity velocity = RadiansPerSecond.zero();
        AngularAcceleration acceleration = RadiansPerSecondPerSecond.zero();

        Voltage voltage = Volts.zero();
        Current current = Amps.zero();
        Temperature temperature = Celsius.zero();
    }

    /** Updates a set of IO inputs with current values. */
    public void updateInputs(ClimbIOInputs inputs);

    /** Sets the angle of the climb motor. */
    public void setAngle(Angle angle);

    /** Sets the percent output of the climb motor. */
    public void setPercent(double percent);

    /** Sets the voltage output of the climb motor. */
    public void setVoltage(Voltage voltage);
}