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
        Angle angle = Radians.zero();
        AngularVelocity velocity = RadiansPerSecond.zero();
        AngularAcceleration acceleration = RadiansPerSecondPerSecond.zero();

        Voltage voltage = Volts.zero();
        Current current = Amps.zero();
        Temperature temperature = Celsius.zero();
    }

    /** Updates a set of IO inputs with current values. */
    public void updateInputs();

    /** Sets the voltage output of the climb motor. */
    public void setVolts(Voltage volts);

    /** Gets the angle of the module as an {@link Angle}. */
    public Angle getAngle();

    /** Gets the angular velocity of the module as an {@link AngularVelocity}. */
    public AngularVelocity getVelocity();

    /** Gets the angular acceleration of the module as an {@link AngularAcceleration}. */
    public AngularAcceleration getAcceleration();

    /** Gets the output volts as a {@link Voltage}. */
    public Voltage getVoltage();

    /** Gets the output current as a {@link Current}. */
    public Current getCurrent();

    /** Gets the temperature as a {@link Temperature}. */
    public Temperature getTemperature();
}