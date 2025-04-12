package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public class ClimbIOInputs {
        double acceleration = 0; // Radians / Second^2
        double current = 0; // Amps
        double position = 0; // Radians
        double percent = 0;
        double temperature = 0; // Celsius
        double velocity = 0; // Radians / Second
        double voltage = 0; // Volts
    }

    /** Updates a set of IO inputs with current values. */
    public void updateInputs(ClimbIOInputs inputs);

    /** 
     * Sets the angle of the climb motor.
     * 
     * @param angle The angle in radians.
     */
    public void setAngle(double angle);

    /** Sets the percent output of the climb motor. */
    public void setPercent(double percent);

    /** Sets the voltage output of the climb motor. */
    public void setVoltage(double voltage);
}