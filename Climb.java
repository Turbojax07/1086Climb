package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    private ClimbIO climbIO;

    public ClimbIOInputsAutoLogged inputs;

    public Climb(ClimbIO climbIO) {
        this.climbIO = climbIO;

        inputs = new ClimbIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        climbIO.updateInputs(inputs);
    }

    /**
     * Sets the angle setpoint of the climb system.
     * 
     * @param angle The angle to go to in radians.
     */
    public void setAngle(double angle) {
        Logger.recordOutput("/Climb/GoalAngle", angle);

        // Clamping the input angle
        if (inputs.position > ClimbConstants.maxAngle) angle = ClimbConstants.maxAngle;
        if (inputs.position < ClimbConstants.minAngle) angle = ClimbConstants.minAngle;

        climbIO.setAngle(angle);
    }

    /**
     * Sets the percent output to run the climb system at.
     * 
     * @param percent The percent output to run at.
     */
    public void setPercent(double percent) {
        Logger.recordOutput("/Climb/GoalPercent", percent);

        // Stopping the motor if at the max or min angles.
        if (inputs.position >= ClimbConstants.maxAngle || inputs.position <= ClimbConstants.minAngle) percent = 0;
        
        climbIO.setPercent(percent);
    }

    /**
     * Sets the voltage output to run the climb system at.
     * 
     * @param voltage The voltage output to run at.
     */
    public void setVoltage(double voltage) {
        Logger.recordOutput("/Climb/GoalVoltage", voltage);

        // Stopping the motor if at the max or min angles.
        if (inputs.position >= ClimbConstants.maxAngle || inputs.position <= ClimbConstants.minAngle) voltage = 0;

        climbIO.setVoltage(voltage);
    }
}