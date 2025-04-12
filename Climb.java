package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
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

    public void setAngle(Angle angle) {
        Logger.recordOutput("/Climb/GoalAngle", angle);

        // Clamping the input angle
        if (inputs.position.gt(ClimbConstants.maxAngle)) angle = ClimbConstants.maxAngle;
        if (inputs.position.lt(ClimbConstants.minAngle)) angle = ClimbConstants.minAngle;

        climbIO.setAngle(angle);
    }

    public void setPercent(double percent) {
        Logger.recordOutput("/Climb/GoalPercent", percent);

        // Stopping the motor if at the max or min angles.
        if (inputs.position.gte(ClimbConstants.maxAngle) || inputs.position.lte(ClimbConstants.minAngle)) percent = 0;
        
        climbIO.setPercent(percent);
    }

    public void setVoltage(Voltage voltage) {
        Logger.recordOutput("/Climb/GoalVoltage", voltage);

        // Stopping the motor if at the max or min angles.
        if (inputs.position.gte(ClimbConstants.maxAngle) || inputs.position.lte(ClimbConstants.minAngle)) voltage = Volts.zero();

        climbIO.setVoltage(voltage);
    }
}