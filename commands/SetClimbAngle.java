package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class SetClimbAngle extends Command {
    private Climb climb;
    private double angle;

    /**
     * Creates a new {@link SetClimbAngle} command.
     * It adjusts the setpoint for the climb system.
     * 
     * @param climb The {@link Climb} subsystem to control.
     * @param angle The angle to go to in radians.
     */
    public SetClimbAngle(Climb climb, double angle) {
        this.climb = climb;
        this.angle = angle;

        addRequirements(climb);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        climb.setAngle(angle);
        cancel();
    }
}