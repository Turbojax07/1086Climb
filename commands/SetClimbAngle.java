package frc.robot.subsystems.climb.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class SetClimbAngle extends Command {
    private Climb climb;
    private Angle angle;

    /**
     * Creates a new {@link SetClimbAngle} command.
     * It adjusts the setpoint for the climb system.
     * 
     * @param climb The {@link Climb} subsystem to control.
     * @param angle The {@link Angle} to go to.
     */
    public SetClimbAngle(Climb climb, Angle angle) {
        this.climb = climb;
        this.angle = angle;

        addRequirements(climb);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        climb.setAngle(angle);
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {}

    /** Returns true when the command should end. */
    @Override
    public boolean isFinished() {
        return true;
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {}
}