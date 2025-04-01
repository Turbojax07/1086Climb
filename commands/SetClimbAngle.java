package frc.robot.subsystems.climb.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class SetClimbAngle extends Command {
    private Climb climb;
    private Angle angle;

    public SetClimbAngle(Climb climb, Angle angle) {
        this.climb = climb;
        this.angle = angle;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setAngle(angle);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}