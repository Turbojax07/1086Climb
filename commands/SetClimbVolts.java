package frc.robot.subsystems.climb.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class SetClimbVolts extends Command {
    private Climb climb;
    private Voltage volts;

    public SetClimbVolts(Climb climb, Voltage volts) {
        this.climb = climb;
        this.volts = volts;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setVolts(volts);
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
        climb.setVolts(Volts.zero());
    }
}