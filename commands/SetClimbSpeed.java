package frc.robot.subsystems.climb.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class SetClimbSpeed extends Command {
    private Climb climb;
    private Supplier<Double> throttle;

    public SetClimbSpeed(Climb climb, Supplier<Double> throttle) {
        this.climb = climb;
        this.throttle = throttle;

        addRequirements(climb);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double speed = throttle.get();

        speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
        speed = Math.copySign(speed * speed, speed);

        climb.setVolts(Volts.of(speed * RobotController.getInputVoltage()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climb.setVolts(Volts.zero());
    }
}