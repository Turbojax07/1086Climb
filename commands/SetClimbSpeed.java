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
    private Supplier<Double> percentSupplier;

    /**
     * Creates a new {@link SetClimbSpeed} command.
     * It sets the voltage of the climb system.
     * 
     * @param climb The {@link Climb} system to control.
     * @param throttle The percent output to run at.
     * @param percentSupplier The max percent to run at.
     */
    public SetClimbSpeed(Climb climb, Supplier<Double> throttle, Supplier<Double> percentSupplier) {
        this.climb = climb;
        this.throttle = throttle;
        this.percentSupplier = percentSupplier;

        addRequirements(climb);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        double speed = throttle.get();

        speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
        speed = Math.copySign(speed * speed, speed);

        climb.setVolts(Volts.of(speed * percentSupplier.get() * RobotController.getInputVoltage()));
    }

    /** Returns true when the command should end. */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        climb.setVolts(Volts.zero());
    }
}