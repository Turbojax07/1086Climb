package frc.robot.subsystems.climb.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class SetClimbVoltage extends Command {
    private Climb climb;
    private Supplier<Double> throttle;

    /**
     * Creates a new {@link SetClimbVoltage} command.
     * It sets the voltage of the climb system.
     * 
     * @param climb The {@link Climb} system to control.
     * @param throttle The percent output to run at.
     */
    public SetClimbVoltage(Climb climb, Supplier<Double> throttle) {
        this.climb = climb;
        this.throttle = throttle;

        addRequirements(climb);
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        double speed = throttle.get();

        speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
        speed = Math.copySign(speed * speed, speed);

        climb.setVoltage(Volts.of(speed * AdjustableValues.getNumber("Climb_Percent") * RobotController.getInputVoltage()));
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        climb.setVoltage(Volts.zero());
    }
}