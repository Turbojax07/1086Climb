package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class ClimbCommands {
    public static Command setAngle(Climb climb, Angle angle) {
        return Commands.runOnce(() -> {
            climb.setAngle(angle);
        }, climb);
    }

    public static Command setPercent(Climb climb, Supplier<Double> throttle) {
        return Commands.runEnd(() -> {
            double speed = throttle.get();

            speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
            speed = Math.copySign(speed * speed, speed);

            climb.setPercent(speed * AdjustableValues.getNumber("Climb_Percent"));
        }, () -> {
            climb.setPercent(0);
        }, climb);
    }

    public static Command setVoltage(Climb climb, Supplier<Double> throttle) {
        return Commands.runEnd(() -> {
            double speed = throttle.get();

            speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
            speed = Math.copySign(speed * speed, speed);

            climb.setVoltage(Volts.of(speed * AdjustableValues.getNumber("Climb_Percent") * RobotController.getInputVoltage()));
        }, () -> {
            climb.setVoltage(Volts.zero());
        }, climb);
    }
}