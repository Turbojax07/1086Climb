package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AdjustableValues;

public class Climb extends SubsystemBase {
    private ClimbIO climbIO;
    private ProfiledPIDController controller;
    private boolean voltageControl = false;

    public Climb(ClimbIO climbIO) {
        this.climbIO = climbIO;

        controller = new ProfiledPIDController(
                AdjustableValues.getNumber("Climb_kP"),
                AdjustableValues.getNumber("Climb_kI"),
                AdjustableValues.getNumber("Climb_kD"),
                new TrapezoidProfile.Constraints(
                        ClimbConstants.maxVelocity.in(RadiansPerSecond),
                        ClimbConstants.maxAcceleration.in(RadiansPerSecondPerSecond)));
    }

    @Override
    public void periodic() {
        if (AdjustableValues.hasChanged("Climb_kP")) controller.setP(AdjustableValues.getNumber("Climb_kP"));
        if (AdjustableValues.hasChanged("Climb_kI")) controller.setI(AdjustableValues.getNumber("Climb_kI"));
        if (AdjustableValues.hasChanged("Climb_kD")) controller.setD(AdjustableValues.getNumber("Climb_kD"));

        // Only runs the system if setAngle() was the last control function used.
        if (!voltageControl) {
            Voltage pidOutput = Volts.of(controller.calculate(getAngle().in(Radians)));

            climbIO.setVolts(pidOutput);
        }

        Logger.recordOutput("/Climb/VoltageControl", voltageControl);

        climbIO.updateInputs();
    }

    public void setAngle(Angle angle) {
        voltageControl = false;

        Logger.recordOutput("/Climb/GoalAngle", angle);

        controller.setGoal(new TrapezoidProfile.State(angle.in(Radians), 0));
    }

    public Angle getAngle() {
        return climbIO.getAngle();
    }

    public AngularVelocity getVelocity() {
        return climbIO.getVelocity();
    }

    public AngularAcceleration getAcceleration() {
        return climbIO.getAcceleration();
    }

    public Voltage getVoltage() {
        return climbIO.getVoltage();
    }

    public Current getCurrent() {
        return climbIO.getCurrent();
    }

    public Temperature getTemperature() {
        return climbIO.getTemperature();
    }

    public void setVolts(Voltage volts) {
        voltageControl = true;

        Logger.recordOutput("/Climb/GoalVolts", volts);

        climbIO.setVolts(volts);
    }
}