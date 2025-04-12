package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.AdjustableValues;

public class ClimbIOSim implements ClimbIO {
    private DCMotorSim motor;

    private ProfiledPIDController controller;

    private boolean closedLoop = false;

    public ClimbIOSim() {
        motor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getNEO(1),
                        ClimbConstants.moi.magnitude(),
                        ClimbConstants.gearRatio),
                DCMotor.getNEO(1));

        controller = new ProfiledPIDController(
            AdjustableValues.getNumber("Climb_kP"),
            AdjustableValues.getNumber("Climb_kI"),
            AdjustableValues.getNumber("Climb_kD"),
            new TrapezoidProfile.Constraints(
                ClimbConstants.maxVelocity.in(RadiansPerSecond),
                ClimbConstants.maxAcceleration.in(RadiansPerSecondPerSecond)));
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        if (AdjustableValues.hasChanged("Climb_kP")) controller.setP(AdjustableValues.getNumber("Climb_kP"));
        if (AdjustableValues.hasChanged("Climb_kI")) controller.setI(AdjustableValues.getNumber("Climb_kI"));
        if (AdjustableValues.hasChanged("Climb_kD")) controller.setD(AdjustableValues.getNumber("Climb_kD"));

        if (closedLoop) motor.setInputVoltage(controller.calculate(motor.getAngularPositionRad()));

        motor.update(0.02);

        inputs.acceleration = motor.getAngularAcceleration();
        inputs.current = Amps.of(motor.getCurrentDrawAmps());
        inputs.percent = motor.getInputVoltage() / RobotController.getInputVoltage();
        inputs.position = motor.getAngularPosition();
        inputs.velocity = motor.getAngularVelocity();
        inputs.voltage = Volts.of(motor.getInputVoltage());
    }

    @Override
    public void setAngle(Angle angle) {
        closedLoop = true;
        
        controller.setGoal(angle.in(Radians));
    }

    @Override
    public void setPercent(double percent) {
        closedLoop = false;

        motor.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        closedLoop = false;

        motor.setInputVoltage(voltage.in(Volts));
    }
}