package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
                        ClimbConstants.moi,
                        ClimbConstants.gearRatio),
                DCMotor.getNEO(1));

        controller = new ProfiledPIDController(
            AdjustableValues.getNumber("Climb_kP"),
            AdjustableValues.getNumber("Climb_kI"),
            AdjustableValues.getNumber("Climb_kD"),
            new TrapezoidProfile.Constraints(
                ClimbConstants.maxVelocity,
                ClimbConstants.maxAcceleration));
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        if (AdjustableValues.hasChanged("Climb_kP")) controller.setP(AdjustableValues.getNumber("Climb_kP"));
        if (AdjustableValues.hasChanged("Climb_kI")) controller.setI(AdjustableValues.getNumber("Climb_kI"));
        if (AdjustableValues.hasChanged("Climb_kD")) controller.setD(AdjustableValues.getNumber("Climb_kD"));

        if (closedLoop) motor.setInputVoltage(controller.calculate(motor.getAngularPositionRad()));

        motor.update(0.02);

        inputs.acceleration = motor.getAngularAccelerationRadPerSecSq();
        inputs.current = motor.getCurrentDrawAmps();
        inputs.percent = motor.getInputVoltage() / RobotController.getInputVoltage();
        inputs.position = motor.getAngularPositionRad();
        inputs.velocity = motor.getAngularVelocityRadPerSec();
        inputs.voltage = motor.getInputVoltage();
    }

    @Override
    public void setAngle(double angle) {
        closedLoop = true;
        
        controller.setGoal(angle);
    }

    @Override
    public void setPercent(double percent) {
        closedLoop = false;

        motor.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setVoltage(double voltage) {
        closedLoop = false;

        motor.setInputVoltage(voltage);
    }
}