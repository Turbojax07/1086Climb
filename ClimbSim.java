package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.AdjustableValues;

public class ClimbSim extends Climb {
    private DCMotorSim motor;

    private ProfiledPIDController controller;

    private boolean closedLoop = false;

    public ClimbSim() {
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
    public void periodic() {
        if (AdjustableValues.hasChanged("Climb_kP")) controller.setP(AdjustableValues.getNumber("Climb_kP"));
        if (AdjustableValues.hasChanged("Climb_kI")) controller.setI(AdjustableValues.getNumber("Climb_kI"));
        if (AdjustableValues.hasChanged("Climb_kD")) controller.setD(AdjustableValues.getNumber("Climb_kD"));

        if (closedLoop) motor.setInputVoltage(controller.calculate(motor.getAngularPositionRad()));

        motor.update(0.02);

        SmartDashboard.putNumber("/Climb/Acceleration", getAcceleration().in(RadiansPerSecondPerSecond));
        SmartDashboard.putNumber("/Climb/Current", getCurrent().in(Amps));
        SmartDashboard.putNumber("/Climb/Percent/Actual", getPercent());
        SmartDashboard.putNumber("/Climb/Position/Actual", getPosition().in(Radians));
        SmartDashboard.putNumber("/Climb/Temperature", getTemperature().in(Celsius));
        SmartDashboard.putNumber("/Climb/Velocity", getVelocity().in(RadiansPerSecond));
        SmartDashboard.putNumber("/Climb/Voltage/Actual", getVoltage().in(Volts));
    }

    @Override
    public void setAngle(Angle angle) {
        SmartDashboard.putNumber("/Climb/Position/Setpoint", angle.in(Radians));

        // Clamping the input angle
        if (getPosition().gt(ClimbConstants.maxAngle)) angle = ClimbConstants.maxAngle;
        if (getPosition().lt(ClimbConstants.minAngle)) angle = ClimbConstants.minAngle;


        closedLoop = true;

        controller.setGoal(angle.in(Radians));
    }

    @Override
    public void setPercent(double percent) {
        SmartDashboard.putNumber("/Climb/Percent/Setpoint", percent);

        // Stopping the motor if at the max or min angles.
        if (getPosition().gte(ClimbConstants.maxAngle) || getPosition().lte(ClimbConstants.minAngle)) percent = 0;

        closedLoop = false;
        
        motor.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        SmartDashboard.putNumber("/Climb/Voltage/Setpoint", voltage.in(Volts));

        // Stopping the motor if at the max or min angles.
        if (getPosition().gte(ClimbConstants.maxAngle) || getPosition().lte(ClimbConstants.minAngle)) voltage = Volts.zero();

        closedLoop = false;

        motor.setInputVoltage(voltage.in(Volts));
    }

    public AngularAcceleration getAcceleration() {
        return motor.getAngularAcceleration();
    }

    public Current getCurrent() {
        return Amps.of(motor.getCurrentDrawAmps());
    }
    
    public Angle getPosition() {
        return motor.getAngularPosition();
    }
    
    public double getPercent() {
        return motor.getInputVoltage() / RobotController.getInputVoltage();
    }
    
    public AngularVelocity getVelocity() {
        return motor.getAngularVelocity();
    }
    
    public Voltage getVoltage() {
        return Volts.of(motor.getInputVoltage());
    }
}