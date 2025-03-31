package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ClimbConstants {
    public static final double gearRatio = 338.33;

    public static final LinearVelocity maxVelocity = MetersPerSecond.of(1.8);
    public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(6.5);

    public static final double positionConversionFactor = 2.0 * Math.PI / gearRatio;
    public static final double velocityConversionFactor = positionConversionFactor / 60.0;

    public static final Current currentLimit = Amps.of(60);

    public static final MomentOfInertia moi = KilogramSquareMeters.of(1);

    public static final double kPDefault = 41.75;
    public static final double kIDefault = 0;
    public static final double kDDefault = 0;

    public static final Angle extended = Radians.of(1.1152);
    public static final Angle stow = Radians.of(0);
    public static final Angle tucked = Radians.of(2.359);
}
