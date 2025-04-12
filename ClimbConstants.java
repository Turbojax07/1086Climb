package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ClimbConstants {
    public static final double gearRatio = 338.33;

    public static final Current currentLimit = Amps.of(60);

    public static final double maxPercent = 1;

    public static final MomentOfInertia moi = KilogramSquareMeters.of(1);

    public static final double kPDefault = 41.75;
    public static final double kIDefault = 0;
    public static final double kDDefault = 0;

    public static final Angle minAngle = Radians.of(0);
    public static final Angle maxAngle = Radians.of(1.2);

    public static final AngularVelocity maxVelocity = RadiansPerSecond.of(Math.PI);
    public static final AngularAcceleration maxAcceleration = RadiansPerSecondPerSecond.of(Math.PI);

    public class ClimbPositions {
        public static final Angle STOW = Radians.of(0);
        public static final Angle GRAB = Radians.of(1.1152);
        public static final Angle HANG = Radians.of(2.359);
    }
}
