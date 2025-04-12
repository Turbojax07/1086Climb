package frc.robot.subsystems.climb;

public class ClimbConstants {
    public static final double gearRatio = 338.33;
    public static final double currentLimit = 60; // Amps
    public static final double moi = 1; // Kilograms^2 * Meters

    public static final double maxPercent = 1;

    public static final double kPDefault = 41.75;
    public static final double kIDefault = 0;
    public static final double kDDefault = 0;

    public static final double maxAngle = 1.2; // Radians
    public static final double minAngle = 0; // Radians

    public static final double maxVelocity = Math.PI; // Radians / Second
    public static final double maxAcceleration = Math.PI; // Radians / Second^2

    public class ClimbPositions {
        public static final double STOW = 0; // Radians
        public static final double GRAB = 1.1152; // Radians
        public static final double HANG = 2.359; // Radians
    }
}
