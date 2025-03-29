// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

/** Add your docs here. */
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

    public static final Angle extended = Degrees.of(260);
    public static final Angle stow = Degrees.of(0);
    public static final Angle tucked = Degrees.of(180);
}
