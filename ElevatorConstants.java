package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static final double mass = Units.lbsToKilograms(25.1); // Kilograms
    public static final double radius = 0.02864789; // Meters
    public static final double maxHeight = Units.inchesToMeters(69); // Meters
    public static final double gearRatio = 9;

    public static final double maxPercent = 0.5;

    public static final double maxVelocity = 1.8; // Meters / Second
    public static final double maxAcceleration = 6.5; // Meters / Second^2

    public static final double metersPerRotation = 2.0 * Math.PI * radius; // Meters / Rotation

    public static final double currentLimit = 60; // Amps

    public static final double kPDefault = 110;
    public static final double kIDefault = 0;
    public static final double kDDefault = 0;
    public static final double kSDefault = 0.7;
    public static final double kGDefault = 0.3;
    public static final double kVDefault = 3.0;
    public static final double kADefault = 0.43;

    public static final double maxProfileVoltage = 6.0; // Volts

    public static final double sysIdMinPosition = 0.1; // Meters
    public static final double sysIdMaxPosition = 1.5; // Meters

    public static final double sysIdRampUp = 2.5;
    public static final double sysIdStep = 5.5;
    public static final double sysIdTimeout = 20.0;

    public class ElevatorPositions {
        public static final double STOW = 0;       // Meters
        public static final double INTAKE = 0.057; // Meters
        public static final double L1 = 0.33;      // Meters
        public static final double L2 = 0.63;      // Meters
        public static final double L2Algae = 0;    // Meters
        public static final double L3 = 1.05;      // Meters
        public static final double L3Algae = 0.81; // Meters
        public static final double L4 = 1.76;      // Meters
    }
}