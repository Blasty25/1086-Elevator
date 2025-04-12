package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public class ElevatorConstants {
    public static final Mass mass = Pounds.of(25.1);
    public static final Distance radius = Meters.of(0.02864789);
    public static final Distance maxHeight = Inches.of(69);
    public static final double gearRatio = 9;

    public static final double maxPercent = 0.5;

    public static final LinearVelocity maxVelocity = MetersPerSecond.of(1.8);
    public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(6.5);

    public static final double positionConversionFactor = 2.0 * Math.PI * radius.in(Meters) / gearRatio;
    public static final double velocityConversionFactor = positionConversionFactor / 60.0;

    public static final Current currentLimit = Amps.of(60);

    public static final double kPDefault = 110;
    public static final double kIDefault = 0;
    public static final double kDDefault = 0;
    public static final double kSDefault = 0.7;
    public static final double kGDefault = 0.3;
    public static final double kVDefault = 3.0;
    public static final double kADefault = 0.43;

    public static final double maxProfileVoltage = 6.0;

    public static final Distance sysIdMinPosition = Meters.of(0.1);
    public static final Distance sysIdMaxPosition = Meters.of(1.5);

    public static final double sysIdRampUp = 2.5;
    public static final double sysIdStep = 5.5;
    public static final double sysIdTimeout = 20.0;

    public class ElevatorPositions {
        public static final Distance STOW = Meters.of(0);
        public static final Distance INTAKE = Meters.of(0.057);
        public static final Distance L1 = Meters.of(0.33);
        public static final Distance L2 = Meters.of(0.63);
        public static final Distance L2Algae = Meters.zero();
        public static final Distance L3 = Meters.of(1.05);
        public static final Distance L3Algae = Meters.of(0.81);
        public static final Distance L4 = Meters.of(1.76);
    }
}