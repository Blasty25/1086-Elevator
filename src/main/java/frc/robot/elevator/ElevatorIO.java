// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs{
        public Distance targetHeight = Meters.zero();
        public double currentHeight = 0.0;

        public Voltage leftVolts = Volts.zero();
        public Voltage rightVolts = Volts.zero();

        public Current leftCurrent = Amps.zero();
        public Current rightCurrent = Amps.zero();

        public Temperature leftMotor = Fahrenheit.zero();
        public Temperature rightMotor = Fahrenheit.zero();
        
        public double velocity = 0.0;

        public double setpoint = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void resetEncoder() {}

}
