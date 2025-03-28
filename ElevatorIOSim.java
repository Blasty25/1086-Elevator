// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim elevator;

    private Voltage volts = Volts.zero();

    public ElevatorIOSim() {
        this.elevator = new ElevatorSim(
            ElevatorConstants.kVDefault,
            ElevatorConstants.kADefaults[0],
            DCMotor.getKrakenX60(2), 0,
            ElevatorConstants.maxHeight.in(Meters),
            true, 0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevator.update(0.02);
        inputs.leftVolts = volts;
        inputs.currentHeight = Meters.of(elevator.getPositionMeters());
        inputs.velocity = MetersPerSecond.of(elevator.getVelocityMetersPerSecond());

        inputs.leftCurrent = Amps.of(elevator.getCurrentDrawAmps());

        inputs.rightVolts = inputs.leftVolts.unaryMinus();
        inputs.rightCurrent = inputs.leftCurrent.unaryMinus();
    }

    @Override
    public void setVolts(Voltage voltage) {
        this.volts = voltage;
        elevator.setInputVoltage(MathUtil.clamp(voltage.in(Volts), -12, 12));
    }

    @Override
    public void resetEncoder() {
        elevator.setInputVoltage(0);
    }
}