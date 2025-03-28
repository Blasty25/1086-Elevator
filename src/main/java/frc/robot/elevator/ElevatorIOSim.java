// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

public class ElevatorIOSim implements ElevatorIO {

    private ElevatorSim elevator;
    private double volts = 0.0;

    public ElevatorIOSim() {
        this.elevator = new ElevatorSim(ElevatorConstants.KV, ElevatorConstants.KA, ElevatorConstants.gearbox,
                ElevatorConstants.minHeight,
                ElevatorConstants.maxHeight, ElevatorConstants.simGravity, ElevatorConstants.maxHeight);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevator.update(0.02);
        inputs.leftVolts = Volts.of(volts);
        inputs.currentHeight = elevator.getPositionMeters();
        inputs.velocity = elevator.getVelocityMetersPerSecond();

        inputs.leftCurrent = Amps.of(elevator.getCurrentDrawAmps());
    }

    @Override
    public void setVolts(double voltage) {
        this.volts = voltage;
        elevator.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    @Override
    public void resetEncoder() {
        elevator.setInputVoltage(0);
    }

}
