// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AdjustableValues;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private ProfiledPIDController pid = new ProfiledPIDController(
            AdjustableValues.getNumber("Elev_kP"),
            AdjustableValues.getNumber("Elev_kI"),
            AdjustableValues.getNumber("Elev_kD"),
            new TrapezoidProfile.Constraints(
                ElevatorConstants.maxVelocity.in(MetersPerSecond),
                ElevatorConstants.maxAcceleration.in(MetersPerSecondPerSecond)));

    private ElevatorFeedforward l1FeedForward = new ElevatorFeedforward(
            AdjustableValues.getNumber("Elev_kS_L1"),
            AdjustableValues.getNumber("Elev_kG_L1"),
            AdjustableValues.getNumber("Elev_kV"),
            AdjustableValues.getNumber("Elev_kA_L1"));

    private ElevatorFeedforward l2FeedForward = new ElevatorFeedforward(
            AdjustableValues.getNumber("Elev_kS_L2"),
            AdjustableValues.getNumber("Elev_kG_L2"),
            AdjustableValues.getNumber("Elev_kV"),
            AdjustableValues.getNumber("Elev_kA_L2"));

    private ElevatorFeedforward l3FeedForward = new ElevatorFeedforward(
            AdjustableValues.getNumber("Elev_kS_L3"),
            AdjustableValues.getNumber("Elev_kG_L3"),
            AdjustableValues.getNumber("Elev_kV"),
            AdjustableValues.getNumber("Elev_kA_L3"));

    private boolean voltageControl;

    public Elevator(ElevatorIO io) {
        this.io = io;

        pid.setTolerance(0.001);
    }

    public void setPosition(Distance position) {
        voltageControl = false;

        pid.setGoal(new State(position.in(Meters), 0));
    }

    public void setVolts(Voltage volts) {
        voltageControl = true;

        io.setVolts(volts);
    }

    public Command resetEncoder() {
        return Commands.runOnce(() -> {
            io.resetEncoder();
        }, this).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        // Updating PID values
        if (AdjustableValues.hasChanged("Elev_kP")) pid.setP(AdjustableValues.getNumber("Elev_kP"));
        if (AdjustableValues.hasChanged("Elev_kI")) pid.setI(AdjustableValues.getNumber("Elev_kI"));
        if (AdjustableValues.hasChanged("Elev_kD")) pid.setD(AdjustableValues.getNumber("Elev_kD"));

        if (AdjustableValues.hasChanged("ELEV_kS_L1")) l1FeedForward.setKs(AdjustableValues.getNumber("ELEV_kS_L1"));
        if (AdjustableValues.hasChanged("ELEV_kG_L1")) l1FeedForward.setKg(AdjustableValues.getNumber("ELEV_kG_L1"));
        if (AdjustableValues.hasChanged("ELEV_kV"))    l1FeedForward.setKv(AdjustableValues.getNumber("ELEV_kV"));
        if (AdjustableValues.hasChanged("ELEV_kA_L1")) l1FeedForward.setKa(AdjustableValues.getNumber("ELEV_kA_L1"));

        if (AdjustableValues.hasChanged("ELEV_kS_L2")) l2FeedForward.setKs(AdjustableValues.getNumber("ELEV_kS_L2"));
        if (AdjustableValues.hasChanged("ELEV_kG_L2")) l2FeedForward.setKg(AdjustableValues.getNumber("ELEV_kG_L2"));
        if (AdjustableValues.hasChanged("ELEV_kV"))    l2FeedForward.setKv(AdjustableValues.getNumber("ELEV_kV"));
        if (AdjustableValues.hasChanged("ELEV_kA_L2")) l2FeedForward.setKa(AdjustableValues.getNumber("ELEV_kA_L2"));

        if (AdjustableValues.hasChanged("ELEV_kS_L3")) l3FeedForward.setKs(AdjustableValues.getNumber("ELEV_kS_L3"));
        if (AdjustableValues.hasChanged("ELEV_kG_L3")) l3FeedForward.setKg(AdjustableValues.getNumber("ELEV_kG_L3"));
        if (AdjustableValues.hasChanged("ELEV_kV"))    l3FeedForward.setKv(AdjustableValues.getNumber("ELEV_kV"));
        if (AdjustableValues.hasChanged("ELEV_kA_L3")) l3FeedForward.setKa(AdjustableValues.getNumber("ELEV_kA_L3"));

        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // Skipping the PID and FeedForward calculations if voltage control is enabled.
        if (voltageControl) return;

        inputs.targetHeight = Meters.of(pid.getGoal().position);

        Logger.recordOutput("Elevator/setpointv", pid.getSetpoint().velocity);

        double ffVolts = 0;
        if (inputs.currentHeight.in(Meters) < 0.33) {
            ffVolts = l1FeedForward.calculateWithVelocities(inputs.velocity.in(MetersPerSecond), pid.getSetpoint().velocity);
        }

	    if (inputs.currentHeight.in(Meters) < 0.65) {
            ffVolts = l2FeedForward.calculateWithVelocities(inputs.velocity.in(MetersPerSecond), pid.getSetpoint().velocity);
        }

        if (inputs.currentHeight.in(Meters) > 0.65) {
            ffVolts = l3FeedForward.calculateWithVelocities(inputs.velocity.in(MetersPerSecond), pid.getSetpoint().velocity);
        }

        double pidOutput = pid.calculate(inputs.currentHeight.in(Meters));

        Logger.recordOutput("Elevator/PIDOutput", pidOutput);
        Logger.recordOutput("Elevator/FFVolts", ffVolts);

        io.setVolts(Volts.of(pidOutput + ffVolts));

        Distance difference = inputs.targetHeight.minus(inputs.currentHeight);
        Logger.recordOutput("Elevator/Difference", difference.in(Meters));
        Logger.recordOutput("Elevator/TargetHeight", inputs.targetHeight);
    }
}