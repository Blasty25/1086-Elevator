// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AdjustableValues;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private Distance difference = Meters.zero();
    //KP is 110 ik but whatever!
    private ProfiledPIDController pid = new ProfiledPIDController(
            AdjustableValues.getNumber("ELEV_kP"),
            AdjustableValues.getNumber("ELEV_kI"),
            AdjustableValues.getNumber("ELEV_kD"),
            new TrapezoidProfile.Constraints(
                ElevatorConstants.maxVelocity.in(MetersPerSecond), 
                ElevatorConstants.maxAcceleration.in(MetersPerSecondPerSecond)));

    public Elevator(ElevatorIO io) {
        this.io = io;

        pid.setTolerance(0.001);
    }

    public void setPosition(Distance position) {
        pid.setGoal(new State(position.in(Meters), 0));
    }

    public Command resetEncoder() {
        return Commands.runOnce(() -> {
            io.resetEncoder();
        }, this).ignoringDisable(true);
    }

    public Command setGoal(Distance position) {
        return Commands.runOnce(() -> {
            setPosition(position);
        }, this);
    }

    @Override
    public void periodic() {
        // Updating PID values
        if (AdjustableValues.hasChanged("ELEV_kP")) pid.setP(AdjustableValues.getNumber("ELEV_kP"));
        if (AdjustableValues.hasChanged("ELEV_kI")) pid.setI(AdjustableValues.getNumber("ELEV_kI"));
        if (AdjustableValues.hasChanged("ELEV_kD")) pid.setD(AdjustableValues.getNumber("ELEV_kD"));

        // Logger.processInputs("Elevator", inputs);
        io.updateInputs(inputs);

        inputs.targetHeight = Meters.of(pid.getGoal().position);

        double pidOutput = pid.calculate(inputs.currentHeight.in(Meters));

        Logger.recordOutput("Elevator/PIDOutput", pidOutput);

        io.setVolts(Volts.of(pidOutput));
        
        difference = (inputs.targetHeight.minus(inputs.currentHeight));
        Logger.recordOutput("/Elevator/Difference", difference.in(Meters));
        Logger.recordOutput("Elevator/TargetHeight", inputs.targetHeight);
    }
}
