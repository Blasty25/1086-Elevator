package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AdjustableValues;
import org.littletonrobotics.junction.Logger;

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
            AdjustableValues.getNumber("Elev_kV_L1"),
            AdjustableValues.getNumber("Elev_kA_L1"));

    private ElevatorFeedforward l2FeedForward = new ElevatorFeedforward(
            AdjustableValues.getNumber("Elev_kS_L2"),
            AdjustableValues.getNumber("Elev_kG_L2"),
            AdjustableValues.getNumber("Elev_kV_L2"),
            AdjustableValues.getNumber("Elev_kA_L2"));

    private ElevatorFeedforward l3FeedForward = new ElevatorFeedforward(
            AdjustableValues.getNumber("Elev_kS_L3"),
            AdjustableValues.getNumber("Elev_kG_L3"),
            AdjustableValues.getNumber("Elev_kV_L3"),
            AdjustableValues.getNumber("Elev_kA_L3"));

    private boolean voltageControl;

    public Elevator(ElevatorIO io) {
        this.io = io;

        pid.setTolerance(0.001);
    }

    @Override
    public void periodic() {
        // Updating PID values
        if (AdjustableValues.hasChanged("Elev_kP")) pid.setP(AdjustableValues.getNumber("Elev_kP"));
        if (AdjustableValues.hasChanged("Elev_kI")) pid.setI(AdjustableValues.getNumber("Elev_kI"));
        if (AdjustableValues.hasChanged("Elev_kD")) pid.setD(AdjustableValues.getNumber("Elev_kD"));

        if (AdjustableValues.hasChanged("ELEV_L1_kS")) l1FeedForward.setKs(AdjustableValues.getNumber("ELEV_L1_kS"));
        if (AdjustableValues.hasChanged("ELEV_L1_kG")) l1FeedForward.setKg(AdjustableValues.getNumber("ELEV_L1_kG"));
        if (AdjustableValues.hasChanged("ELEV_L1_kV")) l1FeedForward.setKv(AdjustableValues.getNumber("ELEV_L1_kV"));
        if (AdjustableValues.hasChanged("ELEV_L1_kA")) l1FeedForward.setKa(AdjustableValues.getNumber("ELEV_L1_kA"));

        if (AdjustableValues.hasChanged("ELEV_L2_kS")) l2FeedForward.setKs(AdjustableValues.getNumber("ELEV_L2_kS"));
        if (AdjustableValues.hasChanged("ELEV_L2_kG")) l2FeedForward.setKg(AdjustableValues.getNumber("ELEV_L2_kG"));
        if (AdjustableValues.hasChanged("ELEV_L2_kV")) l2FeedForward.setKv(AdjustableValues.getNumber("ELEV_L2_kV"));
        if (AdjustableValues.hasChanged("ELEV_L2_kA")) l2FeedForward.setKa(AdjustableValues.getNumber("ELEV_L2_kA"));

        if (AdjustableValues.hasChanged("ELEV_L3_kS")) l3FeedForward.setKs(AdjustableValues.getNumber("ELEV_L3_kS"));
        if (AdjustableValues.hasChanged("ELEV_L3_kG")) l3FeedForward.setKg(AdjustableValues.getNumber("ELEV_L3_kG"));
        if (AdjustableValues.hasChanged("ELEV_L3_kV")) l3FeedForward.setKv(AdjustableValues.getNumber("ELEV_L3_kV"));
        if (AdjustableValues.hasChanged("ELEV_L3_kA")) l3FeedForward.setKa(AdjustableValues.getNumber("ELEV_L3_kA"));

        io.updateInputs(inputs);
        Logger.processInputs("/RealOutputs/Elevator", inputs);
        Logger.recordOutput("/Elevator/CurrentMode", voltageControl ? "Voltage" : "Position");

        // Skipping the PID and FeedForward calculations if voltage control is enabled.
        // As well as the other values calculated during position control.
        if (voltageControl) return;

        TrapezoidProfile.State goalState = pid.getGoal();

        Logger.recordOutput("/Elevator/VelocitySetpoint", goalState.velocity);
        Logger.recordOutput("/Elevator/VelocityError", goalState.velocity - inputs.velocity.in(MetersPerSecond));
        Logger.recordOutput("/Elevator/HeightSetpoint", goalState.position);
        Logger.recordOutput("/Elevator/HeightError", goalState.position - inputs.position.in(Meters));

        double ffVolts = 0;
        if (inputs.position.in(Meters) < 0.33) {
            ffVolts = l1FeedForward.calculateWithVelocities(inputs.velocity.in(MetersPerSecond), pid.getSetpoint().velocity);
        }

        if (inputs.position.in(Meters) < 0.65) {
            ffVolts = l2FeedForward.calculateWithVelocities(inputs.velocity.in(MetersPerSecond), pid.getSetpoint().velocity);
        }

        if (inputs.position.in(Meters) > 0.65) {
            ffVolts = l3FeedForward.calculateWithVelocities(inputs.velocity.in(MetersPerSecond), pid.getSetpoint().velocity);
        }

        double pidOutput = pid.calculate(inputs.position.in(Meters));

        Logger.recordOutput("/Elevator/PIDOutput", pidOutput);
        Logger.recordOutput("/Elevator/FFVolts", ffVolts);

        io.setVolts(Volts.of(pidOutput + ffVolts));
    }

    public void setPosition(Distance position) {
        voltageControl = false;

        // Clamping position setpoints
        if (position.lt(Meters.zero())) {
            position = Meters.zero();
        }

        if (position.gt(ElevatorConstants.maxHeight)) {
            position = ElevatorConstants.maxHeight;
        }

        pid.setGoal(new State(position.in(Meters), 0));
    }

    public void setVolts(Voltage volts) {
        voltageControl = true;

        io.setVolts(volts);
    }

    public void resetEncoder() {
        io.resetEncoder();
    }
}