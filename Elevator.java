package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public enum State {
        Exponential,
        Trapezoid,
        Voltage
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("/RealOutputs/Elevator", inputs);
    }

    public void setPosition(double position) {
        // Clamping position setpoints
        if (position < 0) {
            position = 0;
        }

        if (position > ElevatorConstants.maxHeight) {
            position = ElevatorConstants.maxHeight;
        }

        io.setControl(position, Elevator.State.Trapezoid);
    }

    public void setVolts(double volts) {
        io.setControl(volts, Elevator.State.Voltage);
    }

    public void resetEncoder() {
        io.resetEncoder();
    }
}