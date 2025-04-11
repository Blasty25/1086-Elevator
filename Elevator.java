package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
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

    public void setPosition(Distance position) {
        // Clamping position setpoints
        if (position.lt(Meters.zero())) {
            position = Meters.zero();
        }

        if (position.gt(ElevatorConstants.maxHeight)) {
            position = ElevatorConstants.maxHeight;
        }

        io.setControl(position.in(Meters), Elevator.State.Trapezoid);
    }

    public void setVolts(Voltage volts) {
        io.setControl(volts.in(Volts), Elevator.State.Voltage);
    }

    public void resetEncoder() {
        io.resetEncoder();
    }
}