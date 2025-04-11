package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInputs {
        Elevator.State currentState = Elevator.State.Voltage;

        Current leftCurrent = Amps.zero();
        Current rightCurrent = Amps.zero();

        Temperature leftTemperature = Celsius.zero();
        Temperature rightTemperature = Celsius.zero();

        Voltage leftVolts = Volts.zero();
        Voltage rightVolts = Volts.zero();

        Distance position = Meters.zero();
        LinearVelocity velocity = MetersPerSecond.zero();
    }

    /** Updates a set of {@link ElevatorIOInputs} with new values. */
    public void updateInputs(ElevatorIOInputs inputs);

    /**
     * Sets the control mode of the elevator.
     * 
     * If the state is either {@link Elevator.State#Exponential Exponential} or {@link Elevator.State#Trapezoid Trapezoid} then the measure should be in meters.
     * If the state is {@link Elevator.State#Voltage Voltage} then the measure should be in volts.
     * 
     * @param measure The height to go to in meters or the desired voltage to run at.
     * @param state The {@link Elevator.State Elevator State} to use.
     */
    public void setControl(double measure, Elevator.State state);

    /** Resets the elevator encoder to zero. */
    public void resetEncoder();

    /** Resets the elevator encoder to the {@link Distance} value. */
    public void resetEncoder(Distance height);
}