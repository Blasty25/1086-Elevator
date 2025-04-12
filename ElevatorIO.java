package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInputs {
        Elevator.State currentState = Elevator.State.Voltage;

        double leftCurrent = 0; // Amps
        double rightCurrent = 0; // Amps

        double leftTemperature = 0; // Celsius
        double rightTemperature = 0; // Celsius

        double leftVolts = 0; // Voltage
        double rightVolts = 0; // Voltage

        double position = 0; // Meters
        double velocity = 0; // Meters / Second
        double acceleration = 0; // Meters / Second^2
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

    /** Resets the elevator encoder to the parameter in meters. */
    public void resetEncoder(double height);
}