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
    public static class ElevatorIOInputs {
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

    /** Sets the voltage output of the elevator. */
    public void setVolts(Voltage volts);

    /** Resets the elevator encoder to zero. */
    public void resetEncoder();

    /** Resets the elevator encoder to the {@link Distance} value. */
    public void resetEncoder(Distance height);
}