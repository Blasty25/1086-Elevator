package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    public enum State {
        Exponential,
        Trapezoid,
        Voltage,
        Percent
    }

    public Elevator.State getCurrentState() {
        return Elevator.State.Trapezoid;
    }

    public Current getLeftCurrent() {
        return Amps.zero();
    }

    public Current getRightCurrent() {
        return Amps.zero();
    }

    public Temperature getLeftTemperature() {
        return Celsius.zero();
    }

    public Temperature getRightTemperature() {
        return Celsius.zero();
    }

    public Voltage getLeftVolts() {
        return Volts.zero();
    }

    public Voltage getRightVolts() {
        return Volts.zero();
    }

    public Distance getPosition() {
        return Meters.zero();
    }

    public LinearVelocity getVelocity() {
        return MetersPerSecond.zero();
    }

    public void setPosition(Distance position) {}

    public void setVolts(Voltage volts) {}

    public void setPercent(double percent) {}

    public void resetEncoder() {}

    public void resetEncoder(Distance distance) {}
}
