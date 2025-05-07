package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.TurboLogger;

// This is named weirdly to prevent conflicts with the WPILib ElevatorSim class.
public class SimElevator extends Elevator {
    private ElevatorSim elevator;

    private PIDController controller =
            new PIDController(
                    TurboLogger.get("Elev_kP", ElevatorConstants.kPDefault),
                    TurboLogger.get("Elev_kI", ElevatorConstants.kIDefault),
                    TurboLogger.get("Elev_kD", ElevatorConstants.kDDefault));

    private ExponentialProfile exponentialProfile =
            new ExponentialProfile(
                    ExponentialProfile.Constraints.fromCharacteristics(
                            12,
                            TurboLogger.get("Elev_kV", ElevatorConstants.kVDefault),
                            TurboLogger.get("Elev_kA", ElevatorConstants.kADefault)));

    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            ElevatorConstants.maxVelocity.in(MetersPerSecond),
                            ElevatorConstants.maxAcceleration.in(MetersPerSecondPerSecond)));

    private Elevator.State currentState = Elevator.State.Voltage;
    // The unit of this measure changes based on the current state.
    private double input = 0;

    public SimElevator() {
        this.elevator = new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                        DCMotor.getKrakenX60(2),
                        ElevatorConstants.mass.in(Kilograms),
                        ElevatorConstants.radius.in(Meters),
                        ElevatorConstants.gearRatio),
                DCMotor.getKrakenX60(2), 0,
                ElevatorConstants.maxHeight.in(Meters), true, 0);
    }

    @Override
    public void periodic() {
        if (TurboLogger.hasChanged("Elev_kP")) controller.setP(TurboLogger.get("Elev_kP", ElevatorConstants.kPDefault));
        if (TurboLogger.hasChanged("Elev_kI")) controller.setI(TurboLogger.get("Elev_kI", ElevatorConstants.kIDefault));
        if (TurboLogger.hasChanged("Elev_kD")) controller.setD(TurboLogger.get("Elev_kD", ElevatorConstants.kDDefault));

        if (TurboLogger.hasChanged("Elev_kV") || TurboLogger.hasChanged("Elev_kA")) {
            exponentialProfile = new ExponentialProfile(
                    ExponentialProfile.Constraints.fromCharacteristics(
                            RobotController.getInputVoltage(),
                            TurboLogger.get("Elev_kV", ElevatorConstants.kVDefault),
                            TurboLogger.get("Elev_kA", ElevatorConstants.kADefault)));
        }

        double voltageInput = 0;

        switch (currentState) {
            case Exponential:
                ExponentialProfile.State goalExpoState =
                        exponentialProfile.calculate(
                                0.02,
                                new ExponentialProfile.State(
                                        getPosition().in(Meters),
                                        getVelocity().in(MetersPerSecond)),
                                new ExponentialProfile.State(input, 0));

                voltageInput =
                        controller.calculate(
                                getPosition().in(Meters),
                                goalExpoState.position
                                        + TurboLogger.get("Elev_kG", ElevatorConstants.kGDefault)
                                        + TurboLogger.get("Elev_kS", ElevatorConstants.kSDefault));
                break;

            case Trapezoid:
                TrapezoidProfile.State goalTrapState =
                        trapezoidProfile.calculate(
                                0.02,
                                new TrapezoidProfile.State(
                                        getPosition().in(Meters),
                                        getVelocity().in(MetersPerSecond)),
                                new TrapezoidProfile.State(input, 0));

                voltageInput =
                        controller.calculate(
                                getPosition().in(Meters),
                                goalTrapState.position
                                        + TurboLogger.get("Elev_kG", ElevatorConstants.kGDefault)
                                        + TurboLogger.get("Elev_kS", ElevatorConstants.kSDefault));
                break;

            case Voltage:
                voltageInput = input;
                break;

            case Percent:
                voltageInput = input * RobotController.getInputVoltage();
                break;
        }

        double maxInput = RobotController.getInputVoltage();
        elevator.setInputVoltage(MathUtil.clamp(voltageInput, -maxInput, maxInput));

        elevator.update(0.02);
    }

    public Elevator.State getCurrentState() {
        return currentState;
    }

    public Current getLeftCurrent() {
        return Amps.of(elevator.getCurrentDrawAmps());
    }

    public Current getRightCurrent() {
        return Amps.of(-elevator.getCurrentDrawAmps());
    }

    public Voltage getLeftVolts() {
        return Volts.of(elevator.getInput(0));
    }

    public Voltage getRightVolts() {
        return Volts.of(-elevator.getInput(0));
    }

    public Distance getPosition() {
        return Meters.of(elevator.getPositionMeters());
    }

    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(elevator.getVelocityMetersPerSecond());
    }

    public void setPosition(Distance position) {
        // Clamping position setpoints
        if (position.lt(Meters.zero())) {
            position = Meters.zero();
        }

        if (position.gt(ElevatorConstants.maxHeight)) {
            position = ElevatorConstants.maxHeight;
        }

        currentState = Elevator.State.Trapezoid;

        input = position.in(Meters);
    }

    public void setVolts(Voltage volts) {
        currentState = Elevator.State.Voltage;

        input = volts.in(Volts);
    }

    public void setPercent(double percent) {
        currentState = Elevator.State.Percent;
        input = percent;
    }

    public void resetEncoder() {
        elevator.setState(0, elevator.getVelocityMetersPerSecond());
    }

    public void resetEncoder(Angle angle) {
        elevator.setState(angle.in(Radians) * ElevatorConstants.radius.in(Meters), elevator.getVelocityMetersPerSecond());
    }
}
