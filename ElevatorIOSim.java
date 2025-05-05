package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.TurboLogger;

public class ElevatorIOSim implements ElevatorIO {
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

    public ElevatorIOSim() {
        this.elevator =
                new ElevatorSim(
                        LinearSystemId.createElevatorSystem(
                                DCMotor.getKrakenX60(2),
                                ElevatorConstants.mass.in(Kilograms),
                                ElevatorConstants.radius.in(Meters),
                                ElevatorConstants.gearRatio),
                        DCMotor.getKrakenX60(2),
                        0,
                        ElevatorConstants.maxHeight.in(Meters),
                        true,
                        0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (TurboLogger.hasChanged("Elev_kP"))
            controller.setP(TurboLogger.get("Elev_kP", ElevatorConstants.kPDefault));
        if (TurboLogger.hasChanged("Elev_kI"))
            controller.setI(TurboLogger.get("Elev_kI", ElevatorConstants.kIDefault));
        if (TurboLogger.hasChanged("Elev_kD"))
            controller.setD(TurboLogger.get("Elev_kD", ElevatorConstants.kDDefault));

        if (TurboLogger.hasChanged("Elev_kV") || TurboLogger.hasChanged("Elev_kA")) {
            exponentialProfile =
                    new ExponentialProfile(
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
                                        inputs.position.in(Meters),
                                        inputs.velocity.in(MetersPerSecond)),
                                new ExponentialProfile.State(input, 0));

                voltageInput =
                        controller.calculate(
                                inputs.position.in(Meters),
                                goalExpoState.position
                                        + TurboLogger.get("Elev_kG", ElevatorConstants.kGDefault)
                                        + TurboLogger.get("Elev_kS", ElevatorConstants.kSDefault));
                break;

            case Trapezoid:
                TrapezoidProfile.State goalTrapState =
                        trapezoidProfile.calculate(
                                0.02,
                                new TrapezoidProfile.State(
                                        inputs.position.in(Meters),
                                        inputs.velocity.in(MetersPerSecond)),
                                new TrapezoidProfile.State(input, 0));

                voltageInput =
                        controller.calculate(
                                inputs.position.in(Meters),
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

        inputs.leftCurrent = Amps.of(elevator.getCurrentDrawAmps());
        inputs.rightCurrent = inputs.leftCurrent.unaryMinus();

        inputs.leftVolts = Volts.of(elevator.getInput(0));
        inputs.rightVolts = inputs.leftVolts.unaryMinus();

        inputs.position = Meters.of(elevator.getPositionMeters());
        inputs.velocity = MetersPerSecond.of(elevator.getVelocityMetersPerSecond());
    }

    @Override
    public void setControl(double measure, Elevator.State state) {
        input = measure;
        currentState = state;
    }

    @Override
    public void resetEncoder() {
        elevator.setState(0, elevator.getVelocityMetersPerSecond());
    }

    @Override
    public void resetEncoder(Distance height) {
        elevator.setState(height.in(Meters), elevator.getVelocityMetersPerSecond());
    }
}
