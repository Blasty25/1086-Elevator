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
import frc.robot.util.AdjustableValues;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim elevator;

    private PIDController controller = new PIDController(
            AdjustableValues.getNumber("Elev_kP"),
            AdjustableValues.getNumber("Elev_kI"),
            AdjustableValues.getNumber("Elev_kD"));

    private ExponentialProfile exponentialProfile = new ExponentialProfile(
            ExponentialProfile.Constraints.fromCharacteristics(12,
                AdjustableValues.getNumber("Elev_kV"),
                AdjustableValues.getNumber("Elev_kA")));

    private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConstants.maxVelocity.in(MetersPerSecond),
                ElevatorConstants.maxAcceleration.in(MetersPerSecondPerSecond)));

    private Elevator.State currentState = Elevator.State.Voltage;
    // The unit of this measure changes based on the current state.
    private double input = 0;

    public ElevatorIOSim() {
        this.elevator = new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                        DCMotor.getKrakenX60(2),
                        ElevatorConstants.mass.in(Kilograms),
                        ElevatorConstants.radius.in(Meters),
                        ElevatorConstants.gearRatio),
                DCMotor.getKrakenX60(2), 0,
                ElevatorConstants.maxHeight.in(Meters),
                true, 0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (AdjustableValues.hasChanged("Elev_kP")) controller.setP(AdjustableValues.getNumber("Elev_kP"));
        if (AdjustableValues.hasChanged("Elev_kI")) controller.setI(AdjustableValues.getNumber("Elev_kI"));
        if (AdjustableValues.hasChanged("Elev_kD")) controller.setD(AdjustableValues.getNumber("Elev_kD"));

        if (AdjustableValues.hasChanged("Elev_kV") || AdjustableValues.hasChanged("Elev_kA")) {
            exponentialProfile = new ExponentialProfile(
                    ExponentialProfile.Constraints.fromCharacteristics(
                        RobotController.getInputVoltage(),
                        AdjustableValues.getNumber("Elev_kV"),
                        AdjustableValues.getNumber("Elev_kA")));
        }

        double voltageInput = 0;

        switch (currentState) {
            case Exponential:
                ExponentialProfile.State goalExpoState = exponentialProfile.calculate(0.02, 
                        new ExponentialProfile.State(
                            inputs.position.in(Meters),
                            inputs.velocity.in(MetersPerSecond)),
                        new ExponentialProfile.State(input, 0));

                voltageInput = controller.calculate(inputs.position.in(Meters), goalExpoState.position) + (AdjustableValues.getNumber("Elev_kG") + AdjustableValues.getNumber("Elev_kS"));
                break;

            case Trapezoid:
                TrapezoidProfile.State goalTrapState = trapezoidProfile.calculate(0.02,
                        new TrapezoidProfile.State(
                            inputs.position.in(Meters),
                            inputs.velocity.in(MetersPerSecond)),
                        new TrapezoidProfile.State(input, 0));

                voltageInput = controller.calculate(inputs.position.in(Meters), goalTrapState.position) + (AdjustableValues.getNumber("Elev_kG") + AdjustableValues.getNumber("Elev_kS"));
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