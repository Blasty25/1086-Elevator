package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim elevator;

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
        elevator.update(0.02);

        inputs.leftCurrent = Amps.of(elevator.getCurrentDrawAmps());
        inputs.rightCurrent = inputs.leftCurrent.unaryMinus();

        inputs.leftVolts = Volts.of(elevator.getInput(0));
        inputs.rightVolts = inputs.leftVolts.unaryMinus();

        inputs.position = Meters.of(elevator.getPositionMeters());
        inputs.velocity = MetersPerSecond.of(elevator.getVelocityMetersPerSecond());
    }

    @Override
    public void setVolts(Voltage voltage) {
        elevator.setInputVoltage(MathUtil.clamp(voltage.in(Volts), -12, 12));
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