package frc.robot.subsystems.elevator.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class SetElevatorSpeed extends Command {
    private Elevator elevator;
    private Supplier<Double> throttle;

    public SetElevatorSpeed(Elevator elevator, Supplier<Double> throttle) {
        this.elevator = elevator;
        this.throttle = throttle;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double speed = throttle.get();

        speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
        speed = Math.copySign(speed * speed, speed);

        elevator.setVolts(Volts.of(speed * RobotController.getInputVoltage()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setVolts(Volts.zero());
    }
}