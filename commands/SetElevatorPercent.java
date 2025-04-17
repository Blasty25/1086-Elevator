package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class SetElevatorPercent extends Command {
    private Elevator elevator;
    private Supplier<Double> throttle;

    /**
     * Creates a new {@link SetElevatorPercent} command.
     * It controls the elevator with percent output based on a throttle
     * 
     * @param elevator The {@link Elevator} subsystem to control.
     * @param throttle The percent output to apply.
     */
    public SetElevatorPercent(Elevator elevator, Supplier<Double> throttle) {
        this.elevator = elevator;
        this.throttle = throttle;
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        double speed = throttle.get();

        speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
        speed = Math.copySign(speed * speed, speed);

        elevator.setPercent(speed * AdjustableValues.getNumber("Elevator_Percent"));
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        elevator.setPercent(0);
    }
}