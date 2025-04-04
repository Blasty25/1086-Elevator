package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevatorHeight extends Command {
    private Elevator elevator;
    private Distance height;

    public SetElevatorHeight(Elevator elevator, Distance height) {
        this.elevator = elevator;
        this.height = height;

        addRequirements(elevator);
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {
        elevator.setPosition(height);
        cancel();
    }
}