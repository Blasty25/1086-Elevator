package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevatorHeight extends Command {
    private Elevator elevator;
    private Distance height;

    /**
     * Creates a new {@link SetElevatorHeight} command.
     * This command sets the height setpoint of the elevator.
     * 
     * @param elevator The {@link Elevator} subsystem to control.
     * @param height The height to travel to.
     */
    public SetElevatorHeight(Elevator elevator, Distance height) {
        this.elevator = elevator;
        this.height = height;
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {
        elevator.setPosition(height);
        
        // Cancels immediately because elevator.setPosition() only needs to run once.
        cancel();
    }
}