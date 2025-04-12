package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevatorHeight extends Command {
    private Elevator elevator;
    private double height;

    /**
     * Creates a new {@link SetElevatorHeight} command.
     * This command sets the height setpoint of the elevator.
     * 
     * @param elevator The {@link Elevator} subsystem to control.
     * @param height The height to travel to in meters.
     */
    public SetElevatorHeight(Elevator elevator, double height) {
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