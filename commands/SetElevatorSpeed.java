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
    private Supplier<Double> percentSupplier;

    /**
     * Creates a new {@link SetElevatorSpeed} command.
     * It controls the elevator with voltage output based on a throttle and a percent supplier
     * 
     * @param elevator The {@link Elevator} subsystem to control
     * @param throttle The percent voltage to apply.
     * @param percentSupplier The max percent to run at.
     */
    public SetElevatorSpeed(Elevator elevator, Supplier<Double> throttle, Supplier<Double> percentSupplier) {
        this.elevator = elevator;
        this.throttle = throttle;
        this.percentSupplier = percentSupplier;

        addRequirements(elevator);
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        double speed = throttle.get();

        speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
        speed = Math.copySign(speed * speed, speed);

        elevator.setVolts(Volts.of(speed * percentSupplier.get() * RobotController.getInputVoltage()));
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        elevator.setVolts(Volts.zero());
    }
}