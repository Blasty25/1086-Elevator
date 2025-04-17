package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class ElevatorCommands {
    public static Command setHeight(Elevator elevator, Distance height) {
        return Commands.runOnce(() -> {
            elevator.setPosition(height);
        }, elevator);
    }

    public static Command setPercent(Elevator elevator, Supplier<Double> throttle) {
        return Commands.runEnd(() -> {
            double speed = throttle.get();

            speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
            speed = Math.copySign(speed * speed, speed);

            elevator.setPercent(speed * AdjustableValues.getNumber("Elevator_Percent"));
        }, () -> {
            elevator.setPercent(0);
        }, elevator);
    }

    public static Command setVoltage(Elevator elevator, Supplier<Double> throttle) {
        return Commands.runEnd(() -> {
            double speed = throttle.get();

            speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
            speed = Math.copySign(speed * speed, speed);

            elevator.setVolts(Volts.of(speed * AdjustableValues.getNumber("Elevator_Percent") * RobotController.getInputVoltage()));
        }, () -> {
            elevator.setVolts(Volts.zero());
        }, elevator);
    }
}