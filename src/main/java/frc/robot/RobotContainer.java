// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConstants;
import frc.robot.elevator.ElevatorIOReal;
import frc.robot.elevator.ElevatorIOSim;

public class RobotContainer {

  private Elevator elevator;
  private CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    if (Robot.isReal()) {
      elevator = new Elevator(new ElevatorIOReal());
    } else {
      elevator = new Elevator(new ElevatorIOSim());
    }
    configureBindings();
  }

  private void configureBindings() {
    controller.y().onTrue(elevator.runSetpoint(ElevatorConstants.l4));
    controller.x().onTrue(elevator.runSetpoint(ElevatorConstants.l3));
    controller.b().onTrue(elevator.runSetpoint(ElevatorConstants.l2));
    controller.a().onTrue(elevator.runSetpoint(ElevatorConstants.l1));
    controller.povDown().onTrue(elevator.runSetpoint(ElevatorConstants.stow));

    controller.povUp().onTrue(elevator.resetEncoder());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
