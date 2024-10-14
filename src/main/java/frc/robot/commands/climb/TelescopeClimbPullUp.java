// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TelescopeClimbPullUp extends Command {
    private double startPosL, startPosR;
  public TelescopeClimbPullUp() {
    addRequirements(RobotContainer.CLIMBER_TELESCOPE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPosL = RobotContainer.CLIMBER_TELESCOPE.getPositionL();
    startPosR = RobotContainer.CLIMBER_TELESCOPE.getPositionR();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.CONTROLS_OPERATOR.isRightThumbstickDown())
    {
        RobotContainer.CLIMBER_TELESCOPE.setTargetPosition
            (startPosR - Constants.CLIMBER_TELESCOPE.PULL_UP_INCHES, startPosL - Constants.CLIMBER_TELESCOPE.PULL_UP_INCHES);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.CLIMBER_TELESCOPE.checkPositionL(Constants.CLIMBER_TELESCOPE.POSITION_TOLERANCE)
        && RobotContainer.CLIMBER_TELESCOPE.checkPositionR(Constants.CLIMBER_TELESCOPE.POSITION_TOLERANCE);
  }
}
