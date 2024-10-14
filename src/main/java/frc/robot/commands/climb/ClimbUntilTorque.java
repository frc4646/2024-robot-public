// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimbUntilTorque extends Command {

    private boolean isLeftTorqueSaturated;
    private boolean isRightTorqueSaturated;
    private double timer; //TODO fix later, rough code :(

  public ClimbUntilTorque() {
    addRequirements(RobotContainer.CLIMBER);
  }

  @Override
  public void initialize() {
    isLeftTorqueSaturated = false;
    isRightTorqueSaturated = false;
    timer = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double torqueL = RobotContainer.CLIMBER.getTorqueCurrentL();
    double torqueR = RobotContainer.CLIMBER.getTorqueCurrentR();
    timer++;
    SmartDashboard.putNumber("timer", timer);
    if (!isLeftTorqueSaturated && Math.abs(torqueL) >= Constants.CLIMBER.MAX_LEFT_TORQUE_CURRENT && timer > 30)
    {
        isLeftTorqueSaturated = true;
    }

    if (!isRightTorqueSaturated && Math.abs(torqueR) >= Constants.CLIMBER.MAX_RIGHT_TORQUE_CURRENT && timer > 30)
    {
        isRightTorqueSaturated = true;
    }

    if (isLeftTorqueSaturated)
    {
        RobotContainer.CLIMBER.setOutputL(0);
    } else {
        RobotContainer.CLIMBER.setOutputL(-Constants.CLIMBER.CLIMB_SPEED);
    }

    if (isRightTorqueSaturated)
    {
        RobotContainer.CLIMBER.setOutputR(0);
    } else {
        RobotContainer.CLIMBER.setOutputR(-Constants.CLIMBER.CLIMB_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isLeftTorqueSaturated && isRightTorqueSaturated;
  }
}
