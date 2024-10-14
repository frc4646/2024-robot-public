// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotStatus;
import frc.robot.subsystems.Shooter;

public class ShootAutoRPM extends Command {
    private final Shooter shooter = RobotContainer.SHOOTER;
    private final RobotStatus state = RobotContainer.ROBOT_STATUS;
  /** Creates a new ShootAutoRPM. */
  public ShootAutoRPM() {
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (state.hasPiece() && state.getDistanceToSpeaker() < Constants.SHOOTER.MAXIMUM_DISTANCE)
    {
        double distance = state.getDistanceToSpeaker();
        double rpmR = Constants.SHOOTER.MAP.getRPMRight(distance);
        double rpmL = Constants.SHOOTER.MAP.getRPMLeft(distance);
        shooter.setRPM(rpmL, rpmR);
    } else {
        shooter.setRPM(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
