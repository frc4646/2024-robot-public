// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class JiggleNote extends Command {
    private double startTime; //start time for moving note back
    private final Intake intake = RobotContainer.INTAKE;
    private final Feeder feeder = RobotContainer.FEEDER;
/*
 * Jiggles the note back and forth, hopefully making it alligned in the shooter
 */
  public JiggleNote() {
    addRequirements(intake);
    addRequirements(feeder);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - startTime < Constants.INTAKE_FEEDER.JIGGLE_TIME)
    {
        feeder.setOutput(Constants.INTAKE_FEEDER.REJECT_PERCENT);
        intake.setOutput(Constants.INTAKE_FEEDER.REJECT_PERCENT);
    } else {
        feeder.setOutput(Constants.INTAKE_FEEDER.FEED_PERCENT);
        intake.setOutput(Constants.INTAKE_FEEDER.FEED_PERCENT);
        if (RobotContainer.ROBOT_STATUS.inFeeder())
        {
            startTime = Timer.getFPGATimestamp();
        }
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
