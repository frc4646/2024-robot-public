// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberTelescope;

public class TelescopeClimbPosition extends Command {
    private final ClimberTelescope climber = RobotContainer.CLIMBER_TELESCOPE;
    private double positionL, positionR;
    
  public TelescopeClimbPosition(double positionInL, double positionInR) {
    addRequirements(climber);
    positionL = positionInL;
    positionR = positionInR;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setTargetPosition(positionL, positionR);
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
