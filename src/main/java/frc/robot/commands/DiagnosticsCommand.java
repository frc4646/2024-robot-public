// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DiagnosticsCommand extends Command
{

    /** Creates a new LEDCommand. */
    public DiagnosticsCommand()
    {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.DIAGNOSTICS);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        if (RobotContainer.ROBOT_STATUS.canShoot())
        {
            RobotContainer.DIAGNOSTICS.setState(Constants.DIAGNOSTICS.ALLIGNED);
            RobotContainer.CONTROLS_OPERATOR.setRumble(.75);
        }
        else if (RobotContainer.ROBOT_STATUS.hasPiece())
        {
            RobotContainer.DIAGNOSTICS.setState(Constants.DIAGNOSTICS.INTAKE_NOTE);
            RobotContainer.CONTROLS_OPERATOR.setRumble(0);
        }
        else
        {
            RobotContainer.DIAGNOSTICS.setState(Constants.DIAGNOSTICS.NO_PIECE);
            RobotContainer.CONTROLS_OPERATOR.setRumble(0);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public boolean runsWhenDisabled()
    {
        return true;
    }
}
