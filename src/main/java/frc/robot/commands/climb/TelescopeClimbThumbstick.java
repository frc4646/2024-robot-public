// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberTelescope;

public class TelescopeClimbThumbstick extends Command
{
    private final ClimberTelescope climber = RobotContainer.CLIMBER_TELESCOPE;

    public TelescopeClimbThumbstick()
    {
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        double speedL, speedR;

        if (RobotContainer.CONTROLS_OPERATOR.isLeftTriggerPressed())
        {
            speedL = RobotContainer.CONTROLS_OPERATOR.getClimberJoystick() * Constants.CLIMBER_TELESCOPE.CLIMB_SPEED;
        }
        else
        {
            speedL = 0;
        }

        if (RobotContainer.CONTROLS_OPERATOR.isRightTriggerPressed())
        {
            speedR = RobotContainer.CONTROLS_OPERATOR.getClimberJoystick() * Constants.CLIMBER_TELESCOPE.CLIMB_SPEED;
        }
        else
        {
            speedR = 0;
        }
        SmartDashboard.putNumber("Position R", climber.getPositionR());
        SmartDashboard.putNumber("Position L", climber.getPositionL());
        climber.setTargetVoltage(speedL, speedR);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        climber.setTargetPosition(climber.getPositionL(), climber.getPositionR());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
