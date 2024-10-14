// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterArm;

public class ArmThumbstick extends Command
{
    private final ShooterArm arm = RobotContainer.SHOOTER_ARM;

    /** Creates a new ArmThumbstick. */
    public ArmThumbstick()
    {
        addRequirements(arm);
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
        arm.setTargetVoltage(RobotContainer.CONTROLS_OPERATOR.getArmJoystick() * .5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        // store the current angle as the new setpoint
        arm.setTargetAngle(arm.getAngle());
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
