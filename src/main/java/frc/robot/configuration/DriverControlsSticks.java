// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configuration;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ArmDegrees;
import frc.robot.commands.commandGroups.SetArmThenIntake;
import frc.robot.commands.commandGroups.prepareToAimAndSwerve;
import frc.robot.commands.drive.DriveAllignToStage;
import frc.robot.commands.drive.DriveRotateToAngle;
import frc.team4646.util.Util;

public class DriverControlsSticks implements IDriverControls
{
    private final double kJoystickDeadband = .25;
    private final int kJoystickPower = 1; // square or cube the input to get finer control at low input values
    private final CommandJoystick stickMove;
    private final CommandJoystick stickRotate;

    private final int BUTTON_BOTTOM = 1;
    private final int BUTTON_TOP = 2;
    private final int BUTTON_LEFT = 3;
    private final int BUTTON_RIGHT = 4;
    private boolean invertControlsForRed;

    private SlewRateLimiter xspeedLimiter;
    private SlewRateLimiter yspeedLimiter;
    private SlewRateLimiter rotLimiter;

    public DriverControlsSticks()
    {
        stickMove = new CommandJoystick(0);
        stickRotate = new CommandJoystick(2);

        xspeedLimiter = new SlewRateLimiter(Constants.SWERVE_DRIVE.MaxSpeedAccel);
        yspeedLimiter = new SlewRateLimiter(Constants.SWERVE_DRIVE.MaxSpeedAccel);
        rotLimiter = new SlewRateLimiter(Constants.SWERVE_DRIVE.MaxAngularRateAccel);

        stickMove.button(BUTTON_BOTTOM).whileTrue(new SetArmThenIntake());
        stickMove.button(BUTTON_TOP).whileTrue(new DriveRotateToAngle(Constants.SWERVE_DRIVE.AMP_DEGREES));
        stickRotate.button(BUTTON_BOTTOM).whileTrue(new ArmDegrees(Constants.SHOOTER_ARM.SERVO.DEGREES_MIN));
        stickRotate.button(BUTTON_TOP).whileTrue((new prepareToAimAndSwerve()));
        stickRotate.button(BUTTON_LEFT).whileTrue(new DriveAllignToStage(true));
        stickRotate.button(BUTTON_RIGHT).whileTrue(new DriveAllignToStage(false));
        // SmartDashboard.putData("Reset Gyro", new InstantCommand(RobotContainer.SWERVE::seedFieldRelative,
        // RobotContainer.SWERVE));
        // SmartDashboard.putData("Reset Pose Apriltag", new VisionBackSetPose());

        // stickRotate.button(BUTTON_BOTTOM).whileTrue(new DriveSnapToAngle());

        // if all 4 buttons are pressed
        // stickMove.button(BUTTON_TOP)
        // .and(stickMove.button(BUTTON_BOTTOM))
        // .and(stickRotate.button(BUTTON_TOP))
        // .and(stickRotate.button(BUTTON_BOTTOM))
        // .onTrue(new VisionBackSetPoseAndResetGyro());
    }

    @Override
    public double getXSpeed()
    {
        // Drive left with -X (left)
        double value = -Util.handleJoystick(stickMove.getY(), kJoystickDeadband, kJoystickPower) * Constants.SWERVE_DRIVE.MaxSpeed;
        // SmartDashboard.putNumber("Joystick X", value);
        value = xspeedLimiter.calculate(value);
        return invertControlsForRed ? -value : value;
    }

    @Override
    public double getYSpeed()
    {
        // Drive forward with -Y (forward)
        double value = -Util.handleJoystick(stickMove.getX(), kJoystickDeadband, kJoystickPower) * Constants.SWERVE_DRIVE.MaxSpeed;
        // SmartDashboard.putNumber("Joystick Y", value);
        value = yspeedLimiter.calculate(value);
        return invertControlsForRed ? -value : value;
    }

    @Override
    public double getRotationRate()
    {
        // Drive ccw with -X (left)
        double value = -Util.handleJoystick(stickRotate.getX(), kJoystickDeadband, kJoystickPower) * Constants.SWERVE_DRIVE.MaxAngularRate;
        // SmartDashboard.putNumber("Joystick Rotate", value);
        value = rotLimiter.calculate(value);
        return value;
    }

    @Override
    public int getSnapAngle()
    {
        // get stick angle between 0 and 360
        if (stickRotate.getMagnitude() < kJoystickDeadband)
        {
            return -1;
        }
        double angle = (stickRotate.getDirectionDegrees() + 360) % 360.0;
        // SmartDashboard.putNumber("Joystick Angle", angle);

        // convert to robot heading
        if (angle < 45 || angle > 315)
        {
            return 0;
        }
        else if (angle < 135)
        {
            return 270;
        }
        else if (angle < 225)
        {
            return 180;
        }
        else
        {
            return 90;
        }
    }

    @Override
    public void whileDisabled()
    {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'whileDisabled'");
        invertControlsForRed = RobotContainer.ROBOT_STATUS.isOnRedAlliance();
    }
}
