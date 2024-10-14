package frc.robot.configuration;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ArmDegrees;
import frc.robot.commands.commandGroups.SetArmThenIntake;
import frc.robot.commands.commandGroups.prepareToAimAndSwerve;
import frc.robot.commands.drive.DriveResetGyro;
import frc.robot.commands.drive.DriveRotateToAngle;
import frc.team4646.util.Util;

public class DriverControlsXbox implements IDriverControls
{
    private final double kJoystickDeadband = .2;
    private final int kJoystickPower = 2; // square or cube the input to get finer control at low input values
    private final CommandXboxController controller = new CommandXboxController(0);

    private boolean invertControlsForRed = false;

    private SlewRateLimiter xspeedLimiter;
    private SlewRateLimiter yspeedLimiter;
    private SlewRateLimiter rotLimiter;

    public DriverControlsXbox()
    {
        xspeedLimiter = new SlewRateLimiter(Constants.SWERVE_DRIVE.MaxSpeedAccel);
        yspeedLimiter = new SlewRateLimiter(Constants.SWERVE_DRIVE.MaxSpeedAccel);
        rotLimiter = new SlewRateLimiter(Constants.SWERVE_DRIVE.MaxAngularRateAccel);

        // Reset Gyro
        controller.back().onTrue(new DriveResetGyro());
        // controller.b().whileTrue(new DriveRotateToAngle(Constants.FIELD.AMP.getRotation().getDegrees()));
        // controller.a().whileTrue(new DrivePointAtTargetNudge(Constants.FIELD.SPEAKER, false));
        // controller.x().whileTrue(new DrivePointAtTarget(Constants.FIELD.AMP, true));
        // controller.rightBumper()
        // .whileTrue(new DriveGoToLocation(Constants.FIELD.AMP, false, true));
        // controller.leftBumper().whileTrue(new ArmDegrees(Constants.SHOOTER_ARM.SERVO.DEGREES_MIN));
        // TODO use ABXY for snapping to angle, so you can still use your left hand to translate
        // controller.y().whileTrue(new SnapToAngleDrive(controller, 0));

        controller.leftBumper().whileTrue(new DriveRotateToAngle(Constants.SWERVE_DRIVE.AMP_DEGREES));
        controller.leftTrigger(.5).whileTrue(new SetArmThenIntake());
        controller.rightBumper().whileTrue(new prepareToAimAndSwerve());
        controller.rightTrigger(.5).whileTrue(new ArmDegrees(Constants.SHOOTER_ARM.SERVO.DEGREES_MIN));

    }

    public void whileDisabled()
    {
        invertControlsForRed = RobotContainer.ROBOT_STATUS.isOnRedAlliance();
    }

    @Override
    public double getXSpeed()
    {
        // Drive left with -X (left)
        double value = -Util.handleJoystick(controller.getLeftY(), kJoystickDeadband, kJoystickPower) * Constants.SWERVE_DRIVE.MaxSpeed;
        value = xspeedLimiter.calculate(value);
        return invertControlsForRed ? -value : value;
    }

    @Override
    public double getYSpeed()
    {
        // Drive forward with -Y (forward)
        double value = -Util.handleJoystick(controller.getLeftX(), kJoystickDeadband, kJoystickPower) * Constants.SWERVE_DRIVE.MaxSpeed;
        value = yspeedLimiter.calculate(value);
        return invertControlsForRed ? -value : value;
    }

    @Override
    public double getRotationRate()
    {
        // Drive ccw with -X (left)
        double value = -Util.handleJoystick(controller.getRightX(), kJoystickDeadband, kJoystickPower) * Constants.SWERVE_DRIVE.MaxAngularRate;
        value = rotLimiter.calculate(value);
        return value;
    }

    @Override
    public int getSnapAngle()
    {
        return -1;
    }
}
