package frc.robot.commands.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DrivePointAtTargetAuto extends Command
{
    private final PhoenixPIDController turnPID = new PhoenixPIDController(
            Constants.SWERVE_DRIVE.PID_ROTATION_KP,
            Constants.SWERVE_DRIVE.PID_ROTATION_KI,
            Constants.SWERVE_DRIVE.PID_ROTATION_KD);
    // no way that works irl

    FieldCentricFacingAngle pointAtTarget = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);
    Pose2d desiredTarget;
    boolean flip;

    public DrivePointAtTargetAuto(Pose2d target, boolean flipped)
    {
        desiredTarget = target;
        flip = flipped;
        addRequirements(RobotContainer.SWERVE);
        pointAtTarget.HeadingController = turnPID;
        pointAtTarget.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute()
    {
        Rotation2d testRot = RobotContainer.ROBOT_STATUS.mirrorAndGetAngle(desiredTarget, flip);

        SmartDashboard.putNumber("Point At Target Angle", testRot.getDegrees());

        RobotContainer.SWERVE.setControl(pointAtTarget
                .withTargetDirection(testRot)
                .withRotationalDeadband(.05)
                .withVelocityX(RobotContainer.CONTROLS_DRIVE.getXSpeed())
                .withVelocityY(RobotContainer.CONTROLS_DRIVE.getYSpeed()));
    }
}
