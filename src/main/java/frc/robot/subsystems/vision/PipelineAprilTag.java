package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.team4646.drivers.LimelightHelpers;

public class PipelineAprilTag
{
    public Pose3d botPose;
    public Pose3d targetPose;
    public int primaryID; // ID number of the primary tag used for the AprilTag calculations TODO: is there a way to see all the tags that are
                          // being used?
    public double xDegrees, yDegrees, area, latency;

    public PipelineAprilTag()
    {
        clear();
    }

    public void clear()
    {
        botPose = new Pose3d();
        targetPose = new Pose3d();
        primaryID = -1;
        latency = -1.0;
    }

    public double getDistanceToTarget()
    {
        return Math.sqrt(Math.pow(targetPose.getX(), 2) + Math.pow(targetPose.getZ(), 2)); // TODO: make sure this is the right x and y
    }

    public double getRotationDelta()
    {
        return xDegrees;
    }

    public double getPitchDelta()
    {
        return yDegrees;
    }

    public boolean process(String pipelineName)
    {
        double[] botPoseRaw = LimelightHelpers.getBotPose_wpiBlue(pipelineName);
        double[] targetPoseRaw = LimelightHelpers.getTargetPose_RobotSpace(pipelineName);
        primaryID = (int) LimelightHelpers.getFiducialID(pipelineName);
        xDegrees = LimelightHelpers.getTX(pipelineName);
        yDegrees = LimelightHelpers.getTY(pipelineName);
        area = LimelightHelpers.getTA(pipelineName);

        if (targetPoseRaw.length > 5 || botPoseRaw.length > 5)
        {
            botPose = new Pose3d(botPoseRaw[0], botPoseRaw[1], botPoseRaw[2], new Rotation3d(botPoseRaw[3], botPoseRaw[4], botPoseRaw[5]));
            latency = botPoseRaw[6];
            targetPose = new Pose3d(targetPoseRaw[0], targetPoseRaw[1], targetPoseRaw[2],
                    new Rotation3d(targetPoseRaw[3], targetPoseRaw[4], targetPoseRaw[5]));
        }

        return LimelightHelpers.getTV(pipelineName);
    }
}