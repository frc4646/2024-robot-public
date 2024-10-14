package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.team4646.drivers.LimelightHelpers;
import frc.team4646.drivers.LimelightHelpers.PoseEstimate;
import frc.team4646.util.Util;

public class PipelineLocalization
{
    private final double DISTANCE_ERROR_MAX = 2.0;

    private final double TWO_TAG_DISTANCE_MIN = 1.5;
    private final double TWO_TAG_DISTANCE_MAX = 5.0;
    private final double TWO_TAG_XY_STDS_MIN = 0.5;
    private final double TWO_TAG_XY_STDS_MAX = 1.0;

    public Pose2d poseRobotUnfiltered = new Pose2d();
    public Pose2d poseRobotFiltered = new Pose2d();
    public int tagCount = 0;
    public double avgTagDistM = 0;
    public boolean firstUpdate = true;
    public double distanceError = 0.0;
    public double timestampSeconds = 0.0;
    public final boolean isPrimaryCamera;

    public PipelineLocalization(boolean isPrimaryCamera)
    {
        this.isPrimaryCamera = isPrimaryCamera;
    }

    /** returns true if */
    public boolean process(String pipelineName)
    {
        // Using botpose directly because LimelightHelpers.getBotPose2d_wpiBlue() is unclear if data is good
        final PoseEstimate botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue(pipelineName);

        double xyStds = .7;
        double degStds = 99999;

        // Raw Pose
        poseRobotUnfiltered = botpose.pose;
        // double tagArea = LimelightHelpers.getTA(pipelineName);
        tagCount = botpose.tagCount;
        avgTagDistM = botpose.avgTagDistM;
        distanceError = poseRobotUnfiltered.getTranslation().getDistance(RobotContainer.SWERVE.getPose().getTranslation());
        timestampSeconds = botpose.timestampSeconds;

        // if we see one tag, close to the pose, not far off the swerve pose, and going slowly
        // if (tagCount == 1
        // && avgTagDistM < 2
        // && distanceError < .25
        // && Math.abs(RobotContainer.SWERVE.getCurrentRobotVelocity()) < .25)
        // {
        // // lightly use the xy
        // xyStds = 2.0;
        // degStds = 99999;
        // }
        // // if we see 2 tags
        // else
        if (tagCount == 2)
        {
            if (avgTagDistM > TWO_TAG_DISTANCE_MIN && avgTagDistM < TWO_TAG_DISTANCE_MAX)
            {
                // linearly convert distance to xy std devs
                xyStds = Util.map(avgTagDistM,
                        TWO_TAG_DISTANCE_MIN, TWO_TAG_DISTANCE_MAX,
                        TWO_TAG_XY_STDS_MIN, TWO_TAG_XY_STDS_MAX);
                degStds = 10.0;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }

        // TODO
        // - if we trust the gyro more, make degStds to something crazy high 99999
        // - scale the vision x and y standard deviation by distance from the tag

        // seed our heading/pose first time we get valid data
        if (firstUpdate && isPrimaryCamera)
        {
            RobotContainer.SWERVE.seedFieldRelative(poseRobotUnfiltered);
            firstUpdate = false;
        }

        // Filter: Only measurements within X distance of current pose estimate (wpilib recommends)
        // if (distanceError < DISTANCE_ERROR_MAX)

        {
            poseRobotFiltered = poseRobotUnfiltered;

            RobotContainer.SWERVE.addVisionMeasurement(
                    poseRobotFiltered,
                    timestampSeconds,
                    VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
        }

        return true;
    }

}