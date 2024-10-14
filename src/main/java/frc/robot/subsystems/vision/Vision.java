package frc.robot.subsystems.vision;

import java.util.EnumSet;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.team4646.SmartSubsystem;
import frc.team4646.drivers.LimelightHelpers;

public class Vision extends SmartSubsystem
{
    /** Order must match Limelight docs */
    public enum LEDMode
    {
        PIPELINE,
        OFF,
        BLINK,
        ON
    }

    /** Order must match Limelight docs */
    public enum CamMode
    {
        VISION_PROCESSOR,
        DRIVER_CAMERA
    }

    /** pipeline configuration, order must match configurtion on Limelight */
    public enum VisionPipeline
    {
        LOCALIZATION,
        APRILTAG,
        VISIONTAPE,
        DETECTOR,
        NONE,
    }

    public static class DataCache
    {
        public VisionPipeline desiredPipeline = VisionPipeline.NONE;
        public VisionPipeline selectedPipeline = VisionPipeline.NONE;
        public boolean seesTarget = false;

        public double prevHeartbeat = 1.0;
        public boolean isConnected;
        public boolean dashCreated;
    }

    private final PipelineAprilTag aprilTag;
    private final PipelineVisionTape visionTape;
    private final PipelineDetector detector;
    private final PipelineLocalization localization;
    private final Field2d field;

    // private final double fieldMap_yOffset = 4.0;
    // private final double fieldMap_xOffset = fieldMap_yOffset * (1654.0 / 802.0);

    private final String limelightName;

    private final NetworkTable table;
    private final DataCache cache = new DataCache();

    private int networkTableListenerID = -1;
    private boolean processUpdates = true;

    private final boolean isPrimaryCamera;

    /**
     * New Vision processing subsystem
     * 
     * @param limelightName
     *            name of camera data on network tables
     * @param pipeline
     *            which pipeline to start in, see {@link VisionPipeline}
     * @param isPrimaryCamera
     *            if true will seed the robot position on the field
     */
    public Vision(String limelightName, VisionPipeline pipeline, boolean isPrimaryCamera)
    {
        this.limelightName = limelightName;
        table = LimelightHelpers.getLimelightNTTable(limelightName);
        field = new Field2d();

        if (RobotBase.isSimulation())
        {
            LimelightHelpers.getLimelightNTTableEntry(limelightName, "hb").setDouble(cache.prevHeartbeat + 1);
            LimelightHelpers.getLimelightNTTableEntry(limelightName, "getpipe").setInteger(cache.desiredPipeline.ordinal());
            LimelightHelpers.getLimelightNTTableEntry(limelightName, "json").setString(Double.toString(cache.prevHeartbeat));
            var poseEntry = LimelightHelpers.getLimelightNTTableEntry(limelightName, "botpose_wpiblue");
            poseEntry.setDoubleArray(new Double[]
            {
                    6.0, 2.0, // xy
                    0.0, 0.0, 0.0,
                    0.0, // rotation
                    100.0, // latency
                    2.0, // tags
                    1.0, // span
                    2.0, // dist
                    0.35 // area
            });
            LimelightHelpers.getLimelightNTTableEntry(limelightName, "ta").setDouble(0.35);
        }
        aprilTag = new PipelineAprilTag();
        visionTape = new PipelineVisionTape();
        detector = new PipelineDetector();
        localization = new PipelineLocalization(isPrimaryCamera);

        setPipeline(pipeline);
        cacheSensors();
        createDashboard();

        this.isPrimaryCamera = isPrimaryCamera;
    }

    /** See https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api. */
    @Override
    public void cacheSensors()
    {
        if (RobotBase.isSimulation())
        {
            LimelightHelpers.getLimelightNTTableEntry(limelightName, "hb").setDouble(cache.prevHeartbeat + 1);
            LimelightHelpers.getLimelightNTTableEntry(limelightName, "getpipe").setInteger(cache.desiredPipeline.ordinal());
            LimelightHelpers.getLimelightNTTableEntry(limelightName, "json").setString(Double.toString(cache.prevHeartbeat));
        }

        double heartbeat = LimelightHelpers.getLimelightNTTableEntry(limelightName, "hb").getDouble(0.0);
        cache.isConnected = heartbeat > cache.prevHeartbeat;
        cache.prevHeartbeat = heartbeat;

        cache.selectedPipeline = VisionPipeline.values()[(int) LimelightHelpers.getCurrentPipelineIndex(limelightName)];

        if (cache.selectedPipeline == VisionPipeline.APRILTAG)
        {
            visionTape.clear();
            detector.clear();
            cache.seesTarget = aprilTag.process(limelightName);
            field.setRobotPose(aprilTag.botPose.toPose2d());
        }
        else if (cache.selectedPipeline == VisionPipeline.DETECTOR)
        {
            aprilTag.clear();
            visionTape.clear();
            cache.seesTarget = detector.process(limelightName);
            field.setRobotPose(RobotContainer.SWERVE.getState().Pose);
        }
        else if (cache.selectedPipeline == VisionPipeline.VISIONTAPE)
        {
            aprilTag.clear();
            detector.clear();
            cache.seesTarget = visionTape.process(limelightName);
            field.setRobotPose(RobotContainer.SWERVE.getState().Pose);
        }
    }

    private void onNetworkTableChanged()
    {
        if (cache.selectedPipeline == VisionPipeline.LOCALIZATION)
        {
            cache.seesTarget = localization.process(limelightName);
            field.setRobotPose(localization.poseRobotFiltered);
        }
    }

    @Override
    public void onEnable(boolean isAutonomous)
    {
    }

    @Override
    public void onDisable()
    {
    }

    @Override
    public void whileDisabled()
    {
        if (cache.isConnected)
        {
            // fix for slow limelight boot
            if (cache.desiredPipeline != cache.selectedPipeline)
            {
                LimelightHelpers.setPipelineIndex(limelightName, cache.desiredPipeline.ordinal());
                cache.selectedPipeline = cache.desiredPipeline;
            }

            if (cache.selectedPipeline == VisionPipeline.LOCALIZATION)
            {
                if (networkTableListenerID < 0)
                {
                    networkTableListenerID = table.addListener("json", EnumSet.of(Kind.kValueAll), new Listener());
                }
            }

            createDashboard();
        }
    }

    private class Listener implements TableEventListener
    {
        @Override
        public void accept(NetworkTable table, String key, NetworkTableEvent event)
        {
            if (key.equals("json"))
            {
                if (processUpdates)
                {
                    onNetworkTableChanged();
                }
            }
        }
    }

    public boolean foundAprilTag()
    {
        return aprilTag.primaryID != -1;
    }

    public boolean foundVisionTape()
    {
        return !foundAprilTag() && cache.selectedPipeline == VisionPipeline.VISIONTAPE && cache.seesTarget; // TODO This might be inverted
    }

    public boolean foundGamePiece()
    {
        return detector.classID != "";
    }

    public void setLEDs(boolean on)
    {
        if (on)
        {
            LimelightHelpers.setLEDMode_ForceOn(limelightName);
        }
        else
        {
            LimelightHelpers.setLEDMode_ForceOff(limelightName);
        }
    }

    public void setPipeline(VisionPipeline pipeline)
    {
        cache.desiredPipeline = pipeline;
    }

    /**
     * swaps between vison tape and AprilTag pipelines,
     * 
     * @return the Pipeline it swaps to
     **/
    public VisionPipeline swapPipeline()
    {
        VisionPipeline otherPipeline;
        if (cache.selectedPipeline == VisionPipeline.APRILTAG)
        {
            otherPipeline = VisionPipeline.VISIONTAPE;
        }
        else
        {
            otherPipeline = VisionPipeline.APRILTAG;
        }
        setPipeline(otherPipeline);
        return otherPipeline;
    }

    /** @return the cached apriltag data, see PipelineAprilTag for fields **/
    public PipelineAprilTag getApriltagPositionData()
    {
        return aprilTag;
    }

    /** @return the cached visionTape data, see PipelineBasicTargeting for fields **/
    public PipelineBasicTargeting getVisionTapePosition()
    {
        return visionTape;
    }

    public PipelineBasicTargeting getGamePiecePosition()
    {
        return detector;
    }

    public Pose2d getPose()
    {
        return localization.poseRobotFiltered;
    }

    public double getTagCount()
    {
        return localization.tagCount;
    }

    public double getTagDistance()
    {
        return localization.avgTagDistM;
    }

    public boolean isTargetPresent()
    {
        return cache.seesTarget;
    }

    public double getTimestamp()
    {
        return localization.timestampSeconds;
    }

    public VisionPipeline getCurrentPipeline()
    {
        return VisionPipeline.values()[(int) LimelightHelpers.getCurrentPipelineIndex(limelightName)];
    }

    @Override
    public void runTests()
    {

    }

    public void createDashboard()
    {
        if (cache.dashCreated == false)
        {
            cache.dashCreated = true;
            if (Constants.TUNING.VISION)
            {
                ShuffleboardTab tab = Shuffleboard.getTab(limelightName);

                SmartDashboard.putString("pipelineName", limelightName);
                SmartDashboard.putNumber("pipeline index", (int) LimelightHelpers.getCurrentPipelineIndex(limelightName));

                if (cache.desiredPipeline != VisionPipeline.NONE && cache.desiredPipeline != VisionPipeline.LOCALIZATION)
                {
                    tab.addBoolean("Target", () -> cache.seesTarget).withPosition(0, 0);
                    tab.addBoolean("lined up", () -> Math.abs(visionTape.xDegrees) < 1).withPosition(0, 1);
                    tab.addNumber("Pipeline", () -> cache.selectedPipeline.ordinal()).withPosition(0, 2);
                    tab.addString("Direction", () -> visionTape.desiredDirection).withPosition(0, 3);
                    ShuffleboardLayout gridXY = tab.getLayout("Retrotape", BuiltInLayouts.kGrid).withPosition(1, 0).withSize(2, 2)
                            .withProperties(Map.of("Number of columns", 2, "Number of rows", 2));
                    gridXY.addNumber("X", () -> visionTape.xDegrees).withPosition(0, 0);
                    gridXY.addNumber("Y", () -> visionTape.yDegrees).withPosition(1, 0);
                    gridXY.addNumber("xOffset", () -> visionTape.xOffset).withPosition(0, 1);
                    gridXY.addNumber("yOffset", () -> visionTape.yOffset).withPosition(1, 1);
                    ShuffleboardLayout gridAT = tab.getLayout("AprilTag", BuiltInLayouts.kGrid).withPosition(3, 0).withSize(2, 2)
                            .withProperties(Map.of("Number of columns", 2, "Number of rows", 2));
                    gridAT.addNumber("AprilTag: X", () -> aprilTag.botPose.getX()).withPosition(0, 0);
                    gridAT.addNumber("AprilTag: Y", () -> aprilTag.botPose.getY()).withPosition(1, 0);
                    gridAT.addNumber("AprilTag: Z rotation", () -> aprilTag.botPose.getRotation().getZ()).withPosition(0, 1);
                    gridAT.addNumber("AprilTag: ID", () -> aprilTag.primaryID).withPosition(1, 1);
                    ShuffleboardLayout grid = tab.getLayout("Detector", BuiltInLayouts.kGrid).withPosition(5, 0).withSize(2, 4)
                            .withProperties(Map.of("Number of columns", 2, "Number of rows", 4));
                    grid.addString("Target", () -> detector.classID).withPosition(0, 0);
                    grid.addNumber("Area", () -> detector.area).withPosition(1, 0);
                    grid.addNumber("X", () -> detector.xDegrees).withPosition(0, 1);
                    grid.addNumber("Y", () -> detector.yDegrees).withPosition(1, 1);
                    grid.addNumber("Horizontal", () -> detector.horizontal).withPosition(0, 2);
                    grid.addNumber("Vertical", () -> detector.vertical).withPosition(1, 2);
                    grid.addNumber("Ratio", () -> (detector.vertical / detector.horizontal)).withPosition(0, 3);
                }
                if (cache.desiredPipeline == VisionPipeline.LOCALIZATION)
                {
                    tab.add(field).withPosition(0, 0).withSize(5, 3);
                    tab.addNumber("BP X", () -> localization.poseRobotUnfiltered.getX()).withPosition(5, 0);
                    tab.addNumber("BP Y", () -> localization.poseRobotUnfiltered.getY()).withPosition(5, 1);
                    tab.addNumber("BP Heading", () -> localization.poseRobotUnfiltered.getRotation().getDegrees()).withPosition(5, 2).withSize(1, 1);
                    tab.addNumber("BP Tag Count", () -> localization.tagCount).withPosition(5, 3).withSize(1, 1);
                    tab.addNumber("BP Avg Dist", () -> localization.avgTagDistM).withPosition(5, 4).withSize(1, 1);

                    tab.addNumber("Filtered X", () -> localization.poseRobotFiltered.getX()).withPosition(6, 0);
                    tab.addNumber("Filtered Y", () -> localization.poseRobotFiltered.getY()).withPosition(6, 1);
                    tab.addNumber("Filtered Heading", () -> localization.poseRobotFiltered.getRotation().getDegrees()).withPosition(6, 2).withSize(1, 1);

                    tab.addNumber("Swerve X", () -> RobotContainer.SWERVE.getPose().getX()).withPosition(7, 0);
                    tab.addNumber("Swerve Y", () -> RobotContainer.SWERVE.getPose().getY()).withPosition(7, 1);
                    tab.addNumber("Swerve Heading", () -> RobotContainer.SWERVE.getPose().getRotation().getDegrees()).withPosition(7, 2).withSize(1, 1);

                    tab.addNumber("Distance Error", () -> localization.distanceError).withPosition(8, 0).withSize(1, 1);
                    tab.addBoolean("First Update", () -> localization.firstUpdate).withPosition(8, 1).withSize(1, 1);
                    tab.addBoolean("Target", () -> cache.seesTarget).withPosition(8, 2).withSize(1, 1);
                }
            }
        }
    }
}
