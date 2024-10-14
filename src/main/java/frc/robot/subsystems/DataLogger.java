package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.team4646.SmartSubsystem;

/**
 * Portions from https://github.com/TheGreenMachine/Zero
 */
public class DataLogger extends SmartSubsystem
{
    private ArrayList<DataLoggerIO> dataLoggers;

    public DataLogger()
    {
        dataLoggers = new ArrayList<>();

        var logFileDir = "/home/lvuser/datalogs/";

        // if there is a USB drive use it
        if (Files.exists(Path.of("/media/sda1")))
        {
            logFileDir = "/media/sda1/";
        }
        if (RobotBase.isSimulation())
        {
            if (System.getProperty("os.name").toLowerCase().contains("win"))
            {
                logFileDir = System.getenv("temp") + "\\";
            }
            else
            {
                logFileDir = System.getProperty("user.dir") + "/";
            }
        }

        // start logging
        DataLogManager.logNetworkTables(false);
        DataLogManager.start(logFileDir, "");
        if (RobotBase.isReal())
        {
            cleanLogFiles();
        }
        DriverStation.startDataLog(DataLogManager.getLog(), false);
    }

    public void addLogger(DataLoggerIO dataLogger)
    {
        dataLoggers.add(dataLogger);
    }

    @Override
    public void periodic()
    {
        for (DataLoggerIO dataLogger : dataLoggers)
        {
            dataLogger.update();
        }
    }

    /**
     * needs to be called after log start
     */
    private static void cleanLogFiles()
    {
        var logPath = DataLogManager.getLogDir();
        long day = 1000 * 60 * 60 * 24;
        long now = System.currentTimeMillis();
        try (Stream<Path> stream = Files.list(Paths.get(logPath)))
        {
            var files = stream
                    .filter(file -> !Files.isDirectory(file)) // No folders
                    .filter(file -> file.toString().endsWith(".wpilog")) // Only .wpiLog
                    .filter(file -> file.toString().chars().filter(ch -> ch == '_').count() == 2)
                    .filter(file ->
                    {
                        try
                        {
                            return now - Files.getLastModifiedTime(file).toMillis() > day;
                        } catch (IOException e)
                        {
                            return false;
                        }
                    })
                    .collect(Collectors.toSet());
            for (var file : files)
            {
                System.out.println("Deleting: " + file);
                Files.delete(file);
            }
        } catch (IOException e)
        {
            System.out.print(e);
        }
    }

    public static interface DataLoggerIO
    {
        public void update();
    }

    /**
     * Create a logger for a Vision subsystem
     */
    public static class SwerveDL implements DataLoggerIO
    {
        private DoubleLogEntry x, y, rotation;
        private final SwerveDriveSubsystem swerveSub;

        public SwerveDL(SwerveDriveSubsystem swerveSub)
        {
            this.swerveSub = swerveSub;
            String name = "Swerve";

            x = new DoubleLogEntry(DataLogManager.getLog(), name + "/X");
            y = new DoubleLogEntry(DataLogManager.getLog(), name + "/Y");
            rotation = new DoubleLogEntry(DataLogManager.getLog(), name + "/Rotation");
        }

        public void update()
        {
            long time = RobotController.getFPGATime();
            Pose2d pose = swerveSub.getPose();
            x.append(pose.getX(), time);
            y.append(pose.getY(), time);
            rotation.append(pose.getRotation().getDegrees(), time);
        }
    }

    /**
     * Create a logger for a Vision subsystem
     */
    public static class VisionDL implements DataLoggerIO
    {
        private DoubleLogEntry x, y, rotation, tagCount, distance, timestamp;
        private BooleanLogEntry targetPresent;
        private final Vision visionSub;

        public VisionDL(String name, Vision visionSub)
        {
            this.visionSub = visionSub;

            x = new DoubleLogEntry(DataLogManager.getLog(), name + "/X");
            y = new DoubleLogEntry(DataLogManager.getLog(), name + "/Y");
            rotation = new DoubleLogEntry(DataLogManager.getLog(), name + "/Rotation");
            tagCount = new DoubleLogEntry(DataLogManager.getLog(), name + "/TagCount");
            distance = new DoubleLogEntry(DataLogManager.getLog(), name + "/Distance");
            targetPresent = new BooleanLogEntry(DataLogManager.getLog(), name + "/TargetPresent");
            timestamp = new DoubleLogEntry(DataLogManager.getLog(), name + "/Timestamp");
        }

        public void update()
        {
            long time = RobotController.getFPGATime();
            Pose2d pose = visionSub.getPose();
            x.append(pose.getX(), time);
            y.append(pose.getY(), time);
            rotation.append(pose.getRotation().getDegrees(), time);
            tagCount.append(visionSub.getTagCount(), time);
            distance.append(visionSub.getTagDistance(), time);
            targetPresent.append(visionSub.isTargetPresent(), time);
            timestamp.append(visionSub.getTimestamp(), time);
        }
    }

}
