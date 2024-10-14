package frc.robot.subsystems.vision;

import frc.team4646.drivers.LimelightHelpers;

public class PipelineDetector extends PipelineBasicTargeting
{
    @Override
    public boolean process(String pipelineName)
    {
        xDegrees = LimelightHelpers.getTX(pipelineName);
        yDegrees = LimelightHelpers.getTY(pipelineName);
        area = LimelightHelpers.getTA(pipelineName);
        classID = LimelightHelpers.getLimelightNTString(pipelineName, "tclass");
        horizontal = LimelightHelpers.getLimelightNTDouble(pipelineName, "thor");
        vertical = LimelightHelpers.getLimelightNTDouble(pipelineName, "tvert");
        xOffset = xDegrees + 24.1;
        yOffset = yDegrees + 7.5;

        return LimelightHelpers.getTV(pipelineName);
    }

}
