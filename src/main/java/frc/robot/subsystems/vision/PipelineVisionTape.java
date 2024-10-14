package frc.robot.subsystems.vision;

import frc.team4646.drivers.LimelightHelpers;

public class PipelineVisionTape extends PipelineBasicTargeting
{
    @Override
    public boolean process(String pipelineName)
    {
        xDegrees = LimelightHelpers.getTX(pipelineName);
        yDegrees = LimelightHelpers.getTY(pipelineName);
        area = LimelightHelpers.getTA(pipelineName);
        classID = "";
        horizontal = LimelightHelpers.getLimelightNTDouble(pipelineName, "thor");
        vertical = LimelightHelpers.getLimelightNTDouble(pipelineName, "tvert");
        xOffset = xDegrees + 25.1;
        yOffset = yDegrees + 7.5;
        if (Math.abs(xDegrees + 25.1) < 1.0)
        {
            desiredDirection = "";
        }
        else if (xDegrees > -25.1)
        {
            desiredDirection = "back ";
        }
        else
        {
            desiredDirection = "forward ";
        }
        if (Math.abs(yDegrees + 7.5) < 1.0)
        {
            desiredDirection += "";
        }
        else if (xDegrees > -25.1)
        {
            desiredDirection += "left ";
        }
        else
        {
            desiredDirection += "right ";
        }

        return LimelightHelpers.getTV(pipelineName);
    }

}
