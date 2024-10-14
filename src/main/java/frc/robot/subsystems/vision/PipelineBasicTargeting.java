package frc.robot.subsystems.vision;

public abstract class PipelineBasicTargeting
{
    public double area; // Target visionTape.area (0% of image to 100% of image)
    public double xDegrees; // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    public double yDegrees; // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    public String classID;
    public double horizontal;
    public double vertical;
    public String desiredDirection;
    public double xOffset, yOffset;

    public PipelineBasicTargeting()
    {
        clear();
    }

    public void clear()
    {
        area = -1.0;
        xDegrees = -1.0;
        yDegrees = -1.0;
        horizontal = -1.0;
        vertical = -1.0;
        classID = "";
        desiredDirection = "";
        xOffset = -1.0;
        yOffset = -1.0;
    }

    public abstract boolean process(String pipelineName);
}