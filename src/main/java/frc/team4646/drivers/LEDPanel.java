// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4646.drivers;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANMessageNotFoundException;
import edu.wpi.first.hal.util.UncleanStatusException;
import frc.team4646.SmartSubsystem;

/** Add your docs here. */
public class LEDPanel extends SmartSubsystem
{
    public static enum MODE
    {
        Off,
        MessageBoard,
        GIFs,
        AllianceRed,
        AllianceBlue
    }

    private static final int READ_TIMEOUT_MS = 1;

    protected final int messageID;
    private MODE modeCurrent;

    public LEDPanel()
    {
        messageID = 0; // TODO
        modeCurrent = MODE.Off;
    }

    public void setMode(LEDPanel.MODE mode)
    {
        this.modeCurrent = mode;
    }

    @Override
    public void updateHardware()
    {
        // TODO if write is needed, send data

        // create data buffer
        byte data[] = new byte[8];

        write(data);
    }

    private void write(byte[] data)
    {
        try
        {
            CANJNI.FRCNetCommCANSessionMuxSendMessage(messageID, data, CANJNI.CAN_SEND_PERIOD_NO_REPEAT);
        } catch (UncleanStatusException e)
        {

        }
    }

    private byte[] read()
    {
        IntBuffer idBuffer = ByteBuffer.allocateDirect(4).asIntBuffer();
        idBuffer.clear();
        idBuffer.put(0, Integer.reverseBytes(messageID));
        ByteBuffer timestamp = ByteBuffer.allocate(4);
        byte[] response = null;
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < READ_TIMEOUT_MS)
        {
            try
            {
                response = CANJNI.FRCNetCommCANSessionMuxReceiveMessage(idBuffer, 0x1fffffff, timestamp);
                break;
            } catch (CANMessageNotFoundException e)
            {
            }
        }
        return response;
    }
}
