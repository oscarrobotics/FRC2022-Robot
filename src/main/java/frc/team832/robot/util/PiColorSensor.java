package frc.team832.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;

public class PiColorSensor {
    private static PiColorSensor INSTANCE;
    
    public static PiColorSensor getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new PiColorSensor();
        }

        return INSTANCE;
    }

    private final NetworkTableInstance m_ntInst = NetworkTableInstance.getDefault();
    private NetworkTableEntry m_proximityEntry = m_ntInst.getEntry("proximity1");
    private NetworkTableEntry m_colorEntry = m_ntInst.getEntry("color1");

    private long m_lastProxChange = 0;

    public boolean dataFresh() {
        boolean hasUpdated =  m_proximityEntry.getLastChange() != m_lastProxChange;
        m_lastProxChange = m_proximityEntry.getLastChange();
        return hasUpdated;
    }

    public double getProximity() {
        return m_proximityEntry.getDouble(-1);
    }

    public Color getColor() {
        var colorArr = m_colorEntry.getDoubleArray(new double[4]);
        return new Color(colorArr[0], colorArr[1], colorArr[2]);
    }
}
