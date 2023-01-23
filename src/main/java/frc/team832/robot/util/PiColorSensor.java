package frc.team832.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

public class PiColorSensor {
    private static PiColorSensor INSTANCE;
    
    public static PiColorSensor getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new PiColorSensor();
        }

        return INSTANCE;
    }

    public class REVColor {
        public double r, g, b, a;

        public REVColor(double[] rgba) {
            if (rgba.length == 4) {
                r = rgba[0];
                g = rgba[1];
                b = rgba[2];
                a = rgba[3];
            }
        }
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

    public REVColor getColor() {
        var colorArr = m_colorEntry.getDoubleArray(new double[4]);
        return new REVColor(colorArr);
    }

    public boolean isBallPresent() {
        return dataFresh() && getProximity() >= 750;
    }

    public boolean isBallBlue() {
        return dataFresh() && getColor().b >= 4000;
    }

    public boolean isBallRed() {
        return !isBallBlue();
    }

    public boolean isAllianceRed() {
        return false;
        // return DriverStation.getAlliance() == DriverStation.Alliance.Red;
    }

    public boolean isAllianceBlue() {
        return true;
        // return DriverStation.getAlliance() == DriverStation.Alliance.Blue;
    }

    public boolean isCargoCorrectColor() {
        return isBallPresent() && (
            (isBallBlue() && isAllianceBlue()) ||
            (isBallRed() && isAllianceRed())
        );
    }
}
