package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.robot.constants.RobotConstants;

public class LED extends LEDSubsystem {
    private static LED mInstance;
    private LEDColor allianceColor;

    public static synchronized LED getInstance() {
        mInstance = mInstance == null ? new LED() : mInstance;
        return mInstance;
    }

    public LED() {
        super(RobotConstants.Led.kPort, RobotConstants.Led.kStartIndex, RobotConstants.Led.kEndindex);
    }

    public void setCubeColor() {
        solidhsv(LEDColor.SEXY_PURPLE);
    }

    public void setConeColor() {
        solidhsv(LEDColor.SEXY_YELLOW);
    }

    public void setAllianceColors(LEDColor alliance) {
        this.allianceColor = alliance;
    }

    public LEDColor getAllianceColor() {
        return allianceColor;
    }

}