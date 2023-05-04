package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.robot.constants.RobotConstants;

public class LED extends LEDSubsystem {
    private static LED mInstance;
    private LEDColor allianceColor;
    private int prevPath = -1;

    public static synchronized LED getInstance() {
        if(mInstance==null)
            mInstance=new LED();
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

    public void updateAutonPathColor(int selectedPath) {
        if (prevPath == selectedPath) return;
        
        switch (selectedPath) {
            case 0:
                blinkhsv(LEDColor.LIGHT_BLUE);
                break;
            case 1:
                blinkhsv(LEDColor.SEXY_YELLOW);
                break;
            case 2:
                blinkhsv(LEDColor.SEXY_PURPLE);
                break;
            case 3:
                blinkhsv(LEDColor.GREEN);
                break;
            case 4:
                animatedRainbow(false, 10, 10);
                break;
            case 5:
                blinkhsv(LEDColor.PINK);
                break;
            case 6:
                animatedMustangRainbow(10, 10);
                break;
            default:
                animatedRainbow(false, 10, 10);
        }
        prevPath = selectedPath;
    }

}