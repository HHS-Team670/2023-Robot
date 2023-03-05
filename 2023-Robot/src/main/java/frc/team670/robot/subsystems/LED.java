package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.utils.LEDColor;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.commands.claw.ClawIntake;

public class LED extends LEDSubsystem {

    private LEDColor allianceColor;
    private Claw claw;

    public LED(int port, int startIndex, int endIndex) {
        super(port, startIndex, endIndex);
    }

    public void setColorPurple() {
        Logger.consoleLog("LED COPLOR BEING SET TO PURPLE IN LED SUBSYSTEM Pokjipguhygxdtfyu08tfyui[u/hyfygyuhGy/HIYUg'uhyG?SaJHKVGUYITDFTIGFGYF<CGUOF<CDHXDGDFTYO*GTRDYTY&UY:RDTYUTDP");
        solidrgb(LEDColor.SEXY_PURPLE); // does not need to be changed
    }

    public void setColorYellow() {
        solidrgb(LEDColor.SEXY_YELLOW); // does not need to be changed
    }

    public void setAllianceColors(LEDColor alliance) {
        this.allianceColor = alliance;
    }

    @Override
    public void mustangPeriodic() {
        if(claw.isFull()){
            solidhsv(allianceColor);
        }
    }
}