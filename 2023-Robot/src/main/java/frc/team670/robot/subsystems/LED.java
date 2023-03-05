package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.utils.LEDColor;
import frc.team670.mustanglib.utils.Logger;

public class LED extends LEDSubsystem {

    public LED(int port, int startIndex, int endIndex) {
        super(port, startIndex, endIndex);
    }

    public void setColorPurple() {
        // Logger.consoleLog("LED COPLOR BEING SET TO PURPLE IN LED SUBSYSTEM Pokjipguhygxdtfyu08tfyui[u/hyfygyuhGy/HIYUg'uhyG?SaJHKVGUYITDFTIGFGYF<CGUOF<CDHXDGDFTYO*GTRDYTY&UY:RDTYUTDP");
        solidrgb(LEDColor.SEXY_PURPLE); // does not need to be changed
    }

    public void setColorYellow() {
        solidrgb(LEDColor.SEXY_YELLOW); // does not need to be changed
    }
}