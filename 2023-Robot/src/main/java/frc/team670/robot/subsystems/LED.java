package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.utils.LEDColor;
import frc.team670.mustanglib.utils.Logger;

public class LED extends LEDSubsystem {

    private LEDColor allianceColor, oppositeAllianceColor;
    private Claw claw;

    public LED(int port, int startIndex, int endIndex, Claw claw) {
        super(port, startIndex, endIndex);
        this.claw = claw;
    }

    public void setColorPurple() {
        // Logger.consoleLog("LED COPLOR BEING SET TO PURPLE IN LED SUBSYSTEM Pokjipguhygxdtfyu08tfyui[u/hyfygyuhGy/HIYUg'uhyG?SaJHKVGUYITDFTIGFGYF<CGUOF<CDHXDGDFTYO*GTRDYTY&UY:RDTYUTDP");
        solidrgb(LEDColor.SEXY_PURPLE); // does not need to be changed
    }

    public void setColorYellow() {
        solidrgb(LEDColor.SEXY_YELLOW); // does not need to be changed
    }

    public void setAllianceColors(LEDColor alliance){
        this.allianceColor = alliance;
    }

    @Override
    public void mustangPeriodic(){
        if(allianceColor == LEDColor.WHITE){
            blink(LEDColor.WHITE);
        }
        
        if(DriverStation.isEnabled()){

            if(claw.isFull()){

                solidhsv(allianceColor);
            }
        }
        
        super.mustangPeriodic();
    }
}