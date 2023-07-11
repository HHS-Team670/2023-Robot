// package frc.team670.robot.commands.leds;

// import frc.team670.mustanglib.commands.MustangCommand;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
// import frc.team670.mustanglib.utils.Logger;
// import frc.team670.robot.subsystems.LED;
// import frc.team670.robot.subsystems.Claw;

// import java.util.HashMap;
// import java.util.Map;

// import edu.wpi.first.wpilibj2.command.InstantCommand;

// public class SetIntakeCube extends InstantCommand implements MustangCommand {
//     private LED led;
//     private Claw claw;
//     HashMap<MustangSubsystemBase, HealthState> healthreqs = new HashMap<MustangSubsystemBase, HealthState>();

//     public SetIntakeCube(LED led,Claw claw) {
//         this.led = led;
//         this.claw = claw;
//         addRequirements(led);
//         healthreqs.put(led, HealthState.GREEN);
//     }

//     public void initialize() {
        
//         led.setCubeColor();
//         claw.setGamePiece(Claw.GamePiece.CUBE);
//     }

//     @Override
//     public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
//         return healthreqs;
//     }

// }
