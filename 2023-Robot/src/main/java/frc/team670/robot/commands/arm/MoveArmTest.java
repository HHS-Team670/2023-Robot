package frc.team670.robot.commands.arm;

import static org.junit.Assert.assertEquals;
import java.awt.geom.Point2D;
import org.junit.Test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class MoveArmTest {

    public void testMoveArm() {

        Arm arm = new Arm();
        for (int startID = 0; startID < Arm.NUM_STATES; startID++) {
            for (int endID = 0; endID < Arm.NUM_STATES; endID++) {
                CommandBase moveArmToStart = new MoveDirectlyToTarget(arm, ArmState.getVal(startID));
                MustangScheduler.getInstance().schedule(moveArmToStart, arm);
                moveArmToStart.ignoringDisable(true); 
                // moveArmToStart.
            }
        }
    }

}