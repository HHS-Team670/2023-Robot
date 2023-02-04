package frc.team670.robot.commands.arm;

import java.awt.geom.Point2D;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class MoveArmTest {

    public void testMoveArm() {
        Arm arm = new Arm();
        for (int startID = 0; startID < Arm.NUM_STATES; startID++) {
            for (int endID = 0; endID < Arm.NUM_STATES; endID++) {
                // CommandBase moveArmToStart = new MoveDirectlyToTarget(arm, ArmState.getVal(startID));
                //MustangScheduler.getInstance().schedule(moveArmToStart, arm);
                //moveArmToStart.ignoringDisable(true);
                
                ArmState start = ArmState.getVal(startID);
                ArmState end = ArmState.getVal(endID);
                MustangCommand moveToStart = new MoveDirectlyToTarget(start,arm);
                MustangScheduler.getInstance().schedule(moveToStart);
                moveToStart.setRunWhenDiabled(true);
                moveArmToStart.setRunWhenDiabled(true);
                moveArmToStart.start();
                while(!moveArmToStart.isCompleted()){
                    MustangScheduler.getInstance.run();
                }
                CommandGroup moveArm = new MoveToTarget(end,arm);

                Logger.consoleLog("Path from " + start + " to " + end + " = " + Arm.getValidPath(start, end));
                MustangScheduler.getInstance().add(moveArm);
                moveArm.setRunWhenDiabled(true);
                while(!moveArm.isCompleted()){
                    MustangScheduler.getInstance.run();
                }
                try {
                    assertTrue(arm.hasReachedTargetPosition());
                } catch (Exception e) {
                    // TODO: handle exception
                }
                // moveArmToStart.
            }
        }
    }

}