package frc.team670.robot.commands.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.drivebase.MoveToPose;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.subsystems.drivebase.DriveBase;

/**
 * AutoAlign - autonomously moves the robot to a given target. If no target is given, it moves to
 * the closest one.`
 */
public class AutoAlign extends CommandBase implements MustangCommand {
    private DriveBase driveBase;
    private MoveToPose moveCommand;
    private int goal;
    private List<Pose2d> targets = new ArrayList<>(12);
    private Direction direction;

    // MustangController controller;
    // private final int CONTROLLER_RIGHT = 90;
    // private final int CONTROLLER_LEFT = 90 + 180;

    public static enum Direction {
        CLOSEST, LEFT, RIGHT
    }

    public AutoAlign(DriveBase driveBase, Direction direction) {
        this.driveBase = driveBase;
        // this.controller = controller;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        if (targets.isEmpty())
            loadTargets();

        goal = getClosestTargetIndex(driveBase.getPose());

        // boolean backUp = false;
        alterGoal(direction);

        Pose2d goalPose = targets.get(goal);
        SmartDashboard.putString("AUTOALIGN: END POSE",
                String.format("(%.2f, %.2f)", goalPose.getX(), goalPose.getY()));

        this.moveCommand = new MoveToPose(driveBase, goalPose);
        MustangScheduler.getInstance().schedule(moveCommand, driveBase);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return this.moveCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        this.moveCommand.end(interrupted);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

    private void loadTargets() {
        for (Pose2d p : FieldConstants.Grids.scoringPoses)
            targets.add(FieldConstants.allianceOrientedAllianceFlip(p));
        for (Pose2d p : FieldConstants.LoadingZone.IntakePoses)
            targets.add(FieldConstants.allianceOrientedAllianceFlip(p));

        for (int i = 0; i < targets.size(); i++) {
            Pose2d p = targets.get(i);
            SmartDashboard.putString(i + ": " + p.toString(),
                    String.format("(%.2f, %.2f)", p.getX(), p.getY()));
        }

    }

    private int getClosestTargetIndex(Pose2d robotPose) {
        int closest = 0;

        for (int i = 0; i < targets.size(); i++) {
            Translation2d robot = robotPose.getTranslation();
            double distanceToI = robot.getDistance(targets.get(i).getTranslation());
            double distanceToClosest = robot.getDistance(targets.get(closest).getTranslation());
            // SmartDashboard.putNumber(i + "'th Dist: ", distanceToI);
            // SmartDashboard.putNumber(closest + ": closest Dist: ", distanceToClosest);
            closest = distanceToI < distanceToClosest ? i : closest;
        }
        return closest;
    }

    private void alterGoal(Direction d) {
        int initGoal = goal;
        int allianceColorFlip = DriverStation.getAlliance() == Alliance.Blue ? 1 : -1;
        if (initGoal <= 8) {    // in grid
            if (direction == Direction.RIGHT) goal -= allianceColorFlip;
            else goal += allianceColorFlip;
        } else {    // intake zones
            if (direction == Direction.RIGHT) goal += allianceColorFlip;
            else goal -= allianceColorFlip;
        }
        // goal += n;
        if (initGoal <= 8 && goal > 8)
            goal = 8;
        else if (initGoal >= 9 && goal < 9)
            goal = 9;
        checkGoalInBounds();
    }


    private void checkGoalInBounds() {
        if (goal < 0)
            goal = 0;
        else if (goal > targets.size() - 1)
            goal = targets.size() - 1;
    }
    // private void scheduleNewMove(int goal) {
    // // if tries to go over/under, stays and doesn't do anything
    // if (goal < 0)
    // goal = 0;
    // else if (goal > 11)
    // goal = 11;
    // else {
    // moveCommand.cancel();
    // Pose2d goalPose = targets.get(goal);
    // moveCommand = new MoveToPose(driveBase, goalPose, true);
    // MustangScheduler.getInstance().schedule(moveCommand, driveBase);
    // }
    // }
}
