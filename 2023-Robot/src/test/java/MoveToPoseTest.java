import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team670.robot.constants.FieldConstants;

public class MoveToPoseTest {
    @Test
    void printConstants() {
        System.out.println("------FIELD VALUES------");
        System.out.println("0, 0");
        System.out.println("0, " + FieldConstants.fieldWidth);
        System.out.println(FieldConstants.fieldLength + ", " + FieldConstants.fieldWidth);
        System.out.println(FieldConstants.fieldLength + ", 0");

        System.out.println("------COMPLEX LOW TRANSLATIONS------");
        System.out.println("ALLIANCE: " + DriverStation.getAlliance());
        for (Pose2d p : FieldConstants.Grids.scoringPoses) {
            p = FieldConstants.allianceFlip(p);
            System.out.println(String.format("%f, %f", p.getX(), p.getY()));
        }

        System.out.println("------LOADING ZONE SUBSTATIONS------");
        for (Pose2d p : FieldConstants.LoadingZone.IntakePoses) {
            p = FieldConstants.allianceFlip(p);
            System.out.println(String.format("%f, %f", p.getX(), p.getY()));
        }
        System.out.println("------OBSTACLE CORNERS------");
        for (Translation2d t : FieldConstants.obstacleCorners) {
            t = FieldConstants.allianceFlip(t);
            System.out.println(String.format("%f, %f", t.getX(), t.getY()));
        }
        System.out.println("------------------------------------");
    }

    @Test
    void moveToFirstCone() {
        MoveToPoseSkeleton m = new MoveToPoseSkeleton(new Pose2d(13, 1, new Rotation2d()),
                new Pose2d(0, 0, new Rotation2d()));
        List<State> trajStates = m.traj.getStates();
        System.out.println("------MOVE TO FIRST CONE------");
        System.out.println("------MOVE TO FIRST CONE: Start Pose------");
        System.out.println(String.format("%f, %f", m.startPose.getX(), m.startPose.getY()));
        System.out.println("------MOVE TO FIRST CONE: End Pose------");
        System.out.println(String.format("%f, %f, %f degreees", m.endPose.getX(), m.endPose.getY(),
                m.endPose.getRotation().getDegrees()));
        System.out.println("------MOVE TO FIRST CONE: Trajectory------");
        trajStates.forEach(
                (state) -> {
                    System.out.println(String.format("%f, %f", state.poseMeters.getX(), state.poseMeters.getY()));
                });
    }

    class MoveToPoseSkeleton {
        Pose2d endPose;
        Pose2d startPose;
        PathPlannerTrajectory traj;

        public MoveToPoseSkeleton(Pose2d startPose, Pose2d endPose) {
            this.startPose = startPose;
            this.endPose = endPose;
            traj = PathPlanner.generatePath(new PathConstraints(1, 0.5), calcStartPoint(), calcEndPoint());

        }

        // calcs start point and points directly towards end point
        private PathPoint calcStartPoint() {
            double dx, dy;
            dx = endPose.getX() - startPose.getX();
            dy = endPose.getY() - startPose.getY();
            System.out.println("dy: " + dy);
            System.out.println("dx: " + dx);
            System.out.println("angle: " + Math.toDegrees(Math.atan(dy / dx)));
            return new PathPoint(startPose.getTranslation(), new Rotation2d(dx, dy));
        }

        // end point where robot faces end Pose
        private PathPoint calcEndPoint() {
            return new PathPoint(endPose.getTranslation(), endPose.getRotation().rotateBy(new Rotation2d(Math.PI)),
                    endPose.getRotation().rotateBy(new Rotation2d(Math.PI)));
        }
    }
}
