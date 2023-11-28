package frc.team670.robot.subsystems.drivebase;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.utils.SwervePoseEstimatorBase;
import frc.team670.robot.constants.FieldConstants;

public class SwervePoseEstimator extends SwervePoseEstimatorBase {

    public SwervePoseEstimator(SwerveDrive swerve) {
        super(swerve);
    }

    @Override
    protected Pose2d getAbsoluteFieldOrientedPoseFromAllianceOriented(Pose2d pose) {
        if (DriverStation.getAlliance() == Alliance.Red) {
            return FieldConstants.allianceOrientedAllianceFlip(pose);
        } else {
            return pose;
        }
    }
    

    @Override
    protected List<Pose2d> getTargets() {
        List<Pose2d> targets = new ArrayList<>();

        for (Translation2d p : FieldConstants.Grids.complexLowTranslations)
            targets.add(FieldConstants.allianceFlip(new Pose2d(p, new Rotation2d())));
        for (Pose2d p : FieldConstants.LoadingZone.IntakePoses)
            targets.add(FieldConstants.allianceFlip(p));
        return targets;
    }
    
}
