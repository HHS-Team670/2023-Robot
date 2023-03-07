package frc.team670.robot.commands.drivebase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.subsystems.DriveBase;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DriveBase m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(DriveBase drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        double angle = m_rotationSupplier.getAsDouble();
        double xPos = m_translationXSupplier.getAsDouble();
        double yPos = m_translationYSupplier.getAsDouble();

        // if (Math.abs(xPos) <= 0.1 && Math.abs(yPos) <= 0.1) {
        // angle = prevAngle;
        // }
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xPos,
                        yPos,
                        angle,
                        m_drivetrainSubsystem.getGyroscopeRotation()));

        // prevAngle = angle;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
