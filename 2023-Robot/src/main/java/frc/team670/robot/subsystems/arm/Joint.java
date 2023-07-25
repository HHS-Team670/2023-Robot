package frc.team670.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystemIO;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.arm.Joint.JointIO.JointIOInputs;

public class Joint extends SparkMaxRotatingSubsystem {
    
    private double offset = 0;
    private double orgTargetAngle = 0;
    protected JointIOInputs inputs;
    protected JointIO io;
    public Joint(JointIO io,LoggableInputs inputs) {
        super(io,inputs);
        this.io=io;
        this.inputs=(JointIOInputs)(super.getInputs());
        io.updateInputs(inputs);
    }

    @Override
    public boolean getTimeout() {
        return false;
    }

    @Override
    public void mustangPeriodic() {
        io.checkAlignment(inputs);
    }
    // @Override
    // public void setSystemTargetAngleInDegrees(double targetAngle) {
    //     orgTargetAngle = targetAngle;
    //     super.setSystemTargetAngleInDegrees(targetAngle);
    // }
    private void setOffset(double offset) {
        // if (Math.abs(offset) > RobotConstants.Arm.Elbow.kMaxOverrideDegreees) {
        //     this.offset = RobotConstants.Arm.Elbow.kMaxOverrideDegreees * this.offset
        //             / Math.abs(this.offset);
        // } else {
        //     this.offset = offset;
        // }
        // setSystemTargetAngleInDegrees(orgTargetAngle);

    }

    public void resetOffset() {
        setOffset(0);

    }
    public void resetPositionFromAbsolute() {
        io.resetPositionFromAbsolute(inputs);
    }
    public boolean hasReachedTargetPosition(){
        return io.hasReachedTargetPosition(inputs);
    }
  
    /**
     * Returns whether or not the relative position has been properly set from the absEncoder. When
     * resetPositionFromAbsolute() gets called, this will temporarily be false.
     */
    public boolean isRelativePositionSet(){
        return io.isRelativePositionSet();
    }

    public void addOffset(double offset) {
        setOffset(this.offset + offset);
        // setSystemTargetAngleInDegrees(orgTaroffset);
    }
    


    public static abstract class JointIO extends SparkMaxRotatingSubsystemIO{

        protected DutyCycleEncoder absEncoder;
        protected boolean hasSetAbsolutePosition = false;
        protected int counter = 0;
        protected double previousReading = 0.0;
        protected double calculatedRelativePosition = 0.0;
        protected boolean relativePositionIsSet = false;
        protected double errorCounter = 0;
        
        public JointIO(Config kConfig,int kAbsoluteEncoderID ) {
            super(kConfig);
            absEncoder = new DutyCycleEncoder(kAbsoluteEncoderID);

        }

        @AutoLog
        public static class JointIOInputs extends SparkMaxRotatingSubsystemIOInputs{
            public double absEncoderPos=0;
            public double mEncoderPositionUnadjusted=0;
            public double mRotatorPower=0;

            public void  updateFromSuper(){
                mEncoderPositionUnadjusted= super.mEncoderPositionUnadjusted;
                mRotatorPower=super.mRotatorPower;
            }
        }

        protected abstract void setEncoderPositionFromAbsolute(JointIOInputs inputs);

        

        public void checkAlignment(JointIOInputs inputs){
            if (!hasSetAbsolutePosition) { // before it's set an absolute position...

            if (Math.abs(previousReading - inputs.absEncoderPos) < 0.02 && inputs.absEncoderPos != 0.0) { // If the current
                                                                                  // reading is
                                                                                  // PRECISELY
                                                                                  // 0, then it's
                                                                                  // not valid.
                counter++; // increases the counter if the current reading is close enough to the
                           // last
                           // reading.
                           // We do this because when the absEncoder gets initialized, its reading
                           // fluctuates drastically at the start.
            } else {
                counter = 0;
                previousReading = inputs.absEncoderPos;
            }
            if (counter > 25) { // Once it's maintained a constant value for long enough...
                setEncoderPositionFromAbsolute(inputs);
                hasSetAbsolutePosition = true;
            }
        } else if (!relativePositionIsSet) {
            double position = inputs.mEncoderPositionUnadjusted;
            
            if (Math.abs(position - calculatedRelativePosition) < 0.5) {
                relativePositionIsSet = true;
            } else {
                super.mEncoder.setPosition(calculatedRelativePosition);
            }
            
        }
        }
         /**
     * Returns whether or not the relative position has been properly set from the absEncoder. When
     * resetPositionFromAbsolute() gets called, this will temporarily be false.
     */
    public boolean isRelativePositionSet() {
        return relativePositionIsSet;
    }

    /**
     * Public method to reset the position from the absolute position.
     */
    public void resetPositionFromAbsolute(JointIOInputs inputs) {
        setEncoderPositionFromAbsolute(inputs);
    }

    // @Override
    public boolean hasReachedTargetPosition(JointIOInputs inputs) {
        return (MathUtils.doublesEqual(inputs.mEncoderPositionUnadjusted, mSetpoint,
        RobotConstants.Arm.Elbow.kAllowedErrorRotations));
    }
        @Override
        protected HealthState checkHealth() {
                          
            REVLibError rotatorError = mRotator.getLastError();
            
            if (rotatorError != null && rotatorError != REVLibError.kOk) {
                // Logger.consoleError("Elbow error! Rotator error is " + rotatorError.toString());
                errorCounter++;
            } else {
                errorCounter = 0;
            }
    
            if (errorCounter >= 20) {
                return HealthState.RED;
            }
    
            if (!hasSetAbsolutePosition || !relativePositionIsSet) {
                return HealthState.YELLOW;
            }
    
            return HealthState.GREEN;
        }
        public void updateInputs(LoggableInputs inputs){
            super.updateInputs(inputs);
            JointIOInputs input=(JointIOInputs)inputs;
            input.absEncoderPos=absEncoder.getAbsolutePosition();
            input.updateFromSuper();
            
            
        }
        
    }

   
    
}
