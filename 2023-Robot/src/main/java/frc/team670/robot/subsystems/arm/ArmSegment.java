package frc.team670.robot.subsystems.arm;

public class ArmSegment {
    /**
     * Length is from joint to joint. Units don't matter, but they should be consistent with other arm segments.
     */
    double length;

    /**
     * Units don't matter, but they should be consistent with other arm segments.
     */
    double mass;

    /**
     * Ranges from 0 to 1. If the mass is evenly distributed, this should be 0.5. If it's closer to the base, it should be <0.5, and closer to the tip is >0.5
     */
    double CMDistribution;

    /**
     * This is the measured voltage when this segment (and all further segments) are parallel to the ground
     */
    double arbitraryFF;


    public ArmSegment(double length, double mass, double CMDistribution, double arbitraryFF) {
        this.length = length;
        this.mass = mass;
        this.CMDistribution = CMDistribution;
        this.arbitraryFF = arbitraryFF;
    }
    
    public double getLength() {
        return length;
    }

    public double getMass() {
        return mass;
    }

    public double getCMDistribution() {
        return CMDistribution;
    }

    public double getArbitraryFF() {
        return arbitraryFF;
    }
}
