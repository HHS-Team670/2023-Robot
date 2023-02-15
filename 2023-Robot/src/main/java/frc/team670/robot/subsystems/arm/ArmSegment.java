package frc.team670.robot.subsystems.arm;

public class ArmSegment {
    double length;
    double mass;
    double com_ratio;
    double arbitrary_ff;


    public ArmSegment(double length, double mass, double com_ratio) {
        this.length = length;
        this.mass = mass;
        this.com_ratio = com_ratio;
    }
    
    public double getLength() {
        return length;
    }

    public double getMass() {
        return mass;
    }

    public double getCOM_Ratio() {
        return com_ratio;
    }

}
