package frc.team670.robot.subsystems.arm;

import java.util.ArrayList;

public class XCMCalculator {
    //contain an arraylist of armsegments. 0th index is the shoulder, last index is the furthest-out joint
    ArrayList<ArmSegment> armSegments;

    //constructor that takes in a list of argsegments
    public XCMCalculator(ArrayList<ArmSegment> armSegments) {
        this.armSegments = armSegments;
    }
    
    /**
     * Calculates the x coordinate displacements from each joint.
     * Formula can be found here: https://media.discordapp.net/attachments/1064355953950081176/1075969455458369588/IMG_4856.jpg?width=1770&height=1328
     * @param anglesRelativeToPrevious The angle of each joint relative to the previous joint. 180 degrees means that the given segment is parallel to the previous joint.
                                       The first arm segment, though, MUST PASS IN ITS ANGLE RELATIVE TO THE GROUND.
                                       The angles should be in the order that they are attached, starting with the base joint and ending with the furthest joint.
     * @return The X offset of the center of mass of the remaining arm segments.
     */
    public ArrayList<Double> calculate(ArrayList<Double> anglesRelativeToPrevious) {

        ArrayList<Double> anglesRelativeToGround = new ArrayList<Double>(); //Angles relative to ground in RADIANS
        anglesRelativeToGround.add(Math.toRadians(anglesRelativeToPrevious.get(0)));
        for(int i = 1; i < anglesRelativeToPrevious.size(); i++) {
            double previousAngle = anglesRelativeToGround.get(i - 1);
            anglesRelativeToGround.add(Math.toRadians(previousAngle - (180 - anglesRelativeToPrevious.get(i))));
        }


        ArrayList<Double> xCMDisplacements = new ArrayList<Double>();
        double netMass = 0;

        for(int i = anglesRelativeToGround.size() - 1; i >= 0; i--) {
            ArmSegment armSegment = armSegments.get(i);

            double x_N = armSegment.getLength() * Math.cos(anglesRelativeToGround.get(i));
            double k_N = armSegment.getCMDistribution();
            double m_N = armSegment.getMass();

            if(i == anglesRelativeToGround.size() - 1) { // if it's the first time through the loop (the furthest segment) then add its center of mass manually
                xCMDisplacements.add(x_N * k_N / m_N);
            } else { // for the Nth segment...
                double xCM_previous = xCMDisplacements.get(0);
                double xCM_N = (x_N * k_N * m_N + (x_N + xCM_previous) * (netMass))
                                / (m_N + netMass);
                xCMDisplacements.add(0, xCM_N);
            }

            netMass += m_N;

        }

        return xCMDisplacements;
    }
    
}