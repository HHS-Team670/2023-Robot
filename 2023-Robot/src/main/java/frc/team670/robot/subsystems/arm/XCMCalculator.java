package frc.team670.robot.subsystems.arm;

import java.util.ArrayList;

public class XCMCalculator {
    //contain an arraylist of armsegments

    //constructor that takes in a list of argsegments

    //calculate() method that takes in an arraylist of angles
    //and returns an arraylist of X_CM values, in the order that they're attached
    
    /**
     * Calculates the x coordinate displacements from each joint.
     * @param anglesRelativeToPrevious The angle of each joint relative to the previous joint. 180 degrees means that the given segment is parallel to the previous joint.
                                       The first arm segment, though, MUST PASS IN ITS ANGLE RELATIVE TO THE GROUND.
                                       The angles should be in the order that they are attached, starting with the base joint and ending with the furthest joint.
     * @return The X offset of the center of mass of the remaining arm segments.
     */
    public ArrayList<Double> calculate(ArrayList<Double> anglesRelativeToPrevious) {
        ArrayList<Double> anglesRelativeToGround = new ArrayList<Double>();
        anglesRelativeToGround.add(anglesRelativeToPrevious.get(0));

        for(int i = 1; i < anglesRelativeToPrevious.size(); i++) {
            double previousAngle = anglesRelativeToGround.get(i - 1);
            anglesRelativeToGround.add(previousAngle - (180 - anglesRelativeToPrevious.get(i)));
        }

        return null;
    }
    
}