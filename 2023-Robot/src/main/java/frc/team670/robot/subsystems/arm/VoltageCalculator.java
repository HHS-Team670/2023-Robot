package frc.team670.robot.subsystems.arm;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import frc.team670.mustanglib.utils.ConsoleLogger;



/**
 * Generic class to calculate the necessary voltages for each joint in an N-segmented arm.
 * @author Justin Hwang, Aditi, Nancy
 */
public class VoltageCalculator {
    /**
     * The Array of ArmSegments that this arm contains. 0th index is the shoulder, last index is the furthest out joint.
     */
    private ArmSegment[] armSegments;

  
    private final String ARM_VOLTAGE_STRING_KEY;
    /**
     * Array of MAXIMUM X_CM displacements. the calculate() method compares against these.
     */
    private ArrayList<Double> maxXCMDisplacements;

    /** 
     * An ArrayList of maximum X_CM offsets
     * @param armSegments The list of arm segments, in the order they are attached. The base joint must be first, and the furthest-out joint must be the end
     */
    public VoltageCalculator(ArmSegment... armSegments) {
        ARM_VOLTAGE_STRING_KEY = "Arm/VoltageCalculator output";
        this.armSegments = armSegments;
        
        maxXCMDisplacements = calculateXCMOffsets(new double[armSegments.length]); //Maximum center-of-mass displacements when all joints are parallel to ground
    }
    
    /**
     * Calculates the necessary voltages for each 
     * @param anglesRelativeToPrevious The angle of each joint relative to the previous joint.
                                       The first arm segment MUST PASS IN ITS ANGLE RELATIVE TO THE GROUND.
     *                                 180 degrees means that the given segment is parallel to the previous joint.
                                       The angles should be in the order that they are attached, starting with the base joint and ending with the furthest joint.
                                       You MUST pass in one angle for each joint. Method will return null if an incorrect number of angles are passed in.
     * @return An ArrayList of the X offset of the center of mass of the remaining arm segments, for each joint, from the base joint to the furthest joint.
     */
    public ArrayList<Double> calculateVoltages(double... anglesRelativeToPrevious) {
        // First, ensure that the number of received angles is correct
        if(anglesRelativeToPrevious.length != armSegments.length) {
            ConsoleLogger.consoleError("VoltageCalculator received the wrong number of angles! Received " + anglesRelativeToPrevious.length + " but expected " + armSegments.length);
            return null;
        }

        // Second, calculate each angle relative to the ground
        double[] anglesRelativeToGround = new double[armSegments.length]; //Angles relative to ground in RADIANS
        anglesRelativeToGround[0] = Math.toRadians(anglesRelativeToPrevious[0]); //Put the first joint in place
        for(int i = 1; i < anglesRelativeToPrevious.length; i++) {
            double previousAngle = anglesRelativeToGround[i - 1];
            anglesRelativeToGround[i] = previousAngle - (Math.PI - Math.toRadians(anglesRelativeToPrevious[i])); //Calculate each joint relative to the previous one
        }

        // Third, get the x-displacements of the center of mass of the remaining segments, for each joint
        ArrayList<Double> xCMDisplacements = calculateXCMOffsets(anglesRelativeToGround);

        // Finally, use proportionality to convert the XCM offsets into voltages
        ArrayList<Double> voltages = new ArrayList<Double>();
        for(int i = 0; i < xCMDisplacements.size(); i++) {
            double ratio = xCMDisplacements.get(i) / maxXCMDisplacements.get(i);
            if(ratio > 0.2)
                voltages.add(armSegments[i].getArbitraryFF() * xCMDisplacements.get(i) / maxXCMDisplacements.get(i));
            else
                voltages.add(0.0);
            
        }

        Logger.getInstance().recordOutput(ARM_VOLTAGE_STRING_KEY, voltages.toString());

        return voltages;

    }

    /**
     * Calculates the x-direction displacements for each joint of the center of mass of the remaining segments
     * Formula can be found here: https://media.discordapp.net/attachments/1064355953950081176/1075969455458369588/IMG_4856.jpg?width=1770&height=1328
     * @param anglesRelativeToGround
     */
    private ArrayList<Double> calculateXCMOffsets(double... anglesRelativeToGround) {
        ArrayList<Double> xCMDisplacements = new ArrayList<Double>();
        double netMass = 0;

        for(int i = anglesRelativeToGround.length - 1; i >= 0; i--) {
            ArmSegment armSegment = armSegments[i];

            double x_N = armSegment.getLength() * Math.cos(anglesRelativeToGround[i]);
            double k_N = armSegment.getCMDistribution();
            double m_N = armSegment.getMass();

            if(i == anglesRelativeToGround.length - 1) { // if it's the first time through the loop (the furthest segment) then add its center of mass manually
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