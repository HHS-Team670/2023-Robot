package frc.team670.robot.subsystems.arm;


public class InverseKinematicsTest {
    
    public static void main(String[] args) {
        for(int i = 0; i <= 9; i++){
        
            //ArmState.getVal(i);    
            System.out.println(ArmState.getVal(i)+" Shoulder Angle: "+ (int) ArmState.getVal(i).getShoulderAngle()+" Elbow Angle: "+ (int) ArmState.getVal(i).getElbowAngle());        
        }
        
    }
}
