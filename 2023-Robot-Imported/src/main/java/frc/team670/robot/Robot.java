// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot;

import frc.team670.mustanglib.RobotBase;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update build.gradle
 * 
 * Robot has a RobotContainer and calls corresponding methods (autonomousInit,
 * periodic, etc), but also runs health checks and other useful functions.
 * 
 */
public class Robot extends RobotBase {
	public Robot() {
		super(new RobotContainer());
	}
}
