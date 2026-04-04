// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// 
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
@SuppressWarnings("unused")
public final class Constants {
    public static final int MOTORID_TURRET = 14;
    public static final int MOTORID_SHOOTER1 = 11;
    public static final int MOTORID_SHOOTER2 = 12;
   
    public static final int MOTORID_INTAKEROLLERS1 = 5;
    public static final int MOTORID_INTAKEROLLERS2 = 8;
    public static final int MOTORID_INTAKEAGITATOR = 10;
    public static final int MOTORID_HOOD = 13;
    public static final int MOTORID_INTAKE_SHOULDER = 9;

    public static final int MOTORID_CONVEYER = 7;

    public static final int MOTORID_PRESHOOTER = 6;

    //Using the Climber Motor for hte Agitator for now
    public static final int MOTORID_CLIMBER = 8;
    
    
    public static final int ENCODERID_TURRET_A = 41;
    public static final int ENCODERID_TURRET_B = 42;
    
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }
}