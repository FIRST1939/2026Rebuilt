// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not   put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class ModeConstants {

        public static final Mode simMode = Mode.SIM;
        public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

        public static enum Mode {
            REAL,
            SIM,
            REPLAY
        }
    }

    public static class FieldUtilConstants {

        public static boolean disableHAL = false;

        public static void disableHAL() {
            disableHAL = true;
        }
    }

    public static class ControllerConstants {

        public static final double kJoystickDeadband = 0.1;
    }

    public static class IntakeConstants {

        public static final double kPivotOutSetpoint = 0.35;
        public static final double kPivotIdleSetpoint = 0.1;
        public static final double kPivotAgitateOutSetpoint = 0.3;
        public static final double kPivotAgitateInSetpoint = 0.18;

        public static final double kPivotStowingPercentage = -0.25;
        public static final double kPivotExtendedPercentage = 0.1;
        public static final double kPivotIntakePercentage = 0.25;

        // Not Current Used
        public static final double kRollerConversionFactor = (1.0 / 0.0254) * (1.0 / (3.0 * Math.PI)) * (60.0 / 1.0);
        public static final double kRollerBaseIntakeVelocity = 3000.0;

        // Currently Used
        public static final double kRollerIntakePercentage = 1.0;

        public static final double kRollerAgitateVelocity = 1250.0;
        public static final double kRollerReverseVelocity = -2750.0;
    }

    public static class SpindexerConstants {

        public static final double kSpindexerPercentage = 1.0;
        public static final double kSpindexerReversePercentage = -0.5;
    }

    public static class FeederConstants {

        public static final double kFeederVelocity = 1500.0;
        public static final double kFeederReverseVelocity = -1500.0;
    }

    public static class ShooterConstants {

        public static final double kTestFlywheelVelocity = 1000;
        public static final double kTestHoodPosition = 0.05;

        public static final double kHoodZeroPercentage = -0.1;

        public static final double kHubFlywheelVelocity = 2750;
        public static final double kOutpostFlywheelVelocity = 1500.0;
        public static final double kTowerFlywheelVelocity = 3000;
        public static final double kTrenchFlywheelVelocity = 3250;

        public static final double kHubHoodSetpoint = 0.0125;
        public static final double kOutpostHoodSetpoint = 0.05;
        public static final double kTowerHoodSetpoint = 0.047;
        public static final double kTrenchHoodSetpoint = 0.0458;
        public static final double kCornerFlywheelVelocity = 3600.0;
        public static final double kCornerHoodPosition = 0.0625;
    }

    public static class ClimberConstants {
        
        public static final double kRaisingClimberSetpoint = 14.0;
        public static final double kLoweringClimberSetpoint = 0.5;

        public static final double kRaisingClimberPercentage = 0.75;
        public static final double kLoweringClimberPercentage = -0.75;
    }
}
