package org.frcteam2910.c2020;

public class Constants {
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 10;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 6;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 9;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 7;

    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 31;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 32;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 30;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 33;

    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(136.108681);
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(236.068095);
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(221.871346);
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(151.933288);

    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 0;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 3;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 1;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 2;

    public static final int SHOOTER_ANGLE_MOTOR_PORT = 0;

    public static final int SHOOTER_DRIVE_MOTOR_PORT_1 = 0;
    public static final int SHOOTER_DRIVE_MOTOR_PORT_2 = 0;

    public static final double WHEEL_OF_FORTUNE_RED_HUE = 45;
    public static final double WHEEL_OF_FORTUNE_GREEN_HUE = 130;
    public static final double WHEEL_OF_FORTUNE_BLUE_HUE = 170;
    public static final double WHEEL_OF_FORTUNE_YELLOW_HUE = 90;

    public static final int DRIVETRAIN_PIGEON_PORT = 0;

    public static final int INTAKE_MOTOR = 0;
    public static final int INTAKE_EXTENSION_SOLENOID = 0;

    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;

    public static final int FEEDER_MOTOR_PORT = 0;

    public static final int CLIMBER_DEPLOY_SOLENOID_PORT = 1;
    public static final int CLIMBER_EXTEND_SOLENOID_PORT = 2;
}
