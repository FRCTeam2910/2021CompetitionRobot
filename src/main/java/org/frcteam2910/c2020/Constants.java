package org.frcteam2910.c2020;

public class Constants {
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 6;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 3;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 8;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 9;

    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 4;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 10;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 7;

    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 15;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 14;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 16;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 17;

    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = Math.toRadians(258.31);
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = Math.toRadians(134.56);
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = Math.toRadians(122.519);
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = Math.toRadians(39.81);

    public static final int SHOOTER_DRIVE_MOTOR_PORT_1 = 15;
    public static final int SHOOTER_DRIVE_MOTOR_PORT_2 = 16;

    public static final int HOOD_ANGLE_ENCODER = 0;
    public static final double HOOD_ENCODER_GEAR_RATIO = 15.75;

    public static final double SHOOTER_HOOD_MIN_ANGLE = Math.toRadians(17.5);
    public static final double SHOOTER_HOOD_MAX_ANGLE = Math.toRadians(75);
    public static final double SHOOTER_HOOD_OFFSET = -Math.toRadians(7.6);
    public static final double SHOOTER_HOOD_GEAR_RATIO = 51 / (16.0 * 15);
    public static final double FLYWHEEL_GEAR_RATIO = 1.5;

    public static final int LEFT_INTAKE_MOTOR_PORT = 0;
    public static final int RIGHT_INTAKE_MOTOR_PORT = 2;

    public static final int HOOD_MOTOR_PORT = 5;

    public static final int TOP_RIGHT_SHOOTER_MOTOR_PORT = 11;
    public static final int BOTTOM_RIGHT_SHOOTER_MOTOR_PORT = 13;
    public static final int BOTTOM_LEFT_SHOOTER_MOTOR_PORT = 12;

    public static final int FEEDER_MOTOR_PORT = 18;

    public static final int BOTTOM_INTAKE_EXTENSION_SOLENOID = 0;
    public static final int TOP_INTAKE_EXTENSION_SOLENOID = 1;

    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;


    public static final int FEEDER_IS_FULL_SENSOR_PORT = 2;
    public static final int FEEDER_INTAKE_BALL_SENSOR_PORT = 1;

//    private static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET_COMPETITION = -Math.toRadians(68.2);
//    private static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET_COMPETITION = -Math.toRadians(51.8);
//    private static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET_COMPETITION = -Math.toRadians(354.1);
//    private static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET_COMPETITION = -Math.toRadians(200.25);
    private static final double SHOOTER_HOOD_MIN_ANGLE_COMPETITION = Math.toRadians(26.90);
    private static final double SHOOTER_HOOD_MAX_ANGLE_COMPETITION = Math.toRadians(64.54);
    private static final double SHOOTER_HOOD_OFFSET_COMPETITION = Math.toRadians(50.9) + SHOOTER_HOOD_MAX_ANGLE_COMPETITION;
    private static final double SHOOTER_HOOD_GEAR_RATIO_COMPETITION = 22.0 / 72.0;
//
//    private static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET_PRACTICE = -Math.toRadians(75.5);
//    private static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET_PRACTICE = -Math.toRadians(347.7);
//    private static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET_PRACTICE = -Math.toRadians(298.5);
//    private static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET_PRACTICE = -Math.toRadians(77.0);
//    private static final double SHOOTER_HOOD_MIN_ANGLE_PRACTICE = Math.toRadians(24.0);
//    private static final double SHOOTER_HOOD_MAX_ANGLE_PRACTICE = Math.toRadians(58.0);
//    private static final double SHOOTER_HOOD_OFFSET_PRACTICE = Math.toRadians(57.38);
//    private static final double SHOOTER_HOOD_GEAR_RATIO_PRACTICE = 14.0 / 60.0;

}
