package frc.robot;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

/**
 * DriveSystem
 */
public class DriveSystem {

    private final double trackWidth;
    private final double linearSpeed;
    private final double rotationalSpeed;

    private final DifferentialDriveKinematics kinematics;
    private final ChassisSpeeds chassisSpeeds;
    private final DifferentialDriveWheelSpeeds speeds;

    private final CANSparkMax fl = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax fr = new CANSparkMax(11, MotorType.kBrushless);
    private final CANSparkMax bl = new CANSparkMax(12, MotorType.kBrushless);
    private final CANSparkMax br = new CANSparkMax(13, MotorType.kBrushless);
  
    // Array for motors
    private final CANSparkMax[] motors = {fl, fr, bl, br};
    private final CANPIDController flPID = fl.getPIDController();
    private final CANPIDController frPID = fr.getPIDController();
    private final CANPIDController blPID = bl.getPIDController();
    private final CANPIDController brPID = br.getPIDController();


    DriveSystem(double trackWidth, double linearSpeed, double rotationalSpeed) {
        this.trackWidth = trackWidth;
        this.linearSpeed = linearSpeed;
        this.rotationalSpeed = rotationalSpeed;

        //create the kinematics for the differential drive
        this.kinematics = new DifferentialDriveKinematics(this.trackWidth);
        this.chassisSpeeds = new ChassisSpeeds(this.linearSpeed, 0., this.rotationalSpeed);

        this.speeds = this.kinematics.toWheelSpeeds(this.chassisSpeeds);

        this.init();
    }

    private void init() {
        for(var motor : this.motors){
            var res = motor.restoreFactoryDefaults();
            System.out.println(res);
            var encoder = motor.getEncoder();
            encoder.setVelocityConversionFactor( 
                (1 / encoder.getCountsPerRevolution()) * 4 * Math.PI * 2.54 / 100
                );
                
            var pid = motor.getPIDController();

        }
    }
    public void drive(double x, double r) {
        var speeds = this.kinematics.toWheelSpeeds(new ChassisSpeeds(x, 0., r));
        var left = speeds.leftMetersPerSecond;
        var right = speeds.rightMetersPerSecond;

        System.out.printf("L:%s, R:%s\n", left, right);
        fl.set(left);
        fr.set(right);
        bl.set(left);
        br.set(right);
    }
}