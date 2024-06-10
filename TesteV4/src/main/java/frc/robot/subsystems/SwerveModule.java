package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;
    private final CANcoder canCoder;
    private final SparkPIDController turnPidController;
    private final RelativeEncoder driveEncoder;

    // PID constants - these need to be tuned for your specific setup
    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public SwerveModule(int driveMotorID, int turningMotorID, int canCoderID) {
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        canCoder = new CANcoder(canCoderID);

        CANcoderConfiguration config = new CANcoderConfiguration();
        canCoder.getConfigurator().apply(config);

        turnPidController = turningMotor.getPIDController();
        turnPidController.setP(kP);
        turnPidController.setI(kI);
        turnPidController.setD(kD);

        driveEncoder = driveMotor.getEncoder();
    }
    
    public void setDesiredState(double speed, double angle) {
        // Set the speed of the drive motor
        driveMotor.set(speed);
    
        // Get the current angle from the CANcoder
        // double currentAngle = canCoder.getAbsolutePosition().getValue();
    
        // Normalize the desired angle to the range [0, 360)
        angle = ((angle % 360) + 360) % 360;
    
        // Determine the shortest path to the desired angle
        // double error = angle - currentAngle;
        // if (error > 180) {
        //     error -= 360;
        // } else if (error < -180) {
        //     error += 360;
        // }
        // double targetAngle = currentAngle + error;
    
        // Set the turning motor position to the target angle
        turningMotor.getPIDController().setReference(angle, ControlType.kPosition);
        // double valueT = turningMotor.getPIDController().getRe
        // System.out.println("TurningMotor: " + turningMotor.get);
    }
    
    public void updateSmartDashboard() {
        // Obtenha a posição absoluta em unidades nativas
        double absolutePositionNativeUnits = canCoder.getAbsolutePosition().getValue();

        // Defina a relação entre unidades nativas e graus
        double nativeUnitsPerDegree = 1024.0 / 360.0; // Exemplo: 1024 unidades nativas correspondem a 360 graus

        // Converta para graus
        double absolutePositionDegrees = absolutePositionNativeUnits / nativeUnitsPerDegree;

        // Send current state information to the SmartDashboard for debugging
        SmartDashboard.putNumber("Drive Motor Speed", driveEncoder.getVelocity());
        SmartDashboard.putNumber("Turning Motor Output", turningMotor.get());
        SmartDashboard.putNumber("Current Angle", absolutePositionDegrees); // Corrigido aqui
        SmartDashboard.putNumber("Drive Motor Rotation", driveEncoder.getPosition());
        SmartDashboard.putNumber("Turning Motor Rotation", turningMotor.getEncoder().getPosition());
        // Adicione aqui a rotação do robô, se disponível
    }
    
    public void zeroCanCoder() {
        canCoder.setPosition(0.0);
    }
    
}
