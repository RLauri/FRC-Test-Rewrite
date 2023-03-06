package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmExtension extends ProfiledPIDSubsystem {
    public static final CANSparkMax m_motorController = new CANSparkMax(Constants.IDs.EXTENSION_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(new DigitalInput(Constants.IDs.EXTENSION_ENC_ID));
    private ProfiledPIDController m_pidController;
    // need to do sysid profiling
    // private final ArmFeedforward m_feedforward =
    //   new ArmFeedforward(
    //       ArmConstants.kSVolts, ArmConstants.kGVolts,
    //       ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

    public ArmExtension() {
        super(
        new ProfiledPIDController(
            Constants.Extension.kP,
            Constants.Extension.kI,
            Constants.Extension.kD,
            new TrapezoidProfile.Constraints(
                Constants.Extension.kMaxVelocityRadPerSecond,
                Constants.Extension.kMaxAccelerationRadPerSecSquared),
            Constants.Extension.kDt),
        1);
        
        m_pidController = getController();
        m_pidController.setTolerance(Constants.Extension.kPosTolerance);
        m_encoder.reset();
        zeroingProtocall();
        m_motorController.setIdleMode(IdleMode.kBrake);
        setGoal(Constants.Extension.MIN_POS);
    }

    private void zeroingProtocall() {
        int loop_counter = 0;
        double current_sum = 0; 
        m_motorController.set(-.05);
        current_sum += m_motorController.getOutputCurrent();
        while (true){
            if(m_motorController.getOutputCurrent() > 2 * (current_sum / ++loop_counter)) {
                m_motorController.stopMotor();
                m_encoder.reset();
                break;
            }
            current_sum += m_motorController.getOutputCurrent();
        }
    }
    
    @Override
    public double getMeasurement() {
      // TODO: set distance conversion
      return m_encoder.getDistance();
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    // double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motorController.setVoltage(output);
    }
}
