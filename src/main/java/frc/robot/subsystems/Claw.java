package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    private static final PneumaticHub hub = new PneumaticHub(Constants.IDs.PNEUMATIC_CAN_ID);
    private static final DoubleSolenoid clawSolenoid = hub.makeDoubleSolenoid(Constants.IDs.CLAW_OPEN_PORT, Constants.IDs.CLAW_CLOSE_PORT);
    private static final Compressor compressor = hub.makeCompressor();


    public void toggleClaw() {
        clawSolenoid.toggle();
    }

    public void init() {
        compressor.enableAnalog(90, 120);
        clawSolenoid.set(DoubleSolenoid.Value.kForward);
    }
}
