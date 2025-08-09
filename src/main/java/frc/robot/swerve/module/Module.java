package frc.robot.swerve.module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;

// A single module
public class Module {
    public record ModuleConstants(int id, String prefix, int driveID, int turnID, int cancoderID, Rotation2d cancoderOffset) {}

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    public Module(ModuleIO io) {
        this.io = io;
    }

    // Updates and logs the IO layer inputs
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(
            "Swerve/" + inputs.prefix + " Module", 
            inputs);
    }


}
