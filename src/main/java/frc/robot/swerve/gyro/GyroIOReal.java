package frc.robot.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOReal implements GyroIO {

    private final Pigeon2 pigeon;

    private final StatusSignal<Angle> yaw;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<AngularVelocity> yawVelocity;

    public GyroIOReal(int id) {
        pigeon = new Pigeon2(id, "*");

        yaw = pigeon.getYaw();
        pitch = pigeon.getPitch();
        roll = pigeon.getRoll();
        yawVelocity = pigeon.getAngularVelocityZWorld();

        // TODO: UPDATE YAW AND YAW VELOCITY IN AN ODO THREAD
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, yaw, pitch, roll, yawVelocity);

        pigeon.optimizeBusUtilization();
    }



    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(yaw, pitch, roll, yawVelocity).equals(StatusCode.OK);

        inputs.yaw = new Rotation2d(yaw.getValue());
        inputs.pitch = new Rotation2d(pitch.getValue());
        inputs.roll = new Rotation2d(roll.getValue());
        inputs.yawVelocityRadPerSec = yawVelocity.getValue().in(Units.RadiansPerSecond);
        
    }

    @Override
    public void setYaw(Rotation2d yaw) {
        pigeon.setYaw(yaw.getDegrees());
        
    }

    

}
