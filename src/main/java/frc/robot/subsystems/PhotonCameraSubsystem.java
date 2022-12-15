package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;


public class PhotonCameraSubsystem extends SubsystemBase {
	private static PhotonCameraSubsystem instance = null;
	
	public static PhotonCameraSubsystem getInstance() {
		
		if (instance == null) {
			instance = new PhotonCameraSubsystem();
		}
		return instance;
	}
	
	final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(35);
	final double TARGET_HEIGHT_METER = Units.inchesToMeters(104);
	final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(27);

	PhotonCamera camera = new PhotonCamera("photo");

	@Override
	public void periodic() {
		super.periodic();
	}
}
