#include "Robot.h"

void Robot::VisionThread()
{
	// Get the Axis camera from CameraServer
	cs::AxisCamera camera = CameraServer::GetInstance()->AddAxisCamera("axis-camera.local");
	camera.SetResolution(640, 480);

	/* How we would add an additional camera stream (for use w/ openCV)
	 *
	// Get a CvSink. This will capture Mats from the Camera
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
	// Setup a CvSource. This will send images back to the Dashboard
	cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);
	*/
}
