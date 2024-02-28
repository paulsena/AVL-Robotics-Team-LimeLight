package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.DetectMascotVisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Vision {
    OpenCvCamera camera;
    DetectMascotVisionPipeline detectMascotVisionPipeline = new DetectMascotVisionPipeline();

    public Vision(HardwareMap hardwareMap) {
        // Create camera instance
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set what image pipeline will do processing
        camera.setPipeline(detectMascotVisionPipeline);

        // Open connection to the camera Async and start streaming
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                //camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
                camera.showFpsMeterOnViewport(false);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

    }

    public void setTeamColor(RobotConstants.TEAM_COLOR teamColor) {
        detectMascotVisionPipeline.setTeamColor(teamColor.ordinal());
    }

    public int getMascotSpikePosition(Telemetry telemetry) {
        int position = detectMascotVisionPipeline.getMascotSpikePosition();
        if (telemetry != null) {
            telemetry.addData("Team Color (Red 0, Blue 1)", detectMascotVisionPipeline.getTeamColor());
            telemetry.addData("Mascot Spike Position", position);
        }
        return position;
    }

    public RobotConstants.MascotSpikePosition getMascotSpikeEnum(Telemetry telemetry) {
        int pos = getMascotSpikePosition(telemetry);
        if (pos == 1) {
            return RobotConstants.MascotSpikePosition.LEFT;
        } else if (pos == 2) {
            return RobotConstants.MascotSpikePosition.MIDDLE;
        } else if (pos == 3) {
            return RobotConstants.MascotSpikePosition.RIGHT;
        }

        return RobotConstants.MascotSpikePosition.MIDDLE;
    }

}
