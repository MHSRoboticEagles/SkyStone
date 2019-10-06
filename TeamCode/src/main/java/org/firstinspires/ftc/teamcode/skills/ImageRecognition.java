package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class ImageRecognition {
    private boolean foundVuMark = false;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;

    public void init(HardwareMap hardwareMap){
         /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AUFqqNL/////AAAAGfWhySdgbElnsdW5/zwg6IE6CBoEhUaVxuLn9QakQb4yAaDqeXUSreY68ZkPXGQZfJt8MrZGR7Cc+6MLOnnFsSXJPuNMRfNfkMZjy5gA1FG09zZBB6Gxqsj5kxCDN967n21wmDQjiElUgSIMBu/Esbu0Zi6X4HWItN8O0of9KEQa7YtNRTqZnTvdzVMnJWtnKecsRTK2FVECgx47Y7wAHB7pPVZu455PWHBT90fM5jJd5JXmvTWzn/DAoWZfEk4JOmYSTiIK2AX0nzZqxExlMtu1yIOFQ/yN8nPHCa+b/hPNZk3vAu+ZlSCRxkEPUziLfLjAujmvPtjYwUrx8rxtiQSGcHUl6qpPx2ugwNqiCNZO";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    public void start(){
        relicTrackables.activate();
    }

    public GoldPosition track(Telemetry telemetry){
        GoldPosition result = GoldPosition.None;
        try {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "Found: %s visible", vuMark);
                switch (vuMark){
                    case RIGHT:
                        result = GoldPosition.Right;
                        break;
                    case CENTER:
                        result = GoldPosition.Center;
                        break;
                    case LEFT:
                        result = GoldPosition.Left;
                        break;
                    default:
                        break;
                }

                foundVuMark = true;
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();

        }
        catch (Exception ex){
            telemetry.addData("Problem with image recognition", ex.toString());
        }
        return result;
    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
