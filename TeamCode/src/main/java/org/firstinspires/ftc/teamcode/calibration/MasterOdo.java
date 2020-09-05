package org.firstinspires.ftc.teamcode.calibration;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.autonomous.AutoStep;
import org.firstinspires.ftc.teamcode.bots.BotAction;
import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.bots.YellowBot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.skills.Geometry;
import org.firstinspires.ftc.teamcode.skills.Led;

import java.io.File;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.internal.system.AppUtil.FIRST_FOLDER;
import static org.firstinspires.ftc.teamcode.autonomous.AutoRoute.NAME_BLUE;
import static org.firstinspires.ftc.teamcode.autonomous.AutoRoute.NAME_RED;

@TeleOp(name="Master Odo", group="Robot15173")
public class MasterOdo extends LinearOpMode {

    protected YellowBot bot = new YellowBot();
    ElapsedTime timer = new ElapsedTime();

    private static String CONFIG_FILE_NAME = "master-odo-config.json";
    public static final File ROUTES_FOLDER = new File(FIRST_FOLDER, "/routes/");
    private ArrayList<AutoRoute> routes = new ArrayList<>();

    protected double right = 0;
    protected double left = 0;
    protected double valueHead = 0;
    protected double startHead = 0;
    protected double nextHead = 0;
    protected double desiredHead = 0;
    protected double leftTarget = 0;
    protected double rightTarget = 0;

    protected double ratio = 0;

    protected int startX = 35;
    protected int startY = 24;

    protected int targetX = 0;
    protected int targetY = 0;

    private int MODE_VALUE = 1;
    private boolean MODE_UP = true;

    private int selectedTopMode = 0;
    private int selectedGoToMode = 0;
    private boolean topMode = true;
    private boolean goToMode = false;

    private boolean routeListMode = false;

    private boolean startSettingMode  = false;
    private boolean routeSettingMode  = false;
    private boolean XSettingMode = true;
    private boolean YSettingMode = false;
    private boolean speedSettingMode = false;
    private boolean strategySettingMode = false;
    private boolean waitSettingMode = false;
    private boolean desiredHeadSettingMode = false;
    private boolean routeSavingMode = false;

    private boolean routeNameSettingMode = false;
    private boolean routeIndexSettingMode = false;


    private AutoStep goToInstructions = new AutoStep();
    private AutoRoute newRoute = new AutoRoute();

    private static double CALIB_WEIGHT = 15.2;


    private static final int[] modesTop = new int[]{0, 1, 2, 3};
    private static final String[] modeNamesTop = new String[]{"Go To", "Start Position", "Routes", "Save"};

    private static final int[] modesStep = new int[]{0, 1, 2, 3, 4};
    private static final String[] modeStepName = new String[]{"Destination", "Top Speed", "Strategy", "Wait", "Action"};



    private double DISTANCE_Y = 0;

    private double SPEED = bot.CALIB_SPEED;

    private double SPEED_INCREMENT = 0.1;

    private double COORD_MIN = 1;
    private double COORD_MAX = 100;
    private int COORD_MULTIPLIER = 10;
    private Led led = null;

    Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 500;

    RobotCoordinatePosition locator = null;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            this.led = bot.getLights();
            bot.initGyro();
            bot.initCalibData();
            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
            listRoutes();

            initRoute();

            waitForStart();


            startLocator();

            if (this.led != null){
                this.led.none();
            }

            showConfig();

            while (opModeIsActive()) {
                processCommands();
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
        finally {
            if (locator != null){
                locator.stop();
            }
            newRoute.getSteps().clear();
            newRoute = null;
        }
    }

    private void startLocator(){
        if (locator == null) {
            locator = new RobotCoordinatePosition(bot, new Point(startX, startY), desiredHead, 75);
            locator.reverseHorEncoder();
            Thread positionThread = new Thread(locator);
            positionThread.start();
        }
    }

    private void toggleRouteSettings(){
        if (XSettingMode){
            XSettingMode = false;
            YSettingMode = true;
        }
        else if (YSettingMode){
            YSettingMode = false;
            XSettingMode = true;
        }
    }

    private void toggleRouteNaming(){
        if (routeNameSettingMode){
            routeNameSettingMode = false;
            routeIndexSettingMode = true;
        }
        else if (routeIndexSettingMode){
            routeIndexSettingMode = false;
            routeNameSettingMode = true;
        }
    }


    private void processCommands(){
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.back){

            topMode = true;
            goToMode = false;

            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.dpad_left ){

            if (routeSettingMode || startSettingMode){
                toggleRouteSettings();
            }
            else if (routeSavingMode){
                toggleRouteNaming();
            }

//            MODE_VALUE = -changeIncrement();
            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.dpad_right){
            if (routeSettingMode || startSettingMode){
                toggleRouteSettings();
            }
            else if (routeSavingMode){
                toggleRouteNaming();
            }

//            MODE_VALUE = changeIncrement();
            showConfig();
            gamepadRateLimit.reset();
        }

        //value adjustment
        if (gamepad1.dpad_down){
            if (routeSettingMode){
                if (XSettingMode) {
                    int x = goToInstructions.getTargetX();
                    x -= 5;
                    goToInstructions.setTargetX(x);
                }
                else if (YSettingMode){
                    int y = goToInstructions.getTargetY();
                    y -= 5;
                    goToInstructions.setTargetY(y);
                }

            }
            else if (startSettingMode){
                int x = (int)locator.getXInches();
                int y = (int)locator.getYInches();
                if (XSettingMode) {
                    x -= 5;
                }
                else if (YSettingMode){
                    y -= 5;
                }
                locator.init(new Point(x, y), locator.getInitialOrientation());
            }
            else if (speedSettingMode){
                double speed = goToInstructions.getTopSpeed();
                speed = speed - SPEED_INCREMENT;
                if (speed < 0){
                    speed = 0;
                }
                goToInstructions.setTopSpeed(speed);
            }
            else if (routeListMode){
                for(int i = 0; i < routes.size(); i++){
                    AutoRoute r = routes.get(i);
                    if (r.isSelected()){
                        r.setSelected(false);
                        if (i + 1 >= routes.size()){
                            routes.get(0).setSelected(true);
                        }
                        else{
                            routes.get(i+1).setSelected(true);
                        }
                    }
                }
            }
            else if (strategySettingMode){
                int index = goToInstructions.getMoveStrategy().ordinal();
                int total = MoveStrategy.values().length;
                index--;
                if (index < 0){
                    index = total - 1;
                }
                MoveStrategy updated = MoveStrategy.values()[index];
                goToInstructions.setMoveStrategy(updated);
            }
            else if (desiredHeadSettingMode){
                double desiredHead = goToInstructions.getDesiredHead();
                desiredHead -= 5;
                if (desiredHead < 0){
                    desiredHead = 0;
                }
                goToInstructions.setDesiredHead(desiredHead);
            }
            else if (waitSettingMode){
                int waitMS = goToInstructions.getWaitMS();
                waitMS -= 500;
                if (waitMS < 0){
                    waitMS = 0;
                }
                goToInstructions.setWaitMS(waitMS);
            }
            else if (routeSavingMode) {
                if (routeIndexSettingMode){
                    int i = newRoute.getNameIndex();
                    i -= 1;
                    newRoute.setNameIndex(i);
                }
                else if (routeNameSettingMode){
                    String name = newRoute.getName();
                    if (name.equals(NAME_BLUE)){
                        name = NAME_RED;
                    }
                newRoute.setName(name);
                }
            }
            else{
                if (topMode) {
                    if (selectedTopMode < modesTop.length) {
                        selectedTopMode++;
                    }
                }
                else if (goToMode){
                    if (selectedGoToMode < modesStep.length) {
                        selectedGoToMode++;
                    }
                }
            }
            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.dpad_up){
            if (routeSettingMode){
                if (XSettingMode) {
                    int x = goToInstructions.getTargetX();
                    x += 5;
                    goToInstructions.setTargetX(x);
                }
                else if (YSettingMode){
                    int y = goToInstructions.getTargetY();
                    y += 5;
                    goToInstructions.setTargetY(y);
                }

            }
            else if (startSettingMode){
                int x = (int)locator.getXInches();
                int y = (int)locator.getYInches();
                if (XSettingMode) {
                    x += 5;
                }
                else if (YSettingMode){
                    y += 5;
                }
                locator.init(new Point(x, y), locator.getInitialOrientation());
            }
            else if (speedSettingMode){
                double speed = goToInstructions.getTopSpeed();
                speed = speed + SPEED_INCREMENT;
                if (speed > 1){
                    speed = 1;
                }
                goToInstructions.setTopSpeed(speed);
            }
            else if (routeListMode){
                for(int i = routes.size(); i >= 0; i--){
                    AutoRoute r = routes.get(i);
                    if (r.isSelected()){
                        r.setSelected(false);
                        if (i  < 0){
                            routes.get(routes.size() -1 ).setSelected(true);
                        }
                        else{
                            routes.get(0).setSelected(true);
                        }
                    }
                }
            }
            else if (desiredHeadSettingMode){
                double desiredHead = goToInstructions.getDesiredHead();
                desiredHead += 5;
                goToInstructions.setDesiredHead(desiredHead);
            }
            else if (waitSettingMode){
                int waitMS = goToInstructions.getWaitMS();
                waitMS += 500;
                goToInstructions.setWaitMS(waitMS);
            }
            else if (strategySettingMode){
                int index = goToInstructions.getMoveStrategy().ordinal();
                int total = MoveStrategy.values().length;
                index++;
                if (index >= total){
                    index = 0;
                }
                MoveStrategy updated = MoveStrategy.values()[index];
                goToInstructions.setMoveStrategy(updated);
            }
            else if (routeSavingMode) {
                if (routeIndexSettingMode){
                    int i = newRoute.getNameIndex();
                    i += 1;
                    newRoute.setNameIndex(i);
                }
                else if (routeNameSettingMode){
                    String name = newRoute.getName();
                    if (name.equals(NAME_RED)){
                        name = NAME_BLUE;
                    }
                newRoute.setName(name);
                }
            }
            else{
                if (topMode) {
                    if (selectedTopMode > 0) {
                        selectedTopMode--;
                    }
                }
                else if (goToMode){
                    if (selectedGoToMode > 0) {
                        selectedGoToMode--;
                    }
                }
            }

            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.start){

            if (goToMode){
                goTo(this.goToInstructions);
            }
            else if (routeSavingMode){
                saveRoute();
            }
            else if (routeListMode){
                runSelectedRoute();
            }
            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.x){
            if (routeListMode){
                deleteSelectedRoute();
            }
            showConfig();
            gamepadRateLimit.reset();
        }

        //accept
        if (gamepad1.right_bumper){
            if (topMode){
                switch (selectedTopMode){
                    case 0:
                        topMode = false;
                        goToMode = true;
                        break;
                    case 1:
                        startSettingMode = !startSettingMode;
                        break;
                    case 2:
                        routeListMode = !routeListMode;
                        break;
                    case 3:
                        routeSavingMode = !routeSavingMode;
                        break;
                }
            }
            else if (goToMode){
                switch (selectedGoToMode){
                    case 0:
                        routeSettingMode = !routeSettingMode;
                        break;
                    case 1:
                        speedSettingMode = !speedSettingMode;
                        break;
                    case 2:
                        strategySettingMode = !strategySettingMode;
                        break;
                    case 3:
                        waitSettingMode = !waitSettingMode;
                        break;
                    case 4:
                        desiredHeadSettingMode = !desiredHeadSettingMode;
                        break;
                }
            }
            showConfig();
            gamepadRateLimit.reset();
        }

    }



    private int changeIncrement(){
        int tempVal = Math.abs(MODE_VALUE);
        if(MODE_UP && tempVal < COORD_MAX){
            tempVal = tempVal * COORD_MULTIPLIER;
            if (tempVal == COORD_MAX) {
                MODE_UP = false;
            }
        }
        else {
            if (MODE_UP == false && tempVal > COORD_MIN) {
                tempVal = tempVal / COORD_MULTIPLIER;
                if (tempVal == COORD_MIN) {
                    MODE_UP = true;
                }
            }
        }
        return tempVal;
    }

    private double moveForward(){
        if (this.led != null){
            this.led.move();
        }
        startHead = bot.getGyroHeading();
        double leftStart = bot.getLeftOdometer();
        double rightStart = bot.getRightOdometer();
        leftTarget = bot.getLeftTarget(DISTANCE_Y);
        rightTarget = bot.getRightTarget(DISTANCE_Y);
        bot.moveTo(SPEED, SPEED, DISTANCE_Y);
        this.left = bot.getLeftOdometer() - leftStart;
        this.right = bot.getRightOdometer() - rightStart;

        double ratio = right/left;

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Calib","Waiting for gyro to settle ...");
            telemetry.update();
        }
        nextHead = bot.getGyroHeading();

        if (this.led != null){
            this.led.start();
        }

        return ratio;
    }

    private void spin(){
        if (this.led != null){
            this.led.move();
        }
        startHead = bot.getGyroHeading();

        bot.spinH(desiredHead, SPEED);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Gyroscope","Stabilizing ...");
            telemetry.update();
        }

        nextHead = bot.getGyroHeading();

        if (this.led != null){
            this.led.start();
        }
    }

    private void spin(BotMoveProfile profile){
//        if (this.led != null){
//            this.led.move();
//        }


        bot.spinH(profile.getAngleChange(), profile.getTopSpeed());

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Gyroscope","Stabilizing ...");
            telemetry.update();
        }


//        if (this.led != null){
//            this.led.start();
//        }
    }


    private void goTo(AutoStep instruction){
        waitToStartStep(instruction.getWaitMS());
        MoveStrategy strategy = instruction.getMoveStrategy();
        executeStep(instruction, strategy);
        newRoute.getSteps().add(instruction);

    }

    private void executeStep(AutoStep instruction,  MoveStrategy strategy){
        BotMoveProfile profile = BotMoveProfile.bestRoute(bot, (int)locator.getXInches(), (int)locator.getYInches(), new Point(instruction.getTargetX(), instruction.getTargetY()),
                RobotDirection.Optimal, instruction.getTopSpeed(), strategy, locator);
        switch (profile.getStrategy()){
            case Curve:
                curve(profile);
                break;
            case Spin:
                spin(profile);
                break;
            case Strafe:
                strafe(profile);
                break;
        }
        if (profile.getNextStep() != null){
            executeStep(instruction, profile.getNextStep());
        }
        //set the desired heading
        double realAngleChange = Geometry.getAngle(instruction.getDesiredHead(), locator.getOrientation());
        BotMoveProfile profileSpin = BotMoveProfile.buildSpinProfile(instruction.getTopSpeed(), realAngleChange, null);
        spin(profileSpin);
    }

    private void  curve(BotMoveProfile profile){
        bot.moveCurveCalib(profile, locator);
    }

    private void waitToStartStep(int MS){
        timer.reset();
        while(timer.milliseconds() < MS && opModeIsActive()){

        }
    }

    private void strafe(BotMoveProfile profile){
        double distance = profile.getDistance();
        bot.strafeToCalib(profile.getTopSpeed(), distance, distance > 0, profile.getMotorReduction());
    }


    private String getStepValue(int index){
        String val = "";
        switch (index){
            case 0:
                //destination
                val = goToInstructions.getDestination();
                break;
            case 1:
                val = goToInstructions.getTopSpeedString();
                break;
            case 2:
                val = goToInstructions.getMoveStrategyString();
                break;
            case 3:
                val = goToInstructions.getWaitString();
                break;
            case 4:
                val = goToInstructions.getAction();
                break;
        }
        return val;
    }

    private void showConfig(){
        try {
            if (routeSettingMode) {
                showTarget();
            } else if (startSettingMode) {
                showStart();
            } else if (speedSettingMode) {
                telemetry.addData("Top Speed", "%.2f", goToInstructions.getTopSpeed());
            }
            else if (desiredHeadSettingMode){
                telemetry.addData("Desired Head", "%.2f", goToInstructions.getDesiredHead());
            }
            else if (routeListMode){
                for(AutoRoute r : routes){
                    if (r.isSelected()){
                        telemetry.addData(r.getRouteName(), "*");
                    }
                    else{
                        telemetry.addData(r.getRouteName(), " ");
                    }
                }
            }
            else if (strategySettingMode) {
                for (MoveStrategy s : MoveStrategy.values()) {
                    if (goToInstructions.getMoveStrategy().equals(s)) {
                        telemetry.addData(s.name(), "*");
                    } else {
                        telemetry.addData(s.name(), " ");
                    }
                }
            }
            else if (routeSavingMode){
                if (routeIndexSettingMode) {
                    telemetry.addData(newRoute.getRouteName(), String.format("-%d*", newRoute.getNameIndex()));
                }
                else if(routeNameSettingMode){
                    telemetry.addData(String.format("%s*", newRoute.getRouteName()), String.format("-%d", newRoute.getNameIndex()));
                }
            }
            else if (waitSettingMode) {
                telemetry.addData("Initial Wait Time MS", goToInstructions.getWaitString());
            }
            else if (topMode) {
                for (int i = 0; i < modesTop.length; i++) {
                    String selected = i == selectedTopMode ? "*" : " ";
                    telemetry.addData(selected, modeNamesTop[i]);
                }
            } else if (goToMode) {
                showStart();
                for (int i = 0; i < modesStep.length; i++) {
                    String selected = i == selectedGoToMode ? "*" : " ";
                    telemetry.addData(String.format("%s%s", selected, modeStepName[i]), getStepValue(i));
                }
            }

            telemetry.update();
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
    }

    private void showTarget(){
        String toX = XSettingMode ? "*" : " ";
        String toY = YSettingMode ? "*" : " ";

        telemetry.addData("Target", "%d%s : %d%s", goToInstructions.getTargetX(), toX, goToInstructions.getTargetY(), toY);
    }

    private void showStart(){
        String toX = XSettingMode ? "*" : " ";
        String toY = YSettingMode ? "*" : " ";

        telemetry.addData("Start", "%d%s : %d%s", (int)locator.getXInches(), toX, (int)locator.getYInches(), toY);
    }

    private void showHints(boolean full){
        telemetry.addData("DPAD UP", "Increment value");
        telemetry.addData("DPAD DOWN", "Decrease value");
        if (full) {
            telemetry.addData("DPAD Left/Right:", "Change increment: -100 -10 -1 1 10 100");
        }
    }


    private void showHelp(){
        telemetry.addData("X:", "Set the x coordinate value");
        telemetry.addData("Y", "Set the y coordinate value");
        telemetry.addData("A", "Set speed in increments of 0.1");
        telemetry.addData("Right Bumper", "Set the robot in motion");
        telemetry.update();
    }

    private void showStats(){
        telemetry.addData("Moving to ", "%.2f : %.2f", targetX, targetY);
        telemetry.addData("ratio", ratio);
        telemetry.addData("startHead", startHead);
        telemetry.addData("nextHead", nextHead);
        telemetry.addData("Speed", SPEED);
        telemetry.addData("Horizontal", bot.getHorizontalOdometer());

        telemetry.update();
    }

    private void saveRoute(){
        try{
            if (newRoute.getSteps().size() > 0) {
                String name = newRoute.getRouteName();
                File configFile = getRouteFile(name);

                String jsonPath = newRoute.serialize();
                ReadWriteFile.writeFile(configFile, jsonPath);
                this.routes.add(newRoute);
                initRoute();
            }
        }
        catch (Exception e) {
            telemetry.addData("Error", "Config cannot be saved. %s", e.getMessage());
            telemetry.update();
        }
    }


    private void runSelectedRoute(){
        AutoRoute selected = null;
        for(AutoRoute r : routes){
            if (r.isSelected()){
                selected = r;
                break;
            }
        }
        if (selected != null){
            for(AutoStep s : selected.getSteps()){
                this.goTo(s);
            }
        }
    }

    private void deleteSelectedRoute(){
        AutoRoute selected = null;
        String selectedName = "";
        for(AutoRoute r : routes){
            if (r.isSelected()){
                selectedName = r.getRouteName();
                selected = r;
                break;
            }
        }
        if (selected != null){
            try {
                File f = getRouteFile(selectedName);
                f.delete();
                routes.remove(selected);
            }
            catch (Exception ex){
                telemetry.addData("Error", ex.getMessage());
                telemetry.update();
            }
        }
    }



    public  List<String> listBotActions() {
        final List<String> methods = new ArrayList<String>();
        Class<?> klass = this.bot.getClass();
        while (klass != Object.class) { // need to iterated thought hierarchy in order to retrieve methods from above the current instance
            // iterate though the list of methods declared in the class represented by klass variable, and add those annotated with the specified annotation
            for (final Method method : klass.getDeclaredMethods()) {
                if (method.isAnnotationPresent(BotAction.class)) {
                    BotAction annotInstance = method.getAnnotation(BotAction.class);
                    methods.add(annotInstance.displayName());
                }
            }
            // move to the upper class in the hierarchy in search for more methods
            klass = klass.getSuperclass();
        }
        return methods;
    }

    public  Method findBotActionMethod(String actionName) {
        Method m = null;
        final List<String> methods = new ArrayList<String>();
        Class<?> klass = this.bot.getClass();
        while (klass != Object.class) { // need to iterated thought hierarchy in order to retrieve methods from above the current instance
            // iterate though the list of methods declared in the class represented by klass variable, and add those annotated with the specified annotation
            for (final Method method : klass.getDeclaredMethods()) {
                if (method.isAnnotationPresent(BotAction.class)) {
                    BotAction annotInstance = method.getAnnotation(BotAction.class);
                    if (annotInstance.displayName().equals(actionName)){
                        m = method;
                        break;
                    }
                }
            }
            // move to the upper class in the hierarchy in search for more methods
            klass = klass.getSuperclass();
        }
        return m;
    }

    private File getRouteFile(String filename)
    {
        File file = new File(filename);
        if (!file.isAbsolute())
        {
            AppUtil.getInstance().ensureDirectoryExists(ROUTES_FOLDER);
            file = new File(ROUTES_FOLDER, filename);
        }
        return file;
    }

    private void initRoute(){
        if(newRoute != null){
            newRoute.getSteps().clear();
            newRoute = null;
        }
        newRoute = new AutoRoute();
    }

    private void listRoutes(){
        try {
            int count = 0;
            AppUtil.getInstance().ensureDirectoryExists(ROUTES_FOLDER);
            File [] list = ROUTES_FOLDER.listFiles();
            if (list != null && list.length > 0) {
                for (final File rf : ROUTES_FOLDER.listFiles()) {
                    String jsonData = ReadWriteFile.readFile(rf);
                    AutoRoute route = AutoRoute.deserialize(jsonData);
                    if (count == 0) {
                        route.setSelected(true);
                    } else {
                        route.setSelected(false);
                    }
                    this.routes.add(route);
                    count++;
                }
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
    }

}
