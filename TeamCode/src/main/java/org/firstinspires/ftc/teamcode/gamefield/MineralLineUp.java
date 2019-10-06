package org.firstinspires.ftc.teamcode.gamefield;

import org.firstinspires.ftc.teamcode.skills.GoldPosition;

import java.util.ArrayList;
import java.util.Collections;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;

public class MineralLineUp {
    private static final int CENTER_X = 250;
    private ArrayList<MineralObject> lineUp = new ArrayList<>();
    private GoldPosition goldPosition = GoldPosition.None;
    private boolean goldFound = false;
    private int goldIndex = -1;
    private int view = 0;
    private int actualIndex = -1;

    public MineralLineUp(ArrayList<MineralObject> objectsDetected, boolean withGold, int view){
        for(MineralObject obj :objectsDetected){
            this.lineUp.add(obj);
        }

        this.lineUp = objectsDetected;
        goldFound = withGold;
        this.view = view;
    }

    public static boolean isGold(MineralObject obj){
        if (obj == null || obj.getLabel() == null){
            return false;
        }
        return obj.getLabel().equals(LABEL_GOLD_MINERAL);
    }


    @Override
    public String toString() {
        String s = "LineUp: ";
        String objects = "";
        for(MineralObject obj : lineUp){
            objects += obj.toString() + ";";
        }

        return String.format("%s %s. View: %d. Found: %b. Position: %s. Gold Index:  %d, Actual: %d", s, objects, view, goldFound, goldPosition.name(), goldIndex, actualIndex);
    }

    public GoldPosition compute(){
        Collections.sort(lineUp);
        if (isGoldFound()){
            for(int i = 0; i < lineUp.size(); i++){
                MineralObject obj = lineUp.get(i);
                if (obj.getLabel().equals(LABEL_GOLD_MINERAL)){
                    setGoldIndex(i);
                    break;
                }
            }
        }
        if (lineUp.size() == 3){
            actualIndex = getGoldIndex();
        }
        else if (lineUp.size() == 2){
            //2 elements in the frame
            if (getGoldIndex() >= 0){
                //first 2
                if (view == -1){
                    actualIndex = getGoldIndex();
                }
                //last 2
                else if (view == 1){
                    actualIndex = getGoldIndex() + 1;
                }
            }
            else{
                //gold is last
                if (view == -1){
                    actualIndex = 2;
                }
                //gold is first
                else if (view == 1){
                    actualIndex = 0;
                }
            }
        }
        else if (goldFound && lineUp.size() == 1){
            MineralObject obj = lineUp.get(0);
            if (view == -1){
                if (obj.getLeftPos() < CENTER_X){
                    actualIndex = 0;
                }
                else{
                    actualIndex = 1;
                }
            }
            else if (view == 1){
                if (obj.getLeftPos() < CENTER_X){
                    actualIndex = 1;
                }
                else{
                    actualIndex = 2;
                }
            }
        }
        switch (actualIndex){
            case 0:
                setGoldPosition(GoldPosition.Left);
                break;
            case 1:
                setGoldPosition(GoldPosition.Center);
                break;
            case 2:
                setGoldPosition(GoldPosition.Right);
                break;
        }

        return getGoldPosition();
    }


    public GoldPosition getGoldPosition() {
        return goldPosition;
    }

    public void setGoldPosition(GoldPosition goldPosition) {
        this.goldPosition = goldPosition;
    }

    public boolean isGoldFound() {
        return goldFound;
    }

    public void setGoldFound(boolean goldFound) {
        this.goldFound = goldFound;
    }

    public int getGoldIndex() {
        return goldIndex;
    }

    public void setGoldIndex(int goldIndex) {
        this.goldIndex = goldIndex;
    }

    public void Dispose(){
        this.lineUp.clear();
    }
}
