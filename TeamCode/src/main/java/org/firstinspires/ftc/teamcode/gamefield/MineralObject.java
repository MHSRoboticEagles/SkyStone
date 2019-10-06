package org.firstinspires.ftc.teamcode.gamefield;

import android.support.annotation.NonNull;

public class MineralObject implements Comparable<MineralObject>{
    private String label = "";
    private float leftPos = -1;

    public MineralObject(){

    }

    public MineralObject(String label, float posX){
        this.leftPos = posX;
        this.label = label;
    }

    @Override
    public String toString() {
        return String.format("%s @ %.2f", this.label, this.leftPos);
    }

    @Override
    public int compareTo(@NonNull MineralObject mineralObject) {
        if(this.leftPos < mineralObject.getLeftPos()) {
            return -1;
        }
        if(this.leftPos > mineralObject.getLeftPos()){
            return 1;
        }
        return 0;
    }

    public String getLabel() {
        return label;
    }

    public void setLabel(String label) {
        this.label = label;
    }

    public float getLeftPos() {
        return leftPos;
    }

    public void setLeftPos(float leftPos) {
        this.leftPos = leftPos;
    }
}
