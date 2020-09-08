package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import java.io.Serializable;
import java.util.ArrayList;

public class AutoRoute implements Serializable {
    public static String NAME_BLUE = "Blue";
    public static String NAME_RED = "Red";
    private int nameIndex = 1;
    private String name = NAME_BLUE;
    private boolean selected;
    private ArrayList<AutoStep> steps = new ArrayList<>();
    private int startX;
    private int startY;

    public String serialize() {
        return SimpleGson.getInstance().toJson(this);
    }
    public static AutoRoute deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data, AutoRoute.class);
    }

    public String getRouteName()
    {
        return  String.format("%s-%d", name, nameIndex);
    }


    public ArrayList<AutoStep> getSteps() {
        return steps;
    }

    public void setSteps(ArrayList<AutoStep> steps) {
        this.steps = steps;
    }

    public boolean isSelected() {
        return selected;
    }

    public void setSelected(boolean selected) {
        this.selected = selected;
    }

    public int getNameIndex() {
        return nameIndex;
    }

    public void setNameIndex(int nameIndex) {
        this.nameIndex = nameIndex;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getStartX() {
        return startX;
    }

    public void setStartX(int startX) {
        this.startX = startX;
    }

    public int getStartY() {
        return startY;
    }

    public void setStartY(int startY) {
        this.startY = startY;
    }
}

