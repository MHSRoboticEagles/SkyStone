package org.firstinspires.ftc.teamcode.calibration;

import java.util.ArrayList;

public class MotorReductionList extends ArrayList<MotorReduction> {
    public MotorReductionList(){
        this.add(new MotorReduction(MotorName.LF));
        this.add(new MotorReduction(MotorName.LB));
        this.add(new MotorReduction(MotorName.RF));
        this.add(new MotorReduction(MotorName.RB));
    }

    public double getMotorReduction(MotorName name){
        double reduction = 1;
        for(MotorReduction mr : this ){
            if (mr.getMotorName() == name){
                reduction = mr.getMotorReduction();
            }
        }
        return reduction;
    }

    public void updateList(MotorReduction mr){
        for(MotorReduction obj : this ){
            if (obj.getMotorName() == mr.getMotorName()){
                obj.setMotorReduction(mr.getMotorReduction());
            }
        }
    }

    public void restoreList(){
        for(MotorReduction mr : this ){
            mr.setMotorReduction(MotorReduction.DEFAULT_REDUCTION);
        }
    }

    public MotorReduction getFirsttMR(){
        return this.get(0);
    }

    public MotorReduction getNextMR(MotorName currentMRName){
        int index = -1;
        for (int i= 0; i < this.size(); i++){
            MotorReduction mr = this.get(i);
            if (mr.getMotorName() == currentMRName){
                index = i;
                break;
            }
        }
        if (index + 1 < this.size()){
            return this.get(index +1);
        }
        else{
            return  null;
        }
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (int i= 0; i < this.size(); i++){
            MotorReduction mr = this.get(i);
            sb.append(String.format("%s:%.02f", mr.getMotorName(), mr.getMotorReduction()));
        }
        return sb.toString();
    }
}
