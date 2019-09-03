package smartsystem;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;

/**
 * Created by ves on 6/26/2015.
 */
public class RobotProcess implements Serializable {
    private String toolX;
    private String toolY;
    private String toolZ;

    private String forceX;
    private String forceY;
    private String forceZ;

    private String speedX;
    private String speedY;
    private String speedZ;

    private String iteration;
    private String elapsedTime;
    private String currElapsedTime;

    private String currStep;
    private String prevStep;
    private String nextStep;

    public RobotProcess() {

    }

    private void writeObject(
            ObjectOutputStream aOutputStream
    ) throws IOException {
        //perform the default serialization for all non-transient, non-static fields
        aOutputStream.defaultWriteObject();
    }


    private void readObject(
            ObjectInputStream aInputStream
    ) throws ClassNotFoundException, IOException {
        //always perform the default de-serialization first
        aInputStream.defaultReadObject();

    }

    private static final long serialVersionUID = 0L;


    public String toString() {
        String s = "toolX: " + toolX + "\n";
        s += "toolY: " + toolY + "\n";
        s += "toolZ: " + toolZ + "\n";
        s += "forceX: " + forceX + "\n";
        s += "forceY: " + forceY + "\n";
        s += "forceZ: " + forceZ + "\n";
        s += "speedX: " + speedX + "\n";
        s += "speedY: " + speedY + "\n";
        s += "speedZ: " + speedZ + "\n";
        s += "iteration: " + iteration + "\n";
        s += "elapsed time: " + elapsedTime + "\n";
        s += "current step: " + currStep + "\n";
        s += "previous step: " + prevStep + "\n";
        s += "next step: " + nextStep + "\n";

        return s;
    }

    public String getToolX() {
        return toolX;
    }
    public void setToolX(String x) {
        toolX = x;
    }
    public String getToolY() {
        return toolY;
    }
    public void setToolY(String y) {
        toolY = y;
    }
    public String getToolZ() {
        return toolZ;
    }
    public void setToolZ(String z) {
        toolZ = z;
    }
    public String getIteration() {
        return iteration;
    }
    public void setIteration(String iteration) {
        this.iteration = iteration;
    }
    public String getElapsedTime() {
        return elapsedTime;
    }
    public void setElapsedTime(String elapsedTime) {
        this.elapsedTime = elapsedTime;
    }
    public String getCurrStep() {
        return currStep;
    }
    public void setCurrStep(String currStep) {
        this.currStep = currStep;
    }
    public String getPrevStep() {
        return prevStep;
    }
    public void setPrevStep(String prevStep) {
        this.prevStep = prevStep;
    }
    public String getNextStep() {
        return nextStep;
    }
    public void setNextStep(String nextStep) {
        this.nextStep = nextStep;
    }
    public String getForceX() {
        return forceX;
    }

    public void setForceX(String forceX) {
        this.forceX = forceX;
    }

    public String getForceY() {
        return forceY;
    }

    public void setForceY(String forceY) {
        this.forceY = forceY;
    }

    public String getForceZ() {
        return forceZ;
    }

    public void setForceZ(String forceZ) {
        this.forceZ = forceZ;
    }

    public String getSpeedX() {
        return speedX;
    }

    public void setSpeedX(String speedX) {
        this.speedX = speedX;
    }

    public String getSpeedY() {
        return speedY;
    }

    public void setSpeedY(String speedY) {
        this.speedY = speedY;
    }

    public String getSpeedZ() {
        return speedZ;
    }

    public void setSpeedZ(String speedZ) {
        this.speedZ = speedZ;
    }

    public String getCurrElapsedTime() {
        return currElapsedTime;
    }

    public void setCurrElapsedTime(String currElapsedTime) {
        this.currElapsedTime = currElapsedTime;
    }


}
