package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;

enum BallColor {
    GREEN,
    PURPLE,
    NONE
}

public class ColorIdentifier {
    private ColorSensor colorSensor;
    private double pMinG;
    private double pMaxG;
    private double pMinR;
    private double pMaxR;
    private double pMinB;
    private double pMaxB;
    private double gMinG;
    private double gMaxG;
    private double gMinR;
    private double gMaxR;
    private double gMinB;
    private double gMaxB;


    public double getgMaxB() {
        return gMaxB;
    }

    public void setgMaxB(double gMaxB) {
        this.gMaxB = gMaxB;
    }

    public double getgMinB() {
        return gMinB;
    }

    public void setgMinB(double gMinB) {
        this.gMinB = gMinB;
    }

    public double getgMaxR() {
        return gMaxR;
    }

    public void setgMaxR(double gMaxR) {
        this.gMaxR = gMaxR;
    }

    public double getgMinR() {
        return gMinR;
    }

    public void setgMinR(double gMinR) {
        this.gMinR = gMinR;
    }

    public double getgMaxG() {
        return gMaxG;
    }

    public void setgMaxG(double gMaxG) {
        this.gMaxG = gMaxG;
    }

    public double getgMinG() {
        return gMinG;
    }

    public void setgMinG(double gMinG) {
        this.gMinG = gMinG;
    }

    public double getpMaxB() {
        return pMaxB;
    }

    public void setpMaxB(double pMaxB) {
        this.pMaxB = pMaxB;
    }

    public double getpMinB() {
        return pMinB;
    }

    public void setpMinB(double pMinB) {
        this.pMinB = pMinB;
    }

    public double getpMaxR() {
        return pMaxR;
    }

    public void setpMaxR(double pMaxR) {
        this.pMaxR = pMaxR;
    }

    public double getpMinR() {
        return pMinR;
    }

    public void setpMinR(double pMinR) {
        this.pMinR = pMinR;
    }

    public double getpMaxG() {
        return pMaxG;
    }

    public void setpMaxG(double pMaxG) {
        this.pMaxG = pMaxG;
    }

    public double getpMinG() {
        return pMinG;
    }

    public void setpMinG(double pMinG) {
        this.pMinG = pMinG;
    }
    public void setgMaxB(int gMaxB) {
        this.gMaxB = gMaxB;
    }







    public ColorSensor getColorSensor() {
        return colorSensor;
    }


    private void setColorSensor(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    /**
     * Constructor for the ColorIdentifier class.
     * @param colorSensor The color sensor to use.
     * @param pMinR The minimum red value for the purple color.
     * @param pMaxR The maximum red value for the purple color.
     * @param pMinG The minimum green value for the purple color.
     * @param pMaxG The maximum green value for the purple color.
     * @param pMinB The minimum blue value for the purple color.
     * @param pMaxB The maximum blue value for the purple color.
     * @param gMinR The minimum red value for the green color.
     * @param gMaxR The maximum red value for the green color.
     * @param gMinG The minimum green value for the green color.
     * @param gMaxG The maximum green value for the green color.
     * @param gMinB The minimum blue value for the green color.
     * @param gMaxB The maximum blue value for the green color.
     */
    public ColorIdentifier(ColorSensor colorSensor, double pMinR, double pMaxR, double pMinG, double pMaxG, double pMinB, double pMaxB, double gMinR, double gMaxR, double gMinG, double gMaxG, double gMinB, double gMaxB) {
        this.pMinR = pMinR;
        this.pMaxR = pMaxR;
        this.pMinG = pMinG;
        this.pMaxG = pMaxG;
        this.pMinB = pMinB;
        this.pMaxB = pMaxB;
        this.gMinR = gMinR;
        this.gMaxR = gMaxR;
        this.gMinG = gMinG;
        this.gMaxG = gMaxG;
        this.gMinB = gMinB;
        this.gMaxB = gMaxB;
        this.colorSensor = colorSensor;


    }

    public BallColor getColor(){
        BallColor result;
        double r = colorSensor.red();
        double g = colorSensor.green();
        double b = colorSensor.blue();

        double pBuffer = 15;
        double gBuffer = 15;

        if (r >= (pMinR - pBuffer) && r <= (pMaxR + pBuffer) &&
                g >= (pMinG - pBuffer) && g <= (pMaxG + pBuffer) &&
                b >= (pMinB - pBuffer) && b <= (pMaxB + pBuffer)){
            result = BallColor.PURPLE;
        }
        else if (r >= (gMinR - gBuffer) && r <= (gMaxR + gBuffer) &&
                g >= (gMinG - gBuffer) && g <= (gMaxG + gBuffer) &&
                b >= (gMinB - gBuffer) && b <= (gMaxB + gBuffer)){
            result = BallColor.GREEN;
        }
        else {
            result = BallColor.NONE;
        }
        return result;
    }


}
