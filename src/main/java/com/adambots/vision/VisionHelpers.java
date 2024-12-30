
package com.adambots.vision;

import com.adambots.vision.LimelightHelpers.LimelightTarget_Detector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;


public class VisionHelpers {
    private VisionHelpers() {}

    public static void setPipeline(String limelight,int pipeline) {
        LimelightHelpers.setPipelineIndex(limelight, pipeline); 
    }
    
    public static void blinkLight(String limelight) {
        LimelightHelpers.setLEDMode_ForceBlink(limelight);
    }

    public static void offLight(String limelight) {
        LimelightHelpers.setLEDMode_ForceOff(limelight);
    }
    
     public static int getHeartbeat(String limelight) {
        return (int) NetworkTableInstance.getDefault().getTable(limelight).getEntry("hb").getDouble(0);
    }

    public static Pose3d getCameraPoseTargetSpace(String limelight) {
        return LimelightHelpers.getBotPose3d_TargetSpace(limelight);
    }

    public static double getAprilHorizDist(String limelight){
        return Math.hypot(getCameraPoseTargetSpace(limelight).getX(), getCameraPoseTargetSpace(limelight).getZ());
    }

    // public static double getTimestamp(String limelight){
    //     return LimelightHelpers.getLatestResults(limelight).targetingResults.timestamp_LIMELIGHT_publish;
    // }

    // public static Pose2d getAprilTagPose2d() {
    //     // return LimelightHelpers.getLatestResults(VisionConstants.aprilLimelite).targetingResults.getBotPose2d_wpiRed();
    // }

    // public static int getAprilTagID() {
    //     return (int) NetworkTableInstance.getDefault().getTable(VisionConstants.aprilLimelite).getEntry("tid").getDouble(0);
    // }

    // public static int getHeartbeat() {
    //     // return (int) LimelightHelpers.getFiducialID(VisionConstants.aprilLimelite);
    //     return (int) NetworkTableInstance.getDefault().getTable(VisionConstants.aprilLimelite).getEntry("hb").getDouble(0);
    //     // return null;
    // }

    public static Pose2d getAprilTagBotPose2dBlue(String limelight) {
        return LimelightHelpers.getLatestResults(limelight).targetingResults.getBotPose2d_wpiBlue();  
    }

    public static String getClassName(String limelight) {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(limelight).targetingResults.targets_Detector;

        if (targetDetector == null || targetDetector.length == 0) {
            return "";
        }

        return targetDetector[0].className;
    }

    public static boolean isDetectingPieces(String limelight) {
        return !getClassName(limelight).equals("");
    }

    public static boolean isDetectingPieces(String limelight, String type){
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(limelight).targetingResults.targets_Detector;
        
        int index = -1;
        if(targetDetector != null){
         for(int i = targetDetector.length - 1; i >= 0; i--){
            if(targetDetector[i].className.equals(type)){
                index = i;
            }
         }
        }

        return index != -1;
    }

    public static double getXLocation(String limelight) {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(limelight).targetingResults.targets_Detector;

        if (targetDetector == null || targetDetector.length == 0) {
            return -1.0;
        }

        return targetDetector[0].tx_pixels;
    }

    public static double getXLocation(String limelight, String type) {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(limelight).targetingResults.targets_Detector;

        int index = -1;
        if(targetDetector != null){
         for(int i = targetDetector.length - 1; i >= 0; i--){
            if(targetDetector[i].className.equals(type)){
                index = i;
            }
         }
        }

        if(index == -1){
            return -1.0;
        }

        return targetDetector[index].tx;
    }

    public static double getYLocation(String limelight) {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(limelight).targetingResults.targets_Detector;

        if (targetDetector == null || targetDetector.length == 0) {
            return -1.0;
        }

        return targetDetector[0].ty_pixels;
    }

    public static double getYLocation(String limelight, String type){
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(limelight).targetingResults.targets_Detector;

        int index = -1;
        if(targetDetector != null){
         for(int i = targetDetector.length - 1; i >= 0; i--){
            if(targetDetector[i].className.equals(type)){
                index = i;
            }
         }
        }

        if(index == -1){
            return -1.0;
        }

        return targetDetector[index].ty;
    }

    public static boolean isAligned(String limelight, double thresholdDegrees){
        return Math.abs(getHorizAngle(limelight)) < thresholdDegrees && isDetected(limelight);
    }

    public static double getHorizAngle(String limelight) { //Use this for horizontal alignment
        return LimelightHelpers.getTX(limelight);
    }

    public static double getVertAngle(String limelight) { //Use this for rough distance estimation
        return LimelightHelpers.getTY(limelight);
    }

    public static double getGamePieceArea(String limelight) {
        return LimelightHelpers.getTA(limelight);
    }

    public static boolean isDetected(String limelight) {
        return LimelightHelpers.getTV(limelight);
    }
}