/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.detections.Detections;

import java.util.Collections;
import java.util.Map;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class Vision extends SubsystemBase {
    private NetworkTable m_tableML = NetworkTableInstance.getDefault().getTable("ML");
    private NetworkTableEntry deviceEntry;
    private NetworkTableEntry resolutionEntry;
    private NetworkTableEntry fspEntry;
    private NetworkTableEntry detectionsEntry;
    private NetworkTableEntry xaxisSpeedEntry;
    private NetworkTableEntry zaxisRotateEntry;

    private Detections[] m_detections;
    private boolean haveDetections = false;

    public Vision() {
        
        deviceEntry = m_tableML.getEntry("device");
        resolutionEntry = m_tableML.getEntry("resolution");  
        
        SmartDashboard.putString("OAK Device", deviceEntry.getString("Not getting device"));
        SmartDashboard.putString("Image Resolution:", resolutionEntry.getString("No resolution"));
    }

    @Override
    public void periodic() {
        // Data from Python 
        detectionsEntry = m_tableML.getEntry("detections");
        parseDetections(detectionsEntry.getString("No detections"));

        xaxisSpeedEntry = m_tableML.getEntry("xaxisSpeed");
        zaxisRotateEntry = m_tableML.getEntry("zaxisRotate"); 
        
        if (haveDetections) {
            fspEntry = m_tableML.getEntry("fsp");
            SmartDashboard.putNumber("FSP", fspEntry.getDouble(0.0));

            SmartDashboard.putString("label", getLabel());
            SmartDashboard.putNumber("ymin", getYMin());
            SmartDashboard.putNumber("xmin", getXMin());
            SmartDashboard.putNumber("ymax", getYMax());
            SmartDashboard.putNumber("xmax", getXMax());
            SmartDashboard.putNumber("confidence", getConfidence());
            SmartDashboard.putNumber("X", getXCoord());
            SmartDashboard.putNumber("Y", getYCoord());
            SmartDashboard.putNumber("Z", getZCoord());
        }
        
    }

    public void parseDetections(String json) {
        //create ObjectMapper instance
		ObjectMapper mapper = new ObjectMapper();     
        try {
            //JSON string to Java Object
            m_detections = mapper.readValue(json, Detections[].class);
            haveDetections = true;
        } catch (JsonProcessingException e) {
            System.out.println(e);
            System.out.println(json);
            haveDetections = false;
        } 	 
    }

    public String getLabel() {
        return m_detections[0].label;
    }

    public int getConfidence() {
        return m_detections[0].confidence;
    }

    public Double getXAxisSpeed() {
        return xaxisSpeedEntry.getDouble(0.0);
    }

    public Double getZAxisRotate() {
        return zaxisRotateEntry.getDouble(0.0);
    }

    // Get bounding box components
    public Map<String, Double> getBox() {
        return m_detections[0].box; 
    }

    public Double getYMin() {
        // System.out.println(getBox());
        Double ymin = getBox().get("ymin");
        if (ymin != null) {
            return ymin;
        }
        return -1.0;
    }

    public Double getXMin() {
        return getBox().get("xmin");
    }
    
    public Double getYMax() {
        return getBox().get("ymax");
    }

    public Double getXMax() {
        return getBox().get("xmax");
    }

    // Get spacial components
    public Map<String, Integer> getSpacial() {
        return m_detections[0].spacial; 
    }

    public Integer getXCoord() {
        return getSpacial().get("X");
    }

    public Integer getYCoord() {
        return getSpacial().get("Y");
    }

    public Integer getZCoord() {
        return getSpacial().get("Z");
    }

}

