package org.iowacityrobotics.y2017;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

import java.util.function.Supplier;

/**
 * Created by icrobotics on 2/16/2017.
 * @author Evan Geng
 */
public class VisionDataProvider implements Supplier<VisionDataProvider.VDF> {

    private static final double[] NONE = new double[] {0};
    
    @Override
    public VDF get() {
        NetworkTable tbl = NetworkTable.getTable("myContoursReport");
        double cX = tbl.getNumberArray("centerX", NONE)[0];
        double cY = tbl.getNumberArray("centerY", NONE)[0];
        double a = tbl.getNumberArray("area", NONE)[0];
        double w = tbl.getNumberArray("width", NONE)[0];
        double h = tbl.getNumberArray("height", NONE)[0];
        return new VDF();
    }

    // Vision data frame
    public static class VDF {



    }
    
}
