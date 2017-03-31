package org.iowacityrobotics.y2017;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.util.collection.Pair;
import org.iowacityrobotics.roboed.util.math.Maths;
import org.iowacityrobotics.roboed.util.math.Vector4;

/**
 * VISION DATA STUFF!!!!!
 * @author Evan Geng
 */
public class VisionDataProvider extends Source<Pair<Vector4, Vector4>> {
    
    private final NetworkTable tbl;

    public static long lastFrameTime = -1L;

    public static long timeDiff() {
        return lastFrameTime == -1L ? 0L : System.currentTimeMillis() - lastFrameTime;
    }
    
    public VisionDataProvider() {
        this.tbl = NetworkTable.getTable("gearPlacerVision");
    }
    
    @Override
    public Pair<Vector4, Vector4> get() {
        double[] arrX, arrY, arrW, arrH; // The arrays for x, y, width, and height values
        if ((arrX = tbl.getNumberArray("x", (double[])null)) == null) // Assign x values to arrX
            return null; // Abort if the assignment fails (i.e. the table entry doesn't exist)
        if ((arrY = tbl.getNumberArray("y", (double[])null)) == null) // Same for y
            return null;
        if ((arrW = tbl.getNumberArray("w", (double[])null)) == null) // Same for width
            return null;
        if ((arrH = tbl.getNumberArray("h", (double[])null)) == null) // Same for height
            return null;
        if (arrX.length < 2) // Case: there are less than two contours
            return null;
        lastFrameTime = System.currentTimeMillis();
        if (arrX.length == 2) { // Case: there are only two contours
            return Pair.of( // Just return them
                    new Vector4(arrX[0], arrY[0], arrW[0], arrH[0]),
                    new Vector4(arrX[1], arrY[1], arrW[1], arrH[1])
            );
        } else if (arrX.length == 3) { // Case: there are three contours
            double dX_ab = Math.abs(arrX[0] - arrX[1]); // Find the difference in X coordinates for A and B
            double dX_bc = Math.abs(arrX[1] - arrX[2]); // Same for B and C
            double dX_ac = Math.abs(arrX[0] - arrX[2]); // Same for A and C
            if (dX_ab > 100 && dX_bc > 100 && dX_ac > 100) // If none of them are within 100 of another one...
                return null; // Abort
            if (dX_ab < dX_bc && dX_ab < dX_ac) { // A and B have the least diff
                double h = Math.abs(arrY[1] - arrY[0]) + 0.5D * (arrH[0] + arrH[1]); // Find the height of the contour that should be there
                int lowInd = arrY[0] < arrY[1] ? 0 : 1; // Find the index of the contour with a lower Y value
                return Pair.of(
                        new Vector4( // Create a new contour with...
                                Maths.average(arrX[0], arrX[1]), // An x value equal to the average of those of A and B
                                arrY[lowInd] + (h - arrH[lowInd]) / 2D, // A y value equal to the topmost contour's top edge plus half the height
                                Maths.average(arrW[0], arrW[1]), // A width equal to the average of those of A and B
                                h // The height previously calculated
                        ),
                        new Vector4(arrX[2], arrY[2], arrW[2], arrH[2]) // Use C as it is, since it's intact
                );
            } else if (dX_bc < dX_ac) { // Same for B and C
                double h = Math.abs(arrY[2] - arrY[1]) + 0.5D * (arrH[1] + arrH[2]);
                int lowInd = arrY[1] < arrY[2] ? 0 : 1;
                return Pair.of(
                        new Vector4(
                                Maths.average(arrX[1], arrX[2]),
                                arrY[lowInd] + (h - arrH[lowInd]) / 2D,
                                Maths.average(arrW[1], arrW[2]),
                                h
                        ),
                        new Vector4(arrX[0], arrY[0], arrW[0], arrH[0])
                );
            } else { // Same for A and C
                double h = Math.abs(arrY[2] - arrY[0]) + 0.5D * (arrH[0] + arrH[2]);
                int lowInd = arrY[0] < arrY[2] ? 0 : 1;
                return Pair.of(
                        new Vector4(
                                Maths.average(arrX[0], arrX[2]),
                                arrY[lowInd] + (h - arrH[lowInd]) / 2D,
                                Maths.average(arrW[0], arrW[2]),
                                h
                        ),
                        new Vector4(arrX[1], arrY[1], arrW[1], arrH[1])
                );
            }
        } else { // Case: more than three contours
            return null; // Abort
        }
    }
    
}
