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
    
    public VisionDataProvider() {
        this.tbl = NetworkTable.getTable("myContoursReport");
    }
    
    @Override
    public Pair<Vector4, Vector4> get() {
        double[] arrX, arrY, arrW, arrH;
        if ((arrX = tbl.getNumberArray("x", (double[])null)) == null)
            return null;
        if ((arrY = tbl.getNumberArray("y", (double[])null)) == null)
            return null;
        if ((arrW = tbl.getNumberArray("w", (double[])null)) == null)
            return null;
        if ((arrH = tbl.getNumberArray("h", (double[])null)) == null)
            return null;
        if (arrX.length == 2) {
            return Pair.of(
                    new Vector4(arrX[0], arrY[0], arrW[0], arrH[0]),
                    new Vector4(arrX[1], arrY[1], arrW[1], arrH[1])
            );
        } else if (arrX.length == 3) {
            double dX_ab = Math.abs(arrX[0] - arrX[1]);
            double dX_bc = Math.abs(arrX[1] - arrX[2]);
            double dX_ac = Math.abs(arrX[0] - arrX[2]);
            if (dX_ab + dX_bc + dX_ac > 300)
                return null;
            if (dX_ab < dX_bc && dX_ab < dX_ac) { // A and B have the least diff
                double h = Math.abs(arrY[1] - arrY[0]) + 0.5D * (arrH[0] + arrH[1]);
                int lowInd = arrY[0] < arrY[1] ? 0 : 1;
                return Pair.of(
                        new Vector4(
                                Maths.average(arrX[0], arrX[1]),
                                arrY[lowInd] + (h - arrH[lowInd]) / 2D,
                                Maths.average(arrW[0], arrW[1]),
                                h
                        ),
                        new Vector4(arrX[2], arrY[2], arrW[2], arrH[2])
                );
            } else if (dX_bc < dX_ac) { // B and C have the least diff
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
            } else { // A and C have the least diff
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
        } else {
            return null;
        }
    }
    
}
