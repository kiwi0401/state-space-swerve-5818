package util.ML;

public class BoundingBox {
    public final int ymin;
    public final int ymax;
    public final int xmin;
    public final int xmax;
    public BoundingBox(int ymin, int ymax, int xmin, int xmax) {
        this.ymin = ymin;
        this.ymax = ymax;
        this.xmin = xmin;
        this.xmax = xmax;
    }
}
