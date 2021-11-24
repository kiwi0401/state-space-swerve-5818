package util.ML;

public class MLObject {
    public final String label;
    public final BoundingBox boundingBox;
    public final double tx;
    public final double ty;
    public final double confidence;

    public MLObject(String label, BoundingBox boundingBox, double confidence) {
        this.label = label;
        this.boundingBox = boundingBox;

        double avgX = (boundingBox.xmax + boundingBox.xmin) / 2.0;
        tx = (avgX - MLCore.CAMERA_WIDTH / 2.0) * MLCore.ANGLE_PER_PIXEL_X;

        double avgY = (boundingBox.ymax + boundingBox.ymin) / 2.0;
        ty = (avgY - MLCore.CAMERA_HEIGHT / 2.0) * MLCore.ANGLE_PER_PIXEL_Y;

        this.confidence = confidence;
    }
}
