package util.AIField;

import java.util.ArrayList;

public class AreaWeight {
    public double weight;
    public int x1;
    public int y1;
    public int x2;
    public int y2;

    public AreaWeight(double weight, int x1, int y1, int x2, int y2) {
        this.weight = weight;
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
    }

    public boolean containsNode(FieldNode node) {
        if(node.xValue >= x1 && node.xValue <= x2) {
            return node.yValue >= y1 && node.yValue <= y2;
        }
        return false;
    }
}
