package util.AIField;

import java.util.ArrayList;
import java.util.List;


/**
 * Represents a 2D point on the field. Contains a
 * Heuristic function which is used in A* Pathfinding and
 * connections to nearby nodes.
 */
public class FieldNode implements Comparable<FieldNode> {
    boolean isValid = true;

    public FieldNode parent = null;
    double xValue = 0;
    double yValue = 0;
    double nodeWeight = 0;

    public List<FieldNode.Edge> neighbors;

    public double f = Double.MAX_VALUE;
    public double g = Double.MAX_VALUE;

    FieldNode(double xValue, double yValue){
        this.xValue = xValue;
        this.yValue = yValue;
        this.neighbors = new ArrayList<>();
    }

    @Override
    public int compareTo(FieldNode n) {
        return Double.compare(this.f, n.f);
    }

    public static class Edge {
        Edge(double weight, FieldNode node){
            this.weight = weight;
            this.node = node;
        }

        public double weight;
        public FieldNode node;
    }

    public void addBranch(double weight, FieldNode node){
        FieldNode.Edge newEdge = new FieldNode.Edge(weight, node);
        neighbors.add(newEdge);
    }

    public double calculateHeuristic(FieldNode target){
        double xDist = target.xValue - xValue;
        double yDist = target.yValue - yValue;

        return Math.sqrt(xDist * xDist + yDist * yDist) + nodeWeight;
    }

    @Override
    public String toString() {
        return xValue + " " + yValue;
    }
}
