package util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.spline.Spline;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpiutil.math.MathUtil;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.awt.*;
import java.awt.geom.Line2D;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.nio.file.Path;
import java.util.*;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;


/**
 * Constructs a Field Mesh using parameters found inside the fieldInfo.txt and fieldObstacles.txt files.
 * A* Pathfinding is used to find Trajectories from point a to point b on the mesh, and the mesh supports weighted paths
 * and object avoidance.
 */
public class FieldMesh {
    private static FieldMesh fieldMesh;
    public static FieldMesh getInstance() throws FileNotFoundException {
        if(fieldMesh == null) fieldMesh = new FieldMesh();
        return fieldMesh;
    }



    public int fieldWidth;
    public int fieldHeight;
    private int aiResolution;
    private static final List<Polygon> fieldObstacles = new ArrayList<>();
    private static final List<ArrayList<FieldNode>> nodes = new ArrayList<>();
    public final double MAX_VELOCITY = 3;
    public final double MAX_ACCELERATION = 0.2;

    public FieldMesh() throws FileNotFoundException {
        Path fieldDimension = Filesystem.getDeployDirectory().toPath().resolve("AI/fieldInfo.txt");
        Path fieldObstacles = Filesystem.getDeployDirectory().toPath().resolve("AI/fieldObstacles.txt");
        Scanner sc = new Scanner(new FileReader(fieldDimension.toFile()));

        sc.next();
        fieldWidth = (int) (sc.nextDouble() * 100);
        sc.next();
        fieldHeight = (int) (sc.nextDouble() * 100);
        sc.next();
        aiResolution = (int) (sc.nextDouble());
        sc.close();

        Pattern parseInput = Pattern.compile("\\(([^)]+)\\)");
        sc = new Scanner(fieldObstacles.toFile());
        sc.nextLine();
        while (sc.hasNextLine()) {
            String obstacle = sc.nextLine();
            Matcher m = parseInput.matcher(obstacle);
            Polygon p = new Polygon();
            while (m.find()) {
                String[] in = m.group(1).split(",");
                p.addPoint(Integer.parseInt(in[0].trim()), Integer.parseInt(in[1].trim()));
            }
            FieldMesh.fieldObstacles.add(p);
        }
        updateFieldMesh();
    }

    /**
     * Creates a field mesh of nodes, mesh is represented as a graph where each
     * node is set as a neighbor of adjacent nodes.
     *
     * */
    private void updateFieldMesh() {
        nodes.clear();
        int cnt = 0;
        for (int i = 0; i <= fieldHeight; i += aiResolution) {
            nodes.add(new ArrayList<>());
            for (int j = 0; j <= fieldWidth; j += aiResolution) {
                var nToAdd = new FieldNode(j, i);
                nToAdd.isValid = isValidPoint(nToAdd);
                nodes.get(cnt).add(nToAdd);
            }
            cnt++;
        }
        for (int i = 0; i < nodes.size(); i++) {
            for (int j = 0; j < nodes.get(i).size(); j++) {
                if (!nodes.get(i).get(j).isValid) continue;
                for (int a = Math.max(0, i - 1); a <= Math.min(i + 1, nodes.size() - 1); a++) {
                    for (int b = Math.max(0, j - 1); b <= Math.min(j + 1, nodes.get(i).size() - 1); b++) {
                        if ((i == a && j == b) || !nodes.get(a).get(b).isValid) continue;
                        if (!isValidEdge(nodes.get(i).get(j), nodes.get(a).get(b))) continue;
                        int xDist = i - a;
                        int yDist = j - b;
                        nodes.get(i).get(j).addBranch(Math.sqrt(xDist * xDist + yDist * yDist), nodes.get(a).get(b));
                    }
                }
            }
        }
    }

    /**
     * Adds an obstacle to the field
     *
     * @param values list of points {double x, double y}(cm) that correspond to a polygons location
     */
    public void addObstacle(final ArrayList<int[]> values) {
        Polygon p = new Polygon();
        for (var pt : values) {
            p.addPoint(pt[0], pt[1]);
        }
        fieldObstacles.add(p);
        updateFieldMesh();
    }

    /**
     * Removes an obstacle from the field
     *
     * @param obstacle the obstacle to remove
     */
    public void removeObstacle(Polygon obstacle) {
        fieldObstacles.remove(obstacle);
        updateFieldMesh();
    }

    /**
     * Sets the resolution of the grid (lower is more accurate).
     * for a grid of size 1m by 1m, a resolution of 10 would produce 100 nodes (10*10).
     *
     * @param resolution resolution for the grid
     */
    public void setAiResolution(int resolution) {
        this.aiResolution = resolution;
        updateFieldMesh();
    }


    /**
     * Add weights to the nodes in an area to penalize traveling through it. negative weights will
     * prioritize a zone more than others.
     *
     * @param weight amount of weight you want to add to a node (default is 1)
     * @param x1 top left x location of zone (cm)
     * @param y1 top left y location of zone (cm)
     * @param x2 bottom right x location of zone (cm)
     * @param y2 bottom right y location of zone (cm)
     */
    public void addWeightToArea(double weight, double x1, double y1, double x2, double y2) {
        int cNodeX1 = MathUtil.clamp((int) (x1 / aiResolution), 0, nodes.get(0).size() - 1);
        int cNodeY1 = MathUtil.clamp((int) (y1 / aiResolution), 0, nodes.size() - 1);
        int cNodeX2 = MathUtil.clamp((int) (x2 / aiResolution), 0, nodes.get(0).size() - 1);
        int cNodeY2 = MathUtil.clamp((int) (y2 / aiResolution), 0, nodes.size() - 1);
        for(int i = cNodeX1; i <= cNodeX2; i++) {
            for(int j = cNodeY1; j <= cNodeY2; j++) {
                nodes.get(j).get(i).nodeWeight = weight;
            }
        }
    }

    public boolean isValidPoint(FieldNode s) {
        for (var obstacle : fieldObstacles) {
            if (obstacle.contains(s.xValue, s.yValue)) return false;
        }
        return true;
    }

    public boolean isValidEdge(FieldNode to, FieldNode from) {
        Line2D nodeLines = new Line2D.Double();
        nodeLines.setLine(to.xValue, to.yValue, from.xValue, from.yValue);
        for (var obstacle : fieldObstacles) {
            for (int i = 1; i < obstacle.npoints; i++) {
                Line2D testIntersect = new Line2D.Double();
                testIntersect.setLine(
                        obstacle.xpoints[i],
                        obstacle.ypoints[i],
                        obstacle.xpoints[i - 1],
                        obstacle.ypoints[i - 1]);
                if (testIntersect.intersectsLine(nodeLines)) return false;
            }
        }
        return true;
    }

    public List<Polygon> getObstacles() {
        return fieldObstacles;
    }

    private double angleBetweenNodes(FieldNode a, FieldNode b) {
        return Math.atan2(b.yValue - a.yValue, b.xValue - a.xValue);
    }

    /**
     *
     * Trajectory Generator which utilizes A* Pathfinding to generate a trajectory that avoids
     * any obstacles on the field.
     *
     * @param x1 start x value (m)
     * @param y1 start y value (m)
     * @param x2 end x value (m)
     * @param y2 end y value (m)
     * @param shouldStop whether the path should stop at 0 m/s or continue at max velocity
     * @param initialVelocity initial velocity of robot. useful for periodic path generation
     * @return Trajectory which corresponds to the start and end values on the field. Avoid objects specified in fieldObstacles.txt
     */
    public Trajectory getTrajectory(double x1, double y1, double x2, double y2, boolean shouldStop, double initialVelocity) {
        try {
            List<FieldNode> path = getPath(x1 * 100.0, y1 * 100.0, x2 * 100.0, y2 * 100.0);
            if(path == null || path.size() <= 1) return null;
            List<Pose2d> poseList = new ArrayList<>();
            var prev = path.get(0);
            poseList.add(new Pose2d(new Translation2d(path.get(0).xValue / 100.0, path.get(0).yValue / 100.0), new Rotation2d(angleBetweenNodes(path.get(0), path.get(1)))));
            int cnt = 0;
            for(FieldNode n : path) {
                cnt++;
                double angle = angleBetweenNodes(prev, n);
                prev = n;
                if(cnt == 1) continue;
                poseList.add(new Pose2d(new Translation2d(n.xValue / 100.0, n.yValue / 100.0), new Rotation2d(angle)));
            }
            TrajectoryConfig config = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION);
            config.setEndVelocity(shouldStop ? 0 : MAX_VELOCITY);
            config.setStartVelocity(initialVelocity);
            return TrajectoryGenerator.generateTrajectory(poseList, config);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }

    /**
     * utilizes A* Pathfinding to return the shortest path to the target while avoiding obstacles
     *
     * @param x1 start x value (cm)
     * @param y1 start y value (cm)
     * @param x2 end x value (cm)
     * @param y2 end y value (cm)
     * @return list of mesh nodes which belong to the path
     */
    public List<FieldNode> getPath(double x1, double y1, double x2, double y2) {
        int cNodeX1 = MathUtil.clamp((int) (x1 / aiResolution), 0, nodes.get(0).size() - 1);
        int cNodeY1 = MathUtil.clamp((int) (y1 / aiResolution), 0, nodes.size() - 1);
        int cNodeX2 = MathUtil.clamp((int) (x2 / aiResolution), 0, nodes.get(0).size() - 1);
        int cNodeY2 = MathUtil.clamp((int) (y2 / aiResolution), 0, nodes.size() - 1);
        FieldNode start = nodes.get(cNodeY1).get(cNodeX1);
        FieldNode end = nodes.get(cNodeY2).get(cNodeX2);
        var n = aStar(start, end);
        return printPath(n);
    }

    /**
     * @return All the nodes that make up the field mesh
     */
    public List<ArrayList<FieldNode>> getFieldNodes() {
        return nodes;
    }

    /**
     * Helper utility for A* Algorithm to retrieve the shortest path after calculation
     */
    public List<FieldNode> printPath(FieldNode target) {
        FieldNode n = target;
        if (n == null) return null;
        List<FieldNode> ids = new ArrayList<>();
        while (n.parent != null) {
            ids.add(n);
            n = n.parent;
        }
        ids.add(n);
        Collections.reverse(ids);
        return ids;
    }

    /**
     * have fun :)
     * https://stackabuse.com/graphs-in-java-a-star-algorithm/
     */
    public FieldNode aStar(FieldNode start, FieldNode target) {
        if (!start.isValid || !target.isValid) return null;

        for (var n : nodes) {
            for (var p : n) {
                p.f = Double.MAX_VALUE;
                p.g = Double.MAX_VALUE;
                p.parent = null;
            }
        }

        PriorityQueue<FieldNode> closedList = new PriorityQueue<>();
        PriorityQueue<FieldNode> openList = new PriorityQueue<>();
        start.g = 0;
        start.f = start.g + start.calculateHeuristic(target);
        openList.add(start);

        while (!openList.isEmpty()) {
            FieldNode n = openList.peek();
            if (n == target) {
                return n;
            }

            for (FieldNode.Edge edge : n.neighbors) {
                FieldNode m = edge.node;
                double totalWeight = n.g + edge.weight + edge.node.nodeWeight;
                if (!openList.contains(m) && !closedList.contains(m)) {
                    m.parent = n;
                    m.g = totalWeight;
                    m.f = m.g + m.calculateHeuristic(target);
                    openList.add(m);
                } else {
                    if (totalWeight < m.g) {
                        m.parent = n;
                        m.g = totalWeight;
                        m.f = m.g + m.calculateHeuristic(target);

                        if (closedList.contains(m)) {
                            closedList.remove(m);
                            openList.add(m);
                        }
                    }
                }
            }

            openList.remove(n);
            closedList.add(n);
        }
        return null;
    }

}
