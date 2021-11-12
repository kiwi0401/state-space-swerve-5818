package util;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.util.*;
import java.util.List;
import java.util.concurrent.*;

/**
 * Adds the "AI Mesh" Camera to the CameraServer which is
 * able to be updated with paths as the robot calculates them.
 * <p>
 * The Thread is only here for testing purposes only,
 * Updating the path should be done by calling updatePath
 */
public class AIFieldDisplay implements Runnable {
    private final CvSource outputStream;
    private final FieldMesh fieldMesh;
    private final int imgWidth;
    private final int imgHeight;
    private static List<FieldNode> path = Collections.synchronizedList(new ArrayList<>());
    private static volatile Trajectory generatedTrajectory = new Trajectory();
    private static volatile boolean needsToRender = false;

    public AIFieldDisplay(FieldMesh fieldMesh, int updateRate) throws IOException {
        int scalingFactor = 1000;
        imgHeight = scalingFactor;
        this.fieldMesh = fieldMesh;
        imgWidth = (int) (scalingFactor * ((double) fieldMesh.fieldWidth / fieldMesh.fieldHeight));
        this.outputStream = CameraServer.getInstance().putVideo("AI Mesh", imgWidth, imgHeight);
        startFieldThread(updateRate);
    }

    private void startFieldThread(int updateRate) {
        ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();

        Mat fieldMat = new Mat(imgHeight, imgWidth, CvType.CV_8UC(4), Scalar.all(100));
        Mat fieldRef = createField();

        executor.scheduleAtFixedRate(() -> {
                    try {
                        if (needsToRender) {
                            fieldRef.copyTo(fieldMat);
                            List<FieldNode> tmp = path;
                            if (generatedTrajectory != null) {
                                for (double i = 0; i < generatedTrajectory.getTotalTimeSeconds(); i += 0.1) {
                                    if (generatedTrajectory.getTotalTimeSeconds() < i + 0.1) continue;
                                    var pose1 = generatedTrajectory.sample(i);
                                    var pose2 = generatedTrajectory.sample(i + 0.1);

                                    Imgproc.arrowedLine(fieldMat, new Point(pose1.poseMeters.getX() * 100, pose1.poseMeters.getY() * 100),
                                            new Point(pose2.poseMeters.getX() * 100, pose2.poseMeters.getY() * 100), new Scalar(39, 107, 242), 10);
                                }
                            }

                            for (int i = 1; i < tmp.size(); i++) {
                                var n1 = tmp.get(i - 1);
                                var n2 = tmp.get(i);
                                Imgproc.arrowedLine(fieldMat, new Point(n1.xValue, n1.yValue), new Point(n2.xValue, n2.yValue), new Scalar(0, 0, 255), 2);
                            }

                            needsToRender = false;
                        }
                        outputStream.putFrame(fieldMat);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }, 0, updateRate, TimeUnit.MILLISECONDS
        );
    }

    private void updatePath(List<FieldNode> path, Trajectory trajectory) {
        AIFieldDisplay.path.clear();
        AIFieldDisplay.path.addAll(path);
        generatedTrajectory = trajectory;
        needsToRender = true;
    }

    private Mat createField() {
        Mat field = new Mat(imgHeight, imgWidth, CvType.CV_8UC(4), Scalar.all(100));
        var nodes = fieldMesh.getFieldNodes();
        for (var n : nodes) {
            for (var p : n) {
                if (!p.isValid) continue;
                for (var e : p.neighbors) {
                    Point a = new Point(p.xValue, p.yValue);
                    Point b = new Point(e.node.xValue, e.node.yValue);
                    Imgproc.line(field, a, b, new Scalar(0, 255, 0), 1);
                }
            }
        }

        Scalar color = new Scalar(64, 64, 64);
        int lineType = Imgproc.LINE_8;
        var fieldObstacles = fieldMesh.getObstacles();
        for (var obstacle : fieldObstacles) {
            List<MatOfPoint> list = new ArrayList<>();
            Point[] pts = new Point[obstacle.npoints];
            for (int i = 0; i < obstacle.npoints; i++) {
                pts[i] = (new Point(obstacle.xpoints[i], obstacle.ypoints[i]));
            }
            list.add(new MatOfPoint(pts));
            Imgproc.fillPoly(field, list, color, lineType);
        }
        return field;
    }

    @Override
    public void run() {
        ScheduledExecutorService exe = Executors.newSingleThreadScheduledExecutor();
        exe.scheduleAtFixedRate(
                () -> {
                    List<FieldNode> tmp = null;
                    while (tmp == null) {
                        int startx = (int) (fieldMesh.fieldWidth * Math.random());
                        int endx = (int) (fieldMesh.fieldWidth * Math.random());
                        int starty = (int) (fieldMesh.fieldHeight * Math.random());
                        int endy = (int) (fieldMesh.fieldHeight * Math.random());
                        tmp = fieldMesh.getPath(startx, starty, endx, endy);
                        generatedTrajectory = fieldMesh.getTrajectory(startx / 100.0, starty / 100.0, endx / 100.0, endy / 100.0, true, 0);
                    }
                    updatePath(tmp, generatedTrajectory);
                }, 0, 500, TimeUnit.MILLISECONDS
        );
    }
}
