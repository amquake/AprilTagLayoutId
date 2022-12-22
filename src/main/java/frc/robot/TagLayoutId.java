package frc.robot;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.stream.Collectors;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.MathUtils;

public class TagLayoutId {

    private static final double kBufferLength = 3;
    private static final double kBufferMaxGap = 0.5;
    private final double updateDelta;

    /** The transform/relation from one tag to another */
    class TagRelation {
        final NavigableMap<Double, Pose3d> buffer = new TreeMap<>();
        Pose3d bestTrf = null;
        double bestDistrust = Double.MAX_VALUE;

        private void expire(double now) {
            while(!buffer.isEmpty()) {
                double first = buffer.firstKey();
                if(now - first >= kBufferLength) buffer.remove(first);
                else return;
            }
        }

        // arbitrary scaling values for rotation radians relative to translation meters
        private static final double kRotScale = 5;
        private static final double kRotPow = 2;
        /**
         * Estimates a scalar "distrust" of these standard deviations in 3d space.
         * Deviations with lower distrust are prioritized.
         * 
         * @param stdDevs The standard deviations in 3d space meters/radians
         * @return A scalar double value (>0) representing "distrust"
         */
        public double estDistrust(Pose3d stdDevs) {
            double distrust = 0;
            distrust += stdDevs.getTranslation().getNorm();

            distrust += kRotScale*stdDevs.getRotation().getX();
            distrust += Math.pow(stdDevs.getRotation().getX(), kRotPow);

            distrust += kRotScale*stdDevs.getRotation().getY();
            distrust += Math.pow(stdDevs.getRotation().getY(), kRotPow);

            distrust += kRotScale*stdDevs.getRotation().getZ();
            distrust += Math.pow(stdDevs.getRotation().getZ(), kRotPow);
            
            return distrust;
        }

        public void update(Pose3d relativePose) {
            double now = Timer.getFPGATimestamp();
            // ensure continuous data
            if(!buffer.isEmpty() && now - buffer.lastKey() > kBufferMaxGap) buffer.clear();
            buffer.put(now, relativePose);
            expire(now); // remove old data
            // if we have a full buffer of data
            if(buffer.lastKey() - buffer.firstKey() >= kBufferLength - 2*updateDelta) {
                Pose3d avg = MathUtils.calcAvg(buffer.values().toArray(new Pose3d[0]));
                Pose3d stdDevs = MathUtils.calcStdDev(avg, buffer.values().toArray(new Pose3d[0]));
                // we need to create a scalar representation of "distrust" given pose std devs
                double distrust = estDistrust(stdDevs);
                // if we find a more trustworthy transformation, use that
                if(distrust < bestDistrust) {
                    bestDistrust = distrust;
                    bestTrf = avg;
                }
            }
        }
    }
    
    private final Map<Integer, Map<Integer, TagRelation>> relationMap = new HashMap<>();
    private final Graph<Pose3d> graph = new Graph<>();
    
    public TagLayoutId() {
        this(0.02);
    }
    public TagLayoutId(double updateDelta) {
        this.updateDelta = updateDelta;
    }

    /**
     * Updates known tag relations with visible tags. If multiple tags are visible together
     * long enough, a transform is estimated between them-- the most consistent transforms
     * previously seen are saved as unique edges between two tags. These edges are used to
     * create a graph structure and solve for the Minimum Spanning Tree(Forest).
     * @param tags All visible AprilTags. The poses must all share the same origin.
     */
    public void update(List<AprilTag> tags) {
        if(tags == null || tags.size() <= 1) return;
        // sort. If two tags are seen, the transform will always be from the lowest ID to the highest
        tags.sort(Comparator.comparingInt(t -> t.ID));
        // update all visible relations
        for(int i = 0; i < tags.size(); i++) {
            var tagFrom = tags.get(i);
            for(int j = tags.size()-1; j > i; j--) {
                var tagTo = tags.get(j);
                relationMap.putIfAbsent(tagFrom.ID, new HashMap<>());
                var relations = relationMap.get(tagFrom.ID);
                relations.putIfAbsent(tagTo.ID, new TagRelation());
                var relation = relations.get(tagTo.ID);
                relation.update(tagTo.pose.relativeTo(tagFrom.pose));
            }
        }

        solve();
    }
    /** Fill graph with known TagRelations and solve for MSF */
    private void solve() {
        var vertices = new HashSet<Graph<Pose3d>.Vertex>(); // avoid duplicate vertices
        var edges = new ArrayList<Graph<Pose3d>.Edge>();
        for(var entry : relationMap.entrySet()) {
            for(var relation : entry.getValue().entrySet()) {
                // skip if this relation is invalid
                if(relation.getValue().bestTrf == null) continue;
                // add this relation as an edge to the graph
                edges.add(graph.new Edge(
                    entry.getKey(), // ID from
                    relation.getKey(), // ID to
                    relation.getValue().bestDistrust, // edge weight
                    relation.getValue().bestTrf // data (this relation's transform)
                ));
                // ensure the related vertices are in the graph
                vertices.add(graph.new Vertex(relation.getKey(), null));
                vertices.add(graph.new Vertex(entry.getKey(), null));
            }
        }
        // create graph from our known tag relations
        graph.fill(new ArrayList<>(vertices), edges);
        // solve minimum spanning forest from relations to remove conflicting relations
        graph.solveMSF();
    }

    /**
     * Uses the solved MSF from {@link #update(List)} to reconstruct an AprilTag layout
     * without conflicts.
     * 
     * <p>An origin may be supplied to give a known point in the layout.
     * If this origin is null or is not seen in the previously visible tags, an origin is
     * instead picked as the tag with the lowest ID from the previously visible tags. This
     * is important as the solved MSF may not be a MST, which means all tags may not be
     * part of one connected tree. The tag layout is reconstructed starting from the origin,
     * so only tags with connections to the origin tag will be returned in the layout.
     * @return The AprilTags in the estimated tag layout
     */
    public List<AprilTag> findLayout() {
        return findLayout(null);
    }
    /**
     * Uses the solved MSF from {@link #update(List)} to reconstruct an AprilTag layout
     * without conflicts.
     * 
     * <p>An origin may be supplied to give a known point in the layout.
     * If this origin is null or is not seen in the previously visible tags, an origin is
     * instead picked as the tag with the lowest ID from the previously visible tags. This
     * is important as the solved MSF may not be a MST, which means all tags may not be
     * part of one connected tree. The tag layout is reconstructed starting from the origin,
     * so only tags with connections to the origin tag will be returned in the layout.
     * @param origin Optional origin for seeding the tag layout
     * @return The AprilTags in the estimated tag layout
     */
    public List<AprilTag> findLayout(AprilTag origin) {
        var tagMap = new HashMap<Integer, AprilTag>();
        var vertices = graph.getVertices();
        var vertList = vertices.entrySet().stream()
                .sorted(Comparator.comparingInt(e -> e.getKey()))
                .collect(Collectors.toList());
        if(origin == null && vertList.size() == 0) return new ArrayList<>();
        // decide origin by lowest ID if necessary
        if(origin == null || (vertices.size() > 0 && !vertices.containsKey(origin.ID)))
                origin = new AprilTag(vertList.get(0).getKey(), new Pose3d());
        // only vertices in the same subset as the origin will be added to the layout
        tagMap.putIfAbsent(origin.ID, origin);

        // reconstruct apriltag poses from transforms starting at origin
        var edges = graph.getEdges().stream().collect(Collectors.toList());
        int numEdges;
        do {
            numEdges = edges.size();
            var removedEdges = new ArrayList<Graph<Pose3d>.Edge>();
            for(var edge : edges) {
                // get all edges connected to current tags in tagMap
                if(tagMap.containsKey(edge.idA)) {
                    tagMap.putIfAbsent(edge.idB, new AprilTag(
                        edge.idB,
                        tagMap.get(edge.idA).pose.transformBy(
                            new Transform3d(new Pose3d(), edge.data)
                        )
                    ));
                    removedEdges.add(edge);
                }
                if(tagMap.containsKey(edge.idB)) {
                    tagMap.putIfAbsent(edge.idA, new AprilTag(
                        edge.idA,
                        tagMap.get(edge.idB).pose.transformBy(
                            new Transform3d(new Pose3d(), edge.data).inverse()
                        )
                    ));
                    removedEdges.add(edge);
                }
            }
            edges.removeAll(removedEdges);
        } while(edges.size() != numEdges);

        return List.copyOf(tagMap.values());
    }
}
