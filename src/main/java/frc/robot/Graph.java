package frc.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Graph<T> {

    /** Edge between two vertices */
    public class Edge implements Comparable<Edge> {
        public final int idA;
        public final int idB;
        public final double weight;
        public final T data;

        public Edge(int a, int b, double weight, T data) {
            // to create undirected edges, always start from the smallest ID
            if(b < a) {
                this.idA = b;
                this.idB = a;
            }
            else {
                this.idA = a;
                this.idB = b;
            }
            this.weight = weight;
            this.data = data;
        }

        @Override
        public int compareTo(Edge other) {
            return Double.compare(weight, other.weight);
        }
    }

    /** Subset of vertices */
    public class Vertex {
        public final int id;
        private Vertex root = this; // root of this subset
        private int rank; // rank of the subset root
        public final T data;

        public Vertex(int id, T data) {
            this(id, null, 0, data);
        }
        public Vertex(int id, Vertex root, int rank, T data) {
            this.id = id;
            if(root != null) this.root = root;
            this.rank = rank;
            this.data = data;
        }
        public int getRank() {return rank;}
        /** Find the root vertex of this vertex's subset */
        public Vertex findRoot() {
            if(root.id != id) root = root.findRoot();
            return root;
        }
        /** Combine the subsets of two vertices */
        public void combine(Vertex b) {
            Vertex rootA = findRoot();
            Vertex rootB = b.findRoot();
            if(rootA.rank < rootB.rank) rootA.root = rootB;
            else if(rootA.rank > rootB.rank) rootB.root = rootA;
            else {
                rootB.root = rootA;
                rootA.rank++;
            }
        }
    }

    private Map<Integer, Vertex> vertices = new HashMap<>();
    private List<Edge> edges = new ArrayList<>();

    public Map<Integer, Vertex> getVertices() {return Map.copyOf(vertices);}
    public List<Edge> getEdges() {return List.copyOf(edges);}
    public Graph<T> copy() {
        var g = new Graph<T>();
        g.fill(new ArrayList<>(getVertices().values()), new ArrayList<>(getEdges()));
        return g;
    }

    public void fill(List<Vertex> vertices, List<Edge> edges) {
        if(vertices == null) vertices = new ArrayList<>();
        this.vertices.clear();
        for(var vertex : vertices) {
            this.vertices.putIfAbsent(vertex.id, vertex);
        }
        if(edges == null) edges = new ArrayList<>();
        this.edges = edges;
        Collections.sort(edges);
    }

    /**
     * Solves this graph for the Minimum Spanning Forest using Kruskal's algorithm.
     * This will remove all cyclic edges(by greedily choosing the ones with the least weight),
     * but does not guarantee all vertices are in the same subset(MST).
     * @return If the solved MSF is a MST
     */
    public boolean solveMSF() {
        var result = new ArrayList<Edge>();
        boolean complete = false;

        for(int i = 0; i < edges.size(); i++) {
            Edge next = edges.get(i);
            Vertex rootA = vertices.get(next.idA).findRoot();
            Vertex rootB = vertices.get(next.idB).findRoot();
            if(rootA.id != rootB.id) { // non-cycle edge, add to MST
                result.add(next);
                rootA.combine(rootB);
            }
            // once MST is complete, finish
            if(result.size() >= vertices.size() - 1) {
                complete = true;
                break;
            }
        }

        edges = result;
        return complete;
    }
    public double getMSFCost() {
        double sum = 0;
        for(var edge : edges) {
            sum += edge.weight;
        }
        return sum;
    } 
}
