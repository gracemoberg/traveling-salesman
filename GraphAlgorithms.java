/* Grace Moberg
 * CS231B
 * Lab 09: Graphs
 * GraphAlgorithms.java
 * 28 November 2022
 */

import java.util.ArrayList;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Random;
import java.util.Stack;


public class GraphAlgorithms {

    // reads a csv file and creates a weighted graph where edges are the time in seconds
    // it takes to travel from capital to capital
    public static Graph<String, Object> readData(String filename) {
        try {
            FileReader fr = new FileReader(filename);
            BufferedReader br = new BufferedReader(fr);

            Graph<String, Object> newGraph = new Graph<>();
            HashMap<String, Graph.Vertex<String, Object>> cities = new HashMap<>();

            br.readLine();
            String line = br.readLine();
            while (line != null) {
                String[] contents = line.split(",");

                String state1 = contents[1];
                String state2 = contents[3];

                if (!cities.containsKey(state1))
                    cities.put(state1, newGraph.addVertex(state1));
                if (!cities.containsKey(state2))
                    cities.put(state2, newGraph.addVertex(state2));

                newGraph.addEdge(cities.get(state1), cities.get(state2), Integer.parseInt(contents[5]));
                line = br.readLine();
            }
            br.close();
            return newGraph;
        } catch (IOException ex) {
            System.out.println("GraphAlgorithms.readData():: error reading file" + filename);
        }
        return null;
    }

    // attemped extension to read file of great-circle distances of US counties
    // reads a csv file and creates a weighted graph where edges are the time in seconds
    // it takes to travel from county to county
    public static Graph<String, Object> newReadData(String filename) {
        try {
            FileReader fr = new FileReader(filename);
            BufferedReader br = new BufferedReader(fr);

            Graph<String, Object> newGraph = new Graph<>();
            HashMap<String, Graph.Vertex<String, Object>> counties = new HashMap<>();

            br.readLine();
            String line = br.readLine();
            while (line != null) {
                String[] contents = line.split(",");

                String county1 = contents[0];
                String county2 = contents[2];

                if (!counties.containsKey(county1))
                    counties.put(county1, newGraph.addVertex(county1));
                if (!counties.containsKey(county2))
                    counties.put(county2, newGraph.addVertex(county2));

                newGraph.addEdge(counties.get(county1), counties.get(county2), Double.parseDouble(contents[1]));
                line = br.readLine();
            }
            System.out.println(newGraph.edges.size());
            br.close();
            return newGraph;
        } catch (IOException ex) {
            System.out.println("GraphAlgorithms.newReadData():: error reading file" + filename);
        }
        return null;
    }

    // returns a HashMap containing the shortest distance between a start vertex and
        // all other reachable vertices of the graph
    public static <V, E> HashMap<Graph.Vertex<V, E>, Double> shortestPaths(Graph<V, E> g, Graph.Vertex<V, E> source) {

        HashMap<Graph.Vertex<V, E>, Double> distances = new HashMap<>(); // hashmap that maintains distances from start vertex

        for (Graph.Vertex<V, E> vertex : g.vertices) {
            distances.put(vertex, Double.POSITIVE_INFINITY); // initially set each distance to infinity
        }
        distances.put(source, 0.0); // distance from source to itself is 0

        PriorityQueue<Graph.Vertex<V, E>> queue = new PriorityQueue<>(new Comparator<Graph.Vertex<V, E>>() {

            @Override
            public int compare(Graph.Vertex<V, E> o1, Graph.Vertex<V, E> o2) {
                return distances.get(o1).compareTo(distances.get(o2));
            }
        });

        for (Graph.Vertex<V, E> vertex : g.vertices) { // offer each vertex in the graph to the queue
            queue.offer(vertex);
        }

        while (!queue.isEmpty()) {
            Graph.Vertex<V, E> cur = queue.poll();

            for (Graph.Edge<V, E> edgeOut : cur.edgesOut()) {
                Graph.Vertex<V, E> next = edgeOut.other(cur);

                if (distances.get(cur) + ((Graph.WeightedEdge<V, E>) edgeOut).weight < distances.get(next)) {
                    distances.put(next, distances.get(cur) + ((Graph.WeightedEdge<V, E>) edgeOut).weight);
                    queue.remove(next);
                    queue.offer(next);
                }
            }
        }
        return distances;
    }

    // returns a collection of lists of edges
    // where each list of edges is a path that visits every vertex in the given
    // graph exactly once
    public static <V, E> Collection<List<Graph.Edge<V, E>>> allHamCycles(Graph<V, E> g, Graph.Vertex<V, E> start) {
        Collection<List<Graph.Edge<V, E>>> output = new ArrayList<List<Graph.Edge<V, E>>>(); // create collection
        ArrayList<Graph.Vertex<V, E>> curPath = new ArrayList<>(); // create current path
        curPath.add(start); // add start
        allHamCycles(g, output, curPath); // recurse!
        return output;

    }

    // recursive helper method
    private static <V, E> void allHamCycles(Graph<V, E> g, Collection<List<Graph.Edge<V, E>>> output,
            ArrayList<Graph.Vertex<V, E>> curPath) {
        // base case -> path length is equal to number of vertices in graph
        if (curPath.size() == g.vertices.size()) {
            ArrayList<Graph.Edge<V, E>> edgePath = new ArrayList<Graph.Edge<V, E>>(); // create list of edges that is
                                                                                      // the path
            for (int i = 0; i < curPath.size() - 1; i++) { // iterate through all vertices in the path
                Graph.Vertex<V, E> cur = curPath.get(i); // get current
                Graph.Vertex<V, E> next = curPath.get(i + 1); // get next
                Graph.Edge<V, E> edge = g.getEdge(cur, next); // get edge between two vertices
                edgePath.add(edge); // add edge to list of edges
            }
            Graph.Edge<V, E> edge = g.getEdge(curPath.get(curPath.size() - 1), curPath.get(0)); // get edge from last vertex to first vertex          
            if (edge != null) { // if that edge exists
                edgePath.add(edge); // add it to the edge path
                output.add(edgePath); // add edge path to collection
            }
        } else { // if path is not yet a hamiltonian cycle
            Graph.Vertex<V, E> last = curPath.get(curPath.size() - 1); // get the last visited vertex
            for (Graph.Edge<V, E> edgeOut : last.edgesOut()) { // for every edge leaving that vertex
                Graph.Vertex<V, E> next = edgeOut.other(last); // get the vertex at the end of that edge
                if (!curPath.contains(next)) { // if that vertex hasn't yet been visited
                    curPath.add(next); // add it to the path
                    allHamCycles(g, output, curPath); // recurse
                    curPath.remove(curPath.size() - 1); // remove last added vertex
                }
            }
        }
    }

    // returns the minimum distance Hamiltonian cycle given a graph and a starting
    // vertex
    public static <V, E> List<Graph.Edge<V, E>> minTSP(Graph<V, E> g, Graph.Vertex<V, E> source) {

        Collection<List<Graph.Edge<V, E>>> output = allHamCycles(g, source); // list of all Hamiltonian Cycles
        List<Graph.Edge<V, E>> min = new ArrayList<Graph.Edge<V, E>>(); // initialize list that will be the minimum
        double minCost = Double.POSITIVE_INFINITY; // initialize a minimum cost that is infinity

        for (List<Graph.Edge<V, E>> hamCycle : output) { // for each cycle in the collection of cycles
            double sum = 0; // initialize sum of edges as 0
            for (Graph.Edge<V, E> edge : hamCycle) { // for each edge in the Hamiltonian cycle
                sum += ((Graph.WeightedEdge<V, E>) edge).weight; // sum the weights of the edges (must cast edge to be
                                                                 // weighted)
            }
            if (sum < minCost) { // if the sum of the edges is less than the minimum cost
                minCost = sum; // update minimum cost
                min = hamCycle; // update minumum cycle
            }
        }

        return min;
    }

    // finds the minimum spanning tree in a given graph and returns it as a
    // collection of edges
    public static <V, E> Collection<Graph.Edge<V, E>> mst(Graph<V, E> g) {

        Random rand = new Random();
        // Prim's algorithm

        // select some starting vertex
        Graph.Vertex<V, E> cur = g.getVertex(rand.nextInt(48)); // sets starting vertex equal to the first vertex added to graph

        // initialize an empty set for storing visited vertices, to which we add the start
        HashSet<Graph.Vertex<V, E>> visited = new HashSet<Graph.Vertex<V, E>>();
        visited.add(cur);

        // initialize an empty collection of edges to return
        Collection<Graph.Edge<V, E>> edges = new ArrayList<Graph.Edge<V, E>>();

        // initialize a PQ
        PriorityQueue<Graph.Edge<V, E>> queue = new PriorityQueue<Graph.Edge<V, E>>(new Comparator<Graph.Edge<V, E>>() {

            @Override
            public int compare(Graph.Edge<V, E> o1, Graph.Edge<V, E> o2) {
                if (((Graph.WeightedEdge<V, E>) o1).weight > ((Graph.WeightedEdge<V, E>) o2).weight)
                    return 1;
                else if (((Graph.WeightedEdge<V, E>) o2).weight > ((Graph.WeightedEdge<V, E>) o1).weight)
                    return -1;
                else
                    return 0;
            }
        });
        for (Graph.Edge<V, E> edgeOut : cur.edgesOut()) {
            queue.add(edgeOut); // add all edges leaving vertex into priority queue
        }

        while (visited.size() < g.vertices.size()) { // while not all vertices are visited
            Graph.Edge<V, E> minimalEdge = queue.poll(); // get minimal edge from PQ    
            for (Graph.Vertex<V, E> v : minimalEdge.vertices()) {
                if (visited.contains(v)) continue;
                edges.add(minimalEdge); // add minimal edge to solution
                visited.add(v);
                cur = v;
                for (Graph.Edge<V, E> edgeOut : cur.edgesOut()) {
                    queue.add(edgeOut); // add all edges leaving vertex into priority queue
                }
            }
        }
        return edges;
    }

    // using the supplied graph, computes a minimum spanning tree
    // then performs a depth first search over that to compute a Hamiltonian cycle
    // returns a list of edges that constitutes the path
    public static <V, E> List<Graph.Edge<V, E>> tspApprox(Graph<V, E> g) {

        List<Graph.Edge<V, E>> path = new ArrayList<Graph.Edge<V, E>>(); // solution path
        Collection<Graph.Edge<V, E>> mst = mst(g); // find minimum spanning tree for graph
        Stack<Graph.Vertex<V, E>> stack = new Stack<Graph.Vertex<V, E>>(); // stack for DFS
        HashSet<Graph.Vertex<V, E>> seen = new HashSet<>(); // visited vertices

        Random rand = new Random();
        Graph.Vertex<V, E> start = g.getVertex(rand.nextInt(48)); // get start, a random vertex 
        Graph.Vertex<V, E> last = null; // pointer to last vertex

        // add start vertex to stack and visited vertices
        stack.push(start);
        seen.add(start);

        while (!stack.isEmpty()) {
            Graph.Vertex<V, E> cur = stack.pop();
            if (last != null) { 
                path.add(g.getEdge(last, cur)); // add edge between current vertex and last vertex to path
            }
            if (!seen.contains(cur)) { // if we haven't already visited the vertex, add it
                seen.add(cur);
            }
            for (Graph.Edge<V, E> edge : mst) { // for each edge in the tree
                if (edge.vertices().contains(cur)) { // if that vertex is at one endpoint of the current edge
                    Graph.Vertex<V, E> neighbor = edge.other(cur); // get its neighbor
                    if (seen.contains(neighbor)) // if we've already seen neighbor, skip iteration
                        continue;
                    stack.add(neighbor); // add neighbor to stack
                    seen.add(neighbor); // add neighbor to seen
                }

            }
            last = cur; // update last
        }
        return path;
    }

    // extension function
    // checks if a graph is connected using a breadth-first search
    // returns true if so, false if not
    public static <V,E> boolean isConnected(Graph<V,E> g){
        Graph.Vertex<V,E> start = g.getVertex(0);
        Queue<Graph.Vertex<V,E>> queue = new LinkedList<Graph.Vertex<V,E>>();
        HashSet<Graph.Vertex<V,E>> seen = new HashSet<>();

        queue.offer(start);
        seen.add(start);

        while (!queue.isEmpty()){
            Graph.Vertex<V,E> cur = queue.poll();

            if(!seen.contains(cur)){
                seen.add(cur);
            }
            for (Graph.Edge<V,E> edge: cur.edgesOut()){
                Graph.Vertex<V,E> neighbor = edge.other(cur);
                if (!seen.contains(neighbor)){
                    seen.add(neighbor);
                    queue.offer(neighbor);
                }
            }
        }
        if (seen.size() == g.vertices.size()) return true;
        return false;
    }

    public static void main(String[] args) {

        Graph<String, Object> stateGraph = GraphAlgorithms.readData("StateData.csv");
        System.out.println(GraphAlgorithms.isConnected(stateGraph));
        // Graph.Vertex<String,Object> vert = stateGraph.getVertex(0);
        // System.out.println(GraphAlgorithms.shortestPaths(stateGraph, vert));
        // long time = System.nanoTime();
        // System.out.println(GraphAlgorithms.tspApprox(stateGraph));
        // System.out.println("runtime: " + (System.nanoTime() - time));

        Graph<String, Object> countyGraph = GraphAlgorithms.newReadData("counties.csv");
        System.out.println(GraphAlgorithms.isConnected(countyGraph));
        // // System.out.println(countyGraph.vertices);
        // GraphAlgorithms.mst(countyGraph);
        // System.out.println(GraphAlgorithms.mst(countyGraph));
    }
}