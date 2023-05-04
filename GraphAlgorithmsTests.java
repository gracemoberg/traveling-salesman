/* Grace Moberg
 * CS231B
 * Lab 09: Graphs
 * GraphAlgorithmsTests.java
 * 5 December 2022
 */

public class GraphAlgorithmsTests {

    public static void main(String[] args){
        
        // case 1: testing readData()
        {
            // setup
            Graph<String, Object> stateGraph = GraphAlgorithms.readData("StateData.csv");

            // verify
            System.out.println("size of graph: " + stateGraph.vertices.size() + " == 48");

            // test
            assert stateGraph.vertices.size() == 48 : "Error in GraphAlgorithms.readData()";
        }

        // case 2: testing shortestPath()
        {
            // setup
            Graph<String, Integer> graph = new Graph<String, Integer>();
            Graph.Vertex<String, Integer> MN = graph.addVertex("MN");
            Graph.Vertex<String, Integer> WI = graph.addVertex("WI");
            Graph.Vertex<String, Integer> IL = graph.addVertex("IL");
            graph.addEdge(MN, IL, 3);
            graph.addEdge(MN, WI, 4);
            graph.addEdge(IL, WI, 1);

            // verify
            System.out.println("length of path: " + GraphAlgorithms.shortestPaths(graph, MN).size() + " == 3");

            // test
            assert GraphAlgorithms.shortestPaths(graph, MN).size() == 3 : "Error in GraphAlgorithms.shortestPath()";
        }

        // case 3: testing allHamCycles()
        {
            // setup
            Graph<String, Integer> graph = new Graph<String, Integer>();
            Graph.Vertex<String, Integer> MN = graph.addVertex("MN");
            Graph.Vertex<String, Integer> WI = graph.addVertex("WI");
            Graph.Vertex<String, Integer> IL = graph.addVertex("IL");
            Graph.Vertex<String, Integer> IA = graph.addVertex("IA");
            graph.addEdge(MN, IL, 3);
            graph.addEdge(MN, WI, 4);
            graph.addEdge(IL, WI, 1);
            graph.addEdge(MN, IA, 0.5);
            graph.addEdge(WI,IA, 4);
            graph.addEdge(IL,IA,2);

            // verify
            System.out.println("number of Ham cycles: " + GraphAlgorithms.allHamCycles(graph, MN).size() + " == 6");
            
            // test
            assert GraphAlgorithms.allHamCycles(graph, MN).size() == 6 : "Error in GraphAlgorithms.allHamCycles()";
        }

        // case 4: testing minTSP()
        {
            // setup
            Graph<String, Integer> graph = new Graph<String, Integer>();
            Graph.Vertex<String, Integer> MN = graph.addVertex("MN");
            Graph.Vertex<String, Integer> WI = graph.addVertex("WI");
            Graph.Vertex<String, Integer> IL = graph.addVertex("IL");
            Graph.Vertex<String, Integer> IA = graph.addVertex("IA");
            graph.addEdge(MN, IL, 3);
            graph.addEdge(MN, WI, 4);
            graph.addEdge(IL, WI, 1);
            graph.addEdge(MN, IA, 0.5);
            graph.addEdge(WI,IA, 4);
            graph.addEdge(IL,IA,2);

            // verify
            System.out.println("minTSP size: " + GraphAlgorithms.minTSP(graph, MN).size() + " == 4");

            // test
            assert GraphAlgorithms.minTSP(graph, MN).size() == 4 : "Error in GraphAlgorithms.minTSP()";
        }

        // case 5: testing mst()
        {
            // setup
            Graph<String, Object> stateGraph = GraphAlgorithms.readData("StateData.csv");

            // verify
            System.out.println("size of MST: " + GraphAlgorithms.mst(stateGraph).size() + " == " + (stateGraph.vertices.size() - 1));

            // test 
            assert GraphAlgorithms.mst(stateGraph).size() == (stateGraph.vertices.size() - 1) : "Error in GraphAlgorithms.mst()";
        }

        // case 6: testing tspApprox()
        {
            // setup
            Graph<String, Object> stateGraph = GraphAlgorithms.readData("StateData.csv");

            // verify
            System.out.println("length of tspApprox: " + GraphAlgorithms.tspApprox(stateGraph).size() + " == 47");

            // test
            assert GraphAlgorithms.tspApprox(stateGraph).size() == (stateGraph.vertices.size() - 1): "Error in GraphAlgorithms.tspApprox()";
        }
    }
}