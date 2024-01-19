import java.util.*;
import java.util.PriorityQueue;
import java.util.Hashtable;
import java.util.List;
import java.util.LinkedList;
import java.util.NoSuchElementException;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * This class extends the BaseGraph data structure with additional methods for
 * computing the total cost and list of node data along the shortest path
 * connecting a provided starting to ending nodes.  This class makes use of
 * Dijkstra's shortest path algorithm.
 */
public class DijkstraGraph<NodeType, EdgeType extends Number>
        extends BaseGraph<NodeType,EdgeType>
        implements GraphADT<NodeType, EdgeType> {

    /**
     * While searching for the shortest path between two nodes, a SearchNode
     * contains data about one specific path between the start node and another
     * node in the graph.  The final node in this path is stored in it's node
     * field.  The total cost of this path is stored in its cost field.  And the
     * predecessor SearchNode within this path is referened by the predecessor
     * field (this field is null within the SearchNode containing the starting
     * node in it's node field).
     *
     * SearchNodes are Comparable and are sorted by cost so that the lowest cost
     * SearchNode has the highest priority within a java.util.PriorityQueue.
     */
    protected class SearchNode implements Comparable<SearchNode> {
        public Node node;
        public double cost;
        public SearchNode predecessor;
        public SearchNode(Node node, double cost, SearchNode predecessor) {
            this.node = node;
            this.cost = cost;
            this.predecessor = predecessor;
        }
        public int compareTo(SearchNode other) {
            if( cost > other.cost ) return +1;
            if( cost < other.cost ) return -1;
            return 0;
        }
    }

    /**
     * This helper method creates a network of SearchNodes while computing the
     * shortest path between the provided start and end locations.  The
     * SearchNode that is returned by this method is represents the end of the
     * shortest path that is found: it's cost is the cost of that shortest path,
     * and the nodes linked together through predecessor references represent
     * all the nodes along that shortest path (ordered from end to start).
     *
     * @param start the data item in the starting node for the path
     * @param end the data item in the destination node for the path
     * @return SearchNode for the final end node within the shortest path
     * @throws NoSuchElementException when no path from start to end is found
     *         or when either start or end data do not correspond to a graph node
     */
    protected SearchNode computeShortestPath(NodeType start, NodeType end) {
        // if start and end node do not exist in graph
        if (!this.containsNode(start) || !this.containsNode(end)) {
            throw new NoSuchElementException();
        }

        PriorityQueue<SearchNode> queue = new PriorityQueue<>();
        Hashtable<NodeType, SearchNode> visitedNodes = new Hashtable<>();

        Node startNode = nodes.get(start);
        SearchNode startSearchNode = new SearchNode(startNode, 0, null);
        // add start node to queue
        queue.add(startSearchNode);
        // adds start node to hashtable of visited nodes
        visitedNodes.put(start, startSearchNode);

        while (!queue.isEmpty()) {
            // removes node at beginning of queue
            SearchNode current = queue.poll();
            // checks if the current node is the end node
            if (current.node.data.equals(end)) {
                return current;
            }

            // iterates all edges leaving current node
            for (Edge edge : current.node.edgesLeaving) {
                // computes cost of new path
                double newCost = current.cost + edge.data.doubleValue();
                // checks if node has been visited, or cost of new path is less than visited path
                if (!visitedNodes.containsKey(edge.successor.data) || newCost <
                visitedNodes.get(edge.successor.data).cost) {
                    // creates node with the new cost and current node as predecessor
                    SearchNode newNode = new SearchNode(edge.successor, newCost, current);
                    // adds new node to queue
                    queue.add(newNode);
                    // adds new node to hashtable
                    visitedNodes.put((NodeType)edge.successor.data, newNode);
                }
            }
        }
        // if no path was found
        throw new NoSuchElementException("Path not found");
    }


    /**
     * Returns the list of data values from nodes along the shortest path
     * from the node with the provided start value through the node with the
     * provided end value.  This list of data values starts with the start
     * value, ends with the end value, and contains intermediary values in the
     * order they are encountered while traversing this shorteset path.  This
     * method uses Dijkstra's shortest path algorithm to find this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end the data item in the destination node for the path
     * @return list of data item from node along this shortest path
     */
    public List<NodeType> shortestPathData(NodeType start, NodeType end) {
        // calls helper and finds the shortest path from start node to end node
        SearchNode endSearchNode = computeShortestPath(start, end);
        // list that holds nodes along shortest path from end to start
        LinkedList<NodeType> shortestPath = new LinkedList<>();
        SearchNode current = endSearchNode;
        while (current != null) {
            // add data from current node to the beginning of the list
            shortestPath.addFirst((NodeType)current.node.data);
            // current node becomes predecessor
            current = current.predecessor;
        }
        return shortestPath;
    }

    /**
     * Returns the cost of the path (sum over edge weights) of the shortest
     * path from the node containing the start data to the node containing the
     * end data.  This method uses Dijkstra's shortest path algorithm to find
     * this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end the data item in the destination node for the path
     * @return the cost of the shortest path between these nodes
     */
    public double shortestPathCost(NodeType start, NodeType end) {
        return computeShortestPath(start, end).cost;
    }

    /**
     * Tests the shortest path and cost calculation by comparing the results
     * with a hand computed example. Graph is created and edges with their
     * weights are added between the nodes.
     * The calculated shortest path and cost from node A to D are then
     * compared with expected results.
     */
    @Test
    public void testShortestPathAndCost() {
        DijkstraGraph<String, Integer> graph = new DijkstraGraph<>();

        // building graph
        graph.insertNode("A");
        graph.insertNode("B");
        graph.insertNode("C");
        graph.insertNode("D");

        graph.insertEdge("A", "B", 1);
        graph.insertEdge("B", "C", 2);
        graph.insertEdge("A", "D", 4);
        graph.insertEdge("B", "D", 1);

        List<String> expectedPath = Arrays.asList("A", "B", "D");
        assertEquals(expectedPath, graph.shortestPathData("A", "D"));
        assertEquals(2, graph.shortestPathCost("A", "D"));
    }

    /**
     * Tests the shortest path and cost calculation for different nodes in
     * same graph. The calculated shortest path and cost from node A to C
     * are then compared with expected results.
     */
    @Test
    public void testShortestPathAndCostDifference() {
        DijkstraGraph<String, Integer> graph = new DijkstraGraph<>();

        // building graph
        graph.insertNode("A");
        graph.insertNode("B");
        graph.insertNode("C");
        graph.insertNode("D");

        graph.insertEdge("A", "B", 1);
        graph.insertEdge("B", "C", 2);
        graph.insertEdge("A", "D", 4);
        graph.insertEdge("B", "D", 1);

        List<String> expectedPath = Arrays.asList("A", "B", "C");
        assertEquals(expectedPath, graph.shortestPathData("A", "C"));
        assertEquals(3, graph.shortestPathCost("A", "C"));
    }

    /**
     * Tests the shortest path calculation when there isn't a path from
     * start to end node in graph.
     * The graph structure is created such that there isn't a direct path
     * from A to C.
     * The method is expected to throw a NoSuchElementException
     */
    @Test
    public void testNoPathFound() {
        DijkstraGraph<String, Integer> graph = new DijkstraGraph<>();

        // building graph
        graph.insertNode("A");
        graph.insertNode("B");
        graph.insertNode("C");
        graph.insertNode("D");

        graph.insertEdge("A", "B", 1);
        graph.insertEdge("B", "A", 2);
        graph.insertEdge("A", "D", 4);
        graph.insertEdge("B", "D", 1);

        assertThrows(NoSuchElementException.class, () -> {
            graph.shortestPathData("A", "C");
        });
    }
}
