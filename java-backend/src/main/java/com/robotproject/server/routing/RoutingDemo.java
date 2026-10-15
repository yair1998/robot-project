package com.robotproject.server.routing;

import java.io.IOException;
import java.util.List;

public class RoutingDemo {

    public static void main(String[] args) throws IOException {

        RouteGraph graph = MapLoader.loadFromClasspath("maps/example_map.json");
        AStarRouter router = new AStarRouter();

        System.out.println("Route A -> E, all edges open:");
        printPath(router.findPath(graph, "A", "E"));

        System.out.println("\nBlocking B -> D (remaining width 0.3m, below the 0.6m robot)...");
        graph.getEdge("B", "D").reportObstructionEvent("routing-demo-block", 0.3);

        System.out.println("Route A -> E, B->D blocked:");
        printPath(router.findPath(graph, "A", "E"));

        System.out.println("\nPartially clearing B -> D (remaining width 0.69m, about 30% of the margin band)...");
        graph.getEdge("B", "D").reportObstructionEvent("routing-demo-block", 0.69);

        System.out.println("Route A -> E, B->D at 30% capacity:");
        printPath(router.findPath(graph, "A", "E"));
    }

    private static void printPath(List<GraphNode> path) {
        if (path.isEmpty()) {
            System.out.println("  No path found.");
            return;
        }
        StringBuilder sb = new StringBuilder("  ");
        for (int i = 0; i < path.size(); i++) {
            sb.append(path.get(i).getId());
            if (i < path.size() - 1) {
                sb.append(" -> ");
            }
        }
        System.out.println(sb);
    }
}
