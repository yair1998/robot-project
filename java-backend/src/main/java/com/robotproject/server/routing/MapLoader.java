package com.robotproject.server.routing;

import com.google.gson.Gson;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.nio.charset.StandardCharsets;

public class MapLoader {

    public static RouteGraph loadFromClasspath(String resourcePath) throws IOException {
        try (InputStream in = MapLoader.class.getClassLoader().getResourceAsStream(resourcePath)) {
            if (in == null) {
                throw new IOException("Map resource not found on classpath: " + resourcePath);
            }
            try (Reader reader = new InputStreamReader(in, StandardCharsets.UTF_8)) {
                MapData data = new Gson().fromJson(reader, MapData.class);
                return buildGraph(data);
            }
        }
    }

    private static RouteGraph buildGraph(MapData data) {
        RouteGraph graph = new RouteGraph();
        for (NodeData n : data.nodes) {
            graph.addNode(new GraphNode(n.id, n.x, n.y));
        }
        for (EdgeData e : data.edges) {
            if (e.width != null) {
                graph.addEdge(e.from, e.to, e.bidirectional, e.width);
            } else {
                graph.addEdge(e.from, e.to, e.bidirectional);
            }
        }
        return graph;
    }

    private static class MapData {
        NodeData[] nodes;
        EdgeData[] edges;
    }

    private static class NodeData {
        String id;
        double x;
        double y;
    }

    private static class EdgeData {
        String from;
        String to;
        boolean bidirectional;
        Double width; // nullable - Gson leaves this null when the JSON
                       // edge object omits "width", so existing maps
                       // with no "width" key continue to work unchanged.
    }
}
