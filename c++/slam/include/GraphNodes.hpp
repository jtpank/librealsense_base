#pragma once

//Nodes in the graph are the robot poses and features in the maap.
//Edges between 2 nodes indicate spatial constraint between those nodes.

template <typename T>
class GraphNode {
    private:
        int m_nodeId;
        T m_pose; // 2-D or 3-D pose object
        bool m_fixed; //fixed or not
    public:
        GraphNode() {};
};