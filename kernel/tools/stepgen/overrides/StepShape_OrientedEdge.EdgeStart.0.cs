    public override StepShape_Vertex EdgeStart()
    {
        // Upstream 573b4be651: the chain of oriented edges is followed in a loop, and a chain that comes back on itself
        // has no vertex. OCCT 8.0.1 recursed, so a file whose oriented edge referred to itself overflowed the stack
        // (decision 34306)
        HashSet<StepShape_Edge>? visited = null;
        StepShape_Edge edge = edgeElement;
        bool isStart = Orientation();
        while (edge is StepShape_OrientedEdge oriented)
        {
            if (!(visited ??= new(ReferenceEqualityComparer.Instance)).Add(edge))
            {
                return null;
            }
            if (!oriented.Orientation())
            {
                isStart = !isStart;
            }
            edge = oriented.EdgeElement();
        }
        return edge is null ? null : isStart ? edge.EdgeStart() : edge.EdgeEnd();
    }
