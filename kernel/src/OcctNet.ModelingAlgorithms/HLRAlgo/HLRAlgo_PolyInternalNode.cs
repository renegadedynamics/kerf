// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyInternalNode.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyInternalNode.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

namespace Occt;

/// <summary>A node of the internal triangulation of the polygonal hidden-line removal (to update the outlines).</summary>
/// <remarks>A working object (a transient), as OCCT's; the nested structs are plain data with public fields.</remarks>
public class HLRAlgo_PolyInternalNode : Transient
{
    /// <summary>The node's first segment, flags and edges (plain data).</summary>
    public struct NodeIndices
    {
        public int NdSg;
        public int Flag;
        public int Edg1;
        public int Edg2;
    }

    /// <summary>The node's point, normal, parameters on the surface and on the edges, and scalar (plain data).</summary>
    public struct NodeData
    {
        public Xyz Point;
        public Xyz Normal;
        public Xy UV;
        public double PCu1;
        public double PCu2;
        public double Scal;

        public NodeData()
        {
            Point = new Xyz();
            Normal = new Xyz();
            UV = new Xy();
            PCu1 = 0.0;
            PCu2 = 0.0;
            Scal = 0.0;
        }
    }

    private NodeIndices myIndices;
    private NodeData myData;

    public HLRAlgo_PolyInternalNode()
    {
        myIndices.NdSg = 0;
        myIndices.Flag = 0;
        myIndices.Edg1 = 0;
        myIndices.Edg2 = 0;
        myData = new NodeData();
    }

    /// <summary>The indices, by reference.</summary>
    public ref NodeIndices Indices() => ref myIndices;

    /// <summary>The data, by reference.</summary>
    public ref NodeData Data() => ref myData;
}
