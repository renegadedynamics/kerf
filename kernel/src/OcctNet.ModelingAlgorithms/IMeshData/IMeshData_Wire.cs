// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Wire.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Wire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IMeshData_Status;

namespace Occt;

/// <summary>The discrete model of a wire: an ordered chain of discrete edges with their orientations.</summary>
public abstract class IMeshData_Wire : IMeshData_TessellatedShape, IMeshData_StatusOwner
{
    private int myStatus; // IMeshData_StatusOwner

    /// <summary>An empty model of the wire.</summary>
    protected IMeshData_Wire(in TopoDS_Wire theWire) : base(theWire)
    {
        myStatus = (int)IMeshData_NoError;
    }

    /// <summary>The wire of the model.</summary>
    public TopoDS_Wire GetWire() => TopoDS.Wire(GetShape());

    /// <summary>The number of edges.</summary>
    public abstract int EdgesNb();

    /// <summary>Adds a discrete edge with the given orientation to the wire's chain; returns its index.</summary>
    public abstract int AddEdge(IMeshData_Edge theDEdge, TopAbs_Orientation theOrientation);

    /// <summary>The discrete edge with the given index.</summary>
    public abstract IMeshData_Edge GetEdge(int theIndex);

    /// <summary>The orientation of the discrete edge with the given index.</summary>
    public abstract TopAbs_Orientation GetEdgeOrientation(int theIndex);

    // IMeshData_StatusOwner

    /// <inheritdoc/>
    public bool IsEqual(IMeshData_Status theValue) => (myStatus == (int)theValue);

    /// <inheritdoc/>
    public bool IsSet(IMeshData_Status theValue) => (myStatus & (int)theValue) != 0;

    /// <inheritdoc/>
    public void SetStatus(IMeshData_Status theValue) => myStatus |= (int)theValue;

    /// <inheritdoc/>
    public void UnsetStatus(IMeshData_Status theValue) => myStatus &= ~(int)theValue;

    /// <inheritdoc/>
    public int GetStatusMask() => myStatus;
}
