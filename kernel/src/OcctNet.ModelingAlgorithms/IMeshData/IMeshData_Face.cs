// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Face.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Face.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IMeshData_Status;

namespace Occt;

/// <summary>The discrete model of a face: one or several wires, the first of which is the outer one.</summary>
/// <remarks>
/// The face's <see cref="GetSurface"/> is one <see cref="BRepAdaptor_Surface"/> (without restriction), which the
/// mesher's loops may evaluate from several threads (phase4-plan 2.2: porters give each thread a shallow copy where
/// needed, as decision 431).
/// </remarks>
public abstract class IMeshData_Face : IMeshData_TessellatedShape, IMeshData_StatusOwner
{
    private readonly BRepAdaptor_Surface mySurface;
    private int myStatus; // IMeshData_StatusOwner

    /// <summary>An empty model of the face.</summary>
    protected IMeshData_Face(in TopoDS_Face theFace) : base(theFace)
    {
        myStatus = (int)IMeshData_NoError;
        mySurface = new BRepAdaptor_Surface(GetFace(), false);
    }

    /// <summary>The number of wires.</summary>
    public abstract int WiresNb();

    /// <summary>Adds a wire to the discrete model of the face.</summary>
    public abstract IMeshData_Wire AddWire(in TopoDS_Wire theWire, int theEdgeNb = 0);

    /// <summary>The discrete wire with the given index.</summary>
    public abstract IMeshData_Wire GetWire(int theIndex);

    /// <summary>The face's surface.</summary>
    public BRepAdaptor_Surface GetSurface() => mySurface;

    /// <summary>The face of the model.</summary>
    public TopoDS_Face GetFace() => TopoDS.Face(GetShape());

    /// <summary>Whether the face's discrete model is valid.</summary>
    public bool IsValid()
    {
        return (IsEqual(IMeshData_NoError) || IsEqual(IMeshData_ReMesh)
                || IsEqual(IMeshData_UnorientedWire));
    }

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
