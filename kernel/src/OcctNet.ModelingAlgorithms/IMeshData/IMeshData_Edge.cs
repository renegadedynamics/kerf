// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Edge.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IMeshData_Status;

namespace Occt;

/// <summary>
/// The discrete model of an edge: its 3D curve and its pcurves, one per discrete face (two on a seam), with the
/// edge's flags as the mesher sees them.
/// </summary>
public abstract class IMeshData_Edge : IMeshData_TessellatedShape, IMeshData_StatusOwner
{
    private bool mySameParam;
    private bool mySameRange;
    private bool myDegenerated;
    private double myAngDeflection;
    private IMeshData_Curve? myCurve;
    private int myStatus; // IMeshData_StatusOwner

    /// <summary>An empty model of the edge, with the edge's SameParameter, SameRange and Degenerated flags.</summary>
    protected IMeshData_Edge(in TopoDS_Edge theEdge) : base(theEdge)
    {
        mySameParam = BRep_Tool.SameParameter(theEdge);
        mySameRange = BRep_Tool.SameRange(theEdge);
        myDegenerated = BRep_Tool.Degenerated(theEdge);
        myAngDeflection = RealLast();
        myStatus = (int)IMeshData_NoError;
    }

    /// <summary>The edge of the model.</summary>
    public TopoDS_Edge GetEdge() => TopoDS.Edge(GetShape());

    /// <summary>The number of pcurves of the edge.</summary>
    public abstract int PCurvesNb();

    /// <summary>Adds a discrete pcurve for the given discrete face.</summary>
    public abstract IMeshData_PCurve AddPCurve(IMeshData_Face theDFace, TopAbs_Orientation theOrientation);

    /// <summary>The pcurve for the given discrete face and orientation.</summary>
    public abstract IMeshData_PCurve GetPCurve(IMeshData_Face theDFace, TopAbs_Orientation theOrientation);

    /// <summary>The pcurve with the given index.</summary>
    public abstract IMeshData_PCurve GetPCurve(int theIndex);

    /// <summary>The indices of the pcurves for the given discrete face.</summary>
    public abstract NCollection_List<int> GetPCurves(IMeshData_Face theDFace);

    /// <summary>Clears the curve and all the pcurves of the edge.</summary>
    public void Clear(bool isKeepEndPoints)
    {
        myCurve!.Clear(isKeepEndPoints);
        for (int aPCurveIt = 0; aPCurveIt < PCurvesNb(); ++aPCurveIt)
        {
            GetPCurve(aPCurveIt).Clear(isKeepEndPoints);
        }
    }

    /// <summary>True if the edge is free, that is, it has no pcurves.</summary>
    public bool IsFree() => (PCurvesNb() == 0);

    /// <summary>Sets the 3D curve of the edge.</summary>
    public void SetCurve(IMeshData_Curve? theCurve) => myCurve = theCurve;

    /// <summary>The 3D curve of the edge.</summary>
    public IMeshData_Curve GetCurve() => myCurve!;

    /// <summary>The angular deflection of the discrete model.</summary>
    public double GetAngularDeflection() => myAngDeflection;

    /// <summary>Sets the angular deflection of the discrete model.</summary>
    public void SetAngularDeflection(double theValue) => myAngDeflection = theValue;

    /// <summary>The SameParameter flag; by default the edge's.</summary>
    public bool GetSameParam() => mySameParam;

    /// <summary>Updates the SameParameter flag.</summary>
    public void SetSameParam(bool theValue) => mySameParam = theValue;

    /// <summary>The SameRange flag; by default the edge's.</summary>
    public bool GetSameRange() => mySameRange;

    /// <summary>Updates the SameRange flag.</summary>
    public void SetSameRange(bool theValue) => mySameRange = theValue;

    /// <summary>The Degenerated flag; by default the edge's.</summary>
    public bool GetDegenerated() => myDegenerated;

    /// <summary>Updates the Degenerated flag.</summary>
    public void SetDegenerated(bool theValue) => myDegenerated = theValue;

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
