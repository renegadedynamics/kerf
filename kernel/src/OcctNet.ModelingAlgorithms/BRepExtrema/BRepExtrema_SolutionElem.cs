// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_SolutionElem.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepExtrema_SupportType;

namespace Occt;

/// <summary>
/// One solution of a distance computation on one shape: the distance, the point, and its support (a vertex, or an
/// edge with a parameter, or a face with two parameters).
/// </summary>
/// <remarks>
/// OCCT's class is used by value and has no mutators, so it is a readonly struct. <c>default</c> is OCCT's empty
/// constructor (zeros, a vertex support with null shapes).
/// </remarks>
public readonly struct BRepExtrema_SolutionElem : IEquatable<BRepExtrema_SolutionElem>
{
    private readonly double myDist;
    private readonly gp_Pnt myPoint;
    private readonly BRepExtrema_SupportType mySupType;
    private readonly TopoDS_Vertex myVertex;
    private readonly TopoDS_Edge myEdge;
    private readonly TopoDS_Face myFace;
    private readonly double myPar1;
    private readonly double myPar2;

    /// <summary>The empty solution.</summary>
    public BRepExtrema_SolutionElem()
    {
        myDist = 0.0;
        myPoint = new gp_Pnt(0.0, 0.0, 0.0);
        mySupType = BRepExtrema_IsVertex;
        myPar1 = 0.0;
        myPar2 = 0.0;
    }

    /// <summary>A solution at a vertex.</summary>
    public BRepExtrema_SolutionElem(double theDist, in gp_Pnt thePoint, BRepExtrema_SupportType theSolType, in TopoDS_Vertex theVertex)
    {
        myDist = theDist;
        myPoint = thePoint;
        mySupType = theSolType;
        myVertex = theVertex;
        myPar1 = 0.0;
        myPar2 = 0.0;
    }

    /// <summary>A solution on an edge at the parameter <paramref name="theParam"/>.</summary>
    public BRepExtrema_SolutionElem(double theDist, in gp_Pnt thePoint, BRepExtrema_SupportType theSolType, in TopoDS_Edge theEdge, double theParam)
    {
        myDist = theDist;
        myPoint = thePoint;
        mySupType = theSolType;
        myEdge = theEdge;
        myPar1 = theParam;
        myPar2 = 0.0;
    }

    /// <summary>A solution in a face at the parameters (<paramref name="theU"/>, <paramref name="theV"/>).</summary>
    public BRepExtrema_SolutionElem(double theDist, in gp_Pnt thePoint, BRepExtrema_SupportType theSolType, in TopoDS_Face theFace, double theU, double theV)
    {
        myDist = theDist;
        myPoint = thePoint;
        mySupType = theSolType;
        myFace = theFace;
        myPar1 = theU;
        myPar2 = theV;
    }

    /// <summary>The distance.</summary>
    public double Dist() => myDist;

    /// <summary>The solution point.</summary>
    public gp_Pnt Point() => myPoint;

    /// <summary>The kind of support: a vertex, on an edge or inside a face.</summary>
    public BRepExtrema_SupportType SupportKind() => mySupType;

    /// <summary>The vertex, if the solution is a vertex.</summary>
    public TopoDS_Vertex Vertex() => myVertex;

    /// <summary>The edge, if the solution is on an edge.</summary>
    public TopoDS_Edge Edge() => myEdge;

    /// <summary>The face, if the solution is in a face.</summary>
    public TopoDS_Face Face() => myFace;

    /// <summary>The parameter on the edge.</summary>
    public void EdgeParameter(out double theParam) => theParam = myPar1;

    /// <summary>The parameters on the face.</summary>
    public void FaceParameter(out double theU, out double theV)
    {
        theU = myPar1;
        theV = myPar2;
    }

    public bool Equals(BRepExtrema_SolutionElem other) =>
        myDist.Equals(other.myDist) && myPoint.Equals(other.myPoint) && mySupType == other.mySupType
        && myVertex.Equals(other.myVertex) && myEdge.Equals(other.myEdge) && myFace.Equals(other.myFace)
        && myPar1.Equals(other.myPar1) && myPar2.Equals(other.myPar2);

    public override bool Equals(object? obj) => obj is BRepExtrema_SolutionElem other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(myDist, myPoint, mySupType, myVertex, myEdge, myFace, myPar1, myPar2);

    public static bool operator ==(in BRepExtrema_SolutionElem theLeft, in BRepExtrema_SolutionElem theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in BRepExtrema_SolutionElem theLeft, in BRepExtrema_SolutionElem theRight) => !theLeft.Equals(theRight);

    public override string ToString() =>
        $"BRepExtrema_SolutionElem({mySupType}, {myPoint}, "
        + gp.Format("", myDist, myPar1, myPar2) + ")";
}
