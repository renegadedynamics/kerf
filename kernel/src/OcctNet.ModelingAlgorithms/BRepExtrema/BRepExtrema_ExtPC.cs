// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtPC.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtPC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The extrema of the distance between a vertex and an edge.</summary>
/// <remarks>A working object (decision 21): initialize with an edge, then perform with any number of vertices.</remarks>
public sealed class BRepExtrema_ExtPC
{
    private readonly Extrema_ExtPC myExtPC = new();
    private BRepAdaptor_Curve? myHC;

    public BRepExtrema_ExtPC()
    {
    }

    /// <summary>Computes all the extrema between the vertex <paramref name="V"/> and the edge <paramref name="E"/>.</summary>
    public BRepExtrema_ExtPC(in TopoDS_Vertex V, in TopoDS_Edge E)
    {
        Initialize(E);
        Perform(V);
    }

    public void Initialize(in TopoDS_Edge E)
    {
        if (!BRep_Tool.IsGeometric(E))
        {
            return; // protect against non-geometric type (e.g. polygon)
        }
        double U1, U2;
        myHC = new BRepAdaptor_Curve(E);
        double Tol = Min(BRep_Tool.Tolerance(E), global::Kerf.Engine.Foundation.Precision.Confusion());
        Tol = Max(myHC.Resolution(Tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        BRep_Tool.Range(E, out U1, out U2);
        myExtPC.Initialize(myHC, U1, U2, Tol);
    }

    /// <summary>Computes the extrema for the vertex <paramref name="V"/> (nothing if the edge isn't geometric).</summary>
    public void Perform(in TopoDS_Vertex V)
    {
        if (myHC is not null)
        {
            Point3D P = BRep_Tool.Pnt(V);
            myExtPC.Perform(P);
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myExtPC.IsDone();

    /// <summary>The number of extremum distances.</summary>
    public int NbExt() => myExtPC.NbExt();

    /// <summary>True if the <paramref name="N"/>th extremum distance is a minimum.</summary>
    public bool IsMin(int N) => myExtPC.IsMin(N);

    /// <summary>The <paramref name="N"/>th extremum square distance.</summary>
    public double SquareDistance(int N) => myExtPC.SquareDistance(N);

    /// <summary>The parameter on the edge of the <paramref name="N"/>th extremum distance.</summary>
    public double Parameter(int N) => myExtPC.Point(N).Parameter();

    /// <summary>The point of the <paramref name="N"/>th extremum distance.</summary>
    public Point3D Point(int N) => myExtPC.Point(N).Value();

    /// <summary>For a trimmed curve, the square distances to the points at its first and last parameters.</summary>
    public void TrimmedSquareDistances(out double dist1, out double dist2, out Point3D pnt1, out Point3D pnt2)
        => myExtPC.TrimmedSquareDistances(out dist1, out dist2, out pnt1, out pnt2);
}
