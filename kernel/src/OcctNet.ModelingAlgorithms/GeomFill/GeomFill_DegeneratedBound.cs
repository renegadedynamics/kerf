// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DegeneratedBound.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DegeneratedBound.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A degenerated boundary of a GeomFill_ConstrainedFilling: a point on a parameter range, with no other constraint.
/// </summary>
/// <remarks>A working object (decision 21): <see cref="Reparametrize"/> sets the range.</remarks>
public class GeomFill_DegeneratedBound : GeomFill_Boundary
{
    private readonly Point3D myPoint;
    private double myFirst;
    private double myLast;

    public GeomFill_DegeneratedBound(in Point3D Point, double First, double Last, double Tol3d, double Tolang)
        : base(Tol3d, Tolang)
    {
        myPoint = Point;
        myFirst = First;
        myLast = Last;
    }

    public override Point3D Value(double U) => myPoint;

    public override void D1(double U, out Point3D P, out Vector3D V)
    {
        P = myPoint;
        V = new Vector3D(0.0, 0.0, 0.0);
    }

    public override void Reparametrize(double First, double Last, bool HasDF, bool HasDL, double DF, double DL, bool Rev)
    {
        myFirst = First;
        myLast = Last;
    }

    public override void Bounds(out double First, out double Last)
    {
        First = myFirst;
        Last = myLast;
    }

    public override bool IsDegenerated() => true;
}
