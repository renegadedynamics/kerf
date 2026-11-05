// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DegeneratedBound.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DegeneratedBound.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A degenerated boundary of a GeomFill_ConstrainedFilling: a point on a parameter range, with no other constraint.
/// </summary>
/// <remarks>A working object (decision 21): <see cref="Reparametrize"/> sets the range.</remarks>
internal class DegeneratedBound : Boundary
{
    private readonly Point3D _point;
    private double _first;
    private double _last;

    public DegeneratedBound(in Point3D point, double first, double last, double tol3d, double tolang)
        : base(tol3d, tolang)
    {
        _point = point;
        _first = first;
        _last = last;
    }

    public override Point3D Value(double u) => _point;

    public override void D1(double u, out Point3D p, out Vector3D v)
    {
        p = _point;
        v = new Vector3D(0.0, 0.0, 0.0);
    }

    public override void Reparametrize(double first, double last, bool hasDF, bool hasDL, double df, double dl, bool rev)
    {
        _first = first;
        _last = last;
    }

    public override void Bounds(out double first, out double last)
    {
        first = _first;
        last = _last;
    }

    public override bool IsDegenerated => true;
}
