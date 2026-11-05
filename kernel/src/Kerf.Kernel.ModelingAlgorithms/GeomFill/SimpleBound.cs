// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SimpleBound.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SimpleBound.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A 3d curve as a boundary of a GeomFill_ConstrainedFilling, not attached to a surface (no tangency constraint).
/// The filling surface will be within Tol3d of it; Tolang is kept but not used.
/// </summary>
/// <remarks>
/// A working object (decision 806): it keeps the caller's adaptor, as OCCT keeps the handle, and never changes it.
/// <see cref="Reparametrize"/> sets a reparametrisation law, a Law_BSpFunc whose B-spline is frozen.
/// </remarks>
internal class SimpleBound : Boundary
{
    private readonly CurveAdaptorBase _c3d;
    private LawsFunction? _par;

    public SimpleBound(CurveAdaptorBase curve, double tol3d, double tolang)
        : base(tol3d, tolang)
    {
        _c3d = curve;
    }

    public override Point3D Value(double u)
    {
        double x = u;
        if (_par is not null)
        {
            x = _par.Value(u);
        }
        return _c3d.Value(x);
    }

    public override void D1(double u, out Point3D p, out Vector3D v)
    {
        double x = u, dx = 1.0;
        if (_par is not null)
        {
            _par.D1(u, out x, out dx);
        }
        _c3d.D1(x, out p, out v);
        v = v.Multiplied(dx);
    }

    public override void Reparametrize(double first, double last, bool hasDF, bool hasDL, double df, double dl, bool rev)
    {
        BSpline curve = global::Kerf.Engine.Construction.Laws.Law.Reparametrize(_c3d, first, last, hasDF, hasDL, df, dl, rev, 30);
        _par = new BSpFunc();
        ((BSpFunc)_par).SetCurve(curve);
    }

    public override void Bounds(out double first, out double last)
    {
        if (_par is not null)
        {
            _par.Bounds(out first, out last);
        }
        else
        {
            first = _c3d.FirstParameter();
            last = _c3d.LastParameter();
        }
    }

    public override bool IsDegenerated => false;
}
