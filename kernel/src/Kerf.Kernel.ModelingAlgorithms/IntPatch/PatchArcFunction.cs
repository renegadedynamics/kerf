// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ArcFunction.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ArcFunction.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ArcFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The signed distance to a quadric along an arc of restriction of a parametric surface: F(t) = Q(S(A(t))), with its
/// derivative; the function of the boundary search (IntPatch_TheSOnBounds). Each state saved by
/// <see cref="GetStateNumber"/> keeps the last evaluated point.
/// </summary>
/// <remarks>
/// A working object. It keeps the arc and the surface (adaptors, by reference) and a copy of the quadric (a struct, as
/// OCCT's member). It implements <see cref="IArcFunction"/>, what IntStart_SearchOnBoundaries reads.
/// </remarks>
internal class PatchArcFunction : FunctionWithDerivative, IArcFunction
{
    private Curve2DAdaptorBase? _arc;
    private SurfaceAdaptorBase? _surf;
    private SurfaceQuadric _quad;
    private Point3D _ptsol;
    private readonly NCollection_Sequence<Point3D> _seqpt;

    public PatchArcFunction()
    {
        _quad = new SurfaceQuadric();
        _ptsol = new Point3D();
        _seqpt = new NCollection_Sequence<Point3D>();
    }

    public void SetQuadric(in SurfaceQuadric q)
    {
        _quad = q;
        _seqpt.Clear();
    }

    public void Set(Curve2DAdaptorBase a)
    {
        _arc = a;
        _seqpt.Clear();
    }

    public void Set(SurfaceAdaptorBase s)
    {
        _surf = s;
        _seqpt.Clear();
    }

    public override bool Value(double x, out double f)
    {
        Point2D p2d = _arc!.Value(x);
        _surf!.D0(p2d.X, p2d.Y, out _ptsol);
        f = _quad.Distance(_ptsol);
        return true;
    }

    public override bool Derivative(double x, out double d)
    {
        Vector3D v = new();
        _arc!.D1(x, out Point2D p2d, out Vector2D d2d);
        _surf!.D1(p2d.X, p2d.Y, out _ptsol, out Vector3D d1u, out Vector3D d1v);
        v = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
        d = v.Dot(_quad.Gradient(_ptsol));
        return true;
    }

    public override bool Values(double x, out double f, out double d)
    {
        Vector3D v1 = new();
        Vector3D v2 = new();
        f = 0.0;
        _arc!.D1(x, out Point2D p2d, out Vector2D d2d);
        _surf!.D1(p2d.X, p2d.Y, out _ptsol, out Vector3D d1u, out Vector3D d1v);
        v1 = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
        _quad.ValAndGrad(_ptsol, ref f, ref v2);
        d = v1.Dot(v2);
        return true;
    }

    /// <summary>The number of samples on the arc: the largest of the surface's and the arc's.</summary>
    public int NbSamples()
    {
        return Max(Max(HInterTool.NbSamplesU(_surf!, 0.0, 0.0), HInterTool.NbSamplesV(_surf!, 0.0, 0.0)),
                   HInterTool.NbSamplesOnArc(_arc!));
    }

    /// <summary>Saves the last evaluated point; returns its rank for <see cref="Valpoint"/>.</summary>
    public override int GetStateNumber()
    {
        _seqpt.Append(_ptsol);
        return _seqpt.Length();
    }

    /// <summary>The point saved at state <paramref name="index"/>.</summary>
    public Point3D Valpoint(int index) => _seqpt.Value(index);

    public SurfaceQuadric Quadric() => (_quad);

    public Curve2DAdaptorBase Arc() => (_arc!);

    public SurfaceAdaptorBase Surface() => (_surf!);

    /// <summary>The point computed by the last call to <see cref="Value"/> (or the derivatives).</summary>
    public Point3D LastComputedPoint() => _ptsol;
}
