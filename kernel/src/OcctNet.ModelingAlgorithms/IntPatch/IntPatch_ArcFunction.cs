// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ArcFunction.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ArcFunction.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ArcFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The signed distance to a quadric along an arc of restriction of a parametric surface: F(t) = Q(S(A(t))), with its
/// derivative; the function of the boundary search (IntPatch_TheSOnBounds). Each state saved by
/// <see cref="GetStateNumber"/> keeps the last evaluated point.
/// </summary>
/// <remarks>
/// A working object. It keeps the arc and the surface (adaptors, by reference) and a copy of the quadric (a struct, as
/// OCCT's member). It implements <see cref="IIntStart_ArcFunction"/>, what IntStart_SearchOnBoundaries reads.
/// </remarks>
public class IntPatch_ArcFunction : FunctionWithDerivative, IIntStart_ArcFunction
{
    private Curve2DAdaptorBase? myArc;
    private SurfaceAdaptorBase? mySurf;
    private IntSurf_Quadric myQuad;
    private Point3D ptsol;
    private readonly NCollection_Sequence<Point3D> seqpt;

    public IntPatch_ArcFunction()
    {
        myQuad = new IntSurf_Quadric();
        ptsol = new Point3D();
        seqpt = new NCollection_Sequence<Point3D>();
    }

    public void SetQuadric(in IntSurf_Quadric Q)
    {
        myQuad = Q;
        seqpt.Clear();
    }

    public void Set(Curve2DAdaptorBase A)
    {
        myArc = A;
        seqpt.Clear();
    }

    public void Set(SurfaceAdaptorBase S)
    {
        mySurf = S;
        seqpt.Clear();
    }

    public override bool Value(double X, out double F)
    {
        Point2D p2d = myArc!.Value(X);
        mySurf!.D0(p2d.X(), p2d.Y(), out ptsol);
        F = myQuad.Distance(ptsol);
        return true;
    }

    public override bool Derivative(double X, out double D)
    {
        Vector3D v = new();
        myArc!.D1(X, out Point2D p2d, out Vector2D d2d);
        mySurf!.D1(p2d.X(), p2d.Y(), out ptsol, out Vector3D d1u, out Vector3D d1v);
        v.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
        D = v.Dot(myQuad.Gradient(ptsol));
        return true;
    }

    public override bool Values(double X, out double F, out double D)
    {
        Vector3D v1 = new();
        Vector3D v2 = new();
        F = 0.0;
        myArc!.D1(X, out Point2D p2d, out Vector2D d2d);
        mySurf!.D1(p2d.X(), p2d.Y(), out ptsol, out Vector3D d1u, out Vector3D d1v);
        v1.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
        myQuad.ValAndGrad(ptsol, ref F, ref v2);
        D = v1.Dot(v2);
        return true;
    }

    /// <summary>The number of samples on the arc: the largest of the surface's and the arc's.</summary>
    public int NbSamples()
    {
        return Max(Max(IntPatch_HInterTool.NbSamplesU(mySurf!, 0.0, 0.0), IntPatch_HInterTool.NbSamplesV(mySurf!, 0.0, 0.0)),
                   IntPatch_HInterTool.NbSamplesOnArc(myArc!));
    }

    /// <summary>Saves the last evaluated point; returns its rank for <see cref="Valpoint"/>.</summary>
    public override int GetStateNumber()
    {
        seqpt.Append(ptsol);
        return seqpt.Length();
    }

    /// <summary>The point saved at state <paramref name="Index"/>.</summary>
    public Point3D Valpoint(int Index) => seqpt.Value(Index);

    public IntSurf_Quadric Quadric() => (myQuad);

    public Curve2DAdaptorBase Arc() => (myArc!);

    public SurfaceAdaptorBase Surface() => (mySurf!);

    /// <summary>The point computed by the last call to <see cref="Value"/> (or the derivatives).</summary>
    public Point3D LastComputedPoint() => ptsol;
}
