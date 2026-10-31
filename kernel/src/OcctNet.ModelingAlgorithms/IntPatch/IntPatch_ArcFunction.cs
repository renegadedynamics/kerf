// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ArcFunction.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ArcFunction.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ArcFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
public class IntPatch_ArcFunction : math_FunctionWithDerivative, IIntStart_ArcFunction
{
    private Adaptor2d_Curve2d? myArc;
    private Adaptor3d_Surface? mySurf;
    private IntSurf_Quadric myQuad;
    private gp_Pnt ptsol;
    private readonly NCollection_Sequence<gp_Pnt> seqpt;

    public IntPatch_ArcFunction()
    {
        myQuad = new IntSurf_Quadric();
        ptsol = new gp_Pnt();
        seqpt = new NCollection_Sequence<gp_Pnt>();
    }

    public void SetQuadric(in IntSurf_Quadric Q)
    {
        myQuad = Q;
        seqpt.Clear();
    }

    public void Set(Adaptor2d_Curve2d A)
    {
        myArc = A;
        seqpt.Clear();
    }

    public void Set(Adaptor3d_Surface S)
    {
        mySurf = S;
        seqpt.Clear();
    }

    public override bool Value(double X, out double F)
    {
        gp_Pnt2d p2d = myArc!.Value(X);
        mySurf!.D0(p2d.X(), p2d.Y(), out ptsol);
        F = myQuad.Distance(ptsol);
        return true;
    }

    public override bool Derivative(double X, out double D)
    {
        gp_Vec v = new();
        myArc!.D1(X, out gp_Pnt2d p2d, out gp_Vec2d d2d);
        mySurf!.D1(p2d.X(), p2d.Y(), out ptsol, out gp_Vec d1u, out gp_Vec d1v);
        v.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
        D = v.Dot(myQuad.Gradient(ptsol));
        return true;
    }

    public override bool Values(double X, out double F, out double D)
    {
        gp_Vec v1 = new();
        gp_Vec v2 = new();
        F = 0.0;
        myArc!.D1(X, out gp_Pnt2d p2d, out gp_Vec2d d2d);
        mySurf!.D1(p2d.X(), p2d.Y(), out ptsol, out gp_Vec d1u, out gp_Vec d1v);
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
    public gp_Pnt Valpoint(int Index) => seqpt.Value(Index);

    public IntSurf_Quadric Quadric() => (myQuad);

    public Adaptor2d_Curve2d Arc() => (myArc!);

    public Adaptor3d_Surface Surface() => (mySurf!);

    /// <summary>The point computed by the last call to <see cref="Value"/> (or the derivatives).</summary>
    public gp_Pnt LastComputedPoint() => ptsol;
}
