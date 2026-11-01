// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppSurface.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppSurface.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Approximates a blend's surface (and its curves on the supports) with Approx_SweepApproximation over a sweep
/// function (<see cref="BRepBlend_AppFuncRoot"/>), lowering the continuity asked for when the function cannot be
/// derived at the start.
/// </summary>
/// <remarks>A working object (decision 21); the arrays the accessors return are the approximation's own.</remarks>
public class BRepBlend_AppSurface : AppBlend_Approx
{
    private readonly Approx_SweepApproximation approx;

    public BRepBlend_AppSurface(Approx_SweepFunction Func, double First, double Last, double Tol3d, double Tol2d,
                                double TolAngular, GeomAbs_Shape Continuity = GeomAbs_C0, int Degmax = 11,
                                int Segmax = 50)
    {
        approx = new Approx_SweepApproximation(Func);

        int Nb2d = Func.Nb2dCurves();
        int NbPolSect, NbKnotSect, udeg;
        GeomAbs_Shape continuity = Continuity;

        // (1) Check that the function can be derived
        if (continuity != GeomAbs_C0)
        {
            if (Nb2d == 0)
            {
                Nb2d = 1;
            }
            Func.SectionShape(out NbPolSect, out NbKnotSect, out udeg);
            NCollection_Array1<double> W = new(1, NbPolSect);
            NCollection_Array1<gp_Pnt> P = new(1, NbPolSect);
            NCollection_Array1<gp_Pnt2d> P2d = new(1, Nb2d);
            NCollection_Array1<gp_Vec> V = new(1, NbPolSect);
            NCollection_Array1<gp_Vec2d> V2d = new(1, Nb2d);
            bool Ok;
            if (continuity == GeomAbs_C2)
            {
                Ok = Func.D2(First, First, Last, P, V, V, P2d, V2d, V2d, W, W, W);
                if (!Ok)
                {
                    continuity = GeomAbs_C1;
                }
            }
            if (continuity == GeomAbs_C1)
            {
                Ok = (Func.D1(First, First, Last, P, V, P2d, V2d, W, W));
                if (!Ok)
                {
                    continuity = GeomAbs_C0;
                }
            }
        }

        // (2) Approximation
        approx.Perform(First, Last, Tol3d, Tol3d, Tol2d, TolAngular, continuity, Degmax, Segmax);
    }

    public override bool IsDone() => approx.IsDone();

    public override void SurfShape(out int UDegree, out int VDegree, out int NbUPoles, out int NbVPoles,
                                   out int NbUKnots, out int NbVKnots)
    {
        approx.SurfShape(out UDegree, out VDegree, out NbUPoles, out NbVPoles, out NbUKnots, out NbVKnots);
    }

    public override void Surface(NCollection_Array2<gp_Pnt> TPoles, NCollection_Array2<double> TWeights,
                                 NCollection_Array1<double> TUKnots, NCollection_Array1<double> TVKnots,
                                 NCollection_Array1<int> TUMults, NCollection_Array1<int> TVMults)
    {
        approx.Surface(TPoles, TWeights, TUKnots, TVKnots, TUMults, TVMults);
    }

    public override int UDegree() => approx.UDegree();

    public override int VDegree() => approx.VDegree();

    public override NCollection_Array2<gp_Pnt> SurfPoles() => approx.SurfPoles();

    public override NCollection_Array2<double> SurfWeights() => approx.SurfWeights();

    public override NCollection_Array1<double> SurfUKnots() => approx.SurfUKnots();

    public override NCollection_Array1<double> SurfVKnots() => approx.SurfVKnots();

    public override NCollection_Array1<int> SurfUMults() => approx.SurfUMults();

    public override NCollection_Array1<int> SurfVMults() => approx.SurfVMults();

    /// <summary>The maximum error of the surface's approximation.</summary>
    public double MaxErrorOnSurf()
    {
        return approx.MaxErrorOnSurf();
    }

    public override int NbCurves2d() => approx.NbCurves2d();

    public override void Curves2dShape(out int Degree, out int NbPoles, out int NbKnots)
    {
        approx.Curves2dShape(out Degree, out NbPoles, out NbKnots);
    }

    public override void Curve2d(int Index, NCollection_Array1<gp_Pnt2d> TPoles, NCollection_Array1<double> TKnots,
                                 NCollection_Array1<int> TMults)
    {
        approx.Curve2d(Index, TPoles, TKnots, TMults);
    }

    public override int Curves2dDegree() => approx.Curves2dDegree();

    public override NCollection_Array1<gp_Pnt2d> Curve2dPoles(int Index) => approx.Curve2dPoles(Index);

    public override NCollection_Array1<double> Curves2dKnots() => approx.Curves2dKnots();

    public override NCollection_Array1<int> Curves2dMults() => approx.Curves2dMults();

    /// <summary>The maximum error of the approximation of the 2D curve <paramref name="Index"/>.</summary>
    public double Max2dError(int Index)
    {
        return approx.Max2dError(Index);
    }

    public override double TolCurveOnSurf(int Index)
    {
        return approx.TolCurveOnSurf(Index);
    }

    public override void TolReached(out double Tol3d, out double Tol2d)
    {
        Tol3d = approx.MaxErrorOnSurf();
        Tol2d = 0;
        for (int ii = 1; ii <= approx.NbCurves2d(); ii++)
        {
            Tol2d = Max(Tol2d, approx.Max2dError(ii));
        }
    }

    public void Dump(TextWriter o)
    {
        approx.Dump(o);
    }
}
