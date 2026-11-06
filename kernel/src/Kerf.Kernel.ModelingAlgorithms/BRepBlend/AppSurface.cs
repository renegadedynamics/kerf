// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppSurface.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppSurface.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// Approximates a blend's surface (and its curves on the supports) with Approx_SweepApproximation over a sweep
/// function (<see cref="AppFuncRoot"/>), lowering the continuity asked for when the function cannot be
/// derived at the start.
/// </summary>
/// <remarks>A working object (decision 21); the arrays the accessors return are the approximation's own.</remarks>
internal class AppSurface : Approx
{
    private readonly SweepApproximation _approx;

    public AppSurface(SweepFunction func, double first, double last, double tol3d, double tol2d,
                                double tolAngular, Continuity Continuity = C0, int degmax = 11,
                                int segmax = 50)
    {
        _approx = new SweepApproximation(func);

        int nb2d = func.Nb2dCurves();
        int nbPolSect, nbKnotSect, udeg;
        Continuity continuity = Continuity;

        // (1) Check that the function can be derived
        if (continuity != C0)
        {
            if (nb2d == 0)
            {
                nb2d = 1;
            }
            func.SectionShape(out nbPolSect, out nbKnotSect, out udeg);
            NCollection_Array1<double> w = new(1, nbPolSect);
            NCollection_Array1<Point3D> p = new(1, nbPolSect);
            NCollection_Array1<Point2D> p2d = new(1, nb2d);
            NCollection_Array1<Vector3D> v = new(1, nbPolSect);
            NCollection_Array1<Vector2D> v2d = new(1, nb2d);
            bool ok;
            if (continuity == C2)
            {
                ok = func.D2(first, first, last, p, v, v, p2d, v2d, v2d, w, w, w);
                if (!ok)
                {
                    continuity = C1;
                }
            }
            if (continuity == C1)
            {
                ok = (func.D1(first, first, last, p, v, p2d, v2d, w, w));
                if (!ok)
                {
                    continuity = C0;
                }
            }
        }

        // (2) Approximation
        _approx.Perform(first, last, tol3d, tol3d, tol2d, tolAngular, continuity, degmax, segmax);
    }

    public override bool IsDone => _approx.IsDone;

    public override void SurfShape(out int uDegree, out int vDegree, out int nbUPoles, out int nbVPoles,
                                   out int nbUKnots, out int nbVKnots)
    {
        _approx.SurfShape(out uDegree, out vDegree, out nbUPoles, out nbVPoles, out nbUKnots, out nbVKnots);
    }

    public override void Surface(NCollection_Array2<Point3D> tPoles, NCollection_Array2<double> tWeights,
                                 NCollection_Array1<double> tuKnots, NCollection_Array1<double> tvKnots,
                                 NCollection_Array1<int> tuMults, NCollection_Array1<int> tvMults)
    {
        _approx.Surface(tPoles, tWeights, tuKnots, tvKnots, tuMults, tvMults);
    }

    public override int UDegree() => _approx.UDegree();

    public override int VDegree() => _approx.VDegree();

    public override NCollection_Array2<Point3D> SurfPoles() => _approx.SurfPoles();

    public override NCollection_Array2<double> SurfWeights() => _approx.SurfWeights();

    public override NCollection_Array1<double> SurfUKnots() => _approx.SurfUKnots();

    public override NCollection_Array1<double> SurfVKnots() => _approx.SurfVKnots();

    public override NCollection_Array1<int> SurfUMults() => _approx.SurfUMults();

    public override NCollection_Array1<int> SurfVMults() => _approx.SurfVMults();

    /// <summary>The maximum error of the surface's approximation.</summary>
    public double MaxErrorOnSurf()
    {
        return _approx.MaxErrorOnSurf();
    }

    public override int Curves2dCount => _approx.Curves2dCount;

    public override void Curves2dShape(out int degree, out int nbPoles, out int nbKnots)
    {
        _approx.Curves2dShape(out degree, out nbPoles, out nbKnots);
    }

    public override void Curve2d(int index, NCollection_Array1<Point2D> tPoles, NCollection_Array1<double> tKnots,
                                 NCollection_Array1<int> tMults)
    {
        _approx.Curve2d(index, tPoles, tKnots, tMults);
    }

    public override int Curves2dDegree() => _approx.Curves2dDegree();

    public override NCollection_Array1<Point2D> Curve2dPoles(int index) => _approx.Curve2dPoles(index);

    public override NCollection_Array1<double> Curves2dKnots() => _approx.Curves2dKnots();

    public override NCollection_Array1<int> Curves2dMults() => _approx.Curves2dMults();

    /// <summary>The maximum error of the approximation of the 2D curve <paramref name="index"/>.</summary>
    public double Max2dError(int index)
    {
        return _approx.Max2dError(index);
    }

    public override double TolCurveOnSurf(int index)
    {
        return _approx.TolCurveOnSurf(index);
    }

    public override void TolReached(out double tol3d, out double tol2d)
    {
        tol3d = _approx.MaxErrorOnSurf();
        tol2d = 0;
        for (int ii = 1; ii <= _approx.Curves2dCount; ii++)
        {
            tol2d = Max(tol2d, _approx.Max2dError(ii));
        }
    }

    public void Dump(TextWriter o)
    {
        _approx.Dump(o);
    }
}
