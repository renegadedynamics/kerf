// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ApproxSeewing.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ApproxSeewing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// Approximates a <see cref="FillingMultiLine"/> by a 3D B-spline curve and its two pcurves, through 50 points
/// spread along it (AppDef_Compute), or by polygons when the approximation fails.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT copies the MultiLine; the port shares it, which is the same, because a
/// BRepFill_MultiLine never changes after its construction. The curves handed out are new and unfrozen, as OCCT's.
/// </remarks>
internal sealed class ApproxSeewing
{
    private FillingMultiLine _ml = new();
    private bool _isDone;
    private Curve3D? _curve;
    private Curve2D? _pCurve1;
    private Curve2D? _pCurve2;

    /// <summary>An empty approximation.</summary>
    public ApproxSeewing()
    {
        _isDone = false;
    }

    /// <summary>Approximates <paramref name="ml"/>.</summary>
    public ApproxSeewing(FillingMultiLine ml)
    {
        _isDone = false;
        Perform(ml);
    }

    /// <summary>Approximates <paramref name="ml"/>.</summary>
    public void Perform(FillingMultiLine ml)
    {
        _ml = ml;

        // evaluate the approximative length of the 3dCurve
        int i;
        double length = 0.0;
        double u1 = _ml.FirstParameter();
        double u2 = _ml.LastParameter();
        int nbPoints = 50;
        double dist, dU = (u2 - u1) / (2 * nbPoints - 1);

        NCollection_Array1<Point2D> lp = new(1, 2 * nbPoints); // tableau Longueur <-> Param
        Point3D pnt1, pnt2;
        pnt1 = _ml.Value(u1);

        for (i = 0; i < 2 * nbPoints; i++)
        {
            pnt2 = _ml.Value(u1 + i * dU);
            dist = pnt1.DistanceTo(pnt2);
            length += dist;
            lp[i + 1] = new Point2D(length, u1 + (i * dU));
            pnt1 = pnt2;
        }

        // On cherche a mettre NbPoints dans la curve.
        // on met les points environ a Length/NbPoints.

        MultiLine mls = new(nbPoints);
        MultiPointConstraint mp = new(1, 2);
        Point3D p3d;
        Point2D pf1, pf2;

        ml.Value3dOnF1OnF2(u1, out p3d, out pf1, out pf2);
        mp.SetPoint(1, p3d);
        mp.SetPoint2d(2, pf1);
        mp.SetPoint2d(3, pf2);
        mls.SetValue(1, mp);

        double dCorde = length / (nbPoints - 1);
        double corde = dCorde;
        int index = 1;
        double u, alpha;
        for (i = 2; i < nbPoints; i++)
        {
            while (lp[index].X < corde)
            {
                index++;
            }
            alpha = (corde - lp[index - 1].X) / (lp[index].X - lp[index - 1].X);
            u = lp[index - 1].Y + alpha * (lp[index].Y - lp[index - 1].Y);
            MultiPointConstraint mpc = new(1, 2);
            ml.Value3dOnF1OnF2(u, out p3d, out pf1, out pf2);
            mpc.SetPoint(1, p3d);
            mpc.SetPoint2d(2, pf1);
            mpc.SetPoint2d(3, pf2);
            mls.SetValue(i, mpc);
            corde = i * dCorde;
        }
        MultiPointConstraint mpe = new(1, 2);
        ml.Value3dOnF1OnF2(u2, out p3d, out pf1, out pf2);
        mpe.SetPoint(1, p3d);
        mpe.SetPoint2d(2, pf1);
        mpe.SetPoint2d(3, pf2);
        mls.SetValue(nbPoints, mpe);

        Compute fit = new(mls);

        int nbCurves = fit.MultiCurveCount;

        if (nbCurves == 0)
        {
            NCollection_Array1<double> Knots = new(1, nbPoints);
            NCollection_Array1<int> Mults = new(1, nbPoints);
            Mults.Init(1);
            Mults[1] = Mults[nbPoints] = 2;
            NCollection_Array1<Point3D> p = new(1, nbPoints);
            NCollection_Array1<Point2D> p1 = new(1, nbPoints);
            NCollection_Array1<Point2D> p2 = new(1, nbPoints);

            double uf = ml.FirstParameter();
            double ul = ml.LastParameter();
            double dUlf = (ul - uf) / (nbPoints - 1);
            MultiPointConstraint mpc;
            for (i = 1; i <= nbPoints - 1; i++)
            {
                mpc = mls.Value(i);
                u = uf + (i - 1) * dUlf;
                p[i] = mpc.Point(1);
                p1[i] = mpc.Point2d(2);
                p2[i] = mpc.Point2d(3);
                Knots[i] = u;
            }
            // eval the last point on Ul
            mpc = mls.Value(nbPoints);
            p[nbPoints] = mpc.Point(1);
            p1[nbPoints] = mpc.Point2d(2);
            p2[nbPoints] = mpc.Point2d(3);
            Knots[nbPoints] = ul;

            _curve = new BSplineCurve3D(p, Knots, Mults, 1);
            _pCurve1 = new BSplineCurve2D(p1, Knots, Mults, 1);
            _pCurve2 = new BSplineCurve2D(p2, Knots, Mults, 1);

            _isDone = true;

            return;
        }

        // Les approx sont a priori OK.

        MultiBSpCurve mbSp = fit.SplineValue();
        int nbPoles = mbSp.NbPoles();
        NCollection_Array1<Point3D> poles = new(1, nbPoles);
        NCollection_Array1<Point2D> poles2d1 = new(1, nbPoles);
        NCollection_Array1<Point2D> poles2d2 = new(1, nbPoles);

        mbSp.Curve(1, poles);
        mbSp.Curve(2, poles2d1);
        mbSp.Curve(3, poles2d2);

        NCollection_Array1<double> knots = mbSp.Knots();
        NCollection_Array1<int> mults = mbSp.Multiplicities();
        int degree = mbSp.Degree();

        _curve = new BSplineCurve3D(poles, knots, mults, degree);
        _pCurve1 = new BSplineCurve2D(poles2d1, knots, mults, degree);
        _pCurve2 = new BSplineCurve2D(poles2d2, knots, mults, degree);

        _isDone = true;
    }

    /// <summary>True once <see cref="Perform"/> has run.</summary>
    public bool IsDone => _isDone;

    /// <summary>The 3D curve.</summary>
    /// <exception cref="NotDoneException">Not done.</exception>
    public Curve3D Curve()
    {
        StdFail_NotDone_Raise_if(!_isDone, "BRepFill_ApproxSeewing::Curve");
        return _curve!;
    }

    /// <summary>The pcurve on the first face.</summary>
    /// <exception cref="NotDoneException">Not done.</exception>
    public Curve2D CurveOnF1()
    {
        StdFail_NotDone_Raise_if(!_isDone, "BRepFill_ApproxSeewing::CurveOnF1");
        return _pCurve1!;
    }

    /// <summary>The pcurve on the second face.</summary>
    /// <exception cref="NotDoneException">Not done.</exception>
    public Curve2D CurveOnF2()
    {
        StdFail_NotDone_Raise_if(!_isDone, "BRepFill_ApproxSeewing::CurveOnF2");
        return _pCurve2!;
    }
}
