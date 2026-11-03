// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectOnSurface.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>
/// Projects a curve on a surface along the surface normals. The result (a 3D curve) is an approximation.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Load</c> is public. Keeps the caller's adaptors by reference, as OCCT keeps
/// its handles. <see cref="BSpline"/> freezes the stored result and returns the same object at each call
/// (decision 243). As in OCCT, the result's poles are not the computed ones (occt-issues.md): <c>Load</c> copies
/// each Bezier segment's poles into the wrong array, so the poles after the first degree are the origin.
/// </remarks>
internal class ProjectOnSurface
{
    private CurveAdaptorBase? _curve;
    private SurfaceAdaptorBase? _surface;
    private double _tolerance;
    private bool _isDone;
    private BSplineCurve3D? _result;

    private static Point3D OnSurface_Value(double u, CurveAdaptorBase myCurve, ExtPS myExtPS)
    {
        // Try to find the closest solution point.
        myExtPS.Perform(myCurve.Value(u));

        double dist2Min = RealLast();
        int index = 0;

        for (int i = 1; i <= myExtPS.ExtCount; i++)
        {
            if (myExtPS.SquareDistance(i) < dist2Min)
            {
                index = i;
                dist2Min = myExtPS.SquareDistance(index);
            }
        }
        if (index == 0)
        {
            // OCCT also prints " Extrema non trouve pour U = " to std::cout.
            return new Point3D(0.0, 0.0, 0.0);
        }
        else
        {
            return (myExtPS.Point(index)).Value();
        }
    }

    private static bool OnSurface_D1(double u, CurveAdaptorBase myCurve, ExtPS myExtPS)
    {
        _ = u;
        _ = myCurve;
        _ = myExtPS;
        return false;
    }

    /// <summary>OCCT's file-local class ProjLib_OnSurface: the foot of the curve's point on the surface.</summary>
    private sealed class ProjLib_OnSurface : Function
    {
        private readonly CurveAdaptorBase _curve;
        private readonly ExtPS _extPS;

        public ProjLib_OnSurface(CurveAdaptorBase c, SurfaceAdaptorBase s)
        {
            _curve = c;
            _nbPnt = 1;
            _nbPnt2d = 0;
            double u = _curve.FirstParameter();
            Point3D p = _curve.Value(u);
            double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
            _extPS = new ExtPS(p, s, tol, tol);
        }

        public override double FirstParameter() => _curve.FirstParameter();

        public override double LastParameter() => _curve.LastParameter();

        public override bool Value(double t, NCollection_Array1<Point2D> pnt2d, NCollection_Array1<Point3D> pnt)
        {
            pnt[1] = OnSurface_Value(t, _curve, _extPS);
            return true;
        }

        public override bool D1(double t, NCollection_Array1<Vector2D> vec2d, NCollection_Array1<Vector3D> vec)
        {
            return OnSurface_D1(t, _curve, _extPS);
        }
    }

    //=====================================================================//
    //                                                                     //
    //  D E S C R I P T I O N   O F   T H E   C L A S S  :                 //
    //                                                                     //
    //         P r o j L i b _ A p p r o x P r o j e c t O n P l a n e     //
    //                                                                     //
    //=====================================================================//

    /// <summary>An empty projector.</summary>
    public ProjectOnSurface()
    {
        _tolerance = 0.0;
        _isDone = false;
    }

    /// <summary>A projector normally to the surface <paramref name="s"/>.</summary>
    public ProjectOnSurface(SurfaceAdaptorBase s)
    {
        _tolerance = 0.0;
        _isDone = false;
        _surface = s;
    }

    /// <summary>Sets the surface; the curve has to be loaded to compute the projection.</summary>
    public void Load(SurfaceAdaptorBase s)
    {
        _surface = s;
        _isDone = false;
    }

    /// <summary>Computes the projection of <paramref name="c"/> on the surface.</summary>
    public void Load(CurveAdaptorBase c, double tolerance)
    {
        _tolerance = tolerance;
        _curve = c;
        _isDone = false;
        if (_surface is not null)
        {
            ProjLib_OnSurface f = new(_curve, _surface);

            int deg1, deg2;
            deg1 = 8;
            deg2 = 8;

            FitAndDivide fit = new(f, deg1, deg2, global::Kerf.Engine.Foundation.Precision.Approximation(), global::Kerf.Engine.Foundation.Precision.PApproximation(), true);
            int i;
            int nbCurves = fit.MultiCurveCount;
            int maxDeg = 0;

            // To convert the MultiCurve to BSpline, all constituent Bezier curves
            // must have the same degree -> Calculate MaxDeg
            int nbPoles;
            for (i = 1; i <= nbCurves; i++)
            {
                int deg = fit.Value(i).Degree();
                maxDeg = Max(maxDeg, deg);
            }
            nbPoles = maxDeg * nbCurves + 1; // Poles on the BSpline
            NCollection_Array1<Point3D> poles = new(1, nbPoles);

            NCollection_Array1<Point3D> tempPoles = new(1, maxDeg + 1); // for degree elevation

            NCollection_Array1<double> knots = new(1, nbCurves + 1); // Knots of the BSpline

            int compt = 1;
            for (i = 1; i <= fit.MultiCurveCount; i++)
            {
                fit.Parameters(i, out knots[i], out knots[i + 1]);

                MultiCurve mc = fit.Value(i);                    // Load the i-th Curve
                NCollection_Array1<Point3D> localPoles = new(1, mc.Degree() + 1); // Get the poles
                // OCCT: the poles go to the start of Poles, not into LocalPoles, which stays at the origin; the loops
                // below then copy LocalPoles over Poles, so the result's poles after the first MaxDeg are the
                // origin (the first ones are the last segment's).
                mc.Curve(1, poles);

                // Possible degree elevation
                int inc = maxDeg - mc.Degree();
                if (inc > 0)
                {
                    // OCCT: passes the increment where BSplCLib::IncreaseDegree takes the new degree.
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(inc, localPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), tempPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
                    // update the poles of the PCurve
                    for (int j = 1; j <= maxDeg + 1; j++)
                    {
                        poles.SetValue(compt, tempPoles[j]);
                        compt++;
                    }
                }
                else
                {
                    // update the poles of the PCurve
                    for (int j = 1; j <= maxDeg + 1; j++)
                    {
                        poles.SetValue(compt, localPoles[j]);
                        compt++;
                    }
                }

                compt--;
            }

            // update the fields of ProjLib_Approx

            int nbKnots = nbCurves + 1;

            NCollection_Array1<int> mults = new(1, nbKnots);
            mults.SetValue(1, maxDeg + 1);
            for (i = 2; i <= nbCurves; i++)
            {
                mults.SetValue(i, maxDeg);
            }
            mults.SetValue(nbKnots, maxDeg + 1);
            _result = new BSplineCurve3D(poles, knots, mults, maxDeg, false);
            _isDone = true;
        }
    }

    public bool IsDone => _isDone;

    /// <summary>The projected curve, frozen; the same object at each call.</summary>
    /// <exception cref="NoSuchObjectException">The projection is not done.</exception>
    public BSplineCurve3D BSpline()
    {
        Standard_NoSuchObject_Raise_if(!_isDone, "ProjLib_ProjectOnSurface:BSpline");
        _result!.Freeze();
        return _result;
    }
}
