// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_QuasiUniformAbscissa.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_QuasiUniformAbscissa.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Measurement;

/// <summary>
/// Computes a given number of points on a curve spaced at an approximately uniform curvilinear distance. Bezier and
/// B-spline curves are divided along a polygonal approximation of their length; other curves go through
/// GCPnts_UniformAbscissa.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Initialize</c> recomputes. The templates over the curve type are ported per
/// instantiation (decision 187).
/// </remarks>
internal sealed class QuasiUniformAbscissa
{
    private bool _done;
    private int _nbPoints;
    private NCollection_HArray1<double>? _params;

    /// <summary>An empty algorithm; use <c>Initialize</c>.</summary>
    public QuasiUniformAbscissa()
    {
        _done = false;
        _nbPoints = 0;
    }

    /// <summary>theNbPoints points on the curve at an approximately uniform distance.</summary>
    public QuasiUniformAbscissa(CurveAdaptorBase c, int nbPoints)
        : this()
    {
        Initialize(c, nbPoints);
    }

    /// <summary>
    /// theNbPoints points on the part theU1..theU2 of the curve at an approximately uniform distance; the first and
    /// last are the points of theU1 and theU2.
    /// </summary>
    public QuasiUniformAbscissa(CurveAdaptorBase c, int nbPoints, double u1, double u2)
        : this()
    {
        Initialize(c, nbPoints, u1, u2);
    }

    /// <summary>theNbPoints points on the curve at an approximately uniform distance.</summary>
    public QuasiUniformAbscissa(Curve2DAdaptorBase c, int nbPoints)
        : this()
    {
        Initialize(c, nbPoints);
    }

    /// <summary>theNbPoints points on the part theU1..theU2 of the curve at an approximately uniform distance.</summary>
    public QuasiUniformAbscissa(Curve2DAdaptorBase c, int nbPoints, double u1, double u2)
        : this()
    {
        Initialize(c, nbPoints, u1, u2);
    }

    /// <summary>Recomputes theNbPoints points on the curve.</summary>
    public void Initialize(CurveAdaptorBase c, int nbPoints)
    {
        Initialize(c, nbPoints, c.FirstParameter(), c.LastParameter());
    }

    /// <summary>Recomputes theNbPoints points on the curve.</summary>
    public void Initialize(Curve2DAdaptorBase c, int nbPoints)
    {
        Initialize(c, nbPoints, c.FirstParameter(), c.LastParameter());
    }

    /// <summary>Recomputes theNbPoints points on the part theU1..theU2 of the curve.</summary>
    public void Initialize(CurveAdaptorBase c, int nbPoints, double u1, double u2)
    {
        InitializeCore(c, nbPoints, u1, u2);
    }

    /// <summary>Recomputes theNbPoints points on the part theU1..theU2 of the curve.</summary>
    public void Initialize(Curve2DAdaptorBase c, int nbPoints, double u1, double u2)
    {
        InitializeCore(c, nbPoints, u1, u2);
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => _done;

    /// <summary>The number of points computed.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int PointCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, "GCPnts_QuasiUniformAbscissa::NbPoints()");
            return _nbPoints;
        }
    }

    /// <summary>The parameter of the point of index Index (1 to NbPoints).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Parameter(int index)
    {
        StdFail_NotDone_Raise_if(!_done, "GCPnts_QuasiUniformAbscissa::Parameter()");
        return _params!.Value(index);
    }

    // ---- Templates over the curve type (3D instantiation) ----

    // This function divides given curve on the several parts with equal length.
    // It returns array of parameters in the control points.
    private void InitializeCore(CurveAdaptorBase c, int nbPoints, double u1, double u2)
    {
        if (c.GetType() != BezierCurve && c.GetType() != BSplineCurve)
        {
            UniformAbscissa ua = new(c, nbPoints, u1, u2);
            _done = ua.IsDone;
            // OCCT reads NbPoints() and Parameter() without checking IsDone(), whose checks its release build
            // compiles out: a failed result is copied as it is (decision 189).
            _nbPoints = ua.PointsUncheckedCount;
            _params = new NCollection_HArray1<double>(1, _nbPoints);
            for (int pntIter = 1; pntIter <= _nbPoints; ++pntIter)
            {
                _params.SetValue(pntIter, ua.ParameterUnchecked(pntIter));
            }
            return;
        }

        Standard_ConstructionError_Raise_if(
            nbPoints <= 1,
            "GCPnts_QuasiUniformAbscissa::Initialize(), number of points should be >= 2");

        // evaluate the approximative length of the 3dCurve
        _nbPoints = nbPoints;
        double length = 0.0;
        double dU = (u2 - u1) / (2 * nbPoints - 1);

        NCollection_Array1<Point2D> lp = new(1, 2 * nbPoints); // table Length <-> Param
        Point3D p1, p2;
        p1 = c.Value(u1);

        // Sum all distances
        for (int i = 0; i < 2 * nbPoints; ++i)
        {
            p2 = c.Value(u1 + i * dU);
            double dist = p1.DistanceTo(p2);
            length += dist;
            lp[i + 1] = new Point2D(length, u1 + (i * dU));
            p1 = p2;
        }

        // Try to distribute NbPoints on the curve.
        // Place points approximately at intervals of Length/NbPoints.
        if (IsEqual(length, 0.0))
        { // use usual analytical grid
            double step = (u2 - u1) / (nbPoints - 1);
            _params = new NCollection_HArray1<double>(1, nbPoints);
            _params.SetValue(1, u1);
            for (int i = 2; i < nbPoints; ++i)
            {
                _params.SetValue(i, u1 + step * (i - 1));
            }
        }
        else
        {
            double dCorde = length / (nbPoints - 1);
            double corde = dCorde;
            int index = 1;
            _params = new NCollection_HArray1<double>(1, nbPoints);
            _params.SetValue(1, u1);
            for (int i = 2; i < nbPoints; ++i)
            {
                while (lp[index].X < corde)
                {
                    ++index;
                }
                double alpha = (corde - lp[index - 1].X) / (lp[index].X - lp[index - 1].X);
                double u = lp[index - 1].Y + alpha * (lp[index].Y - lp[index - 1].Y);
                _params.SetValue(i, u);
                corde = i * dCorde;
            }
        }
        _params.SetValue(nbPoints, u2);
        _done = true;
    }

    // ---- Templates over the curve type (2D instantiation) ----

    // This function divides given curve on the several parts with equal length.
    // It returns array of parameters in the control points.
    private void InitializeCore(Curve2DAdaptorBase c, int nbPoints, double u1, double u2)
    {
        if (c.GetType() != BezierCurve && c.GetType() != BSplineCurve)
        {
            UniformAbscissa ua = new(c, nbPoints, u1, u2);
            _done = ua.IsDone;
            // OCCT reads NbPoints() and Parameter() without checking IsDone(), whose checks its release build
            // compiles out: a failed result is copied as it is (decision 189).
            _nbPoints = ua.PointsUncheckedCount;
            _params = new NCollection_HArray1<double>(1, _nbPoints);
            for (int pntIter = 1; pntIter <= _nbPoints; ++pntIter)
            {
                _params.SetValue(pntIter, ua.ParameterUnchecked(pntIter));
            }
            return;
        }

        Standard_ConstructionError_Raise_if(
            nbPoints <= 1,
            "GCPnts_QuasiUniformAbscissa::Initialize(), number of points should be >= 2");

        // evaluate the approximative length of the 3dCurve
        _nbPoints = nbPoints;
        double length = 0.0;
        double dU = (u2 - u1) / (2 * nbPoints - 1);

        NCollection_Array1<Point2D> lp = new(1, 2 * nbPoints); // table Length <-> Param
        Point2D p1, p2;
        p1 = c.Value(u1);

        // Sum all distances
        for (int i = 0; i < 2 * nbPoints; ++i)
        {
            p2 = c.Value(u1 + i * dU);
            double dist = p1.DistanceTo(p2);
            length += dist;
            lp[i + 1] = new Point2D(length, u1 + (i * dU));
            p1 = p2;
        }

        // Try to distribute NbPoints on the curve.
        // Place points approximately at intervals of Length/NbPoints.
        if (IsEqual(length, 0.0))
        { // use usual analytical grid
            double step = (u2 - u1) / (nbPoints - 1);
            _params = new NCollection_HArray1<double>(1, nbPoints);
            _params.SetValue(1, u1);
            for (int i = 2; i < nbPoints; ++i)
            {
                _params.SetValue(i, u1 + step * (i - 1));
            }
        }
        else
        {
            double dCorde = length / (nbPoints - 1);
            double corde = dCorde;
            int index = 1;
            _params = new NCollection_HArray1<double>(1, nbPoints);
            _params.SetValue(1, u1);
            for (int i = 2; i < nbPoints; ++i)
            {
                while (lp[index].X < corde)
                {
                    ++index;
                }
                double alpha = (corde - lp[index - 1].X) / (lp[index].X - lp[index - 1].X);
                double u = lp[index - 1].Y + alpha * (lp[index].Y - lp[index - 1].Y);
                _params.SetValue(i, u);
                corde = i * dCorde;
            }
        }
        _params.SetValue(nbPoints, u2);
        _done = true;
    }
}
