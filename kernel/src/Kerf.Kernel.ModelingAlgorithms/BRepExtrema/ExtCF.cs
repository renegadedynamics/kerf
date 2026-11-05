// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtCF.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtCF.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>The extrema of the distance between an edge and a face (points inside the face or on its boundary).</summary>
/// <remarks>
/// A working object (decision 21). OCCT copies a local surface adaptor into a new handle; the port keeps the adaptor
/// it made (decision 394).
/// </remarks>
internal sealed class ExtCF
{
    private readonly ExtCS _extCS = new();
    private readonly NCollection_Sequence<double> _sqDist = new();
    private readonly NCollection_Sequence<POnSurf> _pointsOnS = new();
    private readonly NCollection_Sequence<POnCurv> _pointsOnC = new();
    private FaceSurfaceAdaptor? _hs;

    public ExtCF()
    {
    }

    /// <summary>Computes all the extrema between the edge and the face.</summary>
    public ExtCF(in Edge e, in Face f)
    {
        Initialize(e, f);
        Perform(e, f);
    }

    public void Initialize(in Edge e, in Face f)
    {
        var surf = new FaceSurfaceAdaptor(f);
        if (surf.GetType() == OtherSurface || !Tool.IsGeometric(e))
        {
            return; // protect against non-geometric type (e.g. triangulation)
        }
        var c = new EdgeCurveAdaptor(e);
        _hs = surf;
        double tolC, tolS;
        //
        tolS = Min(Tool.Tolerance(f), global::Kerf.Engine.Foundation.Precision.Confusion());
        tolS = Min(surf.UResolution(tolS), surf.VResolution(tolS));
        tolS = Max(tolS, global::Kerf.Engine.Foundation.Precision.PConfusion());
        //
        tolC = Min(Tool.Tolerance(e), global::Kerf.Engine.Foundation.Precision.Confusion());
        tolC = c.Resolution(tolC);
        tolC = Max(tolC, global::Kerf.Engine.Foundation.Precision.PConfusion());
        //
        double u1, u2, v1, v2;
        BrepTools.UVBounds(f, out u1, out u2, out v1, out v2);
        _extCS.Initialize(_hs, u1, u2, v1, v2, tolC, tolS);
    }

    /// <summary>
    /// Computes the extrema for the edge. The face is used only to classify the points, not for the surface (which
    /// is the one given to Initialize).
    /// </summary>
    public void Perform(in Edge e, in Face f2)
    {
        _sqDist.Clear();
        _pointsOnS.Clear();
        _pointsOnC.Clear();

        if (_hs is null)
        {
            return; // protect against non-geometric type (e.g. triangulation)
        }

        double u1, u2;
        Tool.Range(e, out u1, out u2);

        var curv = new EdgeCurveAdaptor(e);
        EdgeCurveAdaptor hc = curv;
        _extCS.Perform(hc, u1, u2);

        if (!_extCS.IsDone)
        {
            return;
        }

        if (_extCS.IsParallel)
        {
            _sqDist.Append(_extCS.SquareDistance(1));
        }
        else
        {
            // Exploration of points and classification
            double tol = Tool.Tolerance(f2);
            var classifier = new FClass2d(f2, tol);

            // If the underlying surface of the face is periodic
            // Extrema should return the point within the period,
            // so there is no point to adjust it in classifier.
            bool isAdjustPeriodic = false;

            for (int i = 1; i <= _extCS.ExtCount; i++)
            {
                _extCS.Points(i, out POnCurv p1, out POnSurf p2);
                p2.Parameter(out u1, out u2);
                var puv = new Point2D(u1, u2);
                Containment state = classifier.Perform(puv, isAdjustPeriodic);
                if (state == On || state == In)
                {
                    _sqDist.Append(_extCS.SquareDistance(i));
                    _pointsOnC.Append(p1);
                    _pointsOnS.Append(p2);
                }
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _extCS.IsDone;

    /// <summary>The number of extremum distances.</summary>
    public int ExtCount => _sqDist.Length();

    /// <summary>The <paramref name="n"/>th extremum square distance.</summary>
    public double SquareDistance(int n) => _sqDist.Value(n);

    /// <summary>True if the curve is on a parallel surface.</summary>
    public bool IsParallel => _extCS.IsParallel;

    /// <summary>The parameter on the edge of the <paramref name="n"/>th extremum distance.</summary>
    public double ParameterOnEdge(int n) => _pointsOnC.Value(n).Parameter();

    /// <summary>The parameters on the face of the <paramref name="n"/>th extremum distance.</summary>
    public void ParameterOnFace(int n, out double u, out double v) => _pointsOnS.Value(n).Parameter(out u, out v);

    /// <summary>The point on the edge of the <paramref name="n"/>th extremum distance.</summary>
    public Point3D PointOnEdge(int n) => _pointsOnC.Value(n).Value();

    /// <summary>The point on the face of the <paramref name="n"/>th extremum distance.</summary>
    public Point3D PointOnFace(int n) => _pointsOnS.Value(n).Value();
}
