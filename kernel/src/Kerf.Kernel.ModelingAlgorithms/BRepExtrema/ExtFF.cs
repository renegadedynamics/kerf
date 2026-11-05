// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtFF.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtFF.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
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

/// <summary>The extrema of the distance between two faces (points inside the faces or on their boundaries).</summary>
/// <remarks>
/// A working object (decision 21). OCCT copies local surface adaptors into new handles; the port keeps the adaptors
/// it made (decision 394).
/// </remarks>
internal sealed class ExtFF
{
    private readonly ExtSS _extSS = new();
    private readonly NCollection_Sequence<double> _sqDist = new();
    private readonly NCollection_Sequence<POnSurf> _pointsOnS1 = new();
    private readonly NCollection_Sequence<POnSurf> _pointsOnS2 = new();
    private FaceSurfaceAdaptor? _hs;

    public ExtFF()
    {
    }

    /// <summary>Computes all the extrema between the faces.</summary>
    public ExtFF(in Face f1, in Face f2)
    {
        Initialize(f2);
        Perform(f1, f2);
    }

    public void Initialize(in Face f2)
    {
        var surf = new FaceSurfaceAdaptor(f2);
        if (surf.GetType() == OtherSurface)
        {
            return; // protect against non-geometric type (e.g. triangulation)
        }

        _hs = surf;
        double tol = Min(Tool.Tolerance(f2), global::Kerf.Engine.Foundation.Precision.Confusion());
        tol = Min(surf.UResolution(tol), surf.VResolution(tol));
        tol = Max(tol, global::Kerf.Engine.Foundation.Precision.PConfusion());
        double u1, u2, v1, v2;
        BrepTools.UVBounds(f2, out u1, out u2, out v1, out v2);
        _extSS.Initialize(_hs, u1, u2, v1, v2, tol);
    }

    /// <summary>
    /// Computes the extrema for the face <paramref name="f1"/>. The face <paramref name="f2"/> is used only to classify
    /// the points, not for the surface (which is the one given to Initialize).
    /// </summary>
    public void Perform(in Face f1, in Face f2)
    {
        _sqDist.Clear();
        _pointsOnS1.Clear();
        _pointsOnS2.Clear();

        var surf1 = new FaceSurfaceAdaptor(f1);
        if (_hs is null || surf1.GetType() == OtherSurface)
        {
            return; // protect against non-geometric type (e.g. triangulation)
        }

        FaceSurfaceAdaptor hs1 = surf1;
        double tol1 = Min(Tool.Tolerance(f1), global::Kerf.Engine.Foundation.Precision.Confusion());
        tol1 = Min(surf1.UResolution(tol1), surf1.VResolution(tol1));
        tol1 = Max(tol1, global::Kerf.Engine.Foundation.Precision.PConfusion());
        double u1, u2, v1, v2;
        BrepTools.UVBounds(f1, out u1, out u2, out v1, out v2);
        _extSS.Perform(hs1, u1, u2, v1, v2, tol1);

        if (!_extSS.IsDone)
        {
            return;
        }

        if (_extSS.IsParallel)
        {
            _sqDist.Append(_extSS.SquareDistance(1));
        }
        else
        {
            // Exploration of points and classification
            var classifier = new FaceClassifier();
            double tol2 = Tool.Tolerance(f2);

            int i;
            for (i = 1; i <= _extSS.ExtCount; i++)
            {
                _extSS.Points(i, out POnSurf p1, out POnSurf p2);
                p1.Parameter(out u1, out u2);
                var puv1 = new Point2D(u1, u2);
                classifier.Perform(f1, puv1, tol1);
                Containment state1 = classifier.State();
                if (state1 == On || state1 == In)
                {
                    p2.Parameter(out u1, out u2);
                    var puv2 = new Point2D(u1, u2);
                    classifier.Perform(f2, puv2, tol2);
                    Containment state2 = classifier.State();
                    if (state2 == On || state2 == In)
                    {
                        _sqDist.Append(_extSS.SquareDistance(i));
                        _pointsOnS1.Append(p1);
                        _pointsOnS2.Append(p2);
                    }
                }
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _extSS.IsDone;

    /// <summary>True if the surfaces are parallel.</summary>
    public bool IsParallel => _extSS.IsParallel;

    /// <summary>The number of extremum distances.</summary>
    public int ExtCount => _sqDist.Length();

    /// <summary>The <paramref name="n"/>th extremum square distance.</summary>
    public double SquareDistance(int n) => _sqDist.Value(n);

    /// <summary>The parameters on the face F1 of the <paramref name="n"/>th extremum distance.</summary>
    public void ParameterOnFace1(int n, out double u, out double v) => _pointsOnS1.Value(n).Parameter(out u, out v);

    /// <summary>The parameters on the face F2 of the <paramref name="n"/>th extremum distance.</summary>
    public void ParameterOnFace2(int n, out double u, out double v) => _pointsOnS2.Value(n).Parameter(out u, out v);

    /// <summary>The point on the face F1 of the <paramref name="n"/>th extremum distance.</summary>
    public Point3D PointOnFace1(int n) => _pointsOnS1.Value(n).Value();

    /// <summary>The point on the face F2 of the <paramref name="n"/>th extremum distance.</summary>
    public Point3D PointOnFace2(int n) => _pointsOnS2.Value(n).Value();
}
