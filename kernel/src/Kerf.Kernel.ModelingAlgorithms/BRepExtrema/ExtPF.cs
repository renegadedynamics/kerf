// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtPF.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ExtPF.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Extrema;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Extrema.ExtAlgo;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>The extrema of the distance between a vertex and a face (points inside the face or on its boundary).</summary>
/// <remarks>A working object (decision 21): initialize with a face, then perform with any number of vertices.</remarks>
internal sealed class ExtPF
{
    private readonly ExtPS _extPS = new();
    private readonly NCollection_Sequence<double> _sqDist = new();
    private readonly NCollection_Sequence<POnSurf> _points = new();
    private readonly FaceSurfaceAdaptor _surf = new();

    public ExtPF()
    {
    }

    /// <summary>Computes all the extrema between the vertex and the face.</summary>
    public ExtPF(in Vertex theVertex,
                             in Face theFace,
                             ExtFlag theFlag = ExtFlag_MINMAX,
                             ExtAlgo theAlgo = ExtAlgo_Grad)
    {
        Initialize(theFace, theFlag, theAlgo);
        Perform(theVertex, theFace);
    }

    public void Initialize(in Face theFace,
                           ExtFlag theFlag = ExtFlag_MINMAX,
                           ExtAlgo theAlgo = ExtAlgo_Grad)
    {
        // cette surface doit etre en champ. Extrema ne fait
        // pas de copie et prend seulement un pointeur dessus.
        _surf.Initialize(theFace, false);

        if (_surf.GetType() == OtherSurface)
        {
            return; // protect against non-geometric type (e.g. triangulation)
        }

        double tol = Min(Tool.Tolerance(theFace), global::Kerf.Engine.Foundation.Precision.Confusion());
        double tolU, tolV;
        tolU = Max(_surf.UResolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        tolV = Max(_surf.VResolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        double u1, u2, v1, v2;
        BrepTools.UVBounds(theFace, out u1, out u2, out v1, out v2);
        _extPS.SetFlag(theFlag);
        _extPS.SetAlgo(theAlgo);
        _extPS.Initialize(_surf, u1, u2, v1, v2, tolU, tolV);
    }

    /// <summary>
    /// Computes the extrema for the vertex. The face is used only to classify the points, not for the surface (which
    /// is the one given to Initialize).
    /// </summary>
    public void Perform(in Vertex theVertex, in Face theFace)
    {
        _sqDist.Clear();
        _points.Clear();

        Point3D p = Tool.Pnt(theVertex);
        if (_surf.GetType() == OtherSurface)
        {
            return; // protect against non-geometric type (e.g. triangulation)
        }

        _extPS.Perform(p);

        // Exploration of points and classification
        if (_extPS.IsDone)
        {
            var classifier = new FaceClassifier();
            double u1, u2;
            double tol = Tool.Tolerance(theFace);
            for (int i = 1; i <= _extPS.ExtCount; i++)
            {
                _extPS.Point(i).Parameter(out u1, out u2);
                var puv = new Point2D(u1, u2);
                classifier.Perform(theFace, puv, tol);
                Containment state = classifier.State();
                if (state == On || state == In)
                {
                    _sqDist.Append(_extPS.SquareDistance(i));
                    _points.Append(_extPS.Point(i));
                }
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _extPS.IsDone;

    /// <summary>The number of extremum distances.</summary>
    public int ExtCount => _points.Length();

    /// <summary>The <paramref name="n"/>th extremum square distance.</summary>
    public double SquareDistance(int n) => _sqDist.Value(n);

    /// <summary>The parameters on the face of the <paramref name="n"/>th extremum distance.</summary>
    public void Parameter(int n, out double u, out double v) => _points.Value(n).Parameter(out u, out v);

    /// <summary>The point of the <paramref name="n"/>th extremum distance.</summary>
    public Point3D Point(int n) => _points.Value(n).Value();

    public void SetFlag(ExtFlag f) => _extPS.SetFlag(f);

    public void SetAlgo(ExtAlgo a) => _extPS.SetAlgo(a);
}
