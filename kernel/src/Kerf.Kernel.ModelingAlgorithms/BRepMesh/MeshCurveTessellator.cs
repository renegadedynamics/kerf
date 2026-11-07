// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CurveTessellator.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CurveTessellator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Tessellates an edge with GCPnts_TangentialDeflection at half the discrete edge's deflections, adding the edge's
/// internal vertices and, on its non-planar faces, points where the pcurve's chords deviate from the surface.
/// </summary>
/// <remarks>
/// A working object of one loop body: it evaluates its own adaptor of the edge (and of the face), never the discrete
/// face's shared surface adaptor (decision 490's audit).
/// </remarks>
internal class MeshCurveTessellator : CurveTessellator
{
    private readonly EdgeModel _dEdge;
    private readonly ToolParameters _parameters; // OCCT holds a reference; the caller's doesn't change meanwhile
    private readonly Edge _edge;
    private readonly EdgeCurveAdaptor _curve;
    private readonly int _minPointsNb;
    private readonly TangentialDeflection _discretTool = new();
    private double _squareEdgeDef;
    private double _squareMinSize;
    private double _edgeSqTol;
    private readonly double[] _faceRangeU = new double[2];
    private readonly double[] _faceRangeV = new double[2];

    /// <summary>A tessellator of the edge's 3D curve.</summary>
    /// <exception cref="FailureException">The parameters' MinSize isn't positive.</exception>
    public MeshCurveTessellator(EdgeModel edge,
                                     in ToolParameters parameters,
                                     int minPointsNb = 2)
    {
        _dEdge = edge;
        _parameters = parameters;
        _edge = edge.GetEdge();
        _curve = new EdgeCurveAdaptor(_edge);
        _minPointsNb = minPointsNb;
        Init();
    }

    /// <summary>A tessellator of the edge, with the orientation, as a curve on the face.</summary>
    /// <exception cref="FailureException">The parameters' MinSize isn't positive.</exception>
    public MeshCurveTessellator(EdgeModel edge,
                                     Orientation orientation,
                                     FaceModel face,
                                     in ToolParameters parameters,
                                     int minPointsNb = 2)
    {
        _dEdge = edge;
        _parameters = parameters;
        _edge = ShapeCasts.Edge(edge.GetEdge().Oriented(orientation));
        _curve = new EdgeCurveAdaptor(_edge, face.GetFace());
        _minPointsNb = minPointsNb;
        Init();
    }

    // Performs initialization of this tool.
    private void Init()
    {
        if (_parameters.MinSize <= 0.0)
        {
            throw new FailureException("The structure \"myParameters\" is not initialized");
        }

        double preciseAngDef = 0.5 * _dEdge.GetAngularDeflection();
        double preciseLinDef = 0.5 * _dEdge.GetDeflection();
        if (_edge.Orientation == Internal)
        {
            preciseLinDef *= 0.5;
        }

        preciseLinDef = Max(preciseLinDef, global::Kerf.Engine.Foundation.Precision.Confusion());
        preciseAngDef = Max(preciseAngDef, global::Kerf.Engine.Foundation.Precision.Angular());

        double minSize = _parameters.MinSize;
        if (_parameters.AdjustMinSize)
        {
            minSize = Min(minSize,
                           ToolParameters.RelMinSize()
                               * AbscissaPoint.Length(_curve,
                                                             _curve.FirstParameter(),
                                                             _curve.LastParameter(),
                                                             preciseLinDef));
        }

        _squareEdgeDef = preciseLinDef * preciseLinDef;
        _squareMinSize = Max(_squareEdgeDef, minSize * minSize);

        _edgeSqTol = Tool.Tolerance(_edge);
        _edgeSqTol *= _edgeSqTol;

        int minPntThreshold = 2;
        switch (_curve.GetType())
        {
            case Circle:
            case Ellipse:
            case Parabola:
            case Hyperbola:
                minPntThreshold = 4;
                break;

            default:
                break;
        }

        int minPntNb = Max(_minPointsNb, minPntThreshold); // OCC287
        _discretTool.Initialize(_curve,
                                 _curve.FirstParameter(),
                                 _curve.LastParameter(),
                                 preciseAngDef,
                                 preciseLinDef,
                                 minPntNb,
                                 global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                 minSize);

        if (_curve.IsCurveOnSurface)
        {
            CurveOnSurface curve = _curve.CurveOnSurface();
            SurfaceAdaptorBase surface = curve.GetSurface()!;

            double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
            double du = surface.UResolution(tol);
            double dv = surface.VResolution(tol);

            _faceRangeU[0] = surface.FirstUParameter() - du;
            _faceRangeU[1] = surface.LastUParameter() + du;

            _faceRangeV[0] = surface.FirstVParameter() - dv;
            _faceRangeV[1] = surface.LastVParameter() + dv;
        }

        // Kerf: an edge alone can be over the budget, as a damaged file's far too long one is (decision 1915)
        _parameters.Budget?.Check(_discretTool.PointCount);
        AddInternalVertices();
        SplitByDeflection2d();
    }

    /// <inheritdoc/>
    public override int PointsNb() => _discretTool.PointCount;

    // Check deflection in 2d space for improvement of edge tessellation.
    private void SplitByDeflection2d()
    {
        int nodesNb = _discretTool.PointCount;
        if (!_dEdge.IsFree && _dEdge.GetSameParam() && _dEdge.GetSameRange() && nodesNb > 1)
        {
            for (int pCurveIt = 0; pCurveIt < _dEdge.PCurvesNb(); ++pCurveIt)
            {
                PCurveModel pCurve = _dEdge.GetPCurve(pCurveIt);
                Face face = pCurve.GetFace().GetFace();
                Surface surface = Tool.Surface(face, out Placement _)!;
                if (surface.IsInstance(typeof(PlaneSurface)))
                {
                    continue;
                }

                Edge currEdge = ShapeCasts.Edge(_edge.Oriented(pCurve.GetOrientation()));

                Curve2D curve2d = Tool.CurveOnSurface(currEdge, face, out double _, out double _)!;
                NCollection_Array1<double> paramArray = new(1, nodesNb);
                for (int i = 1; i <= nodesNb; ++i)
                {
                    paramArray.SetValue(i, _discretTool.Parameter(i));
                }

                for (int i = 1; i < nodesNb; ++i)
                {
                    SplitSegment(surface, curve2d, paramArray[i], paramArray[i + 1], 1);
                }
            }
        }
    }

    // Adds internal vertices to discrete polygon.
    private void AddInternalVertices()
    {
        // PTv, chl/922/G9, Take into account internal vertices
        // it is necessary for internal edges, which do not split other edges, by their vertex
        Explorer vertexIt = new(_edge, ShapeType.Vertex);
        for (; vertexIt.More(); vertexIt.Next())
        {
            Vertex vertex = ShapeCasts.Vertex(vertexIt.Current());
            if (vertex.Orientation != Internal)
            {
                continue;
            }

            _discretTool.AddPoint(Tool.Pnt(vertex), Tool.Parameter(vertex, _edge), true);
        }
    }

    /// <inheritdoc/>
    /// <remarks>
    /// On a B-spline, Bezier or other non-periodic surface, a point whose UV lies outside the surface's range is
    /// valid only if the surface's point there is within the edge's tolerance.
    /// </remarks>
    public override bool Value(int index, out Point3D point, out double parameter)
    {
        point = _discretTool.Value(index);
        parameter = _discretTool.Parameter(index);

        if (!_curve.IsCurveOnSurface)
        {
            return true;
        }

        // If point coordinates are out of surface range,
        // it is necessary to re-project point.
        CurveOnSurface curve = _curve.CurveOnSurface();
        SurfaceAdaptorBase surface = curve.GetSurface()!;
        if (surface.GetType() != SurfaceKind.BSplineSurface && surface.GetType() != SurfaceKind.BezierSurface
            && surface.GetType() != OtherSurface)
        {
            return true;
        }

        // Let skip periodic case.
        if (surface.IsUPeriodic || surface.IsVPeriodic)
        {
            return true;
        }

        curve.GetCurve()!.D0(parameter, out Point2D uv);
        // Point lies within the surface range - nothing to do.
        if (uv.X > _faceRangeU[0] && uv.X < _faceRangeU[1] && uv.Y > _faceRangeV[0]
            && uv.Y < _faceRangeV[1])
        {
            return true;
        }

        surface.D0(uv.X, uv.Y, out Point3D pntOnSurf);

        return (point.SquareDistanceTo(pntOnSurf) < _edgeSqTol);
    }

    private void SplitSegment(Surface surf,
                              Curve2D curve2d,
                              double first,
                              double last,
                              int nbIter)
    {
        // limit iteration depth
        if (nbIter > 10)
        {
            return;
        }

        Point2D uvf, uvl, uvm;
        Point3D p3dF, p3dL, midP3dFromSurf;
        double midpar;

        if (Math.Abs(last - first) < 2 * global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return;
        }

        if ((curve2d.FirstParameter - first > global::Kerf.Engine.Foundation.Precision.PConfusion())
            || (last - curve2d.LastParameter > global::Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            // E.g. test bugs moddata_3 bug30133
            return;
        }

        curve2d.D0(first, out uvf);
        curve2d.D0(last, out uvl);

        p3dF = surf.Value(uvf.X, uvf.Y);
        p3dL = surf.Value(uvl.X, uvl.Y);

        if (p3dF.SquareDistanceTo(p3dL) < _squareMinSize)
        {
            return;
        }

        uvm = new Point2D((uvf.XY() + uvl.XY()) * 0.5);
        midP3dFromSurf = surf.Value(uvm.X, uvm.Y);

        Xyz vec1 = midP3dFromSurf.XYZ() - p3dF.XYZ();
        if (vec1.SquareModulus() < _squareMinSize)
        {
            return;
        }

        Xyz vec = p3dL.XYZ() - p3dF.XYZ();
        vec.Normalize();

        double modulus = vec1.Dot(vec);
        Xyz proj = vec * modulus;
        Xyz dist = vec1 - proj;

        if (dist.SquareModulus() < _squareEdgeDef)
        {
            return;
        }

        midpar = (first + last) * 0.5;
        _curve.D0(midpar, out Point3D midP3d);
        _discretTool.AddPoint(midP3d, midpar, false);
        _parameters.Budget?.Check(_discretTool.PointCount);
        if ((_discretTool.PointCount & 63) == 0)
        {
            Cancellation.ThrowIfRequested();
        }

        SplitSegment(surf, curve2d, first, midpar, nbIter + 1);
        SplitSegment(surf, curve2d, midpar, last, nbIter + 1);
    }
}
