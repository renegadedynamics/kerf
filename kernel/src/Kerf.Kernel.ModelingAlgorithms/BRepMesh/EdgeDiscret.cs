// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_EdgeDiscret.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_EdgeDiscret.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Meshing.Model.StatusModel;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Discretizes the model's edges: an edge whose polygon on a face's triangulation (or, for a free edge, whose 3D
/// polygon) fits the deflection is read back from it; any other is tessellated, and outdated polygons are flagged.
/// </summary>
/// <remarks>
/// <para>
/// The edges are processed in parallel when the parameters ask for it. Each loop body writes only its own discrete
/// edge (its flags, deflections, 3D curve and pcurves) and reads the shape, triangulations included; the tessellators
/// evaluate their own adaptors, never a discrete face's shared surface adaptor. So a parallel run gives the serial
/// run's model (decision 490).
/// </para>
/// <para>
/// An edge whose processing throws a failure (<see cref="ModelAlgo.IsCaughtFailure"/>) gets
/// <see cref="Failure"/>, as OCCT's <c>catch (Standard_Failure)</c>.
/// </para>
/// </remarks>
internal class EdgeDiscret : ModelAlgo
{
    private ModelModel? _model;
    private ToolParameters _parameters = new();

    /// <summary>An edge discretizer.</summary>
    public EdgeDiscret()
    {
    }

    /// <summary>A tessellator of the free edge's 3D curve.</summary>
    public static CurveTessellator CreateEdgeTessellator(EdgeModel dEdge,
                                                                    in ToolParameters parameters,
                                                                    int minPointsNb = 2)
    {
        return new MeshCurveTessellator(dEdge, parameters, minPointsNb);
    }

    /// <summary>
    /// A tessellator of the edge: of its 3D curve if it is SameParameter, otherwise of its curve on the face with the
    /// orientation.
    /// </summary>
    public static CurveTessellator CreateEdgeTessellator(EdgeModel dEdge,
                                                                    Orientation orientation,
                                                                    FaceModel dFace,
                                                                    in ToolParameters parameters,
                                                                    int minPointsNb = 2)
    {
        return dEdge.GetSameParam()
                   ? new MeshCurveTessellator(dEdge, parameters, minPointsNb)
                   : new MeshCurveTessellator(dEdge,
                                                   orientation,
                                                   dFace,
                                                   parameters,
                                                   minPointsNb);
    }

    /// <summary>An extractor of the edge's polygon on the face's triangulation.</summary>
    public static CurveTessellator CreateEdgeTessellationExtractor(EdgeModel dEdge,
                                                                              FaceModel dFace)
    {
        return new EdgeTessellationExtractor(dEdge, dFace);
    }

    /// <summary>
    /// Fills the discrete edge's 3D curve from the tessellator: all its points, the ends at the vertices, if
    /// <paramref name="updateEnds"/>; otherwise only the inner points, inserted before the last one.
    /// </summary>
    public static void Tessellate3d(EdgeModel dEdge,
                                    CurveTessellator tessellator,
                                    bool updateEnds)
    {
        // Create 3d polygon.
        CurveModel curve = dEdge.GetCurve();

        Edge edge = dEdge.GetEdge();
        Exploration.Vertices(edge, out Vertex firstVertex, out Vertex lastVertex);

        if (firstVertex.IsNull || lastVertex.IsNull)
        {
            return;
        }

        if (updateEnds)
        {
            tessellator.Value(1, out Point3D _, out double param);
            curve.AddPoint(Tool.Pnt(firstVertex), param);
        }

        if (!dEdge.GetDegenerated())
        {
            for (int i = 2; i < tessellator.PointsNb(); ++i)
            {
                if (!tessellator.Value(i, out Point3D point, out double param))
                {
                    continue;
                }

                if (updateEnds)
                {
                    curve.AddPoint(point, param);
                }
                else
                {
                    curve.InsertPoint(curve.ParametersNb() - 1, point, param);
                }
            }
        }

        if (updateEnds)
        {
            tessellator.Value(tessellator.PointsNb(), out Point3D _, out double param);
            curve.AddPoint(Tool.Pnt(lastVertex), param);
        }
    }

    /// <summary>
    /// Fills the discrete edge's pcurves from its 3D curve: all the points if <paramref name="updateEnds"/>,
    /// otherwise only the inner ones, inserted before the last one.
    /// </summary>
    public static void Tessellate2d(EdgeModel dEdge, bool updateEnds)
    {
        CurveModel curve = dEdge.GetCurve();
        for (int pCurveIt = 0; pCurveIt < dEdge.PCurvesNb(); ++pCurveIt)
        {
            PCurveModel pCurve = dEdge.GetPCurve(pCurveIt);
            FaceModel dFace = pCurve.GetFace();
            ParametersListArrayAdaptorModel<CurveModel> curveArray = new(curve);
            EdgeParameterProvider<ParametersListArrayAdaptorModel<CurveModel>> provider =
                new(dEdge, pCurve.GetOrientation(), dFace, curveArray);

            Curve2DAdaptorBase geomPCurve = provider.GetPCurve()!;

            int paramIdx, paramNb;
            if (updateEnds)
            {
                paramIdx = 0;
                paramNb = curve.ParametersNb();
            }
            else
            {
                paramIdx = 1;
                paramNb = curve.ParametersNb() - 1;
            }

            for (; paramIdx < paramNb; ++paramIdx)
            {
                double param = provider.Parameter(paramIdx, curve.GetPoint(paramIdx));

                geomPCurve.D0(param, out Point2D point2d);
                if (updateEnds)
                {
                    pCurve.AddPoint(point2d, param);
                }
                else
                {
                    pCurve.InsertPoint(pCurve.ParametersNb() - 1, point2d, param);
                }
            }
        }
    }

    /// <summary>Discretizes the model's edges; false without a model.</summary>
    protected override bool PerformInternal(ModelModel? model,
                                            in ToolParameters parameters,
                                            ProgressRange range)
    {
        _ = range;
        _model = model;
        _parameters = parameters;

        if (_model is null)
        {
            return false;
        }

        OsdParallel.For(0, _model.EdgesNb(), Process, !_parameters.InParallel);

        _model = null; // Do not hold link to model.
        return true;
    }

    // Checks existing discretization of the edge and updates data model.
    private void Process(int edgeIndex)
    {
        EdgeModel dEdge = _model!.GetEdge(edgeIndex);
        try
        {
            Deflection.ComputeDeflection(dEdge, _model.GetMaxSize(), _parameters);

            CurveTessellator edgeTessellator;
            if (!dEdge.IsFree)
            {
                // Iterate over pcurves and check deflection on corresponding face.
                double minDeflection = RealLast();
                int minPCurveIndex = -1;
                for (int pCurveIt = 0; pCurveIt < dEdge.PCurvesNb(); ++pCurveIt)
                {
                    PCurveModel pCurve = dEdge.GetPCurve(pCurveIt);
                    double tmpDeflection = CheckExistingPolygonAndUpdateStatus(dEdge, pCurve);
                    if (tmpDeflection < minDeflection)
                    {
                        // Identify pcurve with the smallest deflection in order to
                        // retrieve polygon that represents the most smooth discretization.
                        minDeflection = tmpDeflection;
                        minPCurveIndex = pCurveIt;
                    }

                    ShapeTool.CheckAndUpdateFlags(dEdge, pCurve);
                }

                if (minPCurveIndex != -1)
                {
                    dEdge.SetDeflection(minDeflection);
                    FaceModel dFace = dEdge.GetPCurve(minPCurveIndex).GetFace();
                    edgeTessellator = CreateEdgeTessellationExtractor(dEdge, dFace);
                }
                else
                {
                    PCurveModel pCurve = dEdge.GetPCurve(0);
                    FaceModel dFace = pCurve.GetFace();
                    edgeTessellator = CreateEdgeTessellator(dEdge,
                                                             pCurve.GetOrientation(),
                                                             dFace,
                                                             _parameters);
                }
            }
            else
            {
                Polygon3D? poly3D = Tool.Polygon3D(dEdge.GetEdge(), out Placement _);
                if (poly3D is not null)
                {
                    if (poly3D.HasParameters
                        && Deflection.IsConsistent(poly3D.Deflection(),
                                                            dEdge.GetDeflection(),
                                                            _parameters.AllowQualityDecrease))
                    {
                        // Edge already has suitable 3d polygon.
                        dEdge.SetStatus(StatusModel.Reused);
                        return;
                    }
                    else
                    {
                        dEdge.SetStatus(StatusModel.Outdated);
                    }
                }

                edgeTessellator = CreateEdgeTessellator(dEdge, _parameters);
            }

            Tessellate3d(dEdge, edgeTessellator, true);
            if (!dEdge.IsFree)
            {
                Tessellate2d(dEdge, true);
            }
        }
        catch (Exception exception) when (IsCaughtFailure(exception))
        {
            dEdge.SetStatus(StatusModel.Failure);
        }
    }

    // Checks existing polygon on triangulation does it fit edge deflection or not.
    // Returns deflection of polygon or RealLast () in case if edge has no polygon or it was dropped.
    private double CheckExistingPolygonAndUpdateStatus(EdgeModel dEdge, PCurveModel pCurve)
    {
        Edge edge = dEdge.GetEdge();
        Face face = pCurve.GetFace().GetFace();

        Triangulation? faceTriangulation = Tool.Triangulation(face, out Placement loc);

        double deflection = RealLast();
        if (faceTriangulation is null)
        {
            return deflection;
        }

        PolygonOnTriangulation? polygon = Tool.PolygonOnTriangulation(edge, faceTriangulation, loc);

        if (polygon is not null)
        {
            bool isConsistent = polygon.HasParameters
                                && Deflection.IsConsistent(polygon.Deflection(),
                                                                    dEdge.GetDeflection(),
                                                                    _parameters.AllowQualityDecrease);

            if (!isConsistent)
            {
                // Nullify edge data and mark discrete pcurve to
                // notify necessity to mesh the entire face.
                dEdge.SetStatus(StatusModel.Outdated);
            }
            else
            {
                deflection = polygon.Deflection();
            }
        }

        return deflection;
    }
}
