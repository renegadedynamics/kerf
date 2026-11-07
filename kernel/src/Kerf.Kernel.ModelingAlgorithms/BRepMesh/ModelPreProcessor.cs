// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelPreProcessor.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelPreProcessor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Meshing.Model.StatusModel;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Pre-processes the discrete model: adds points to the seam edges of cones, marks the faces whose triangulation fits
/// the deflection <see cref="Reused"/> and the others <see cref="Outdated"/>, and removes the
/// outdated triangulations and polygons from the shape.
/// </summary>
/// <remarks>
/// <para>
/// Writes into the shape being meshed, as OCCT does: <see cref="ShapeTool.NullifyFace"/> and
/// <c>NullifyEdge</c> remove outdated meshes. The mesher runs on an unfrozen shape in place, or on a frozen one in a
/// working overlay (decision 489, unit 10H6); a reused triangulation is only read, so it may stay frozen.
/// </para>
/// <para>
/// The two face loops run in parallel when the parameters ask for it: each body writes only its face's status and
/// deflection, and the seam amplifier the discrete seam edge of its cone (its 3D curve and its two pcurves on that
/// face); the face's surface adaptor is evaluated by that face's body only. The cleaning loop is serial, as in OCCT.
/// </para>
/// </remarks>
internal class ModelPreProcessor : ModelAlgo
{
    /// <summary>A pre-processor.</summary>
    public ModelPreProcessor()
    {
    }

    /// <summary>Pre-processes the model; false without a model.</summary>
    protected override bool PerformInternal(ModelModel? model,
                                            in ToolParameters parameters,
                                            ProgressRange range)
    {
        _ = range;
        if (model is null)
        {
            return false;
        }

        int facesNb = model.FacesNb();
        bool isOneThread = !parameters.InParallel;
        OsdParallel.For(0, facesNb, new SeamEdgeAmplifier(model, parameters).Process, isOneThread);
        OsdParallel.For(0,
                         facesNb,
                         new TriangulationConsistency(model, parameters.AllowQualityDecrease).Process,
                         isOneThread);

        // Clean edges and faces from outdated polygons.
        HashSet<FaceModel> usedFaces = new(ReferenceEqualityComparer.Instance);
        for (int edgeIt = 0; edgeIt < model.EdgesNb(); ++edgeIt)
        {
            EdgeModel dEdge = model.GetEdge(edgeIt);
            if (dEdge.IsFree)
            {
                if (dEdge.IsSet(StatusModel.Outdated))
                {
                    Tool.Polygon3D(dEdge.GetEdge(), out Placement loc);
                    ShapeTool.NullifyEdge(dEdge.GetEdge(), loc);
                }

                continue;
            }

            for (int pCurveIt = 0; pCurveIt < dEdge.PCurvesNb(); ++pCurveIt)
            {
                // Find adjacent outdated face.
                FaceModel dFace = dEdge.GetPCurve(pCurveIt).GetFace();
                if (!usedFaces.Contains(dFace))
                {
                    usedFaces.Add(dFace);
                    if (dFace.IsSet(StatusModel.Outdated))
                    {
                        Triangulation? triangulation = Tool.Triangulation(dFace.GetFace(), out Placement loc);

                        // Clean all edges of oudated face.
                        for (int wireIt = 0; wireIt < dFace.WiresNb(); ++wireIt)
                        {
                            WireModel dWire = dFace.GetWire(wireIt);
                            for (int wireEdgeIt = 0; wireEdgeIt < dWire.EdgesNb(); ++wireEdgeIt)
                            {
                                EdgeModel tmpDEdge = dWire.GetEdge(wireEdgeIt);
                                ShapeTool.NullifyEdge(tmpDEdge.GetEdge(), triangulation, loc);
                            }
                        }

                        ShapeTool.NullifyFace(dFace.GetFace());
                    }
                }
            }
        }

        return true;
    }

    // Checks consistency of triangulation stored in topological face.
    private sealed class TriangulationConsistency
    {
        private readonly ModelModel _model;
        private readonly bool _allowQualityDecrease; // Flag used for consistency check

        public TriangulationConsistency(ModelModel model, bool allowQualityDecrease)
        {
            _model = model;
            _allowQualityDecrease = allowQualityDecrease;
        }

        // Main functor.
        public void Process(int faceIndex)
        {
            FaceModel dFace = _model.GetFace(faceIndex);
            if (dFace.IsSet(StatusModel.Outdated) || dFace.GetFace().IsNull)
            {
                return;
            }

            Triangulation? triangulation = Tool.Triangulation(dFace.GetFace(), out Placement _);

            if (triangulation is not null)
            {
                // If there is an info about initial parameters, use it due to deflection kept
                // by Poly_Triangulation is generally an estimation upon generated mesh and can
                // be either less or even greater than specified value.
                TriangulationParameters? sourceParams = triangulation.Parameters();
                double deflection = (sourceParams is not null && sourceParams.HasDeflection)
                                         ? sourceParams.Deflection()
                                         : triangulation.Deflection();

                bool isTriangulationConsistent = Deflection.IsConsistent(deflection,
                                                                                 dFace.GetDeflection(),
                                                                                 _allowQualityDecrease);

                if (isTriangulationConsistent)
                {
                    // #25080: check that indices of links forming triangles are in range.
                    Span<int> node = stackalloc int[3];
                    for (int i = 1; i <= triangulation.TriangleCount && isTriangulationConsistent; ++i)
                    {
                        Triangle1 triangle = triangulation.Triangle(i);

                        triangle.Get(out node[0], out node[1], out node[2]);
                        for (int j = 0; j < 3 && isTriangulationConsistent; ++j)
                        {
                            isTriangulationConsistent = (node[j] >= 1 && node[j] <= triangulation.NodeCount);
                        }
                    }
                }

                if (isTriangulationConsistent)
                {
                    dFace.SetStatus(StatusModel.Reused);
                    dFace.SetDeflection(triangulation.Deflection());
                }
                else
                {
                    dFace.SetStatus(StatusModel.Outdated);
                }
            }
        }
    }

    // Adds additional points to seam edges on specific surfaces.
    private sealed class SeamEdgeAmplifier
    {
        private readonly ModelModel _model;
        private readonly ToolParameters _parameters;

        public SeamEdgeAmplifier(ModelModel model, in ToolParameters parameters)
        {
            _model = model;
            _parameters = parameters;
        }

        // Main functor.
        public void Process(int faceIndex)
        {
            FaceModel dFace = _model.GetFace(faceIndex);
            if (dFace.GetSurface().GetType() != SurfaceKind.Cone || dFace.IsSet(StatusModel.Failure))
            {
                return;
            }

            WireModel dWire = dFace.GetWire(0);
            for (int edgeIdx = 0; edgeIdx < dWire.EdgesNb() - 1; ++edgeIdx)
            {
                EdgeModel dEdge = dWire.GetEdge(edgeIdx);

                if (dEdge.GetPCurve(dFace, Forward) != dEdge.GetPCurve(dFace, Reversed))
                {
                    if (dEdge.GetCurve().ParametersNb() == 2)
                    {
                        if (SplitEdge(dEdge, dFace, Math.Abs(GetConeStep(dFace))))
                        {
                            Triangulation? triangulation =
                                Tool.Triangulation(dFace.GetFace(), out Placement _);

                            if (triangulation is not null)
                            {
                                dFace.SetStatus(StatusModel.Outdated);
                            }
                        }
                    }
                    return;
                }
            }
        }

        // Returns step for splitting seam edge of a cone.
        private double GetConeStep(FaceModel dFace)
        {
            ConeRangeSplitter splitter = new();
            splitter.Reset(dFace, _parameters);

            WireModel dWire = dFace.GetWire(0);
            for (int edgeIt = 0; edgeIt < dWire.EdgesNb(); ++edgeIt)
            {
                EdgeModel dEdge = dWire.GetEdge(edgeIt);
                PCurveModel pCurve = dEdge.GetPCurve(dFace, dWire.GetEdgeOrientation(edgeIt));

                for (int pointIt = 0; pointIt < pCurve.ParametersNb(); ++pointIt)
                {
                    Point2D pnt2d = pCurve.GetPoint(pointIt);
                    splitter.AddPoint(pnt2d);
                }
            }

            (double first, double second) steps = splitter.GetSplitSteps(_parameters, out (int first, int second) _);
            return steps.second;
        }

        // Splits 3D and all pcurves accordingly using the specified step.
        private static bool SplitEdge(EdgeModel dEdge, FaceModel dFace, double du)
        {
            Edge e = dEdge.GetEdge();
            Face f = dFace.GetFace();

            Curve3D? hc = Tool.Curve(e, out double fParam, out double lParam);

            PCurveModel ipc1 = dEdge.GetPCurve(0);
            PCurveModel ipc2 = dEdge.GetPCurve(1);

            // Calculate the step by parameter of the curve.
            Point2D fPntOfIPC1 = ipc1.GetPoint(0);
            Point2D lPntOfIPC1 = ipc1.GetPoint(ipc1.ParametersNb() - 1);
            double mod = Math.Abs(fPntOfIPC1.Y - lPntOfIPC1.Y);

            if (mod < Constants.Resolution())
            {
                return false;
            }

            double dt = Math.Abs(lParam - fParam) / mod * du;

            if (!SplitCurve(hc!, dEdge.GetCurve(), dt))
            {
                return false;
            }

            // Define two pcurves of the seam-edge.
            e = ShapeCasts.Edge(e.Oriented(Forward));
            Curve2D? pc1 = Tool.CurveOnSurface(e, f, out double start, out double _);

            e = ShapeCasts.Edge(e.Oriented(Reversed));
            Curve2D? pc2 = Tool.CurveOnSurface(e, f, out double _, out double _);

            if (pc1 is null || pc2 is null)
            {
                return false;
            }

            // Select the correct pcurve of the seam-edge. Not at the curve's own first parameter, as OCCT has it: a line's is
            // far off, so one 2e-14 off vertical, as a STEP file's can be, is far from its edge, the pcurves swap, and the
            // seam's inner nodes land on its other side, which leaves a cone's face with no triangles (#572, decision 1930)
            Point2D fPntOfPC1 = pc1.Value(start);

            if (Math.Abs(lPntOfIPC1.X - fPntOfPC1.X) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                (pc1, pc2) = (pc2, pc1);
            }

            SplitCurve(pc1, ipc1, dt);
            SplitCurve(pc2, ipc2, dt);

            return true;
        }

        // Splits the given curve using the specified step (OCCT's template, instantiated for the 3D curve).
        private static bool SplitCurve(Curve3D geomCurve, CurveModel curve, double dt)
        {
            bool isUpdated = false;

            double firstParam = curve.GetParameter(0);
            double lastParam = curve.GetParameter(curve.ParametersNb() - 1);
            bool isReversed = firstParam > lastParam;

            for (int pointIdx = 1; ; ++pointIdx)
            {
                double currParam = firstParam + pointIdx * dt * (isReversed ? -1.0 : 1.0);
                if ((isReversed && (currParam - lastParam < global::Kerf.Engine.Foundation.Precision.PConfusion()))
                    || (!isReversed && !(currParam - lastParam < -global::Kerf.Engine.Foundation.Precision.PConfusion())))
                {
                    break;
                }

                curve.InsertPoint(curve.ParametersNb() - 1,
                                     geomCurve.Value(currParam),
                                     currParam);

                isUpdated = true;
            }

            return isUpdated;
        }

        // Splits the given curve using the specified step (OCCT's template, instantiated for a pcurve).
        private static bool SplitCurve(Curve2D geomCurve, PCurveModel curve, double dt)
        {
            bool isUpdated = false;

            double firstParam = curve.GetParameter(0);
            double lastParam = curve.GetParameter(curve.ParametersNb() - 1);
            bool isReversed = firstParam > lastParam;

            for (int pointIdx = 1; ; ++pointIdx)
            {
                double currParam = firstParam + pointIdx * dt * (isReversed ? -1.0 : 1.0);
                if ((isReversed && (currParam - lastParam < global::Kerf.Engine.Foundation.Precision.PConfusion()))
                    || (!isReversed && !(currParam - lastParam < -global::Kerf.Engine.Foundation.Precision.PConfusion())))
                {
                    break;
                }

                curve.InsertPoint(curve.ParametersNb() - 1,
                                     geomCurve.Value(currParam),
                                     currParam);

                isUpdated = true;
            }

            return isUpdated;
        }
    }
}
