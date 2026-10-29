// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelPreProcessor.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelPreProcessor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.IMeshData_Status;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// Pre-processes the discrete model: adds points to the seam edges of cones, marks the faces whose triangulation fits
/// the deflection <see cref="IMeshData_Reused"/> and the others <see cref="IMeshData_Outdated"/>, and removes the
/// outdated triangulations and polygons from the shape.
/// </summary>
/// <remarks>
/// <para>
/// Writes into the shape being meshed, as OCCT does: <see cref="BRepMesh_ShapeTool.NullifyFace"/> and
/// <c>NullifyEdge</c> remove outdated meshes, which throws on a frozen face or edge. The mesher runs on an unfrozen
/// shape or an exact copy of a frozen one (decision 489); a reused triangulation is only read, so it may stay frozen.
/// </para>
/// <para>
/// The two face loops run in parallel when the parameters ask for it: each body writes only its face's status and
/// deflection, and the seam amplifier the discrete seam edge of its cone (its 3D curve and its two pcurves on that
/// face); the face's surface adaptor is evaluated by that face's body only. The cleaning loop is serial, as in OCCT.
/// </para>
/// </remarks>
public class BRepMesh_ModelPreProcessor : IMeshTools_ModelAlgo
{
    /// <summary>A pre-processor.</summary>
    public BRepMesh_ModelPreProcessor()
    {
    }

    /// <summary>Pre-processes the model; false without a model.</summary>
    protected override bool performInternal(IMeshData_Model? theModel,
                                            in IMeshTools_Parameters theParameters,
                                            Message_ProgressRange theRange)
    {
        _ = theRange;
        if (theModel is null)
        {
            return false;
        }

        int aFacesNb = theModel.FacesNb();
        bool isOneThread = !theParameters.InParallel;
        OSD_Parallel.For(0, aFacesNb, new SeamEdgeAmplifier(theModel, theParameters).Process, isOneThread);
        OSD_Parallel.For(0,
                         aFacesNb,
                         new TriangulationConsistency(theModel, theParameters.AllowQualityDecrease).Process,
                         isOneThread);

        // Clean edges and faces from outdated polygons.
        NCollection_Map<IMeshData_Face> aUsedFaces = new(ReferenceEqualityComparer.Instance);
        for (int aEdgeIt = 0; aEdgeIt < theModel.EdgesNb(); ++aEdgeIt)
        {
            IMeshData_Edge aDEdge = theModel.GetEdge(aEdgeIt);
            if (aDEdge.IsFree())
            {
                if (aDEdge.IsSet(IMeshData_Outdated))
                {
                    BRep_Tool.Polygon3D(aDEdge.GetEdge(), out TopLoc_Location aLoc);
                    BRepMesh_ShapeTool.NullifyEdge(aDEdge.GetEdge(), aLoc);
                }

                continue;
            }

            for (int aPCurveIt = 0; aPCurveIt < aDEdge.PCurvesNb(); ++aPCurveIt)
            {
                // Find adjacent outdated face.
                IMeshData_Face aDFace = aDEdge.GetPCurve(aPCurveIt).GetFace();
                if (!aUsedFaces.Contains(aDFace))
                {
                    aUsedFaces.Add(aDFace);
                    if (aDFace.IsSet(IMeshData_Outdated))
                    {
                        Poly_Triangulation? aTriangulation = BRep_Tool.Triangulation(aDFace.GetFace(), out TopLoc_Location aLoc);

                        // Clean all edges of oudated face.
                        for (int aWireIt = 0; aWireIt < aDFace.WiresNb(); ++aWireIt)
                        {
                            IMeshData_Wire aDWire = aDFace.GetWire(aWireIt);
                            for (int aWireEdgeIt = 0; aWireEdgeIt < aDWire.EdgesNb(); ++aWireEdgeIt)
                            {
                                IMeshData_Edge aTmpDEdge = aDWire.GetEdge(aWireEdgeIt);
                                BRepMesh_ShapeTool.NullifyEdge(aTmpDEdge.GetEdge(), aTriangulation, aLoc);
                            }
                        }

                        BRepMesh_ShapeTool.NullifyFace(aDFace.GetFace());
                    }
                }
            }
        }

        return true;
    }

    // Checks consistency of triangulation stored in topological face.
    private sealed class TriangulationConsistency
    {
        private readonly IMeshData_Model myModel;
        private readonly bool myAllowQualityDecrease; // Flag used for consistency check

        public TriangulationConsistency(IMeshData_Model theModel, bool theAllowQualityDecrease)
        {
            myModel = theModel;
            myAllowQualityDecrease = theAllowQualityDecrease;
        }

        // Main functor.
        public void Process(int theFaceIndex)
        {
            IMeshData_Face aDFace = myModel.GetFace(theFaceIndex);
            if (aDFace.IsSet(IMeshData_Outdated) || aDFace.GetFace().IsNull())
            {
                return;
            }

            Poly_Triangulation? aTriangulation = BRep_Tool.Triangulation(aDFace.GetFace(), out TopLoc_Location _);

            if (aTriangulation is not null)
            {
                // If there is an info about initial parameters, use it due to deflection kept
                // by Poly_Triangulation is generally an estimation upon generated mesh and can
                // be either less or even greater than specified value.
                Poly_TriangulationParameters? aSourceParams = aTriangulation.Parameters();
                double aDeflection = (aSourceParams is not null && aSourceParams.HasDeflection())
                                         ? aSourceParams.Deflection()
                                         : aTriangulation.Deflection();

                bool isTriangulationConsistent = BRepMesh_Deflection.IsConsistent(aDeflection,
                                                                                 aDFace.GetDeflection(),
                                                                                 myAllowQualityDecrease);

                if (isTriangulationConsistent)
                {
                    // #25080: check that indices of links forming triangles are in range.
                    Span<int> aNode = stackalloc int[3];
                    for (int i = 1; i <= aTriangulation.NbTriangles() && isTriangulationConsistent; ++i)
                    {
                        Poly_Triangle aTriangle = aTriangulation.Triangle(i);

                        aTriangle.Get(out aNode[0], out aNode[1], out aNode[2]);
                        for (int j = 0; j < 3 && isTriangulationConsistent; ++j)
                        {
                            isTriangulationConsistent = (aNode[j] >= 1 && aNode[j] <= aTriangulation.NbNodes());
                        }
                    }
                }

                if (isTriangulationConsistent)
                {
                    aDFace.SetStatus(IMeshData_Reused);
                    aDFace.SetDeflection(aTriangulation.Deflection());
                }
                else
                {
                    aDFace.SetStatus(IMeshData_Outdated);
                }
            }
        }
    }

    // Adds additional points to seam edges on specific surfaces.
    private sealed class SeamEdgeAmplifier
    {
        private readonly IMeshData_Model myModel;
        private readonly IMeshTools_Parameters myParameters;

        public SeamEdgeAmplifier(IMeshData_Model theModel, in IMeshTools_Parameters theParameters)
        {
            myModel = theModel;
            myParameters = theParameters;
        }

        // Main functor.
        public void Process(int theFaceIndex)
        {
            IMeshData_Face aDFace = myModel.GetFace(theFaceIndex);
            if (aDFace.GetSurface().GetType() != GeomAbs_Cone || aDFace.IsSet(IMeshData_Failure))
            {
                return;
            }

            IMeshData_Wire aDWire = aDFace.GetWire(0);
            for (int aEdgeIdx = 0; aEdgeIdx < aDWire.EdgesNb() - 1; ++aEdgeIdx)
            {
                IMeshData_Edge aDEdge = aDWire.GetEdge(aEdgeIdx);

                if (aDEdge.GetPCurve(aDFace, TopAbs_FORWARD) != aDEdge.GetPCurve(aDFace, TopAbs_REVERSED))
                {
                    if (aDEdge.GetCurve().ParametersNb() == 2)
                    {
                        if (splitEdge(aDEdge, aDFace, Math.Abs(getConeStep(aDFace))))
                        {
                            Poly_Triangulation? aTriangulation =
                                BRep_Tool.Triangulation(aDFace.GetFace(), out TopLoc_Location _);

                            if (aTriangulation is not null)
                            {
                                aDFace.SetStatus(IMeshData_Outdated);
                            }
                        }
                    }
                    return;
                }
            }
        }

        // Returns step for splitting seam edge of a cone.
        private double getConeStep(IMeshData_Face theDFace)
        {
            BRepMesh_ConeRangeSplitter aSplitter = new();
            aSplitter.Reset(theDFace, myParameters);

            IMeshData_Wire aDWire = theDFace.GetWire(0);
            for (int aEdgeIt = 0; aEdgeIt < aDWire.EdgesNb(); ++aEdgeIt)
            {
                IMeshData_Edge aDEdge = aDWire.GetEdge(aEdgeIt);
                IMeshData_PCurve aPCurve = aDEdge.GetPCurve(theDFace, aDWire.GetEdgeOrientation(aEdgeIt));

                for (int aPointIt = 0; aPointIt < aPCurve.ParametersNb(); ++aPointIt)
                {
                    gp_Pnt2d aPnt2d = aPCurve.GetPoint(aPointIt);
                    aSplitter.AddPoint(aPnt2d);
                }
            }

            (double first, double second) aSteps = aSplitter.GetSplitSteps(myParameters, out (int first, int second) _);
            return aSteps.second;
        }

        // Splits 3D and all pcurves accordingly using the specified step.
        private static bool splitEdge(IMeshData_Edge theDEdge, IMeshData_Face theDFace, double theDU)
        {
            TopoDS_Edge aE = theDEdge.GetEdge();
            TopoDS_Face aF = theDFace.GetFace();

            Geom_Curve? aHC = BRep_Tool.Curve(aE, out double aFParam, out double aLParam);

            IMeshData_PCurve aIPC1 = theDEdge.GetPCurve(0);
            IMeshData_PCurve aIPC2 = theDEdge.GetPCurve(1);

            // Calculate the step by parameter of the curve.
            gp_Pnt2d aFPntOfIPC1 = aIPC1.GetPoint(0);
            gp_Pnt2d aLPntOfIPC1 = aIPC1.GetPoint(aIPC1.ParametersNb() - 1);
            double aMod = Math.Abs(aFPntOfIPC1.Y() - aLPntOfIPC1.Y());

            if (aMod < gp.Resolution())
            {
                return false;
            }

            double aDT = Math.Abs(aLParam - aFParam) / aMod * theDU;

            if (!splitCurve(aHC!, theDEdge.GetCurve(), aDT))
            {
                return false;
            }

            // Define two pcurves of the seam-edge.
            aE = TopoDS.Edge(aE.Oriented(TopAbs_FORWARD));
            Geom2d_Curve? aPC1 = BRep_Tool.CurveOnSurface(aE, aF, out double _, out double _);

            aE = TopoDS.Edge(aE.Oriented(TopAbs_REVERSED));
            Geom2d_Curve? aPC2 = BRep_Tool.CurveOnSurface(aE, aF, out double _, out double _);

            if (aPC1 is null || aPC2 is null)
            {
                return false;
            }

            // Select the correct pcurve of the seam-edge.
            gp_Pnt2d aFPntOfPC1 = aPC1.Value(aPC1.FirstParameter());

            if (Math.Abs(aLPntOfIPC1.X() - aFPntOfPC1.X()) > Precision.Confusion())
            {
                (aPC1, aPC2) = (aPC2, aPC1);
            }

            splitCurve(aPC1, aIPC1, aDT);
            splitCurve(aPC2, aIPC2, aDT);

            return true;
        }

        // Splits the given curve using the specified step (OCCT's template, instantiated for the 3D curve).
        private static bool splitCurve(Geom_Curve theGeomCurve, IMeshData_Curve theCurve, double theDT)
        {
            bool isUpdated = false;

            double aFirstParam = theCurve.GetParameter(0);
            double aLastParam = theCurve.GetParameter(theCurve.ParametersNb() - 1);
            bool isReversed = aFirstParam > aLastParam;

            for (int aPointIdx = 1; ; ++aPointIdx)
            {
                double aCurrParam = aFirstParam + aPointIdx * theDT * (isReversed ? -1.0 : 1.0);
                if ((isReversed && (aCurrParam - aLastParam < Precision.PConfusion()))
                    || (!isReversed && !(aCurrParam - aLastParam < -Precision.PConfusion())))
                {
                    break;
                }

                theCurve.InsertPoint(theCurve.ParametersNb() - 1,
                                     theGeomCurve.Value(aCurrParam),
                                     aCurrParam);

                isUpdated = true;
            }

            return isUpdated;
        }

        // Splits the given curve using the specified step (OCCT's template, instantiated for a pcurve).
        private static bool splitCurve(Geom2d_Curve theGeomCurve, IMeshData_PCurve theCurve, double theDT)
        {
            bool isUpdated = false;

            double aFirstParam = theCurve.GetParameter(0);
            double aLastParam = theCurve.GetParameter(theCurve.ParametersNb() - 1);
            bool isReversed = aFirstParam > aLastParam;

            for (int aPointIdx = 1; ; ++aPointIdx)
            {
                double aCurrParam = aFirstParam + aPointIdx * theDT * (isReversed ? -1.0 : 1.0);
                if ((isReversed && (aCurrParam - aLastParam < Precision.PConfusion()))
                    || (!isReversed && !(aCurrParam - aLastParam < -Precision.PConfusion())))
                {
                    break;
                }

                theCurve.InsertPoint(theCurve.ParametersNb() - 1,
                                     theGeomCurve.Value(aCurrParam),
                                     aCurrParam);

                isUpdated = true;
            }

            return isUpdated;
        }
    }
}
