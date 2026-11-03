// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelPostProcessor.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelPostProcessor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.IMeshData_Status;

namespace Occt;

/// <summary>
/// Post-processes the discrete model: stores the edges' polygons on the faces' triangulations (and the free edges' 3D
/// polygons) into the shape, then each new triangulation's deflection and the parameters it was made with.
/// </summary>
/// <remarks>
/// <para>
/// Writes into the shape being meshed, as OCCT does, through <see cref="BRepMesh_ShapeTool"/>'s non-freezing attach
/// (decision 488); the polygons and triangulations freeze with the result when it is published. Faces that failed or
/// whose triangulation was reused (frozen, from the input) are skipped, so a reused triangulation is never written.
/// </para>
/// <para>
/// The edge loop is serial, as OCCT forces it ("data races on edges sharing the same TShape"). The face loop runs in
/// parallel when the parameters ask for it: each body updates only its face's triangulation, and shares one
/// immutable <see cref="TriangulationParameters"/>, as OCCT does.
/// </para>
/// </remarks>
public class BRepMesh_ModelPostProcessor : IMeshTools_ModelAlgo
{
    /// <summary>A post-processor.</summary>
    public BRepMesh_ModelPostProcessor()
    {
    }

    /// <summary>Post-processes the model; false without a model.</summary>
    protected override bool performInternal(IMeshData_Model? theModel,
                                            in IMeshTools_Parameters theParameters,
                                            ProgressRange theRange)
    {
        _ = theRange;
        if (theModel is null)
        {
            return false;
        }

        // TODO: Force single threaded solution due to data races on edges sharing the same TShape
        OsdParallel.For(0,
                         theModel.EdgesNb(),
                         new PolygonCommitter(theModel).Process,
                         true /*!theParameters.InParallel*/);

        // Estimate deflection here due to BRepLib::EstimateDeflection requires
        // existence of both Poly_Triangulation and Poly_PolygonOnTriangulation.
        OsdParallel.For(0,
                         theModel.FacesNb(),
                         new DeflectionEstimator(theModel, theParameters).Process,
                         !theParameters.InParallel);
        return true;
    }

    // Commits 3D polygons and polygons on triangulations for corresponding edges.
    private sealed class PolygonCommitter
    {
        private readonly IMeshData_Model myModel;

        public PolygonCommitter(IMeshData_Model theModel)
        {
            myModel = theModel;
        }

        // Main functor.
        public void Process(int theEdgeIndex)
        {
            IMeshData_Edge aDEdge = myModel.GetEdge(theEdgeIndex);
            if (aDEdge.GetCurve().ParametersNb() == 0)
            {
                return;
            }

            if (aDEdge.IsFree())
            {
                if (!aDEdge.IsSet(IMeshData_Reused))
                {
                    commitPolygon3D(aDEdge);
                }
            }
            else
            {
                commitPolygons(aDEdge);
            }
        }

        // Commits 3d polygon to topological edge
        private static void commitPolygon3D(IMeshData_Edge theDEdge)
        {
            IMeshData_Curve aCurve = theDEdge.GetCurve();

            NCollection_Array1<Point3D> aNodes = new(1, aCurve.ParametersNb());
            NCollection_Array1<double> aUVNodes = new(1, aCurve.ParametersNb());
            for (int i = 1; i <= aCurve.ParametersNb(); ++i)
            {
                aNodes[i] = aCurve.GetPoint(i - 1);
                aUVNodes[i] = aCurve.GetParameter(i - 1);
            }

            Polygon3D aPoly3D = new(aNodes, aUVNodes);
            aPoly3D.Deflection(theDEdge.GetDeflection());

            BRepMesh_ShapeTool.UpdateEdge(theDEdge.GetEdge(), aPoly3D);
        }

        // Commits all polygons on triangulations correspondent to the given edge.
        private static void commitPolygons(IMeshData_Edge theDEdge)
        {
            // Collect pcurves associated with the given edge on the specific surface.
            NCollection_IndexedDataMap<IMeshData_Face, NCollection_List<IMeshData_PCurve>> aMapOfPCurves =
                new(ReferenceEqualityComparer.Instance);
            for (int aPCurveIt = 0; aPCurveIt < theDEdge.PCurvesNb(); ++aPCurveIt)
            {
                IMeshData_PCurve aPCurve = theDEdge.GetPCurve(aPCurveIt);
                IMeshData_Face aDFacePtr = aPCurve.GetFace();
                IMeshData_Face aDFace = aDFacePtr;
                if (aDFace.IsSet(IMeshData_Failure) || aDFace.IsSet(IMeshData_Reused))
                {
                    continue;
                }

                if (!aMapOfPCurves.Contains(aDFacePtr))
                {
                    aMapOfPCurves.Add(aDFacePtr, new NCollection_List<IMeshData_PCurve>());
                }

                NCollection_List<IMeshData_PCurve> aPCurves = aMapOfPCurves.ChangeFromKey(aDFacePtr);
                aPCurves.Append(aPCurve);
            }

            // Commit polygons related to separate face.
            Edge aEdge = theDEdge.GetEdge();
            for (int aPolygonIt = 1; aPolygonIt <= aMapOfPCurves.Extent(); ++aPolygonIt)
            {
                Face aFace = aMapOfPCurves.FindKey(aPolygonIt).GetFace();

                Triangulation? aTriangulation = Tool.Triangulation(aFace, out Placement aLoc);

                if (aTriangulation is not null)
                {
                    NCollection_List<IMeshData_PCurve> aPCurves = aMapOfPCurves.FindFromIndex(aPolygonIt);
                    // OCCT: the first pcurve becomes the forward edge's polygon, the last the reversed edge's, whatever
                    // their orientations (occt-issues.md).
                    if (aPCurves.Size() == 2)
                    {
                        BRepMesh_ShapeTool.UpdateEdge(aEdge,
                                                      collectPolygon(aPCurves.First(), theDEdge.GetDeflection()),
                                                      collectPolygon(aPCurves.Last(), theDEdge.GetDeflection()),
                                                      aTriangulation,
                                                      aLoc);
                    }
                    else
                    {
                        BRepMesh_ShapeTool.UpdateEdge(aEdge,
                                                      collectPolygon(aPCurves.First(), theDEdge.GetDeflection()),
                                                      aTriangulation,
                                                      aLoc);
                    }
                }
            }
        }

        // Collects polygonal data for the given pcurve
        private static PolygonOnTriangulation collectPolygon(IMeshData_PCurve thePCurve, double theDeflection)
        {
            NCollection_Array1<int> aNodes = new(1, thePCurve.ParametersNb());
            NCollection_Array1<double> aParams = new(1, thePCurve.ParametersNb());
            for (int i = 1; i <= thePCurve.ParametersNb(); ++i)
            {
                aNodes[i] = thePCurve.GetIndex(i - 1);
                aParams[i] = thePCurve.GetParameter(i - 1);
            }

            PolygonOnTriangulation aPolygon = new(aNodes, aParams);

            aPolygon.Deflection(theDeflection);
            return aPolygon;
        }
    }

    // Estimates and updates deflection of triangulations for corresponding faces.
    private sealed class DeflectionEstimator
    {
        private readonly IMeshData_Model myModel;
        private readonly TriangulationParameters myParams;

        public DeflectionEstimator(IMeshData_Model theModel, in IMeshTools_Parameters theParams)
        {
            myModel = theModel;
            myParams = new TriangulationParameters(theParams.Deflection,
                                                        theParams.Angle,
                                                        theParams.MinSize);
        }

        // Main functor.
        public void Process(int theFaceIndex)
        {
            IMeshData_Face aDFace = myModel.GetFace(theFaceIndex);
            if (aDFace.IsSet(IMeshData_Failure) || aDFace.IsSet(IMeshData_Reused))
            {
                return;
            }

            BRepLib.UpdateDeflection(aDFace.GetFace());

            Triangulation? aTriangulation = Tool.Triangulation(aDFace.GetFace(), out Placement _);

            if (aTriangulation is not null)
            {
                aTriangulation.Parameters(myParams);
            }
        }
    }
}
