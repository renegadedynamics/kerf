// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_EdgeDiscret.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_EdgeDiscret.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.IMeshData_Status;

namespace Occt;

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
/// An edge whose processing throws a failure (<see cref="IMeshTools_ModelAlgo.IsCaughtFailure"/>) gets
/// <see cref="IMeshData_Failure"/>, as OCCT's <c>catch (Standard_Failure)</c>.
/// </para>
/// </remarks>
public class BRepMesh_EdgeDiscret : IMeshTools_ModelAlgo
{
    private IMeshData_Model? myModel;
    private IMeshTools_Parameters myParameters = new();

    /// <summary>An edge discretizer.</summary>
    public BRepMesh_EdgeDiscret()
    {
    }

    /// <summary>A tessellator of the free edge's 3D curve.</summary>
    public static IMeshTools_CurveTessellator CreateEdgeTessellator(IMeshData_Edge theDEdge,
                                                                    in IMeshTools_Parameters theParameters,
                                                                    int theMinPointsNb = 2)
    {
        return new BRepMesh_CurveTessellator(theDEdge, theParameters, theMinPointsNb);
    }

    /// <summary>
    /// A tessellator of the edge: of its 3D curve if it is SameParameter, otherwise of its curve on the face with the
    /// orientation.
    /// </summary>
    public static IMeshTools_CurveTessellator CreateEdgeTessellator(IMeshData_Edge theDEdge,
                                                                    TopAbs_Orientation theOrientation,
                                                                    IMeshData_Face theDFace,
                                                                    in IMeshTools_Parameters theParameters,
                                                                    int theMinPointsNb = 2)
    {
        return theDEdge.GetSameParam()
                   ? new BRepMesh_CurveTessellator(theDEdge, theParameters, theMinPointsNb)
                   : new BRepMesh_CurveTessellator(theDEdge,
                                                   theOrientation,
                                                   theDFace,
                                                   theParameters,
                                                   theMinPointsNb);
    }

    /// <summary>An extractor of the edge's polygon on the face's triangulation.</summary>
    public static IMeshTools_CurveTessellator CreateEdgeTessellationExtractor(IMeshData_Edge theDEdge,
                                                                              IMeshData_Face theDFace)
    {
        return new BRepMesh_EdgeTessellationExtractor(theDEdge, theDFace);
    }

    /// <summary>
    /// Fills the discrete edge's 3D curve from the tessellator: all its points, the ends at the vertices, if
    /// <paramref name="theUpdateEnds"/>; otherwise only the inner points, inserted before the last one.
    /// </summary>
    public static void Tessellate3d(IMeshData_Edge theDEdge,
                                    IMeshTools_CurveTessellator theTessellator,
                                    bool theUpdateEnds)
    {
        // Create 3d polygon.
        IMeshData_Curve aCurve = theDEdge.GetCurve();

        TopoDS_Edge aEdge = theDEdge.GetEdge();
        TopExp.Vertices(aEdge, out TopoDS_Vertex aFirstVertex, out TopoDS_Vertex aLastVertex);

        if (aFirstVertex.IsNull() || aLastVertex.IsNull())
        {
            return;
        }

        if (theUpdateEnds)
        {
            theTessellator.Value(1, out Point3D _, out double aParam);
            aCurve.AddPoint(BRep_Tool.Pnt(aFirstVertex), aParam);
        }

        if (!theDEdge.GetDegenerated())
        {
            for (int i = 2; i < theTessellator.PointsNb(); ++i)
            {
                if (!theTessellator.Value(i, out Point3D aPoint, out double aParam))
                {
                    continue;
                }

                if (theUpdateEnds)
                {
                    aCurve.AddPoint(aPoint, aParam);
                }
                else
                {
                    aCurve.InsertPoint(aCurve.ParametersNb() - 1, aPoint, aParam);
                }
            }
        }

        if (theUpdateEnds)
        {
            theTessellator.Value(theTessellator.PointsNb(), out Point3D _, out double aParam);
            aCurve.AddPoint(BRep_Tool.Pnt(aLastVertex), aParam);
        }
    }

    /// <summary>
    /// Fills the discrete edge's pcurves from its 3D curve: all the points if <paramref name="theUpdateEnds"/>,
    /// otherwise only the inner ones, inserted before the last one.
    /// </summary>
    public static void Tessellate2d(IMeshData_Edge theDEdge, bool theUpdateEnds)
    {
        IMeshData_Curve aCurve = theDEdge.GetCurve();
        for (int aPCurveIt = 0; aPCurveIt < theDEdge.PCurvesNb(); ++aPCurveIt)
        {
            IMeshData_PCurve aPCurve = theDEdge.GetPCurve(aPCurveIt);
            IMeshData_Face aDFace = aPCurve.GetFace();
            IMeshData_ParametersListArrayAdaptor<IMeshData_Curve> aCurveArray = new(aCurve);
            BRepMesh_EdgeParameterProvider<IMeshData_ParametersListArrayAdaptor<IMeshData_Curve>> aProvider =
                new(theDEdge, aPCurve.GetOrientation(), aDFace, aCurveArray);

            Adaptor2d_Curve2d aGeomPCurve = aProvider.GetPCurve()!;

            int aParamIdx, aParamNb;
            if (theUpdateEnds)
            {
                aParamIdx = 0;
                aParamNb = aCurve.ParametersNb();
            }
            else
            {
                aParamIdx = 1;
                aParamNb = aCurve.ParametersNb() - 1;
            }

            for (; aParamIdx < aParamNb; ++aParamIdx)
            {
                double aParam = aProvider.Parameter(aParamIdx, aCurve.GetPoint(aParamIdx));

                aGeomPCurve.D0(aParam, out Point2D aPoint2d);
                if (theUpdateEnds)
                {
                    aPCurve.AddPoint(aPoint2d, aParam);
                }
                else
                {
                    aPCurve.InsertPoint(aPCurve.ParametersNb() - 1, aPoint2d, aParam);
                }
            }
        }
    }

    /// <summary>Discretizes the model's edges; false without a model.</summary>
    protected override bool performInternal(IMeshData_Model? theModel,
                                            in IMeshTools_Parameters theParameters,
                                            ProgressRange theRange)
    {
        _ = theRange;
        myModel = theModel;
        myParameters = theParameters;

        if (myModel is null)
        {
            return false;
        }

        OsdParallel.For(0, myModel.EdgesNb(), process, !myParameters.InParallel);

        myModel = null; // Do not hold link to model.
        return true;
    }

    // Checks existing discretization of the edge and updates data model.
    private void process(int theEdgeIndex)
    {
        IMeshData_Edge aDEdge = myModel!.GetEdge(theEdgeIndex);
        try
        {
            BRepMesh_Deflection.ComputeDeflection(aDEdge, myModel.GetMaxSize(), myParameters);

            IMeshTools_CurveTessellator aEdgeTessellator;
            if (!aDEdge.IsFree())
            {
                // Iterate over pcurves and check deflection on corresponding face.
                double aMinDeflection = RealLast();
                int aMinPCurveIndex = -1;
                for (int aPCurveIt = 0; aPCurveIt < aDEdge.PCurvesNb(); ++aPCurveIt)
                {
                    IMeshData_PCurve aPCurve = aDEdge.GetPCurve(aPCurveIt);
                    double aTmpDeflection = checkExistingPolygonAndUpdateStatus(aDEdge, aPCurve);
                    if (aTmpDeflection < aMinDeflection)
                    {
                        // Identify pcurve with the smallest deflection in order to
                        // retrieve polygon that represents the most smooth discretization.
                        aMinDeflection = aTmpDeflection;
                        aMinPCurveIndex = aPCurveIt;
                    }

                    BRepMesh_ShapeTool.CheckAndUpdateFlags(aDEdge, aPCurve);
                }

                if (aMinPCurveIndex != -1)
                {
                    aDEdge.SetDeflection(aMinDeflection);
                    IMeshData_Face aDFace = aDEdge.GetPCurve(aMinPCurveIndex).GetFace();
                    aEdgeTessellator = CreateEdgeTessellationExtractor(aDEdge, aDFace);
                }
                else
                {
                    IMeshData_PCurve aPCurve = aDEdge.GetPCurve(0);
                    IMeshData_Face aDFace = aPCurve.GetFace();
                    aEdgeTessellator = CreateEdgeTessellator(aDEdge,
                                                             aPCurve.GetOrientation(),
                                                             aDFace,
                                                             myParameters);
                }
            }
            else
            {
                Polygon3D? aPoly3D = BRep_Tool.Polygon3D(aDEdge.GetEdge(), out Placement _);
                if (aPoly3D is not null)
                {
                    if (aPoly3D.HasParameters
                        && BRepMesh_Deflection.IsConsistent(aPoly3D.Deflection(),
                                                            aDEdge.GetDeflection(),
                                                            myParameters.AllowQualityDecrease))
                    {
                        // Edge already has suitable 3d polygon.
                        aDEdge.SetStatus(IMeshData_Reused);
                        return;
                    }
                    else
                    {
                        aDEdge.SetStatus(IMeshData_Outdated);
                    }
                }

                aEdgeTessellator = CreateEdgeTessellator(aDEdge, myParameters);
            }

            Tessellate3d(aDEdge, aEdgeTessellator, true);
            if (!aDEdge.IsFree())
            {
                Tessellate2d(aDEdge, true);
            }
        }
        catch (Exception anException) when (IsCaughtFailure(anException))
        {
            aDEdge.SetStatus(IMeshData_Failure);
        }
    }

    // Checks existing polygon on triangulation does it fit edge deflection or not.
    // Returns deflection of polygon or RealLast () in case if edge has no polygon or it was dropped.
    private double checkExistingPolygonAndUpdateStatus(IMeshData_Edge theDEdge, IMeshData_PCurve thePCurve)
    {
        TopoDS_Edge aEdge = theDEdge.GetEdge();
        TopoDS_Face aFace = thePCurve.GetFace().GetFace();

        Triangulation? aFaceTriangulation = BRep_Tool.Triangulation(aFace, out Placement aLoc);

        double aDeflection = RealLast();
        if (aFaceTriangulation is null)
        {
            return aDeflection;
        }

        PolygonOnTriangulation? aPolygon = BRep_Tool.PolygonOnTriangulation(aEdge, aFaceTriangulation, aLoc);

        if (aPolygon is not null)
        {
            bool isConsistent = aPolygon.HasParameters
                                && BRepMesh_Deflection.IsConsistent(aPolygon.Deflection(),
                                                                    theDEdge.GetDeflection(),
                                                                    myParameters.AllowQualityDecrease);

            if (!isConsistent)
            {
                // Nullify edge data and mark discrete pcurve to
                // notify necessity to mesh the entire face.
                theDEdge.SetStatus(IMeshData_Outdated);
            }
            else
            {
                aDeflection = aPolygon.Deflection();
            }
        }

        return aDeflection;
    }
}
