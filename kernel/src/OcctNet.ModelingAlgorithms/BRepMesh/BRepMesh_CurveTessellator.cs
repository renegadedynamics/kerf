// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CurveTessellator.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CurveTessellator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Tessellates an edge with GCPnts_TangentialDeflection at half the discrete edge's deflections, adding the edge's
/// internal vertices and, on its non-planar faces, points where the pcurve's chords deviate from the surface.
/// </summary>
/// <remarks>
/// A working object of one loop body: it evaluates its own adaptor of the edge (and of the face), never the discrete
/// face's shared surface adaptor (decision 490's audit).
/// </remarks>
public class BRepMesh_CurveTessellator : IMeshTools_CurveTessellator
{
    private readonly IMeshData_Edge myDEdge;
    private readonly IMeshTools_Parameters myParameters; // OCCT holds a reference; the caller's doesn't change meanwhile
    private readonly TopoDS_Edge myEdge;
    private readonly BRepAdaptor_Curve myCurve;
    private readonly int myMinPointsNb;
    private readonly GCPnts_TangentialDeflection myDiscretTool = new();
    private TopoDS_Vertex myFirstVertex;
    private TopoDS_Vertex myLastVertex;
    private double mySquareEdgeDef;
    private double mySquareMinSize;
    private double myEdgeSqTol;
    private readonly double[] myFaceRangeU = new double[2];
    private readonly double[] myFaceRangeV = new double[2];

    /// <summary>A tessellator of the edge's 3D curve.</summary>
    /// <exception cref="FailureException">The parameters' MinSize isn't positive.</exception>
    public BRepMesh_CurveTessellator(IMeshData_Edge theEdge,
                                     in IMeshTools_Parameters theParameters,
                                     int theMinPointsNb = 2)
    {
        myDEdge = theEdge;
        myParameters = theParameters;
        myEdge = theEdge.GetEdge();
        myCurve = new BRepAdaptor_Curve(myEdge);
        myMinPointsNb = theMinPointsNb;
        init();
    }

    /// <summary>A tessellator of the edge, with the orientation, as a curve on the face.</summary>
    /// <exception cref="FailureException">The parameters' MinSize isn't positive.</exception>
    public BRepMesh_CurveTessellator(IMeshData_Edge theEdge,
                                     TopAbs_Orientation theOrientation,
                                     IMeshData_Face theFace,
                                     in IMeshTools_Parameters theParameters,
                                     int theMinPointsNb = 2)
    {
        myDEdge = theEdge;
        myParameters = theParameters;
        myEdge = TopoDS.Edge(theEdge.GetEdge().Oriented(theOrientation));
        myCurve = new BRepAdaptor_Curve(myEdge, theFace.GetFace());
        myMinPointsNb = theMinPointsNb;
        init();
    }

    // Performs initialization of this tool.
    private void init()
    {
        if (myParameters.MinSize <= 0.0)
        {
            throw new FailureException("The structure \"myParameters\" is not initialized");
        }

        TopExp.Vertices(myEdge, out myFirstVertex, out myLastVertex);

        double aPreciseAngDef = 0.5 * myDEdge.GetAngularDeflection();
        double aPreciseLinDef = 0.5 * myDEdge.GetDeflection();
        if (myEdge.Orientation() == TopAbs_INTERNAL)
        {
            aPreciseLinDef *= 0.5;
        }

        aPreciseLinDef = Max(aPreciseLinDef, global::Kerf.Engine.Foundation.Precision.Confusion());
        aPreciseAngDef = Max(aPreciseAngDef, global::Kerf.Engine.Foundation.Precision.Angular());

        double aMinSize = myParameters.MinSize;
        if (myParameters.AdjustMinSize)
        {
            aMinSize = Min(aMinSize,
                           IMeshTools_Parameters.RelMinSize()
                               * GCPnts_AbscissaPoint.Length(myCurve,
                                                             myCurve.FirstParameter(),
                                                             myCurve.LastParameter(),
                                                             aPreciseLinDef));
        }

        mySquareEdgeDef = aPreciseLinDef * aPreciseLinDef;
        mySquareMinSize = Max(mySquareEdgeDef, aMinSize * aMinSize);

        myEdgeSqTol = BRep_Tool.Tolerance(myEdge);
        myEdgeSqTol *= myEdgeSqTol;

        int aMinPntThreshold = 2;
        switch (myCurve.GetType())
        {
            case Circle:
            case Ellipse:
            case Parabola:
            case Hyperbola:
                aMinPntThreshold = 4;
                break;

            default:
                break;
        }

        int aMinPntNb = Max(myMinPointsNb, aMinPntThreshold); // OCC287
        myDiscretTool.Initialize(myCurve,
                                 myCurve.FirstParameter(),
                                 myCurve.LastParameter(),
                                 aPreciseAngDef,
                                 aPreciseLinDef,
                                 aMinPntNb,
                                 global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                 aMinSize);

        if (myCurve.IsCurveOnSurface())
        {
            Adaptor3d_CurveOnSurface aCurve = myCurve.CurveOnSurface();
            Adaptor3d_Surface aSurface = aCurve.GetSurface()!;

            double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
            double aDu = aSurface.UResolution(aTol);
            double aDv = aSurface.VResolution(aTol);

            myFaceRangeU[0] = aSurface.FirstUParameter() - aDu;
            myFaceRangeU[1] = aSurface.LastUParameter() + aDu;

            myFaceRangeV[0] = aSurface.FirstVParameter() - aDv;
            myFaceRangeV[1] = aSurface.LastVParameter() + aDv;
        }

        addInternalVertices();
        splitByDeflection2d();
    }

    /// <inheritdoc/>
    public override int PointsNb() => myDiscretTool.NbPoints();

    // Check deflection in 2d space for improvement of edge tessellation.
    private void splitByDeflection2d()
    {
        int aNodesNb = myDiscretTool.NbPoints();
        if (!myDEdge.IsFree() && myDEdge.GetSameParam() && myDEdge.GetSameRange() && aNodesNb > 1)
        {
            for (int aPCurveIt = 0; aPCurveIt < myDEdge.PCurvesNb(); ++aPCurveIt)
            {
                IMeshData_PCurve aPCurve = myDEdge.GetPCurve(aPCurveIt);
                TopoDS_Face aFace = aPCurve.GetFace().GetFace();
                Geom_Surface aSurface = BRep_Tool.Surface(aFace, out Placement _)!;
                if (aSurface.IsInstance(typeof(Geom_Plane)))
                {
                    continue;
                }

                TopoDS_Edge aCurrEdge = TopoDS.Edge(myEdge.Oriented(aPCurve.GetOrientation()));

                Geom2d_Curve aCurve2d = BRep_Tool.CurveOnSurface(aCurrEdge, aFace, out double _, out double _)!;
                NCollection_Array1<double> aParamArray = new(1, aNodesNb);
                for (int i = 1; i <= aNodesNb; ++i)
                {
                    aParamArray.SetValue(i, myDiscretTool.Parameter(i));
                }

                for (int i = 1; i < aNodesNb; ++i)
                {
                    splitSegment(aSurface, aCurve2d, aParamArray[i], aParamArray[i + 1], 1);
                }
            }
        }
    }

    // Adds internal vertices to discrete polygon.
    private void addInternalVertices()
    {
        // PTv, chl/922/G9, Take into account internal vertices
        // it is necessary for internal edges, which do not split other edges, by their vertex
        TopExp_Explorer aVertexIt = new(myEdge, TopAbs_VERTEX);
        for (; aVertexIt.More(); aVertexIt.Next())
        {
            TopoDS_Vertex aVertex = TopoDS.Vertex(aVertexIt.Current());
            if (aVertex.Orientation() != TopAbs_INTERNAL)
            {
                continue;
            }

            myDiscretTool.AddPoint(BRep_Tool.Pnt(aVertex), BRep_Tool.Parameter(aVertex, myEdge), true);
        }
    }

    // Checks whether the given point lies within tolerance of the vertex (unused, as in OCCT).
    private static bool isInToleranceOfVertex(in Point3D thePoint, in TopoDS_Vertex theVertex)
    {
        Point3D aPoint = BRep_Tool.Pnt(theVertex);
        double aTolerance = BRep_Tool.Tolerance(theVertex);

        return (thePoint.SquareDistance(aPoint) < aTolerance * aTolerance);
    }

    /// <inheritdoc/>
    /// <remarks>
    /// On a B-spline, Bezier or other non-periodic surface, a point whose UV lies outside the surface's range is
    /// valid only if the surface's point there is within the edge's tolerance.
    /// </remarks>
    public override bool Value(int theIndex, out Point3D thePoint, out double theParameter)
    {
        thePoint = myDiscretTool.Value(theIndex);
        theParameter = myDiscretTool.Parameter(theIndex);

        if (!myCurve.IsCurveOnSurface())
        {
            return true;
        }

        // If point coordinates are out of surface range,
        // it is necessary to re-project point.
        Adaptor3d_CurveOnSurface aCurve = myCurve.CurveOnSurface();
        Adaptor3d_Surface aSurface = aCurve.GetSurface()!;
        if (aSurface.GetType() != BSplineSurface && aSurface.GetType() != BezierSurface
            && aSurface.GetType() != OtherSurface)
        {
            return true;
        }

        // Let skip periodic case.
        if (aSurface.IsUPeriodic() || aSurface.IsVPeriodic())
        {
            return true;
        }

        aCurve.GetCurve()!.D0(theParameter, out Point2D aUV);
        // Point lies within the surface range - nothing to do.
        if (aUV.X() > myFaceRangeU[0] && aUV.X() < myFaceRangeU[1] && aUV.Y() > myFaceRangeV[0]
            && aUV.Y() < myFaceRangeV[1])
        {
            return true;
        }

        aSurface.D0(aUV.X(), aUV.Y(), out Point3D aPntOnSurf);

        return (thePoint.SquareDistance(aPntOnSurf) < myEdgeSqTol);
    }

    private void splitSegment(Geom_Surface theSurf,
                              Geom2d_Curve theCurve2d,
                              double theFirst,
                              double theLast,
                              int theNbIter)
    {
        // limit iteration depth
        if (theNbIter > 10)
        {
            return;
        }

        Point2D uvf, uvl, uvm;
        Point3D P3dF, P3dL, midP3dFromSurf;
        double midpar;

        if (Math.Abs(theLast - theFirst) < 2 * global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return;
        }

        if ((theCurve2d.FirstParameter() - theFirst > global::Kerf.Engine.Foundation.Precision.PConfusion())
            || (theLast - theCurve2d.LastParameter() > global::Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            // E.g. test bugs moddata_3 bug30133
            return;
        }

        theCurve2d.D0(theFirst, out uvf);
        theCurve2d.D0(theLast, out uvl);

        P3dF = theSurf.Value(uvf.X(), uvf.Y());
        P3dL = theSurf.Value(uvl.X(), uvl.Y());

        if (P3dF.SquareDistance(P3dL) < mySquareMinSize)
        {
            return;
        }

        uvm = new Point2D((uvf.XY() + uvl.XY()) * 0.5);
        midP3dFromSurf = theSurf.Value(uvm.X(), uvm.Y());

        Xyz Vec1 = midP3dFromSurf.XYZ() - P3dF.XYZ();
        if (Vec1.SquareModulus() < mySquareMinSize)
        {
            return;
        }

        Xyz aVec = P3dL.XYZ() - P3dF.XYZ();
        aVec.Normalize();

        double aModulus = Vec1.Dot(aVec);
        Xyz aProj = aVec * aModulus;
        Xyz aDist = Vec1 - aProj;

        if (aDist.SquareModulus() < mySquareEdgeDef)
        {
            return;
        }

        midpar = (theFirst + theLast) * 0.5;
        myCurve.D0(midpar, out Point3D midP3d);
        myDiscretTool.AddPoint(midP3d, midpar, false);

        splitSegment(theSurf, theCurve2d, theFirst, midpar, theNbIter + 1);
        splitSegment(theSurf, theCurve2d, midpar, theLast, theNbIter + 1);
    }
}
