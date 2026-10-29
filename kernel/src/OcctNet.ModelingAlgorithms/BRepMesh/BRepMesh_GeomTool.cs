// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_GeomTool.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_GeomTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.CSLib_DerivativeStatus;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_IsoType;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The mesher's geometric tools: the discretization points of a curve or of an iso curve of a surface for given
/// deflections (through <see cref="GCPnts_TangentialDeflection"/>), and the statics the mesher shares (normals,
/// segment intersections, the deflection of a segment, the cell counts of the circles' grid).
/// </summary>
/// <remarks>A working object, as OCCT's: <see cref="AddPoint"/> changes the discretization in place.</remarks>
public sealed class BRepMesh_GeomTool
{
    /// <summary>The states of the intersection check of two segments.</summary>
    public enum IntFlag
    {
        NoIntersection,
        Cross,
        EndPointTouch,
        PointOnSegment,
        Glued,
        Same,
    }

    private readonly TopoDS_Edge? myEdge;
    private readonly GCPnts_TangentialDeflection myDiscretTool = new();
    private readonly GeomAbs_IsoType myIsoType;

    /// <summary>
    /// Discretizes the curve between the parameters with the linear and angular deflections, in at least
    /// <paramref name="theMinPointsNb"/> points, segments not shorter than <paramref name="theMinSize"/>.
    /// </summary>
    public BRepMesh_GeomTool(BRepAdaptor_Curve theCurve,
                             double theFirstParam,
                             double theLastParam,
                             double theLinDeflection,
                             double theAngDeflection,
                             int theMinPointsNb = 2,
                             double theMinSize = 1.0e-7 /* Precision::Confusion() */)
    {
        myEdge = theCurve.Edge();
        myIsoType = GeomAbs_NoneIso;
        myDiscretTool.Initialize(theCurve,
                                 theFirstParam,
                                 theLastParam,
                                 theAngDeflection,
                                 theLinDeflection,
                                 theMinPointsNb,
                                 Precision.PConfusion(),
                                 theMinSize);
    }

    /// <summary>
    /// Discretizes the iso curve of the surface (U or V iso at <paramref name="theParamIso"/>) between the parameters
    /// with the linear and angular deflections.
    /// </summary>
    public BRepMesh_GeomTool(BRepAdaptor_Surface theSurface,
                             GeomAbs_IsoType theIsoType,
                             double theParamIso,
                             double theFirstParam,
                             double theLastParam,
                             double theLinDeflection,
                             double theAngDeflection,
                             int theMinPointsNb = 2,
                             double theMinSize = 1.0e-7 /* Precision::Confusion() */)
    {
        myEdge = null;
        myIsoType = theIsoType;
        Adaptor3d_IsoCurve aIso = new(theSurface, theIsoType, theParamIso, theFirstParam, theLastParam);

        myDiscretTool.Initialize(aIso,
                                 theFirstParam,
                                 theLastParam,
                                 theAngDeflection,
                                 theLinDeflection,
                                 theMinPointsNb,
                                 Precision.PConfusion(),
                                 theMinSize);
    }

    /// <summary>
    /// Adds a point to the computed points, or replaces one within the parametric tolerance if
    /// <paramref name="theIsReplace"/>; returns its index.
    /// </summary>
    public int AddPoint(in gp_Pnt thePoint, double theParam, bool theIsReplace = true)
    {
        return myDiscretTool.AddPoint(thePoint, theParam, theIsReplace);
    }

    /// <summary>The number of discretization points.</summary>
    public int NbPoints() => myDiscretTool.NbPoints();

    /// <summary>
    /// The discretization point with the given index (from 1) on the iso curve: its parameter, the 3D point and the
    /// point in the surface's parametric space, with <paramref name="theIsoParam"/> as the other coordinate.
    /// </summary>
    public bool Value(int theIndex, double theIsoParam, out double theParam, out gp_Pnt thePoint, out gp_Pnt2d theUV)
    {
        theParam = 0.0;
        thePoint = new gp_Pnt();
        theUV = new gp_Pnt2d();
        if (theIndex < 1 || theIndex > NbPoints())
        {
            return false;
        }

        thePoint = myDiscretTool.Value(theIndex);
        theParam = myDiscretTool.Parameter(theIndex);

        if (myIsoType == GeomAbs_IsoU)
        {
            theUV.SetCoord(theIsoParam, theParam);
        }
        else
        {
            theUV.SetCoord(theParam, theIsoParam);
        }

        return true;
    }

    /// <summary>
    /// The discretization point with the given index (from 1) on the edge: its parameter, the 3D point and the point
    /// of the edge's pcurve on the surface's face. False for an iso curve.
    /// </summary>
    public bool Value(int theIndex, BRepAdaptor_Surface theSurface, out double theParam, out gp_Pnt thePoint,
                      out gp_Pnt2d theUV)
    {
        theParam = 0.0;
        thePoint = new gp_Pnt();
        theUV = new gp_Pnt2d();
        if (theIndex < 1 || theIndex > NbPoints())
        {
            return false;
        }

        if (myEdge is null)
        {
            return false;
        }

        thePoint = myDiscretTool.Value(theIndex);
        theParam = myDiscretTool.Parameter(theIndex);

        TopoDS_Face aFace = theSurface.Face();

        Geom2d_Curve? aCurve = BRep_Tool.CurveOnSurface(myEdge.Value, aFace, out _, out _);

        aCurve!.D0(theParam, out theUV);
        return true;
    }

    // static API

    /// <summary>
    /// The point and the normal of the surface at (U, V), reversed on a reversed face; false if the normal can't be
    /// computed.
    /// </summary>
    public static bool Normal(BRepAdaptor_Surface theSurface,
                              double theParamU,
                              double theParamV,
                              out gp_Pnt thePoint,
                              out gp_Dir theNormal)
    {
        bool isOK = true;
        theNormal = new gp_Dir();
        theSurface.D1(theParamU, theParamV, out thePoint, out gp_Vec aD1U, out gp_Vec aD1V);

        CSLib.Normal(aD1U, aD1V, Precision.Angular(), out CSLib_DerivativeStatus aStatus, ref theNormal);
        if (aStatus != CSLib_Done)
        {
            theSurface.D2(theParamU, theParamV, out thePoint, out aD1U, out aD1V, out gp_Vec aD2U, out gp_Vec aD2V,
                          out gp_Vec aD2UV);
            CSLib.Normal(aD1U,
                         aD1V,
                         aD2U,
                         aD2V,
                         aD2UV,
                         Precision.Angular(),
                         out isOK,
                         out CSLib_NormalStatus _,
                         ref theNormal);
        }

        if (!isOK)
        {
            return false;
        }

        TopoDS_Face aFace = theSurface.Face();
        TopAbs_Orientation aOri = aFace.Orientation();
        if (aOri == TopAbs_REVERSED)
        {
            theNormal.Reverse();
        }

        return true;
    }

    /// <summary>
    /// The intersection of the two lines through the segments' points: <see cref="IntFlag.Cross"/> with the point and
    /// its parameters on both segments (0 at the start point, 1 at the end point), <see cref="IntFlag.Same"/> for one
    /// line, or <see cref="IntFlag.NoIntersection"/> for parallel lines.
    /// </summary>
    /// <param name="theStartPnt1">The start point of the first segment.</param>
    /// <param name="theEndPnt1">The end point of the first segment.</param>
    /// <param name="theStartPnt2">The start point of the second segment.</param>
    /// <param name="theEndPnt2">The end point of the second segment.</param>
    /// <param name="theIntPnt">The intersection point, written on a cross.</param>
    /// <param name="theParamOnSegment">Two values (OCCT's <c>double (&amp;)[2]</c>), written on a cross.</param>
    public static IntFlag IntLinLin(in gp_XY theStartPnt1,
                                    in gp_XY theEndPnt1,
                                    in gp_XY theStartPnt2,
                                    in gp_XY theEndPnt2,
                                    ref gp_XY theIntPnt,
                                    Span<double> theParamOnSegment)
    {
        gp_XY aVec1 = theEndPnt1 - theStartPnt1;
        gp_XY aVec2 = theEndPnt2 - theStartPnt2;
        gp_XY aVecO1O2 = theStartPnt2 - theStartPnt1;

        double aCrossD1D2 = aVec1 ^ aVec2;
        double aCrossD1D3 = aVecO1O2 ^ aVec2;

        double aPrec = gp.Resolution();
        // Are edgegs codirectional
        if (Math.Abs(aCrossD1D2) < aPrec)
        {
            // Just a parallel case?
            if (Math.Abs(aCrossD1D3) < aPrec)
            {
                return IntFlag.Same;
            }
            else
            {
                return IntFlag.NoIntersection;
            }
        }

        theParamOnSegment[0] = aCrossD1D3 / aCrossD1D2;
        theIntPnt = theStartPnt1 + theParamOnSegment[0] * aVec1;

        double aCrossD2D3 = aVecO1O2 ^ aVec1;
        theParamOnSegment[1] = aCrossD2D3 / aCrossD1D2;
        return IntFlag.Cross;
    }

    /// <summary>
    /// The intersection of the two segments, within both. A touch at end points is
    /// <see cref="IntFlag.EndPointTouch"/> if <paramref name="isConsiderEndPointTouch"/> (else no intersection); an end
    /// point on the other segment is <see cref="IntFlag.PointOnSegment"/> (with the point) if
    /// <paramref name="isConsiderPointOnSegment"/>; <see cref="IntFlag.Cross"/> writes the intersection point.
    /// </summary>
    public static IntFlag IntSegSeg(in gp_XY theStartPnt1,
                                    in gp_XY theEndPnt1,
                                    in gp_XY theStartPnt2,
                                    in gp_XY theEndPnt2,
                                    bool isConsiderEndPointTouch,
                                    bool isConsiderPointOnSegment,
                                    ref gp_Pnt2d theIntPnt)
    {
        Span<int> aPointHash = [classifyPoint(theStartPnt1, theEndPnt1, theStartPnt2),
                                classifyPoint(theStartPnt1, theEndPnt1, theEndPnt2),
                                classifyPoint(theStartPnt2, theEndPnt2, theStartPnt1),
                                classifyPoint(theStartPnt2, theEndPnt2, theEndPnt1)];

        int aPosHash = aPointHash[0] + aPointHash[1] + aPointHash[2] + aPointHash[3];

        // Consider case when edges have shared vertex
        if (aPointHash[0] < 0 || aPointHash[1] < 0)
        {
            if (aPosHash == -1)
            {
                // -1 means, that 2 points are equal, and 1 point is on another curve
                return IntFlag.Glued;
            }
            else
            {
                if (isConsiderEndPointTouch)
                {
                    return IntFlag.EndPointTouch;
                }

                return IntFlag.NoIntersection;
            }
        }

        // 1) hash code == 1: an end point of one segment on the other.
        // 2) hash code == 2: overlapping segments.
        if (aPosHash == 1)
        {
            if (isConsiderPointOnSegment)
            {
                if (aPointHash[0] == 1)
                {
                    theIntPnt = new gp_Pnt2d(theStartPnt1);
                }
                else if (aPointHash[1] == 1)
                {
                    theIntPnt = new gp_Pnt2d(theEndPnt1);
                }
                else if (aPointHash[2] == 1)
                {
                    theIntPnt = new gp_Pnt2d(theStartPnt2);
                }
                else
                {
                    theIntPnt = new gp_Pnt2d(theEndPnt2);
                }

                return IntFlag.PointOnSegment;
            }

            return IntFlag.NoIntersection;
        }
        else if (aPosHash == 2)
        {
            return IntFlag.Glued;
        }

        Span<double> aParam = stackalloc double[2];
        IntFlag aIntFlag =
            IntLinLin(theStartPnt1, theEndPnt1, theStartPnt2, theEndPnt2, ref theIntPnt.ChangeCoord(), aParam);
        if (aIntFlag == IntFlag.NoIntersection)
        {
            return IntFlag.NoIntersection;
        }

        if (aIntFlag == IntFlag.Same)
        {
            if (aPosHash < -2)
            {
                return IntFlag.Same;
            }
            else if (aPosHash == -1)
            {
                return IntFlag.Glued;
            }

            return IntFlag.NoIntersection;
        }

        // Cross
        // Intersection is out of segments ranges
        double aPrec = Precision.PConfusion();
        double aEndPrec = 1 - aPrec;
        for (int i = 0; i < 2; ++i)
        {
            if (aParam[i] < aPrec || aParam[i] > aEndPrec)
            {
                return IntFlag.NoIntersection;
            }
        }

        return IntFlag.Cross;
    }

    /// <summary>
    /// The square of the deflection of the segment from its middle point: the distance from the line through the
    /// end points, or from the first point if they coincide.
    /// </summary>
    public static double SquareDeflectionOfSegment(in gp_Pnt theFirstPoint, in gp_Pnt theLastPoint, in gp_Pnt theMidPoint)
    {
        // 23.03.2010 skl for OCC21645 - change precision for comparison
        if (theFirstPoint.SquareDistance(theLastPoint) > Precision.SquareConfusion())
        {
            gp_Lin aLin = new(theFirstPoint, new gp_Dir(new gp_Vec(theFirstPoint, theLastPoint)));
            return aLin.SquareDistance(theMidPoint);
        }

        return theFirstPoint.SquareDistance(theMidPoint);
    }

    /// <summary>
    /// The cell counts along U and V of the circles' grid, estimated from the face's deflection and complexity:
    /// <c>2 ^ log10(estimated points count)</c> along each parametric direction, and
    /// <c>2 ^ log10(initial vertex count)</c> along a linear one. (-1, -1) without a range splitter.
    /// </summary>
    public static (int, int) CellsCount(Adaptor3d_Surface theSurface,
                                        int theVerticesNb,
                                        double theDeflection,
                                        BRepMesh_DefaultRangeSplitter? theRangeSplitter)
    {
        if (theRangeSplitter is null)
        {
            return (-1, -1);
        }

        GeomAbs_SurfaceType aType = theSurface.GetType();

        ComputeErrFactors(theDeflection, theSurface, out double anErrFactorU, out double anErrFactorV);

        (double first, double second) aRangeU = theRangeSplitter.GetRangeU();
        (double first, double second) aRangeV = theRangeSplitter.GetRangeV();
        (double first, double second) aDelta = theRangeSplitter.GetDelta();

        int aCellsCountU, aCellsCountV;
        if (aType == GeomAbs_Torus)
        {
            aCellsCountU =
                (int)Math.Ceiling(Math.Pow(2, Math.Log10((aRangeU.second - aRangeU.first) / aDelta.first)));
            aCellsCountV =
                (int)Math.Ceiling(Math.Pow(2, Math.Log10((aRangeV.second - aRangeV.first) / aDelta.second)));
        }
        else if (aType == GeomAbs_Cylinder)
        {
            aCellsCountU =
                (int)Math.Ceiling(Math.Pow(2,
                                           Math.Log10((aRangeU.second - aRangeU.first) / aDelta.first
                                                      / (aRangeV.second - aRangeV.first))));
            aCellsCountV =
                (int)Math.Ceiling(Math.Pow(2, Math.Log10((aRangeV.second - aRangeV.first) / anErrFactorV)));
        }
        else
        {
            aCellsCountU = (int)Math.Ceiling(
                Math.Pow(2, Math.Log10((aRangeU.second - aRangeU.first) / aDelta.first / anErrFactorU)));
            aCellsCountV = (int)Math.Ceiling(
                Math.Pow(2, Math.Log10((aRangeV.second - aRangeV.first) / aDelta.second / anErrFactorV)));
        }

        AdjustCellsCounts(theSurface, theVerticesNb, ref aCellsCountU, ref aCellsCountV);
        return (aCellsCountU, aCellsCountV);
    }

    // Classifies the point in case of coincidence of two vectors: zero if the point is out of the segment, -1 if it
    // coincides with an end point, 1 if it is between them.
    private static int classifyPoint(in gp_XY thePoint1, in gp_XY thePoint2, in gp_XY thePointToCheck)
    {
        gp_XY aP1 = thePoint2 - thePoint1;
        gp_XY aP2 = thePointToCheck - thePoint1;

        double aPrec = Precision.PConfusion();
        double aSqPrec = aPrec * aPrec;
        double aDist = Math.Abs(aP1 ^ aP2);
        if (aDist > aPrec)
        {
            aDist = (aDist * aDist) / aP1.SquareModulus();
            if (aDist > aSqPrec)
            {
                return 0; // out
            }
        }

        gp_XY aMult = aP1.Multiplied(aP2);
        if (aMult.X() < 0.0 || aMult.Y() < 0.0)
        {
            return 0; // out
        }

        if (aP1.SquareModulus() < aP2.SquareModulus())
        {
            return 0; // out
        }

        if (thePointToCheck.IsEqual(thePoint1, aPrec) || thePointToCheck.IsEqual(thePoint2, aPrec))
        {
            return -1; // coincides with an end point
        }

        return 1;
    }

    private static void ComputeErrFactors(double theDeflection,
                                          Adaptor3d_Surface theFace,
                                          out double theErrFactorU,
                                          out double theErrFactorV)
    {
        theErrFactorU = theDeflection * 10.0;
        theErrFactorV = theDeflection * 10.0;

        switch (theFace.GetType())
        {
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
            case GeomAbs_Torus:
                break;

            case GeomAbs_SurfaceOfExtrusion:
            case GeomAbs_SurfaceOfRevolution:
            {
                Adaptor3d_Curve aCurve = theFace.BasisCurve();
                if (aCurve.GetType() == GeomAbs_BSplineCurve && aCurve.Degree() > 2)
                {
                    theErrFactorV /= (aCurve.Degree() * aCurve.NbKnots());
                }
                break;
            }
            case GeomAbs_BezierSurface:
            {
                if (theFace.UDegree() > 2)
                {
                    theErrFactorU /= (theFace.UDegree());
                }
                if (theFace.VDegree() > 2)
                {
                    theErrFactorV /= (theFace.VDegree());
                }
                break;
            }
            case GeomAbs_BSplineSurface:
            {
                if (theFace.UDegree() > 2)
                {
                    theErrFactorU /= (theFace.UDegree() * theFace.NbUKnots());
                }
                if (theFace.VDegree() > 2)
                {
                    theErrFactorV /= (theFace.VDegree() * theFace.NbVKnots());
                }
                break;
            }

            case GeomAbs_Plane:
            default:
                theErrFactorU = theErrFactorV = 1.0;
                break;
        }
    }

    private static void AdjustCellsCounts(Adaptor3d_Surface theFace,
                                          int theNbVertices,
                                          ref int theCellsCountU,
                                          ref int theCellsCountV)
    {
        GeomAbs_SurfaceType aType = theFace.GetType();
        if (aType == GeomAbs_OtherSurface)
        {
            // fallback to the default behavior
            theCellsCountU = theCellsCountV = -1;
            return;
        }

        double aSqNbVert = theNbVertices;
        if (aType == GeomAbs_Plane)
        {
            theCellsCountU = theCellsCountV = (int)Math.Ceiling(Math.Pow(2, Math.Log10(aSqNbVert)));
        }
        else if (aType == GeomAbs_Cylinder || aType == GeomAbs_Cone)
        {
            theCellsCountV = (int)Math.Ceiling(Math.Pow(2, Math.Log10(aSqNbVert)));
        }
        else if (aType == GeomAbs_SurfaceOfExtrusion || aType == GeomAbs_SurfaceOfRevolution)
        {
            Adaptor3d_Curve aCurve = theFace.BasisCurve();
            if (aCurve.GetType() == GeomAbs_Line
                || (aCurve.GetType() == GeomAbs_BSplineCurve && aCurve.Degree() < 2))
            {
                // planar, cylindrical, conical cases
                if (aType == GeomAbs_SurfaceOfExtrusion)
                {
                    theCellsCountU = (int)Math.Ceiling(Math.Pow(2, Math.Log10(aSqNbVert)));
                }
                else
                {
                    theCellsCountV = (int)Math.Ceiling(Math.Pow(2, Math.Log10(aSqNbVert)));
                }
            }
            if (aType == GeomAbs_SurfaceOfExtrusion)
            {
                // V is always a line
                theCellsCountV = (int)Math.Ceiling(Math.Pow(2, Math.Log10(aSqNbVert)));
            }
        }
        else if (aType == GeomAbs_BezierSurface || aType == GeomAbs_BSplineSurface)
        {
            if (theFace.UDegree() < 2)
            {
                theCellsCountU = (int)Math.Ceiling(Math.Pow(2, Math.Log10(aSqNbVert)));
            }
            if (theFace.VDegree() < 2)
            {
                theCellsCountV = (int)Math.Ceiling(Math.Pow(2, Math.Log10(aSqNbVert)));
            }
        }

        theCellsCountU = Max(theCellsCountU, 2);
        theCellsCountV = Max(theCellsCountV, 2);
    }
}
