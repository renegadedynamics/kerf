// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_FindSurface.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_FindSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Finds a surface through the edges of a shape: a surface on which every edge has a pcurve (<see cref="Existed"/>),
/// or else a plane fitted by least squares to points sampled along the edges, accepted if no point is farther than
/// the tolerance.
/// </summary>
/// <remarks>
/// <para>The tolerance is the larger of the given one and the edges' largest; a negative tolerance accepts a plane up
/// to five times the edges' tolerance. A plane through a closed wire is oriented so that the wire bounds a finite
/// area.</para>
/// <para>A working object; it only reads the shape. <see cref="Surface"/> freezes the surface it returns.</para>
/// </remarks>
public class BRepLib_FindSurface
{
    private Geom_Surface? mySurface;
    private double myTolerance;
    private double myTolReached;
    private bool isExisted;
    private TopLoc_Location myLocation = new();

    private static double Controle(NCollection_Sequence<gp_Pnt> thePoints, Geom_Plane thePlane)
    {
        double dfMaxDist = 0.0;
        double dist;
        int ii;
        thePlane.Coefficients(out double a, out double b, out double c, out double d);
        for (ii = 1; ii <= thePoints.Length(); ii++)
        {
            gp_XYZ xyz = thePoints[ii].XYZ();
            dist = Math.Abs(a * xyz.X() + b * xyz.Y() + c * xyz.Z() + d);
            if (dist > dfMaxDist)
            {
                dfMaxDist = dist;
            }
        }

        return dfMaxDist;
    }

    // Return true if the last vertex of theEdge1 coincides with
    // the first vertex of theEdge2 in parametric space of theFace
    private static bool Is2DConnected(in TopoDS_Edge theEdge1,
                                      in TopoDS_Edge theEdge2,
                                      Geom_Surface theSurface,
                                      in TopLoc_Location theLocation)
    {
        double f, l;
        Geom2d_Curve? aCurve;
        gp_Pnt2d p1, p2;

        // get 2D points
        aCurve = BRep_Tool.CurveOnSurface(theEdge1, theSurface, theLocation, out f, out l);
        p1 = aCurve!.Value(theEdge1.Orientation() == TopAbs_FORWARD ? l : f);
        aCurve = BRep_Tool.CurveOnSurface(theEdge2, theSurface, theLocation, out f, out l);
        p2 = aCurve!.Value(theEdge2.Orientation() == TopAbs_FORWARD ? f : l);

        // compare 2D points
        var aSurface = new GeomAdaptor_Surface(theSurface);
        TopoDS_Vertex aV = TopExp.FirstVertex(theEdge2, true);
        double tol3D = BRep_Tool.Tolerance(aV);
        double tol2D = aSurface.UResolution(tol3D) + aSurface.VResolution(tol3D);
        double dist2 = p1.SquareDistance(p2);
        return dist2 < tol2D * tol2D;
    }

    // Return true if edges of theShape form a closed wire in
    // parametric space of theSurface
    private static bool Is2DClosed(in TopoDS_Shape theShape, Geom_Surface theSurface, in TopLoc_Location theLocation)
    {
        try
        {
            // get a wire theShape
            var aWireExp = new TopExp_Explorer(theShape, TopAbs_WIRE);
            if (!aWireExp.More())
            {
                return false;
            }
            TopoDS_Wire aWire = TopoDS.Wire(aWireExp.Current());
            // a tmp face
            TopoDS_Face aTmpFace = new BRepLib_MakeFace(theSurface, Precision.PConfusion()).InternalFace();

            // check topological closeness using wire explorer, if the wire is not closed
            // the 1st and the last vertices of wire are different
            var aWireExplorer = new BRepTools_WireExplorer(aWire, aTmpFace);
            if (!aWireExplorer.More())
            {
                return false;
            }
            // remember the 1st and the last edges of aWire
            TopoDS_Edge aFisrtEdge = aWireExplorer.Current(), aLastEdge = aFisrtEdge;
            // check if edges connected topologically (that is assured by BRepTools_WireExplorer)
            // are connected in 2D
            TopoDS_Edge aPrevEdge = aFisrtEdge;
            for (aWireExplorer.Next(); aWireExplorer.More(); aWireExplorer.Next())
            {
                aLastEdge = aWireExplorer.Current();
                if (!Is2DConnected(aPrevEdge, aLastEdge, theSurface, theLocation))
                {
                    return false;
                }
                aPrevEdge = aLastEdge;
            }
            // wire is closed if ( 1st vertex of aFisrtEdge ) ==
            // ( last vertex of aLastEdge ) in 2D
            TopoDS_Vertex aV1 = TopExp.FirstVertex(aFisrtEdge, true);
            TopoDS_Vertex aV2 = TopExp.LastVertex(aLastEdge, true);
            return (aV1.IsSame(aV2) && Is2DConnected(aLastEdge, aFisrtEdge, theSurface, theLocation));
        }
        catch (Standard_Failure)
        {
            return false;
        }
        catch (NullReferenceException)
        {
            // OCCT dereferences a null pcurve under OCC_CATCH_SIGNALS, which turns the access violation into a
            // Standard_Failure caught here.
            return false;
        }
    }

    /// <summary>Creates an empty finder.</summary>
    public BRepLib_FindSurface()
    {
        myTolerance = 0.0;
        myTolReached = 0.0;
        isExisted = false;
    }

    /// <summary>Finds a surface through the edges of <paramref name="S"/> (see <see cref="Init"/>).</summary>
    public BRepLib_FindSurface(in TopoDS_Shape S, double Tol = -1, bool OnlyPlane = false, bool OnlyClosed = false)
    {
        Init(S, Tol, OnlyPlane, OnlyClosed);
    }

    private static void fillParams(NCollection_Array1<double> theKnots,
                                   int theDegree,
                                   double theParMin,
                                   double theParMax,
                                   NCollection_DynamicArray<double> theParams)
    {
        double aPrevPar = theParMin;
        theParams.Append(aPrevPar);

        int aNbP = Max(theDegree, 1);

        for (int i = 1; (i < theKnots.Length()) && (theKnots[i] < (theParMax - Precision.PConfusion())); ++i)
        {
            if (theKnots[i + 1] < theParMin + Precision.PConfusion())
            {
                continue;
            }

            double aStep = (theKnots[i + 1] - theKnots[i]) / aNbP;
            for (int k = 1; k <= aNbP; ++k)
            {
                double aPar = theKnots[i] + k * aStep;
                if (aPar > theParMax - Precision.PConfusion())
                {
                    break;
                }

                if (aPar > aPrevPar + Precision.PConfusion())
                {
                    theParams.Append(aPar);
                    aPrevPar = aPar;
                }
            }
        }
        theParams.Append(theParMax);
    }

    private static void fillPoints(BRepAdaptor_Curve theCurve,
                                   NCollection_DynamicArray<double> theParams,
                                   NCollection_Sequence<gp_Pnt> thePoints,
                                   NCollection_Sequence<double> theWeights)
    {
        double aDistPrev = 0.0, aDistNext;
        gp_Pnt aPPrev = theCurve.Value(theParams[0]), aPNext = new();

        for (int iP = 1; iP <= theParams.Length(); ++iP)
        {
            if (iP < theParams.Length())
            {
                double aParam = theParams[iP];
                aPNext = theCurve.Value(aParam);
                aDistNext = aPPrev.Distance(aPNext);
            }
            else
            {
                aDistNext = 0.0;
            }

            thePoints.Append(aPPrev);
            theWeights.Append(aDistPrev + aDistNext);
            aDistPrev = aDistNext;
            aPPrev = aPNext;
        }
    }

    /// <summary>
    /// Finds a surface through the edges of <paramref name="S"/>: an existing surface of the first edge's pcurves that
    /// every edge has a pcurve on (with <paramref name="OnlyPlane"/>, only a plane; with <paramref name="OnlyClosed"/>,
    /// only one on which the shape's wire is closed in 2D), otherwise a least-squares plane within the tolerance.
    /// </summary>
    public void Init(in TopoDS_Shape S, double Tol = -1, bool OnlyPlane = false, bool OnlyClosed = false)
    {
        myTolerance = Tol;
        myTolReached = 0.0;
        isExisted = false;
        myLocation.Identity();
        mySurface = null;

        // compute the tolerance
        var ex = new TopExp_Explorer();

        for (ex.Init(S, TopAbs_EDGE); ex.More(); ex.Next())
        {
            double t = BRep_Tool.Tolerance(TopoDS.Edge(ex.Current()));
            if (t > myTolerance)
            {
                myTolerance = t;
            }
        }

        // search an existing surface
        ex.Init(S, TopAbs_EDGE);
        if (!ex.More())
        {
            return; // no edges ....
        }

        TopoDS_Edge E = TopoDS.Edge(ex.Current());
        double f = 0, l = 0, ff = 0, ll = 0;
        Geom2d_Curve? PC = null, aPPC = null;
        Geom_Surface? SS = null;
        var L = new TopLoc_Location();
        int i = 0, j;

        // iterate on the surfaces of the first edge
        for (;;)
        {
            i++;
            BRep_Tool.CurveOnSurface(E, ref PC, ref mySurface, ref myLocation, ref f, ref l, i);
            if (mySurface is null)
            {
                break;
            }
            // check the other edges
            for (ex.Init(S, TopAbs_EDGE); ex.More(); ex.Next())
            {
                if (!E.IsSame(ex.Current()))
                {
                    j = 0;
                    for (;;)
                    {
                        j++;
                        BRep_Tool.CurveOnSurface(TopoDS.Edge(ex.Current()), ref aPPC, ref SS, ref L, ref ff, ref ll, j);
                        if (SS is null)
                        {
                            break;
                        }
                        if (ReferenceEquals(SS, mySurface) && (L.IsEqual(myLocation)))
                        {
                            break;
                        }
                        SS = null;
                    }

                    if (SS is null)
                    {
                        mySurface = null;
                        break;
                    }
                }
            }

            // if OnlyPlane, eval if mySurface is a plane.
            if (OnlyPlane && mySurface is not null)
            {
                if (mySurface is Geom_RectangularTrimmedSurface aRTS)
                {
                    mySurface = aRTS.BasisSurface();
                }
                mySurface = mySurface as Geom_Plane;
            }

            if (mySurface is not null)
            {
                // if S is e.g. the bottom face of a cylinder, mySurface can be the
                // lateral (cylindrical) face of the cylinder; reject an improper mySurface
                if (!OnlyClosed || Is2DClosed(S, mySurface, myLocation))
                {
                    break;
                }
            }
        }

        if (mySurface is not null)
        {
            isExisted = true;
            return;
        }
        //
        // no existing surface, search a plane
        // 07/02/02 akm vvv : (OCC157) changed algorithm
        //                    1. Collect the points along all edges of the shape
        //                       For each point calculate the WEIGHT = sum of
        //                       distances from neighboring points (_only_ same edge)
        //                    2. Minimizing the weighed sum of squared deviations
        //                       compute coefficients of the sought plane.

        var aPoints = new NCollection_Sequence<gp_Pnt>();
        var aWeight = new NCollection_Sequence<double>();

        // ======================= Step #1
        for (ex.Init(S, TopAbs_EDGE); ex.More(); ex.Next())
        {
            var c = new BRepAdaptor_Curve(TopoDS.Edge(ex.Current()));

            double dfUf = c.FirstParameter();
            double dfUl = c.LastParameter();
            if (IsEqual(dfUf, dfUl))
            {
                // Degenerate
                continue;
            }
            int iNbPoints = 0;

            // Fill the parameters of the sampling points
            var aParams = new NCollection_DynamicArray<double>();
            switch (c.GetType())
            {
                case GeomAbs_BezierCurve:
                {
                    Geom_BezierCurve GC = c.Bezier();
                    var aKnots = new NCollection_Array1<double>(1, 2);
                    aKnots.SetValue(1, GC.FirstParameter());
                    aKnots.SetValue(2, GC.LastParameter());

                    fillParams(aKnots, GC.Degree(), dfUf, dfUl, aParams);
                    break;
                }
                case GeomAbs_BSplineCurve:
                {
                    Geom_BSplineCurve GC = c.BSpline();
                    fillParams(GC.InternalKnots(), GC.Degree(), dfUf, dfUl, aParams);
                    break;
                }
                case GeomAbs_Line:
                {
                    // Two points on a straight segment
                    aParams.Append(dfUf);
                    aParams.Append(dfUl);
                    break;
                }
                case GeomAbs_Circle:
                case GeomAbs_Ellipse:
                case GeomAbs_Hyperbola:
                case GeomAbs_Parabola:
                default:
                {
                    // Four points on other analytical curves
                    if (c.GetType() is GeomAbs_Circle or GeomAbs_Ellipse or GeomAbs_Hyperbola or GeomAbs_Parabola)
                    {
                        iNbPoints = 4;
                    }
                    // Put some points on other curves
                    if (iNbPoints == 0)
                    {
                        iNbPoints = 15 + c.NbIntervals(GeomAbs_Shape.GeomAbs_C3);
                    }

                    var aBounds = new NCollection_Array1<double>(1, 2);
                    aBounds.SetValue(1, dfUf);
                    aBounds.SetValue(2, dfUl);

                    fillParams(aBounds, iNbPoints - 1, dfUf, dfUl, aParams);
                    break;
                }
            }

            // Add the points with weights to the sequences
            fillPoints(c, aParams, aPoints, aWeight);
        }

        if (aPoints.Length() < 3)
        {
            return;
        }

        // ======================= Step #2
        myLocation.Identity();
        int iPoint;
        var aMat = new math_Matrix(1, 3, 1, 3, 0.0);
        var aVec = new math_Vector(1, 3, 0.0);
        // Find the barycenter and normalize weights
        double dfMaxWeight = 0.0;
        var aBaryCenter = new gp_XYZ(0.0, 0.0, 0.0);
        double dfSumWeight = 0.0;
        for (iPoint = 1; iPoint <= aPoints.Length(); iPoint++)
        {
            double dfW = aWeight[iPoint];
            aBaryCenter += dfW * aPoints[iPoint].XYZ();
            dfSumWeight += dfW;
            if (dfW > dfMaxWeight)
            {
                dfMaxWeight = dfW;
            }
        }
        aBaryCenter /= dfSumWeight;

        // Fill the matrix and the right vector
        for (iPoint = 1; iPoint <= aPoints.Length(); iPoint++)
        {
            gp_XYZ p = aPoints[iPoint].XYZ() - aBaryCenter;
            double w = aWeight[iPoint] / dfMaxWeight;
            aMat[1, 1] += w * p.X() * p.X();
            aMat[1, 2] += w * p.X() * p.Y();
            aMat[1, 3] += w * p.X() * p.Z();
            //
            aMat[2, 2] += w * p.Y() * p.Y();
            aMat[2, 3] += w * p.Y() * p.Z();
            //
            aMat[3, 3] += w * p.Z() * p.Z();
        }
        aMat[2, 1] = aMat[1, 2];
        aMat[3, 1] = aMat[1, 3];
        aMat[3, 2] = aMat[2, 3];
        //
        var anEignval = new math_Jacobi(aMat);
        math_Vector anEVals;
        bool isSolved = anEignval.IsDone();
        int isol = 0;
        if (isSolved)
        {
            anEVals = anEignval.Values();
            // We need vector with eigenvalue ~ 0.
            double anEMin = RealLast();
            double anEMax = -anEMin;
            for (i = 1; i <= 3; ++i)
            {
                double anE = Math.Abs(anEVals[i]);
                if (anEMin > anE)
                {
                    anEMin = anE;
                    isol = i;
                }
                if (anEMax < anE)
                {
                    anEMax = anE;
                }
            }

            if (isol == 0)
            {
                isSolved = false;
            }
            else
            {
                double eps = Epsilon(anEMax);
                if (anEMin <= eps)
                {
                    anEignval.Vector(isol, aVec);
                }
                else
                {
                    // try using vector product of other axes
                    Span<int> ind = [0, 0];
                    for (i = 1; i <= 3; ++i)
                    {
                        if (i == isol)
                        {
                            continue;
                        }
                        if (ind[0] == 0)
                        {
                            ind[0] = i;
                            continue;
                        }
                        if (ind[1] == 0)
                        {
                            ind[1] = i;
                            continue;
                        }
                    }
                    var aVec1 = new math_Vector(1, 3, 0.0);
                    var aVec2 = new math_Vector(1, 3, 0.0);
                    anEignval.Vector(ind[0], aVec1);
                    anEignval.Vector(ind[1], aVec2);
                    var aV1 = new gp_Vec(aVec1[1], aVec1[2], aVec1[3]);
                    var aV2 = new gp_Vec(aVec2[1], aVec2[2], aVec2[3]);
                    gp_Vec aN0 = aV1 ^ aV2;
                    aVec[1] = aN0.X();
                    aVec[2] = aN0.Y();
                    aVec[3] = aN0.Z();
                }
                if (aVec.Norm2() < gp.Resolution())
                {
                    isSolved = false;
                }
            }
        }

        if (!isSolved)
        {
            return;
        }
        // Removing very small values
        double aMaxV = Max(Math.Abs(aVec[1]), Max(Math.Abs(aVec[2]), Math.Abs(aVec[3])));
        double eps2 = Epsilon(aMaxV);
        for (i = 1; i <= 3; ++i)
        {
            if (Math.Abs(aVec[i]) <= eps2)
            {
                aVec[i] = 0.0;
            }
        }
        var aN = new gp_Vec(aVec[1], aVec[2], aVec[3]);
        var aPlane = new Geom_Plane(new gp_Pnt(aBaryCenter), new gp_Dir(aN));
        myTolReached = Controle(aPoints, aPlane);
        const double aWeakness = 5.0;
        if (myTolReached <= myTolerance || (Tol < 0 && myTolReached < myTolerance * aWeakness))
        {
            mySurface = aPlane;
            // If S is wire, try to orient surface according to orientation of wire.
            if (S.ShapeType() == TopAbs_WIRE && S.Closed())
            {
                TopoDS_Wire aW = TopoDS.Wire(S);
                TopoDS_Face aTmpFace = new BRepLib_MakeFace(mySurface, Precision.Confusion()).InternalFace();
                var BB = new BRep_Builder();
                BB.Add(aTmpFace, aW);
                var FClass = new BRepTopAdaptor_FClass2d(aTmpFace, 0.0);
                if (FClass.PerformInfinitePoint() == TopAbs_IN)
                {
                    gp_Dir aNorm = aPlane.Position().Direction();
                    aNorm.Reverse();
                    mySurface = new Geom_Plane(aPlane.Position().Location(), aNorm);
                }
            }
        }
    }

    /// <summary>True if a surface was found.</summary>
    public bool Found()
    {
        return mySurface is not null;
    }

    /// <summary>The surface found (frozen), or null.</summary>
    public Geom_Surface? Surface()
    {
        mySurface?.Freeze();
        return mySurface;
    }

    /// <summary>The tolerance: the larger of the given one and the edges' largest.</summary>
    public double Tolerance()
    {
        return myTolerance;
    }

    /// <summary>The largest distance of the sampled points to the fitted plane (0 for an existing surface).</summary>
    public double ToleranceReached()
    {
        return myTolReached;
    }

    /// <summary>True if the surface is one of the edges' existing surfaces.</summary>
    public bool Existed()
    {
        return isExisted;
    }

    /// <summary>The location of the surface (of an existing surface; identity for a fitted plane).</summary>
    public TopLoc_Location Location()
    {
        return myLocation;
    }
}
