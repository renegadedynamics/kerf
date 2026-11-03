// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_Intersector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_Intersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntRes2d_Position;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>
/// Intersects an edge (its pcurve on its face) with a segment of a 2D line: the intersector of the face classifier.
/// </summary>
/// <remarks>
/// A working object (decision 21): an <see cref="IntRes2d_Intersection"/> whose results are those of the last
/// <see cref="Perform(in Line2D, double, double, in BRepClass_Edge)"/>.
/// </remarks>
public class BRepClass_Intersector : Geom2dInt_IntConicCurveOfGInter, ITopClass_Intersector<BRepClass_Edge>
{
    /// <summary>An empty intersector.</summary>
    public BRepClass_Intersector()
    {
    }

    private static double MaxTol2DCurEdge(in Vertex theV1,
                                          in Vertex theV2,
                                          in Face theF,
                                          double theTol)
    {
        double aTolV3D1, aTolV3D2;
        if (theV1.IsNull)
        {
            aTolV3D1 = 0.0;
        }
        else
        {
            aTolV3D1 = Tool.Tolerance(theV1);
        }

        if (theV2.IsNull)
        {
            aTolV3D2 = 0.0;
        }
        else
        {
            aTolV3D2 = Tool.Tolerance(theV2);
        }

        double aTol2D, anUr, aVr;

        double aTolV3D = Max(aTolV3D1, aTolV3D2);
        var aS = new FaceSurfaceAdaptor(theF, false);

        anUr = aS.UResolution(aTolV3D);
        aVr = aS.VResolution(aTolV3D);
        aTol2D = Max(anUr, aVr);
        //
        aTol2D = Max(aTol2D, theTol);
        return aTol2D;
    }

    private static bool IsInter(in Box2d theBox, in Line2D theL, double theP)
    {
        bool aStatusInter = true;
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theP))
        {
            aStatusInter = theBox.IsOut(theL);
        }
        else
        {
            Point2D aPntF = theL.Location();
            Point2D aPntL = global::Kerf.Engine.Geometry.ElCLib.Value(theP, theL);
            aStatusInter = theBox.IsOut(aPntF, aPntL);
        }

        return !aStatusInter;
    }

    private static bool CheckOn(ref IntRes2d_IntersectionPoint thePntInter,
                                in Face theF,
                                in Line2D theL,
                                CurveAdaptor2D theCur,
                                ref double theTolZ,
                                double theFin,
                                double theDeb)
    {
        var anExtPC2d = new ExtPC2d(theL.Location(), theCur);
        double aMinDist = RealLast();
        int aMinInd = 0;
        if (anExtPC2d.IsDone)
        {
            int aNbPnts = anExtPC2d.ExtCount;
            for (int i = 1; i <= aNbPnts; ++i)
            {
                double aDist = anExtPC2d.SquareDistance(i);

                if (aDist < aMinDist)
                {
                    aMinDist = aDist;
                    aMinInd = i;
                }
            }
        }

        if (aMinInd != 0)
        {
            aMinDist = Math.Sqrt(aMinDist);
        }

        if (aMinDist <= theTolZ)
        {
            Point2D aPntExact = (anExtPC2d.Point(aMinInd)).Value();
            double aPar = (anExtPC2d.Point(aMinInd)).Parameter();
            //
            RefineTolerance(theF, theCur, aPar, ref theTolZ);
            //
            if (aMinDist <= theTolZ)
            {
                var aTrOnLin = new IntRes2d_Transition(IntRes2d_Head);
                IntRes2d_Position aPosOnCurve = IntRes2d_Middle;
                if ((Math.Abs(aPar - theDeb) <= global::Kerf.Engine.Foundation.Precision.Confusion()) || (aPar < theDeb))
                {
                    aPosOnCurve = IntRes2d_Head;
                }
                else if ((Math.Abs(aPar - theFin) <= global::Kerf.Engine.Foundation.Precision.Confusion()) || (aPar > theFin))
                {
                    aPosOnCurve = IntRes2d_End;
                }

                //
                var aTrOnCurve = new IntRes2d_Transition(aPosOnCurve);
                thePntInter = new IntRes2d_IntersectionPoint(aPntExact, 0.0, aPar, aTrOnLin, aTrOnCurve, false);
                //
                return true;
            }
        }

        return false;
    }

    private static void CheckSkip(ref Geom2dInt_GInter theInter,
                                  in Line2D theL,
                                  in BRepClass_Edge theE,
                                  Curve2D theC2D,
                                  in IntRes2d_Domain theDL,
                                  CurveAdaptor2D theCur,
                                  CurveAdaptor2D theCGA,
                                  ref double theFin,
                                  ref double theDeb,
                                  double theMaxTol,
                                  ref Point2D thePdeb,
                                  ref Point2D thePfin)
    {
        if (theE.Edge().IsNull || theE.Face().IsNull)
        {
            return;
        }

        bool anIsLSkip = false;
        Vertex aVl; // the last vertex of current edge

        Curve2D? aSkipC2D;

        aVl = Exploration.LastVertex(theE.Edge(), true);
        if (aVl.IsNull)
        {
            return;
        }

        if (!(Tool.Tolerance(aVl) > theMaxTol) || theE.NextEdge().IsNull)
        {
            return;
        }

        double aLdeb = 0.0, aLfin = 0.0;
        Curve2D? aLC2D; // the next curve

        aLC2D = Tool.CurveOnSurface(theE.NextEdge(), theE.Face(), out aLdeb, out aLfin);
        if (aLC2D is null)
        {
            return;
        }

        double anA, aB, aC; // coefficients of the straight line
        double aX1, anY1, aX2, anY2; // coordinates of the ends of edges
        Point2D aP1, aP2; // the ends of edges

        theL.Coefficients(out anA, out aB, out aC);

        double at1 = theFin;
        if (theE.Edge().Orientation() != Forward)
        {
            at1 = theDeb;
        }

        double at2 = aLdeb;
        if (theE.NextEdge().Orientation() != Forward)
        {
            at2 = aLfin;
        }

        aP1 = theC2D.Value(at1);
        aP2 = aLC2D.Value(at2);

        // Check if points belong to DL domain
        double aPar1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theL, aP1);
        double aPar2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theL, aP2);

        if (aPar1 <= theDL.FirstParameter() || aPar1 >= theDL.LastParameter()
            || aPar2 <= theDL.FirstParameter() || aPar2 >= theDL.LastParameter())
        {
            return;
        }

        aX1 = aP1.X();
        anY1 = aP1.Y();
        aX2 = aP2.X();
        anY2 = aP2.Y();
        double aFV = anA * aX1 + aB * anY1 + aC;
        double aSV = anA * aX2 + aB * anY2 + aC;

        // Check for getting into vertex with high tolerance
        if ((aFV * aSV) >= 0)
        {
            anIsLSkip = false;
        }
        else
        {
            anIsLSkip = true;
            var aMkSeg = new MakeSegment2d(aP1, aP2);
            if (!aMkSeg.IsDone)
            {
                return;
            }

            aSkipC2D = aMkSeg.Value();

            if (aSkipC2D is null || !anIsLSkip)
            {
                return;
            }

            // if we got
            theCur.Load(aSkipC2D);
            if (theCur.Curve() is null)
            {
                return;
            }

            double atoldeb = 1.0e-5, atolfin = 1.0e-5;

            theDeb = theCur.FirstParameter();
            theFin = theCur.LastParameter();
            theCur.D0(theDeb, out thePdeb);
            theCur.D0(theFin, out thePfin);

            var aDE = new IntRes2d_Domain(thePdeb, theDeb, atoldeb, thePfin, theFin, atolfin);
            // temporary periodic domain
            if (theCur.Curve()!.IsPeriodic)
            {
                aDE.SetEquivalentParameters(theCur.FirstParameter(),
                                            theCur.FirstParameter() + theCur.Curve()!.LastParameter()
                                            - theCur.Curve()!.FirstParameter());
            }

            theInter = new Geom2dInt_GInter(theCGA,
                                            theDL,
                                            theCur,
                                            aDE,
                                            global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                            global::Kerf.Engine.Foundation.Precision.PIntersection());
        }
    }

    /// <summary>Intersects the segment of <paramref name="L"/> from its origin to the parameter <paramref name="P"/> with the edge.</summary>
    public void Perform(in Line2D L, double P, double Tol, in BRepClass_Edge E)
    {
        double deb = 0.0, fin = 0.0, aTolZ = Tol;
        Curve2D? aC2D;
        //
        Edge EE = E.Edge();
        Face F = E.Face();

        //
        aC2D = Tool.CurveOnSurface(EE, F, out deb, out fin);
        if (aC2D is null)
        {
            done = false; // !IsDone()
            return;
        }

        //
        Box2d aBond = new();
        Point2D aPntF = new();
        bool anUseBndBox = E.UseBndBox();
        if (anUseBndBox)
        {
            Add2dCurve.Add(aC2D, deb, fin, 0.0, ref aBond);
            aBond.SetGap(aTolZ);
            aPntF = L.Location();
        }

        //
        var C = new CurveAdaptor2D(aC2D, deb, fin);
        //
        // Case of "ON": direct check of belonging to edge
        // taking into account the tolerance
        if (!anUseBndBox || (anUseBndBox && !aBond.IsOut(aPntF)))
        {
            bool aStatusOn = false;
            IntRes2d_IntersectionPoint aPntInter = new();
            double aDebTol = deb;
            double aFinTol = fin;
            if (aTolZ > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                aDebTol = deb - aTolZ;
                aFinTol = fin + aTolZ;
            }

            var aCurAdaptor = new CurveAdaptor2D(aC2D, aDebTol, aFinTol);

            aStatusOn = CheckOn(ref aPntInter, F, L, aCurAdaptor, ref aTolZ, fin, deb);
            if (aStatusOn)
            {
                Append(aPntInter);
                done = true;
                return;
            }
        }

        //
        if (anUseBndBox)
        {
            Exploration.Vertices(EE, out Vertex aVF, out Vertex aVL);

            aTolZ = MaxTol2DCurEdge(aVF, aVL, F, Tol);
            aBond.SetGap(aTolZ);

            if (!IsInter(aBond, L, P))
            {
                done = false;
                return;
            }
        }

        C.D0(deb, out Point2D pdeb);
        C.D0(fin, out Point2D pfin);
        double toldeb = 1.0e-5, tolfin = 1.0e-5;

        IntRes2d_Domain DL = new();
        //
        if (P != RealLast())
        {
            DL.SetValues(L.Location(),
                         0.0,
                         global::Kerf.Engine.Foundation.Precision.PConfusion(),
                         global::Kerf.Engine.Geometry.ElCLib.Value(P, L),
                         P,
                         global::Kerf.Engine.Foundation.Precision.PConfusion());
        }
        else
        {
            DL.SetValues(L.Location(), 0.0, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);
        }

        var DE = new IntRes2d_Domain(pdeb, deb, toldeb, pfin, fin, tolfin);
        // temporary periodic domain
        if (C.Curve()!.IsPeriodic)
        {
            DE.SetEquivalentParameters(C.FirstParameter(),
                                       C.FirstParameter() + C.Curve()!.LastParameter()
                                       - C.Curve()!.FirstParameter());
        }

        var GL = new LineCurve2D(L);
        var CGA = new CurveAdaptor2D(GL);
        var Inter = new Geom2dInt_GInter(CGA, DL, C, DE, global::Kerf.Engine.Foundation.Precision.PConfusion(), global::Kerf.Engine.Foundation.Precision.PIntersection());
        //
        // The check is for hitting the intersector to
        // a vertex with high tolerance
        if (Inter.IsEmpty())
        {
            CheckSkip(ref Inter, L, E, aC2D, DL, C, CGA, ref fin, ref deb, E.MaxTolerance(), ref pdeb, ref pfin);
        }

        //
        SetValues(Inter);
    }

    /// <summary>
    /// The tangent <paramref name="Tang"/>, normal <paramref name="Norm"/> and curvature <paramref name="C"/> of the
    /// edge's pcurve at the parameter <paramref name="U"/>.
    /// </summary>
    /// <remarks>
    /// Where OCCT leaves the tangent as the caller had it (no tangent and a chord too short), the port gives OCCT's
    /// callers' default, (1, 0).
    /// </remarks>
    public void LocalGeometry(in BRepClass_Edge E, double U, out Direction2D Tang, out Direction2D Norm, out double C)
    {
        Tang = new Direction2D();
        Norm = new Direction2D();
        Curve2D aPCurve = Tool.CurveOnSurface(E.Edge(), E.Face(), out double fpar, out double lpar)!;
        var Prop = new CLProps2d(aPCurve, U, 2, global::Kerf.Engine.Foundation.Precision.PConfusion());

        C = 0.0;
        if (Prop.IsTangentDefined())
        {
            Prop.Tangent(out Tang);
            C = Prop.Curvature();
        }
        else
        {
            GetTangentAsChord(aPCurve, ref Tang, U, fpar, lpar);
        }

        if (C > global::Kerf.Engine.Foundation.Precision.PConfusion() && !global::Kerf.Engine.Foundation.Precision.IsInfinite(C))
        {
            Prop.Normal(out Norm);
        }
        else
        {
            Norm.SetCoord(Tang.Y(), -Tang.X());
        }
    }

    private static void RefineTolerance(in Face aF,
                                        CurveAdaptor2D aC,
                                        double aT,
                                        ref double aTolZ)
    {
        SurfaceKind aTypeS;
        //
        var aBAS = new FaceSurfaceAdaptor(aF, false);
        //
        aTypeS = aBAS.GetType();
        if (aTypeS == SurfaceKind.Cylinder)
        {
            double aURes, aVRes, aTolX;
            //
            aURes = aBAS.UResolution(aTolZ);
            aVRes = aBAS.VResolution(aTolZ);
            //
            aC.D1(aT, out Point2D _, out Vector2D aV2D);
            Direction2D aD2D = new(aV2D);
            //
            aTolX = aURes * aD2D.Y() + aVRes * aD2D.X();
            if (aTolX < 0.0)
            {
                aTolX = -aTolX;
            }

            //
            if (aTolX < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                aTolX = global::Kerf.Engine.Foundation.Precision.Confusion();
            }

            //
            if (aTolX < aTolZ)
            {
                aTolZ = aTolX;
            }
        }
    }

    private static void GetTangentAsChord(Curve2D thePCurve,
                                          ref Direction2D theTangent,
                                          double theParam,
                                          double theFirst,
                                          double theLast)
    {
        double Offset = 0.1 * (theLast - theFirst);

        if (theLast - theParam < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            // theParam == theLast
            Offset *= -1;
        }
        else if (theParam + Offset > theLast)
        {
            //<theParam> is close to <theLast>
            Offset = 0.5 * (theLast - theParam);
        }

        Point2D aPnt2d = thePCurve.Value(theParam);
        Point2D OffsetPnt2d = thePCurve.Value(theParam + Offset);

        Vector2D aChord = new(aPnt2d, OffsetPnt2d);
        if (Offset < 0.0)
        {
            aChord.Reverse();
        }

        double SqLength = aChord.SquareMagnitude();
        if (SqLength > global::Kerf.Engine.Foundation.Precision.SquarePConfusion())
        {
            theTangent = new Direction2D(aChord);
        }
    }
}
