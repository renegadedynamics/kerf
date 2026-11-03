// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;

namespace Occt;

public static partial class BRepLib
{
    //=======================================================================
    // function: findNearestValidPoint
    // purpose : Starting from the appointed end of the curve, find the nearest
    //           point on the curve that is an intersection with the sphere with
    //           center theVertPnt and radius theTol.
    //=======================================================================
    private static bool findNearestValidPoint(CurveAdaptorBase theCurve,
                                              double theFirst,
                                              double theLast,
                                              bool isFirst,
                                              in Point3D theVertPnt,
                                              double theTol,
                                              double theEps,
                                              ref double thePar)
    {
        // 1. Check that the needed end is inside the sphere

        double aStartU = theFirst;
        double anEndU = theLast;
        if (!isFirst)
        {
            (aStartU, anEndU) = (anEndU, aStartU);
        }
        Point3D aP = theCurve.Value(aStartU);
        double aSqTol = theTol * theTol;
        if (aP.SquareDistance(theVertPnt) > aSqTol)
        {
            // the vertex does not cover the corresponding to this vertex end of the curve
            return false;
        }

        // 2. Find a nearest point that is outside

        // stepping along the curve by theTol till go out
        //
        // the general step is computed using general curve resolution
        double aStep = theCurve.Resolution(theTol) * 1.01;
        if (aStep < theEps)
        {
            aStep = theEps;
        }
        // aD1Mag is a threshold to consider local derivative magnitude too small
        // and to accelerate going out of sphere
        // (inverse of resolution is the maximal derivative);
        // this is actual for bezier and b-spline types only
        double aD1Mag = 0.0;
        CurveKind aType = theCurve.GetType();
        if (aType == OffsetCurve)
        {
            OffsetCurve3D anOffsetCurve = theCurve.OffsetCurve();
            Curve3D aBaseCurve = anOffsetCurve.BasisCurve();
            aType = new CurveAdaptor(aBaseCurve).GetType();
        }
        if (aType == BezierCurve || aType == BSplineCurve)
        {
            aD1Mag = 1.0 / theCurve.Resolution(1.0) * 0.01;
            aD1Mag *= aD1Mag;
        }
        if (!isFirst)
        {
            aStep = -aStep;
        }
        bool isOut = false;
        double anUIn = aStartU;
        double anUOut = anUIn;
        while (!isOut)
        {
            anUIn = anUOut;
            anUOut += aStep;
            if ((isFirst && anUOut > anEndU) || (!isFirst && anUOut < anEndU))
            {
                // step is too big and we go out of bounds,
                // check if the opposite bound is outside
                aP = theCurve.Value(anEndU);
                isOut = (aP.SquareDistance(theVertPnt) > aSqTol);
                if (!isOut)
                {
                    // all range is inside sphere
                    return false;
                }
                anUOut = anEndU;
                break;
            }
            if (aD1Mag > 0.0)
            {
                double aStepLocal = aStep;
                for (;;)
                {
                    // cycle to go out of local singularity
                    theCurve.D1(anUOut, out aP, out Vector3D aD1);
                    isOut = (aP.SquareDistance(theVertPnt) > aSqTol);
                    if (!isOut && aD1.SquareMagnitude() < aD1Mag)
                    {
                        aStepLocal *= 2.0;
                        anUOut += aStepLocal;
                        if ((isFirst && anUOut < anEndU) || (!isFirst && anUOut > anEndU))
                        {
                            // still in range
                            continue;
                        }
                        // went out of range, so check if the end point has out state
                        anUOut = anEndU;
                        aP = theCurve.Value(anUOut);
                        isOut = (aP.SquareDistance(theVertPnt) > aSqTol);
                        if (!isOut)
                        {
                            // all range is inside sphere
                            return false;
                        }
                    }
                    break;
                }
            }
            else
            {
                aP = theCurve.Value(anUOut);
            }
            if (!isOut)
            {
                isOut = (aP.SquareDistance(theVertPnt) > aSqTol);
            }
        }

        // 3. Precise solution with binary search

        double aDelta = Math.Abs(anUOut - anUIn);
        while (aDelta > theEps)
        {
            double aMidU = (anUIn + anUOut) * 0.5;
            aP = theCurve.Value(aMidU);
            isOut = (aP.SquareDistance(theVertPnt) > aSqTol);
            if (isOut)
            {
                anUOut = aMidU;
            }
            else
            {
                anUIn = aMidU;
            }
            aDelta = Math.Abs(anUOut - anUIn);
        }
        thePar = (anUIn + anUOut) * 0.5;
        return true;
    }

    /// <summary>
    /// For an edge given by a 3D curve and a tolerance, and its vertices by points, parameters and tolerances, finds
    /// the range of the curve between the vertices that their tolerances don't cover. Returns false if there is none;
    /// otherwise sets <paramref name="theFirst"/> and <paramref name="theLast"/> (which may be written before a false
    /// return, as in OCCT).
    /// </summary>
    public static bool FindValidRange(CurveAdaptorBase theCurve,
                                      double theTolE,
                                      double theParV1,
                                      in Point3D thePntV1,
                                      double theTolV1,
                                      double theParV2,
                                      in Point3D thePntV2,
                                      double theTolV2,
                                      ref double theFirst,
                                      ref double theLast)
    {
        if (theParV2 - theParV1 < Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        bool isInfParV1 = Kerf.Engine.Foundation.Precision.IsInfinite(theParV1), isInfParV2 = Kerf.Engine.Foundation.Precision.IsInfinite(theParV2);

        double aMaxPar = 0.0;
        if (!isInfParV1)
        {
            aMaxPar = Math.Abs(theParV1);
        }
        if (!isInfParV2)
        {
            aMaxPar = Max(aMaxPar, Math.Abs(theParV2));
        }

        double anEps = Max(Max(theCurve.Resolution(theTolE) * 0.1, Epsilon(aMaxPar)), Kerf.Engine.Foundation.Precision.PConfusion());

        if (isInfParV1)
        {
            theFirst = theParV1;
        }
        else
        {
            if (!findNearestValidPoint(theCurve, theParV1, theParV2, true, thePntV1, theTolV1, anEps, ref theFirst))
            {
                return false;
            }
            if (theParV2 - theFirst < anEps)
            {
                return false;
            }
        }

        if (isInfParV2)
        {
            theLast = theParV2;
        }
        else
        {
            if (!findNearestValidPoint(theCurve, theParV1, theParV2, false, thePntV2, theTolV2, anEps, ref theLast))
            {
                return false;
            }
            if (theLast - theParV1 < anEps)
            {
                return false;
            }
        }

        // check found parameters
        if (theFirst > theLast)
        {
            // overlapping, not valid range
            return false;
        }

        return true;
    }

    /// <summary>
    /// The range of the edge's 3D curve that its vertices' tolerances (increased by the confusion precision) don't
    /// cover. Returns false if there is none.
    /// </summary>
    public static bool FindValidRange(in Edge theEdge, ref double theFirst, ref double theLast)
    {
        if (Tool.Curve(theEdge, out _, out _, out _) is null)
        {
            return false;
        }
        var anAC = new EdgeCurveAdaptor(theEdge);
        Span<double> aParV = [anAC.FirstParameter(), anAC.LastParameter()];
        if (aParV[1] - aParV[0] < Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        // get vertices
        var aV = new Vertex[2];
        Exploration.Vertices(theEdge, out aV[0], out aV[1]);

        double aTolE = Tool.Tolerance(theEdge);
        // to have correspondence with intersection precision
        // the tolerances of vertices are increased on Precision::Confusion()
        Span<double> aTolV = [Kerf.Engine.Foundation.Precision.Confusion(), Kerf.Engine.Foundation.Precision.Confusion()];
        var aPntV = new Point3D[] { new(), new() };
        for (int i = 0; i < 2; i++)
        {
            if (!aV[i].IsNull)
            {
                aTolV[i] += Tool.Tolerance(aV[i]);
                aPntV[i] = Tool.Pnt(aV[i]);
            }
            else if (!Kerf.Engine.Foundation.Precision.IsInfinite(aParV[i]))
            {
                aTolV[i] += aTolE;
                aPntV[i] = anAC.Value(aParV[i]);
            }
        }
        return FindValidRange(anAC, aTolE, aParV[0], aPntV[0], aTolV[0], aParV[1], aPntV[1], aTolV[1], ref theFirst, ref theLast);
    }

    /// <summary>
    /// Stores on the edge its pcurve on the face when the face is a plane and the pcurve was only computed (in place:
    /// internal).
    /// </summary>
    internal static void BuildPCurveForEdgeOnPlane(in Edge aE, in Face aF)
    {
        bool bToUpdate;
        double aTolE;
        var aBB = new BrepBuilder();
        //
        BuildPCurveForEdgeOnPlane(aE, aF, out Curve2D? aC2D, out bToUpdate);
        if (bToUpdate)
        {
            aTolE = Tool.Tolerance(aE);
            aBB.UpdateEdge(aE, aC2D, aF, aTolE);
        }
    }

    /// <summary>
    /// The pcurve of the edge on the face, and whether it would have to be stored on the edge: true when it was
    /// computed (the face is a plane and the edge has no pcurve on it). Reads only.
    /// </summary>
    public static void BuildPCurveForEdgeOnPlane(in Edge aE, in Face aF, out Curve2D? aC2D, out bool bToUpdate)
    {
        aC2D = Tool.CurveOnSurface(aE, aF, out _, out _, out bool isStored);
        bToUpdate = !isStored && aC2D is not null;
    }

    /// <summary>Builds the pcurves of the edges of <paramref name="theLE"/> on the face if it is a plane (in place: internal).</summary>
    internal static void BuildPCurveForEdgesOnPlane(IEnumerable<Shape> theLE, in Face theF)
    {
        foreach (Shape aS in theLE)
        {
            Edge aE = ShapeCasts.Edge(aS);
            if (!aE.IsNull)
            {
                BuildPCurveForEdgeOnPlane(aE, theF);
            }
        }
    }
}
