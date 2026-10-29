// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// General utilities for BRep: the default precision and plane of the makers, the computation of missing 3D curves
/// and pcurves, SameParameter and SameRange, tolerances, regularity, orientation of closed solids.
/// </summary>
public static partial class BRepLib
{
    // OCCT's file statics. Process-wide as in OCCT; their setters are internal, so callers always see the defaults.
    private static double thePrecision = Occt.Precision.Confusion();
    private static Geom_Plane? thePlane;

    /// <summary>Sets the default precision (internal: process-wide state).</summary>
    internal static void Precision(double P)
    {
        thePrecision = P;
    }

    /// <summary>Returns the default precision of the makers.</summary>
    public static double Precision()
    {
        return thePrecision;
    }

    /// <summary>Sets the current plane (internal: process-wide state).</summary>
    internal static void Plane(Geom_Plane P)
    {
        P.Freeze();
        thePlane = P;
    }

    /// <summary>Returns the current plane of the 2D makers (frozen).</summary>
    public static Geom_Plane Plane()
    {
        if (thePlane is null)
        {
            var aPlane = new Geom_Plane(new gp_Ax3(gp.XOY()));
            aPlane.Freeze();
            Interlocked.CompareExchange(ref thePlane, aPlane, null);
        }
        return thePlane;
    }

    // TODO(3I): the statics below are being ported; signatures are final.

    internal static bool BuildCurves3d(in TopoDS_Shape S, double Tolerance, GeomAbs_Shape Continuity = GeomAbs_Shape.GeomAbs_C1,
                                       int MaxDegree = 14, int MaxSegment = 0)
        => throw new NotImplementedException();

    internal static bool BuildCurves3d(in TopoDS_Shape S) => throw new NotImplementedException();

    internal static void SameParameter(in TopoDS_Edge theEdge, double Tolerance = 1.0e-5) => throw new NotImplementedException();

    internal static void SameParameter(in TopoDS_Shape S, double Tolerance = 1.0e-5, bool forced = false)
        => throw new NotImplementedException();

    internal static void UpdateTolerances(in TopoDS_Shape S, bool verifyFaceTolerance = false) => throw new NotImplementedException();

    internal static void EncodeRegularity(in TopoDS_Shape S, double TolAng = 1.0e-10) => throw new NotImplementedException();

    /// <summary>
    /// The bounding sphere of the vertices of <paramref name="theLV"/>: its centre and radius, for a new vertex
    /// covering them all. With fewer than two vertices the outputs are left unchanged, as in OCCT.
    /// </summary>
    public static void BoundingVertex(NCollection_List<TopoDS_Shape> theLV, ref gp_Pnt theNewCenter, ref double theNewTol)
    {
        int aNb;
        //
        aNb = theLV.Extent();
        if (aNb < 2)
        {
            return;
        }
        //
        else if (aNb == 2)
        {
            int m, n;
            double dR, aD, aEps;
            Span<double> aR = stackalloc double[2];
            var aV = new TopoDS_Vertex[2];
            var aP = new gp_Pnt[2];
            //
            aEps = RealEpsilon();
            for (m = 0; m < aNb; ++m)
            {
                aV[m] = (m == 0) ? TopoDS.Vertex(theLV.First()) : TopoDS.Vertex(theLV.Last());
                aP[m] = BRep_Tool.Pnt(aV[m]);
                aR[m] = BRep_Tool.Tolerance(aV[m]);
            }
            //
            m = 0; // max R
            n = 1; // min R
            if (aR[0] < aR[1])
            {
                m = 1;
                n = 0;
            }
            //
            dR = aR[m] - aR[n]; // dR >= 0.
            var aVD = new gp_Vec(aP[m], aP[n]);
            aD = aVD.Magnitude();
            //
            if (aD <= dR || aD < aEps)
            {
                theNewCenter = aP[m];
                theNewTol = aR[m];
            }
            else
            {
                double aRr;
                gp_XYZ aXYZr;
                var aPr = new gp_Pnt();
                //
                aRr = 0.5 * (aR[m] + aR[n] + aD);
                aXYZr = 0.5 * (aP[m].XYZ() + aP[n].XYZ() - aVD.XYZ() * (dR / aD));
                aPr.SetXYZ(aXYZr);
                //
                theNewCenter = aPr;
                theNewTol = aRr;
            }
            return;
        }
        //
        else
        { // if (aNb>2)
            // compute the point
            //
            // issue 0027540 - sum of doubles may depend on the order
            // of addition, thus sort the coordinates for stable result
            int i;
            var aPoints = new gp_Pnt[aNb];
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(theLV);
            for (i = 0; aIt.More(); aIt.Next(), ++i)
            {
                TopoDS_Vertex aVi = TopoDS.Vertex(aIt.Value());
                gp_Pnt aPi = BRep_Tool.Pnt(aVi);
                aPoints[i] = aPi;
            }
            //
            Array.Sort(aPoints, comparePoints);
            //
            var aXYZ = new gp_XYZ(0.0, 0.0, 0.0);
            for (i = 0; i < aNb; ++i)
            {
                aXYZ += aPoints[i].XYZ();
            }
            aXYZ.Divide((double)aNb);
            //
            var aP = new gp_Pnt(aXYZ);
            //
            // compute the tolerance for the new vertex
            double aTi, aDi, aDmax;
            //
            aDmax = -1.0;
            aIt.Initialize(theLV);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Vertex aVi = TopoDS.Vertex(aIt.Value());
                gp_Pnt aPi = BRep_Tool.Pnt(aVi);
                aTi = BRep_Tool.Tolerance(aVi);
                aDi = aP.SquareDistance(aPi);
                aDi = Math.Sqrt(aDi);
                aDi = aDi + aTi;
                if (aDi > aDmax)
                {
                    aDmax = aDi;
                }
            }
            //
            theNewCenter = aP;
            theNewTol = aDmax;
        }
    }

    // BRepLib_ComparePoints: lexicographic order of the coordinates (a strict weak order, as std::sort needs).
    private static int comparePoints(gp_Pnt theP1, gp_Pnt theP2)
    {
        for (int i = 1; i <= 3; ++i)
        {
            if (theP1.Coord(i) < theP2.Coord(i))
            {
                return -1;
            }
            else if (theP1.Coord(i) > theP2.Coord(i))
            {
                return 1;
            }
        }
        return 0;
    }
}
