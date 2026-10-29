// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepCheck_Status;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>Tools for the shape checks: status lists, their printing, and the numeric precision of curves and surfaces.</summary>
public static class BRepCheck
{
    /// <summary>
    /// Adds <paramref name="stat"/> to the list unless it is there already; a real error replaces
    /// <see cref="BRepCheck_NoError"/>.
    /// </summary>
    public static void Add(NCollection_List<BRepCheck_Status> lst, BRepCheck_Status stat)
    {
        var it = new NCollection_List<BRepCheck_Status>.Iterator(lst);
        while (it.More())
        {
            if (it.Value() == BRepCheck_NoError && stat != BRepCheck_NoError)
            {
                lst.Remove(it);
            }
            else
            {
                if (it.Value() == stat)
                {
                    return;
                }

                it.Next();
            }
        }

        lst.Append(stat);
    }

    /// <summary>Writes the name of the status and a new line (nothing for an unknown value).</summary>
    public static void Print(BRepCheck_Status stat, TextWriter OS)
    {
        switch (stat)
        {
            case BRepCheck_NoError:
                OS.Write("BRepCheck_NoError\n");
                break;
            case BRepCheck_InvalidPointOnCurve:
                OS.Write("BRepCheck_InvalidPointOnCurve\n");
                break;
            case BRepCheck_InvalidPointOnCurveOnSurface:
                OS.Write("BRepCheck_InvalidPointOnCurveOnSurface\n");
                break;
            case BRepCheck_InvalidPointOnSurface:
                OS.Write("BRepCheck_InvalidPointOnSurface\n");
                break;
            case BRepCheck_No3DCurve:
                OS.Write("BRepCheck_No3DCurve\n");
                break;
            case BRepCheck_Multiple3DCurve:
                OS.Write("BRepCheck_Multiple3DCurve\n");
                break;
            case BRepCheck_Invalid3DCurve:
                OS.Write("BRepCheck_Invalid3DCurve\n");
                break;
            case BRepCheck_NoCurveOnSurface:
                OS.Write("BRepCheck_NoCurveOnSurface\n");
                break;
            case BRepCheck_InvalidCurveOnSurface:
                OS.Write("BRepCheck_InvalidCurveOnSurface\n");
                break;
            case BRepCheck_InvalidCurveOnClosedSurface:
                OS.Write("BRepCheck_InvalidCurveOnClosedSurface\n");
                break;
            case BRepCheck_InvalidSameRangeFlag:
                OS.Write("BRepCheck_InvalidSameRangeFlag\n");
                break;
            case BRepCheck_InvalidSameParameterFlag:
                OS.Write("BRepCheck_InvalidSameParameterFlag\n");
                break;
            case BRepCheck_InvalidDegeneratedFlag:
                OS.Write("BRepCheck_InvalidDegeneratedFlag\n");
                break;
            case BRepCheck_FreeEdge:
                OS.Write("BRepCheck_FreeEdge\n");
                break;
            case BRepCheck_InvalidMultiConnexity:
                OS.Write("BRepCheck_InvalidMultiConnexity\n");
                break;
            case BRepCheck_InvalidRange:
                OS.Write("BRepCheck_InvalidRange\n");
                break;
            case BRepCheck_EmptyWire:
                OS.Write("BRepCheck_EmptyWire\n");
                break;
            case BRepCheck_RedundantEdge:
                OS.Write("BRepCheck_RedundantEdge\n");
                break;
            case BRepCheck_SelfIntersectingWire:
                OS.Write("BRepCheck_SelfIntersectingWire\n");
                break;
            case BRepCheck_NoSurface:
                OS.Write("BRepCheck_NoSurface\n");
                break;
            case BRepCheck_InvalidWire:
                OS.Write("BRepCheck_InvalidWire\n");
                break;
            case BRepCheck_RedundantWire:
                OS.Write("BRepCheck_RedundantWire\n");
                break;
            case BRepCheck_IntersectingWires:
                OS.Write("BRepCheck_IntersectingWires\n");
                break;
            case BRepCheck_InvalidImbricationOfWires:
                OS.Write("BRepCheck_InvalidImbricationOfWires\n");
                break;
            case BRepCheck_EmptyShell:
                OS.Write("BRepCheck_EmptyShell\n");
                break;
            case BRepCheck_RedundantFace:
                OS.Write("BRepCheck_RedundantFace\n");
                break;
            case BRepCheck_UnorientableShape:
                OS.Write("BRepCheck_UnorientableShape\n");
                break;
            case BRepCheck_NotClosed:
                OS.Write("BRepCheck_NotClosed\n");
                break;
            case BRepCheck_NotConnected:
                OS.Write("BRepCheck_NotConnected\n");
                break;
            case BRepCheck_SubshapeNotInShape:
                OS.Write("BRepCheck_SubshapeNotInShape\n");
                break;
            case BRepCheck_BadOrientation:
                OS.Write("BRepCheck_BadOrientation\n");
                break;
            case BRepCheck_BadOrientationOfSubshape:
                OS.Write("BRepCheck_BadOrientationOfSubshape\n");
                break;
            case BRepCheck_CheckFail:
                OS.Write("BRepCheck_CheckFail\n");
                break;
            case BRepCheck_InvalidPolygonOnTriangulation:
                OS.Write("BRepCheck_InvalidPolygonOnTriangulation\n");
                break;
            case BRepCheck_InvalidToleranceValue:
                OS.Write("BRepCheck_InvalidToleranceValue\n");
                break;
            case BRepCheck_InvalidImbricationOfShells:
                OS.Write("BRepCheck_InvalidImbricationOfShells\n");
                break;
            case BRepCheck_EnclosedRegion:
                OS.Write("BRepCheck_EnclosedRegion\n");
                break;
            default:
                break;
        }
    }

    /// <summary>
    /// True if the wire <paramref name="W"/> intersects itself on the face <paramref name="myFace"/>
    /// (<see cref="BRepCheck_Wire.SelfIntersect"/>); <paramref name="RetE1"/> and <paramref name="RetE2"/> then receive
    /// the intersecting edges (<paramref name="RetE2"/> unchanged for an edge that intersects itself or has no pcurve).
    /// </summary>
    /// <remarks>The edges are <c>ref</c>: OCCT leaves them unchanged when nothing is found.</remarks>
    public static bool SelfIntersection(in TopoDS_Wire W, in TopoDS_Face myFace, ref TopoDS_Edge RetE1, ref TopoDS_Edge RetE2)
    {
        var chkw = new BRepCheck_Wire(W);
        BRepCheck_Status stat = chkw.SelfIntersect(myFace, ref RetE1, ref RetE2);
        return stat == BRepCheck_SelfIntersectingWire;
    }

    /// <summary>
    /// The precision of the curve's numbers: Epsilon of the largest of an ellipse's centre coordinates and radii,
    /// RealEpsilon() for other curves.
    /// </summary>
    public static double PrecCurve(Adaptor3d_Curve aAC3D)
    {
        double aXEmax = RealEpsilon();
        //
        GeomAbs_CurveType aCT = aAC3D.GetType();
        if (aCT == GeomAbs_Ellipse)
        {
            Span<double> aX = stackalloc double[5];
            //
            gp_Elips aEL3D = aAC3D.Ellipse();
            aEL3D.Location().Coord(out aX[0], out aX[1], out aX[2]);
            aX[3] = aEL3D.MajorRadius();
            aX[4] = aEL3D.MinorRadius();
            aXEmax = -1.0;
            for (int i = 0; i < 5; ++i)
            {
                if (aX[i] < 0.0)
                {
                    aX[i] = -aX[i];
                }

                double aXE = Epsilon(aX[i]);
                if (aXE > aXEmax)
                {
                    aXEmax = aXE;
                }
            }
        }

        //
        return aXEmax;
    }

    /// <summary>
    /// The precision of the surface's numbers: Epsilon of the largest of a cone's apex coordinates and reference
    /// radius, RealEpsilon() for other surfaces.
    /// </summary>
    public static double PrecSurface(Adaptor3d_Surface aAHSurf)
    {
        double aXEmax = RealEpsilon();
        //
        GeomAbs_SurfaceType aST = aAHSurf.GetType();
        if (aST == GeomAbs_Cone)
        {
            gp_Cone aCone = aAHSurf.Cone();
            Span<double> aX = stackalloc double[4];
            //
            aCone.Location().Coord(out aX[0], out aX[1], out aX[2]);
            aX[3] = aCone.RefRadius();
            aXEmax = -1.0;
            for (int i = 0; i < 4; ++i)
            {
                if (aX[i] < 0.0)
                {
                    aX[i] = -aX[i];
                }

                double aXE = Epsilon(aX[i]);
                if (aXE > aXEmax)
                {
                    aXEmax = aXE;
                }
            }
        }

        return aXEmax;
    }
}
