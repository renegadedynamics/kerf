// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Healing.Checks;

/// <summary>Tools for the shape checks: status lists, their printing, and the numeric precision of curves and surfaces.</summary>
internal static class BRepCheckCheck
{
    /// <summary>
    /// Adds <paramref name="stat"/> to the list unless it is there already; a real error replaces
    /// <see cref="NoError"/>.
    /// </summary>
    public static void Add(NCollection_List<StatusCheck> lst, StatusCheck stat)
    {
        var it = new NCollection_List<StatusCheck>.Iterator(lst);
        while (it.More())
        {
            if (it.Value() == StatusCheck.NoError && stat != StatusCheck.NoError)
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
    public static void Print(StatusCheck stat, TextWriter os)
    {
        switch (stat)
        {
            case StatusCheck.NoError:
                os.Write("BRepCheck_NoError\n");
                break;
            case StatusCheck.InvalidPointOnCurve:
                os.Write("BRepCheck_InvalidPointOnCurve\n");
                break;
            case StatusCheck.InvalidPointOnCurveOnSurface:
                os.Write("BRepCheck_InvalidPointOnCurveOnSurface\n");
                break;
            case StatusCheck.InvalidPointOnSurface:
                os.Write("BRepCheck_InvalidPointOnSurface\n");
                break;
            case StatusCheck.No3DCurve:
                os.Write("BRepCheck_No3DCurve\n");
                break;
            case StatusCheck.Multiple3DCurve:
                os.Write("BRepCheck_Multiple3DCurve\n");
                break;
            case StatusCheck.Invalid3DCurve:
                os.Write("BRepCheck_Invalid3DCurve\n");
                break;
            case StatusCheck.NoCurveOnSurface:
                os.Write("BRepCheck_NoCurveOnSurface\n");
                break;
            case StatusCheck.InvalidCurveOnSurface:
                os.Write("BRepCheck_InvalidCurveOnSurface\n");
                break;
            case StatusCheck.InvalidCurveOnClosedSurface:
                os.Write("BRepCheck_InvalidCurveOnClosedSurface\n");
                break;
            case StatusCheck.InvalidSameRangeFlag:
                os.Write("BRepCheck_InvalidSameRangeFlag\n");
                break;
            case StatusCheck.InvalidSameParameterFlag:
                os.Write("BRepCheck_InvalidSameParameterFlag\n");
                break;
            case StatusCheck.InvalidDegeneratedFlag:
                os.Write("BRepCheck_InvalidDegeneratedFlag\n");
                break;
            case StatusCheck.FreeEdge:
                os.Write("BRepCheck_FreeEdge\n");
                break;
            case StatusCheck.InvalidMultiConnexity:
                os.Write("BRepCheck_InvalidMultiConnexity\n");
                break;
            case StatusCheck.InvalidRange:
                os.Write("BRepCheck_InvalidRange\n");
                break;
            case StatusCheck.EmptyWire:
                os.Write("BRepCheck_EmptyWire\n");
                break;
            case StatusCheck.RedundantEdge:
                os.Write("BRepCheck_RedundantEdge\n");
                break;
            case StatusCheck.SelfIntersectingWire:
                os.Write("BRepCheck_SelfIntersectingWire\n");
                break;
            case StatusCheck.NoSurface:
                os.Write("BRepCheck_NoSurface\n");
                break;
            case StatusCheck.InvalidWire:
                os.Write("BRepCheck_InvalidWire\n");
                break;
            case StatusCheck.RedundantWire:
                os.Write("BRepCheck_RedundantWire\n");
                break;
            case StatusCheck.IntersectingWires:
                os.Write("BRepCheck_IntersectingWires\n");
                break;
            case StatusCheck.InvalidImbricationOfWires:
                os.Write("BRepCheck_InvalidImbricationOfWires\n");
                break;
            case StatusCheck.EmptyShell:
                os.Write("BRepCheck_EmptyShell\n");
                break;
            case StatusCheck.RedundantFace:
                os.Write("BRepCheck_RedundantFace\n");
                break;
            case StatusCheck.UnorientableShape:
                os.Write("BRepCheck_UnorientableShape\n");
                break;
            case StatusCheck.NotClosed:
                os.Write("BRepCheck_NotClosed\n");
                break;
            case StatusCheck.NotConnected:
                os.Write("BRepCheck_NotConnected\n");
                break;
            case StatusCheck.SubshapeNotInShape:
                os.Write("BRepCheck_SubshapeNotInShape\n");
                break;
            case StatusCheck.BadOrientation:
                os.Write("BRepCheck_BadOrientation\n");
                break;
            case StatusCheck.BadOrientationOfSubshape:
                os.Write("BRepCheck_BadOrientationOfSubshape\n");
                break;
            case StatusCheck.CheckFail:
                os.Write("BRepCheck_CheckFail\n");
                break;
            case StatusCheck.InvalidPolygonOnTriangulation:
                os.Write("BRepCheck_InvalidPolygonOnTriangulation\n");
                break;
            case StatusCheck.InvalidToleranceValue:
                os.Write("BRepCheck_InvalidToleranceValue\n");
                break;
            case StatusCheck.InvalidImbricationOfShells:
                os.Write("BRepCheck_InvalidImbricationOfShells\n");
                break;
            case StatusCheck.EnclosedRegion:
                os.Write("BRepCheck_EnclosedRegion\n");
                break;
            default:
                break;
        }
    }

    /// <summary>
    /// True if the wire <paramref name="w"/> intersects itself on the face <paramref name="myFace"/>
    /// (<see cref="WireCheck.SelfIntersect"/>); <paramref name="retE1"/> and <paramref name="retE2"/> then receive
    /// the intersecting edges (<paramref name="retE2"/> unchanged for an edge that intersects itself or has no pcurve).
    /// </summary>
    /// <remarks>The edges are <c>ref</c>: OCCT leaves them unchanged when nothing is found.</remarks>
    public static bool SelfIntersection(in Wire w, in Face myFace, ref Edge retE1, ref Edge retE2)
    {
        var chkw = new WireCheck(w);
        StatusCheck stat = chkw.SelfIntersect(myFace, ref retE1, ref retE2);
        return stat == StatusCheck.SelfIntersectingWire;
    }

    /// <summary>
    /// The precision of the curve's numbers: Epsilon of the largest of an ellipse's centre coordinates and radii,
    /// RealEpsilon() for other curves.
    /// </summary>
    public static double PrecCurve(CurveAdaptorBase aAC3D)
    {
        double xEmax = RealEpsilon();
        //
        CurveKind ct = aAC3D.GetType();
        if (ct == Ellipse)
        {
            Span<double> x = stackalloc double[5];
            //
            Ellipse3D el3D = aAC3D.Ellipse();
            el3D.Location.Coord(out x[0], out x[1], out x[2]);
            x[3] = el3D.MajorRadius;
            x[4] = el3D.MinorRadius;
            xEmax = -1.0;
            for (int i = 0; i < 5; ++i)
            {
                if (x[i] < 0.0)
                {
                    x[i] = -x[i];
                }

                double xe = Epsilon(x[i]);
                if (xe > xEmax)
                {
                    xEmax = xe;
                }
            }
        }

        //
        return xEmax;
    }

    /// <summary>
    /// The precision of the surface's numbers: Epsilon of the largest of a cone's apex coordinates and reference
    /// radius, RealEpsilon() for other surfaces.
    /// </summary>
    public static double PrecSurface(SurfaceAdaptorBase aAHSurf)
    {
        double xEmax = RealEpsilon();
        //
        SurfaceKind st = aAHSurf.GetType();
        if (st == SurfaceKind.Cone)
        {
            Cone cone = aAHSurf.Cone();
            Span<double> x = stackalloc double[4];
            //
            cone.Location.Coord(out x[0], out x[1], out x[2]);
            x[3] = cone.RefRadius;
            xEmax = -1.0;
            for (int i = 0; i < 4; ++i)
            {
                if (x[i] < 0.0)
                {
                    x[i] = -x[i];
                }

                double xe = Epsilon(x[i]);
                if (xe > xEmax)
                {
                    xEmax = xe;
                }
            }
        }

        return xEmax;
    }
}
