// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeSegment2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeSegment2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeSegment.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a line segment in 2D as a Geom2d_TrimmedCurve: between two points, from a point and a direction
/// to the projection of a point, or on a line between two points or parameters.
/// </summary>
public sealed class GC_MakeSegment2d : GC_Root
{
    private Geom2d_TrimmedCurve? TheSegment;

    /// <summary>
    /// The segment of the line through P1 with direction V, from P1 to the projection of P2; fails with
    /// gce_ConfusedPoints if that projection is P1.
    /// </summary>
    public GC_MakeSegment2d(in gp_Pnt2d P1, in gp_Dir2d V, in gp_Pnt2d P2)
    {
        gp_Lin2d Line = new gp_Lin2d(P1, V);
        double Ulast = ElCLib.Parameter(Line, P2);
        if (Math.Abs(Ulast) > gp.Resolution())
        {
            Geom2d_Line L = new Geom2d_Line(Line);
            TheSegment = new Geom2d_TrimmedCurve(L, 0.0, Ulast, true);
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_ConfusedPoints;
        }
    }

    /// <summary>The segment from P1 to P2; fails with gce_ConfusedPoints if they coincide.</summary>
    public GC_MakeSegment2d(in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        double dist = P1.Distance(P2);
        if (dist > gp.Resolution())
        {
            // GC_MakeLine2d(P1, P2) succeeds here: it only fails below gp::Resolution().
            Geom2d_Line L = new GC_MakeLine2d(P1, P2).Value();
            TheSegment = new Geom2d_TrimmedCurve(L, 0.0, dist, true);
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_ConfusedPoints;
        }
    }

    /// <summary>The segment of Line from the parameter of Point to the parameter U.</summary>
    public GC_MakeSegment2d(in gp_Lin2d Line, in gp_Pnt2d Point, double U)
    {
        double Ufirst = ElCLib.Parameter(Line, Point);
        Geom2d_Line L = new Geom2d_Line(Line);
        TheSegment = new Geom2d_TrimmedCurve(L, Ufirst, U, true);
        TheError = gce_Done;
    }

    /// <summary>The segment of Line between the parameters of P1 and P2.</summary>
    public GC_MakeSegment2d(in gp_Lin2d Line, in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        double Ufirst = ElCLib.Parameter(Line, P1);
        double Ulast = ElCLib.Parameter(Line, P2);
        Geom2d_Line L = new Geom2d_Line(Line);
        TheSegment = new Geom2d_TrimmedCurve(L, Ufirst, Ulast, true);
        TheError = gce_Done;
    }

    /// <summary>The segment of Line between the parameters U1 and U2.</summary>
    public GC_MakeSegment2d(in gp_Lin2d Line, double U1, double U2)
    {
        Geom2d_Line L = new Geom2d_Line(Line);
        TheSegment = new Geom2d_TrimmedCurve(L, U1, U2, true);
        TheError = gce_Done;
    }

    /// <summary>The constructed segment (frozen).</summary>
    /// <exception cref="StdFail_NotDone">No segment was constructed.</exception>
    public Geom2d_TrimmedCurve Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeSegment2d::Value() - no result");
        TheSegment!.Freeze();
        return TheSegment;
    }

    /// <summary>The constructed segment (OCCT's implicit conversion).</summary>
    public static explicit operator Geom2d_TrimmedCurve(GC_MakeSegment2d theMaker) => theMaker.Value();
}
