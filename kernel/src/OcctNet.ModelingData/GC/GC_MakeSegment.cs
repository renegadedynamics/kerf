// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeSegment.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeSegment.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a line segment in 3D as a Geom_TrimmedCurve: between two points, or on a line between two
/// points or parameters.
/// </summary>
public sealed class GC_MakeSegment : GC_Root
{
    private Geom_TrimmedCurve? TheSegment;

    /// <summary>The segment from P1 to P2, parameterised by length from 0 at P1.</summary>
    /// <exception cref="StdFail_NotDone">
    /// P1 and P2 coincide. OCCT doesn't check: its release build goes on with a null line and crashes.
    /// </exception>
    public GC_MakeSegment(in gp_Pnt P1, in gp_Pnt P2)
    {
        double dist = P1.Distance(P2);
        // OCCT converts GC_MakeLine implicitly; Value() throws here when P1 and P2 coincide (see the exception).
        Geom_Line L = new GC_MakeLine(P1, P2).Value();
        TheSegment = new Geom_TrimmedCurve(L, 0.0, dist, true);
        TheError = gce_Done;
    }

    /// <summary>The segment of Line from the parameter of Point to the parameter U.</summary>
    public GC_MakeSegment(in gp_Lin Line, in gp_Pnt Point, double U)
    {
        double Ufirst = ElCLib.Parameter(Line, Point);
        Geom_Line L = new Geom_Line(Line);
        TheSegment = new Geom_TrimmedCurve(L, Ufirst, U, true);
        TheError = gce_Done;
    }

    /// <summary>The segment of Line between the parameters of P1 and P2.</summary>
    public GC_MakeSegment(in gp_Lin Line, in gp_Pnt P1, in gp_Pnt P2)
    {
        double Ufirst = ElCLib.Parameter(Line, P1);
        double Ulast = ElCLib.Parameter(Line, P2);
        Geom_Line L = new Geom_Line(Line);
        TheSegment = new Geom_TrimmedCurve(L, Ufirst, Ulast, true);
        TheError = gce_Done;
    }

    /// <summary>The segment of Line between the parameters U1 and U2.</summary>
    public GC_MakeSegment(in gp_Lin Line, double U1, double U2)
    {
        Geom_Line L = new Geom_Line(Line);
        TheSegment = new Geom_TrimmedCurve(L, U1, U2, true);
        TheError = gce_Done;
    }

    /// <summary>The constructed segment (frozen).</summary>
    /// <exception cref="StdFail_NotDone">No segment was constructed.</exception>
    public Geom_TrimmedCurve Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeSegment::Value() - no result");
        TheSegment!.Freeze();
        return TheSegment;
    }

    /// <summary>The constructed segment (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_TrimmedCurve(GC_MakeSegment theMaker) => theMaker.Value();
}
