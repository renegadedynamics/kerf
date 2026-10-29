// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCircle.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCircle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a Geom_Circle: from a gp_Circ, an axis system and a radius, coaxial to a circle at a distance or
/// through a point, through three points, from a center and a normal, a center and a point on the axis, or an axis.
/// </summary>
public sealed class GC_MakeCircle : GC_Root
{
    private Geom_Circle? TheCircle;

    /// <summary>A circle from the gp_Circ C.</summary>
    public GC_MakeCircle(in gp_Circ C)
    {
        TheError = gce_Done;
        TheCircle = new Geom_Circle(C);
    }

    /// <summary>A circle of radius Radius in the plane of A2; fails with gce_NegativeRadius if Radius is negative.</summary>
    public GC_MakeCircle(in gp_Ax2 A2, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheError = gce_Done;
            TheCircle = new Geom_Circle(new gp_Circ(A2, Radius));
        }
    }

    /// <summary>A circle coaxial to Circ through Point.</summary>
    public GC_MakeCircle(in gp_Circ Circ, in gp_Pnt Point)
    {
        // gce_MakeCirc(Circ, Point) always succeeds, so its implicit conversion can't throw.
        gp_Circ C = new gce_MakeCirc(Circ, Point).Value();
        TheCircle = new Geom_Circle(C);
        TheError = gce_Done;
    }

    /// <summary>A circle coaxial to Circ at the signed distance Dist; fails with gce_NegativeRadius.</summary>
    public GC_MakeCircle(in gp_Circ Circ, double Dist)
    {
        gce_MakeCirc C = new gce_MakeCirc(Circ, Dist);
        TheError = C.Status();
        if (TheError == gce_Done)
        {
            TheCircle = new Geom_Circle(C.Value());
        }
    }

    /// <summary>A circle through three points; fails with the status of gce_MakeCirc.</summary>
    public GC_MakeCircle(in gp_Pnt P1, in gp_Pnt P2, in gp_Pnt P3)
    {
        gce_MakeCirc C = new gce_MakeCirc(P1, P2, P3);
        TheError = C.Status();
        if (TheError == gce_Done)
        {
            TheCircle = new Geom_Circle(C.Value());
        }
    }

    /// <summary>A circle of center Point, normal Norm and radius Radius.</summary>
    public GC_MakeCircle(in gp_Pnt Point, in gp_Dir Norm, double Radius)
    {
        gce_MakeCirc C = new gce_MakeCirc(Point, Norm, Radius);
        TheError = C.Status();
        if (TheError == gce_Done)
        {
            TheCircle = new Geom_Circle(C.Value());
        }
    }

    /// <summary>A circle of center Point, with its axis through PtAxis, and radius Radius.</summary>
    public GC_MakeCircle(in gp_Pnt Point, in gp_Pnt PtAxis, double Radius)
    {
        gce_MakeCirc C = new gce_MakeCirc(Point, PtAxis, Radius);
        TheError = C.Status();
        if (TheError == gce_Done)
        {
            TheCircle = new Geom_Circle(C.Value());
        }
    }

    /// <summary>A circle of axis Axis and radius Radius.</summary>
    public GC_MakeCircle(in gp_Ax1 Axis, double Radius)
    {
        gce_MakeCirc C = new gce_MakeCirc(Axis, Radius);
        TheError = C.Status();
        if (TheError == gce_Done)
        {
            TheCircle = new Geom_Circle(C.Value());
        }
    }

    /// <summary>The constructed circle (frozen).</summary>
    /// <exception cref="StdFail_NotDone">No circle was constructed.</exception>
    public Geom_Circle Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeCircle::Value() - no result");
        TheCircle!.Freeze();
        return TheCircle;
    }

    /// <summary>The constructed circle (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_Circle(GC_MakeCircle theMaker) => theMaker.Value();
}
