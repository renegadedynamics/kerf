// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCircle2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCircle2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeCircle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a Geom2d_Circle: from a gp_Circ2d, an axis and a radius, concentric to a circle through a point
/// or at a distance, through three points, or from a center and a radius or a point.
/// </summary>
public sealed class GC_MakeCircle2d : GC_Root
{
    private Geom2d_Circle? TheCircle;

    /// <summary>A circle from the gp_Circ2d C.</summary>
    public GC_MakeCircle2d(in Circle2D C)
    {
        TheError = gce_Done;
        TheCircle = new Geom2d_Circle(C);
    }

    /// <summary>A circle of radius Radius on the axis A; fails with gce_NegativeRadius.</summary>
    public GC_MakeCircle2d(in Axis2D A, double Radius, bool Sense = true)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheError = gce_Done;
            TheCircle = new Geom2d_Circle(A, Radius, Sense);
        }
    }

    /// <summary>A circle of radius Radius in the axis system A; fails with gce_NegativeRadius.</summary>
    public GC_MakeCircle2d(in Frame2D A, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheError = gce_Done;
            TheCircle = new Geom2d_Circle(A, Radius);
        }
    }

    /// <summary>A circle concentric to Circ through Point.</summary>
    public GC_MakeCircle2d(in Circle2D Circ, in Point2D Point)
    {
        gce_MakeCirc2d C = new gce_MakeCirc2d(Circ, Point);
        TheError = C.Status();
        if (TheError == gce_Done)
        {
            TheCircle = new Geom2d_Circle(C.Value());
        }
    }

    /// <summary>A circle concentric to Circ at the signed distance Dist; fails with gce_NegativeRadius.</summary>
    public GC_MakeCircle2d(in Circle2D Circ, double Dist)
    {
        gce_MakeCirc2d C = new gce_MakeCirc2d(Circ, Dist);
        TheError = C.Status();
        if (TheError == gce_Done)
        {
            TheCircle = new Geom2d_Circle(C.Value());
        }
    }

    /// <summary>A circle through three points; fails with the status of gce_MakeCirc2d.</summary>
    public GC_MakeCircle2d(in Point2D P1, in Point2D P2, in Point2D P3)
    {
        gce_MakeCirc2d C = new gce_MakeCirc2d(P1, P2, P3);
        TheError = C.Status();
        if (TheError == gce_Done)
        {
            TheCircle = new Geom2d_Circle(C.Value());
        }
    }

    /// <summary>A circle of center Point and radius Radius; fails with gce_NegativeRadius.</summary>
    public GC_MakeCircle2d(in Point2D Point, double Radius, bool Sense = true)
    {
        gce_MakeCirc2d C = new gce_MakeCirc2d(Point, Radius, Sense);
        TheError = C.Status();
        if (TheError == gce_Done)
        {
            TheCircle = new Geom2d_Circle(C.Value());
        }
    }

    /// <summary>A circle of center Center through Point.</summary>
    public GC_MakeCircle2d(in Point2D Center, in Point2D Point, bool Sense = true)
    {
        gce_MakeCirc2d C = new gce_MakeCirc2d(Center, Point, Sense);
        TheError = C.Status();
        if (TheError == gce_Done)
        {
            TheCircle = new Geom2d_Circle(C.Value());
        }
    }

    /// <summary>The constructed circle (frozen).</summary>
    /// <exception cref="NotDoneException">No circle was constructed.</exception>
    public Geom2d_Circle Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeCircle2d::Value() - no result");
        TheCircle!.Freeze();
        return TheCircle;
    }

    /// <summary>The constructed circle (OCCT's implicit conversion).</summary>
    public static explicit operator Geom2d_Circle(GC_MakeCircle2d theMaker) => theMaker.Value();
}
