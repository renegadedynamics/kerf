// Port of: src/ModelingData/TKGeomBase/GC/GC_MakePlane.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakePlane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a Geom_Plane: from a gp_Pln, a point and a normal, a cartesian equation, three points, parallel
/// to a plane at a distance or through a point, or normal to an axis.
/// </summary>
public sealed class GC_MakePlane : GC_Root
{
    private Geom_Plane? ThePlane;

    /// <summary>A plane from the gp_Pln Pl.</summary>
    public GC_MakePlane(in gp_Pln Pl)
    {
        TheError = gce_Done;
        ThePlane = new Geom_Plane(Pl);
    }

    /// <summary>A plane through P with normal V.</summary>
    public GC_MakePlane(in gp_Pnt P, in gp_Dir V)
    {
        TheError = gce_Done;
        ThePlane = new Geom_Plane(P, V);
    }

    /// <summary>The plane A·X + B·Y + C·Z + D = 0; fails with gce_BadEquation if (A, B, C) is null.</summary>
    public GC_MakePlane(double A, double B, double C, double D)
    {
        if (Math.Sqrt(A * A + B * B + C * C) <= gp.Resolution())
        {
            TheError = gce_BadEquation;
        }
        else
        {
            TheError = gce_Done;
            ThePlane = new Geom_Plane(new gp_Pln(A, B, C, D));
        }
    }

    /// <summary>The plane through three points; fails with the status of gce_MakePln.</summary>
    public GC_MakePlane(in gp_Pnt P1, in gp_Pnt P2, in gp_Pnt P3)
    {
        gce_MakePln Pl = new gce_MakePln(P1, P2, P3);
        TheError = Pl.Status();
        if (TheError == gce_Done)
        {
            ThePlane = new Geom_Plane(Pl.Value());
        }
    }

    /// <summary>A plane parallel to Pl at the signed distance Dist.</summary>
    public GC_MakePlane(in gp_Pln Pl, double Dist)
    {
        gce_MakePln aMaker = new gce_MakePln(Pl, Dist);
        TheError = aMaker.Status();
        if (TheError == gce_Done)
        {
            ThePlane = new Geom_Plane(aMaker.Value());
        }
    }

    /// <summary>A plane parallel to Pl through Point.</summary>
    public GC_MakePlane(in gp_Pln Pl, in gp_Pnt Point)
    {
        gce_MakePln aMaker = new gce_MakePln(Pl, Point);
        TheError = aMaker.Status();
        if (TheError == gce_Done)
        {
            ThePlane = new Geom_Plane(aMaker.Value());
        }
    }

    /// <summary>The plane through the location of Axis, normal to it.</summary>
    public GC_MakePlane(in gp_Ax1 Axis)
    {
        gce_MakePln aMaker = new gce_MakePln(Axis);
        TheError = aMaker.Status();
        if (TheError == gce_Done)
        {
            ThePlane = new Geom_Plane(aMaker.Value());
        }
    }

    /// <summary>The constructed plane (frozen).</summary>
    /// <exception cref="StdFail_NotDone">No plane was constructed.</exception>
    public Geom_Plane Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakePlane::Value() - no result");
        ThePlane!.Freeze();
        return ThePlane;
    }

    /// <summary>The constructed plane (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_Plane(GC_MakePlane theMaker) => theMaker.Value();
}
