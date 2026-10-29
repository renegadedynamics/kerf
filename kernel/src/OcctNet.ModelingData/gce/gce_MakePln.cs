// Port of: src/ModelingData/TKGeomBase/gce/gce_MakePln.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakePln.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Pln: from an axis system, a point and a normal, a cartesian equation, parallel to a plane
/// through a point or at a signed distance, through three points, through a point normal to the direction to a second
/// point, or through the location of an axis normal to it.
/// </summary>
public sealed class gce_MakePln : gce_Root
{
    private gp_Pln ThePln = new();

    /// <summary>
    /// A plane whose coordinate system is A2: its direction is the normal, its location the origin, and its X and Y
    /// directions parametrize the plane.
    /// </summary>
    public gce_MakePln(in gp_Ax2 A2)
    {
        ThePln = new gp_Pln(new gp_Ax3(A2));
        TheError = gce_Done;
    }

    /// <summary>A plane through P (its location) with normal V.</summary>
    public gce_MakePln(in gp_Pnt P, in gp_Dir V)
    {
        ThePln = new gp_Pln(P, V);
        TheError = gce_Done;
    }

    /// <summary>A plane through P1, normal to P1P2; fails with gce_ConfusedPoints if they coincide.</summary>
    public gce_MakePln(in gp_Pnt P1, in gp_Pnt P2)
    {
        if (P1.Distance(P2) <= gp.Resolution())
        {
            TheError = gce_ConfusedPoints;
        }
        else
        {
            gp_Dir dir = new(P2.XYZ() - P1.XYZ());
            ThePln = new gp_Pln(P1, dir);
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// The plane A * X + B * Y + C * Z + D = 0; fails with gce_BadEquation if A*A + B*B + C*C &lt;= gp::Resolution().
    /// </summary>
    public gce_MakePln(double A, double B, double C, double D)
    {
        if (A * A + B * B + C * C <= gp.Resolution())
        {
            TheError = gce_BadEquation;
        }
        else
        {
            ThePln = new gp_Pln(A, B, C, D);
            TheError = gce_Done;
        }
    }

    /// <summary>A plane through three points; fails with gce_ColinearPoints if they are collinear.</summary>
    public gce_MakePln(in gp_Pnt P1, in gp_Pnt P2, in gp_Pnt P3)
    {
        gp_XYZ V1 = P2.XYZ() - P1.XYZ();
        gp_XYZ V2 = P3.XYZ() - P1.XYZ();
        gp_XYZ Norm = V1.Crossed(V2);
        if (Norm.Modulus() < gp.Resolution())
        {
            TheError = gce_ColinearPoints;
        }
        else
        {
            gp_Dir DNorm = new(Norm);
            gp_Dir Dx = new(V1);
            ThePln = new gp_Pln(new gp_Ax3(P1, DNorm, Dx));
            TheError = gce_Done;
        }
    }

    /// <summary>A plane parallel to Pl at the signed distance Dist (along its normal when positive).</summary>
    public gce_MakePln(in gp_Pln Pl, double Dist)
    {
        gp_Pnt Center = new(Pl.Location().XYZ() + Dist * Pl.Axis().Direction().XYZ());
        ThePln = new gp_Pln(new gp_Ax3(Center, Pl.Axis().Direction(), Pl.XAxis().Direction()));
        TheError = gce_Done;
    }

    /// <summary>A plane parallel to Pl through Point.</summary>
    public gce_MakePln(in gp_Pln Pl, in gp_Pnt Point)
    {
        ThePln = new gp_Pln(new gp_Ax3(Point, Pl.Axis().Direction(), Pl.XAxis().Direction()));
        TheError = gce_Done;
    }

    /// <summary>A plane through the location of Axis, normal to its direction.</summary>
    public gce_MakePln(in gp_Ax1 Axis)
    {
        ThePln = new gp_Pln(Axis.Location(), Axis.Direction());
        TheError = gce_Done;
    }

    /// <summary>The constructed plane.</summary>
    /// <exception cref="StdFail_NotDone">No plane was constructed.</exception>
    public gp_Pln Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakePln::Value() - no result");
        return ThePln;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Pln Operator() => Value();

    /// <summary>The constructed plane (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Pln(gce_MakePln theMaker) => theMaker.Operator();
}
