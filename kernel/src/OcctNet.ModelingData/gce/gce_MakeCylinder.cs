// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCylinder.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Cylinder: from an axis system and a radius, coaxial to a cylinder through a point or at a
/// signed offset, from three points, from an axis and a radius, or from a circular base.
/// </summary>
public sealed class gce_MakeCylinder : gce_Root
{
    private gp_Cylinder TheCylinder = new();

    /// <summary>A cylinder from an axis system and a radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public gce_MakeCylinder(in gp_Ax2 A2, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheCylinder = new gp_Cylinder(new gp_Ax3(A2), Radius);
            TheError = gce_Done;
        }
    }

    /// <summary>A cylinder from its axis and a radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public gce_MakeCylinder(in gp_Ax1 Axis, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            gp_Dir D = Axis.Direction();
            gp_Dir Direc = new();
            double x = D.X();
            double y = D.Y();
            double z = D.Z();
            if (Math.Abs(x) > gp.Resolution())
            {
                Direc = new gp_Dir(-y, x, 0.0);
            }
            else if (Math.Abs(y) > gp.Resolution())
            {
                Direc = new gp_Dir(0.0, -z, y);
            }
            else if (Math.Abs(z) > gp.Resolution())
            {
                Direc = new gp_Dir(z, 0.0, -x);
            }
            TheCylinder = new gp_Cylinder(new gp_Ax3(new gp_Ax2(Axis.Location(), D, Direc)), Radius);
            TheError = gce_Done;
        }
    }

    /// <summary>A cylinder whose axis and radius are those of a circle.</summary>
    public gce_MakeCylinder(in gp_Circ Circ)
    {
        TheCylinder = new gp_Cylinder(new gp_Ax3(Circ.Position()), Circ.Radius());
        TheError = gce_Done;
    }

    /// <summary>
    /// A cylinder whose axis passes through P1 and P2 and whose radius is the distance from P3 to it; fails with
    /// gce_ConfusedPoints if P1 and P2 coincide.
    /// </summary>
    public gce_MakeCylinder(in gp_Pnt P1, in gp_Pnt P2, in gp_Pnt P3)
    {
        // P1 and P2 define the cylinder axis, the distance from P3 to the axis
        // gives the cylinder radius.
        if (P1.Distance(P2) < gp.Resolution())
        {
            TheError = gce_ConfusedPoints;
        }
        else
        {
            gp_Dir D1 = new(P2.XYZ() - P1.XYZ());
            gp_Dir D2 = new();
            double x = D1.X();
            double y = D1.Y();
            double z = D1.Z();
            if (Math.Abs(x) > gp.Resolution())
            {
                D2 = new gp_Dir(-y, x, 0.0);
            }
            else if (Math.Abs(y) > gp.Resolution())
            {
                D2 = new gp_Dir(0.0, -z, y);
            }
            else if (Math.Abs(z) > gp.Resolution())
            {
                D2 = new gp_Dir(z, 0.0, -x);
            }
            TheCylinder = new gp_Cylinder(new gp_Ax3(new gp_Ax2(P1, D1, D2)), new gp_Lin(P1, D1).Distance(P3));
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// A cylinder coaxial to Cyl at the signed distance Dist; fails with gce_NegativeRadius if the radius would be
    /// negative.
    /// </summary>
    public gce_MakeCylinder(in gp_Cylinder Cyl, double Dist)
    {
        double Rad = Cyl.Radius() + Dist;
        if (Rad < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheCylinder = Cyl;
            TheCylinder.SetRadius(Rad);
            TheError = gce_Done;
        }
    }

    /// <summary>A cylinder coaxial to Cyl through the point P.</summary>
    public gce_MakeCylinder(in gp_Cylinder Cyl, in gp_Pnt P)
    {
        gp_Lin L = new(Cyl.Axis());
        double Rad = L.Distance(P);
        TheCylinder = Cyl;
        TheCylinder.SetRadius(Rad);
        TheError = gce_Done;
    }

    /// <summary>The constructed cylinder.</summary>
    /// <exception cref="StdFail_NotDone">No cylinder was constructed.</exception>
    public gp_Cylinder Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeCylinder::Value() - no result");
        return TheCylinder;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Cylinder Operator() => Value();

    /// <summary>The constructed cylinder (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Cylinder(gce_MakeCylinder theMaker) => theMaker.Operator();
}
