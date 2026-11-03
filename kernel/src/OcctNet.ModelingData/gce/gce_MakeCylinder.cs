// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCylinder.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Cylinder: from an axis system and a radius, coaxial to a cylinder through a point or at a
/// signed offset, from three points, from an axis and a radius, or from a circular base.
/// </summary>
public sealed class gce_MakeCylinder : gce_Root
{
    private Cylinder TheCylinder = new();

    /// <summary>A cylinder from an axis system and a radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public gce_MakeCylinder(in Frame3D A2, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheCylinder = new Cylinder(new CoordinateSystem(A2), Radius);
            TheError = gce_Done;
        }
    }

    /// <summary>A cylinder from its axis and a radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public gce_MakeCylinder(in Axis3D Axis, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            Direction3D D = Axis.Direction();
            Direction3D Direc = new();
            double x = D.X();
            double y = D.Y();
            double z = D.Z();
            if (Math.Abs(x) > Constants.Resolution())
            {
                Direc = new Direction3D(-y, x, 0.0);
            }
            else if (Math.Abs(y) > Constants.Resolution())
            {
                Direc = new Direction3D(0.0, -z, y);
            }
            else if (Math.Abs(z) > Constants.Resolution())
            {
                Direc = new Direction3D(z, 0.0, -x);
            }
            TheCylinder = new Cylinder(new CoordinateSystem(new Frame3D(Axis.Location(), D, Direc)), Radius);
            TheError = gce_Done;
        }
    }

    /// <summary>A cylinder whose axis and radius are those of a circle.</summary>
    public gce_MakeCylinder(in Circle3D Circ)
    {
        TheCylinder = new Cylinder(new CoordinateSystem(Circ.Position()), Circ.Radius());
        TheError = gce_Done;
    }

    /// <summary>
    /// A cylinder whose axis passes through P1 and P2 and whose radius is the distance from P3 to it; fails with
    /// gce_ConfusedPoints if P1 and P2 coincide.
    /// </summary>
    public gce_MakeCylinder(in Point3D P1, in Point3D P2, in Point3D P3)
    {
        // P1 and P2 define the cylinder axis, the distance from P3 to the axis
        // gives the cylinder radius.
        if (P1.Distance(P2) < Constants.Resolution())
        {
            TheError = gce_ConfusedPoints;
        }
        else
        {
            Direction3D D1 = new(P2.XYZ() - P1.XYZ());
            Direction3D D2 = new();
            double x = D1.X();
            double y = D1.Y();
            double z = D1.Z();
            if (Math.Abs(x) > Constants.Resolution())
            {
                D2 = new Direction3D(-y, x, 0.0);
            }
            else if (Math.Abs(y) > Constants.Resolution())
            {
                D2 = new Direction3D(0.0, -z, y);
            }
            else if (Math.Abs(z) > Constants.Resolution())
            {
                D2 = new Direction3D(z, 0.0, -x);
            }
            TheCylinder = new Cylinder(new CoordinateSystem(new Frame3D(P1, D1, D2)), new Line3D(P1, D1).Distance(P3));
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// A cylinder coaxial to Cyl at the signed distance Dist; fails with gce_NegativeRadius if the radius would be
    /// negative.
    /// </summary>
    public gce_MakeCylinder(in Cylinder Cyl, double Dist)
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
    public gce_MakeCylinder(in Cylinder Cyl, in Point3D P)
    {
        Line3D L = new(Cyl.Axis());
        double Rad = L.Distance(P);
        TheCylinder = Cyl;
        TheCylinder.SetRadius(Rad);
        TheError = gce_Done;
    }

    /// <summary>The constructed cylinder.</summary>
    /// <exception cref="NotDoneException">No cylinder was constructed.</exception>
    public Cylinder Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeCylinder::Value() - no result");
        return TheCylinder;
    }

    /// <summary>Alias for Value().</summary>
    public Cylinder Operator() => Value();

    /// <summary>The constructed cylinder (OCCT's implicit conversion operator).</summary>
    public static explicit operator Cylinder(gce_MakeCylinder theMaker) => theMaker.Operator();
}
