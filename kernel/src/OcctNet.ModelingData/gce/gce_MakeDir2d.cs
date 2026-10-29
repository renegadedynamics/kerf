// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeDir2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeDir2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>Constructions of a gp_Dir2d: from a vector, coordinates or two points.</summary>
public sealed class gce_MakeDir2d : gce_Root
{
    private gp_Dir2d TheDir2d = new();

    /// <summary>A direction from two points; fails with gce_ConfusedPoints if they are coincident.</summary>
    public gce_MakeDir2d(in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        if (P1.Distance(P2) <= gp.Resolution())
        {
            TheError = gce_ConfusedPoints;
        }
        else
        {
            TheDir2d = new gp_Dir2d(P2.XY() - P1.XY());
            TheError = gce_Done;
        }
    }

    /// <summary>A direction from a coordinate vector; fails with gce_NullVector if its modulus is too small.</summary>
    public gce_MakeDir2d(in gp_XY Coord)
    {
        if (Coord.Modulus() <= gp.Resolution())
        {
            TheError = gce_NullVector;
        }
        else
        {
            TheDir2d = new gp_Dir2d(Coord);
            TheError = gce_Done;
        }
    }

    /// <summary>The normalised vector V; fails with gce_NullVector if its magnitude is too small.</summary>
    public gce_MakeDir2d(in gp_Vec2d V)
    {
        if (V.Magnitude() <= gp.Resolution())
        {
            TheError = gce_NullVector;
        }
        else
        {
            TheDir2d = new gp_Dir2d(V);
            TheError = gce_Done;
        }
    }

    /// <summary>A direction from its coordinates; fails with gce_NullVector if Xv*Xv + Yv*Yv is too small.</summary>
    public gce_MakeDir2d(double Xv, double Yv)
    {
        if (Xv * Xv + Yv * Yv <= gp.Resolution())
        {
            TheError = gce_NullVector;
        }
        else
        {
            TheDir2d = new gp_Dir2d(Xv, Yv);
            TheError = gce_Done;
        }
    }

    /// <summary>The constructed direction.</summary>
    /// <exception cref="StdFail_NotDone">No direction was constructed.</exception>
    public gp_Dir2d Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeDir2d::Value() - no result");
        return TheDir2d;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Dir2d Operator() => Value();

    /// <summary>The constructed direction (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Dir2d(gce_MakeDir2d theMaker) => theMaker.Operator();
}
