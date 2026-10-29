// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeDir.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeDir.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>Constructions of a gp_Dir: from a vector, coordinates or two points.</summary>
public sealed class gce_MakeDir : gce_Root
{
    private gp_Dir TheDir = new();

    /// <summary>A direction from two points; fails with gce_ConfusedPoints if they are coincident.</summary>
    public gce_MakeDir(in gp_Pnt P1, in gp_Pnt P2)
    {
        if (P1.Distance(P2) <= gp.Resolution())
        {
            TheError = gce_ConfusedPoints;
        }
        else
        {
            TheDir = new gp_Dir(P2.XYZ() - P1.XYZ());
            TheError = gce_Done;
        }
    }

    /// <summary>A direction from a coordinate vector; fails with gce_NullVector if its modulus is too small.</summary>
    public gce_MakeDir(in gp_XYZ Coord)
    {
        if (Coord.Modulus() <= gp.Resolution())
        {
            TheError = gce_NullVector;
        }
        else
        {
            TheDir = new gp_Dir(Coord);
            TheError = gce_Done;
        }
    }

    /// <summary>The normalised vector V; fails with gce_NullVector if its magnitude is too small.</summary>
    public gce_MakeDir(in gp_Vec V)
    {
        if (V.Magnitude() <= gp.Resolution())
        {
            TheError = gce_NullVector;
        }
        else
        {
            TheDir = new gp_Dir(V);
            TheError = gce_Done;
        }
    }

    /// <summary>A direction from its coordinates; fails with gce_NullVector if Xv*Xv + Yv*Yv + Zv*Zv is too small.</summary>
    public gce_MakeDir(double Xv, double Yv, double Zv)
    {
        if (Xv * Xv + Yv * Yv + Zv * Zv <= gp.Resolution())
        {
            TheError = gce_NullVector;
        }
        else
        {
            TheDir = new gp_Dir(Xv, Yv, Zv);
            TheError = gce_Done;
        }
    }

    /// <summary>The constructed direction.</summary>
    /// <exception cref="StdFail_NotDone">No direction was constructed.</exception>
    public gp_Dir Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeDir::Value() - no result");
        return TheDir;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Dir Operator() => Value();

    /// <summary>The constructed direction (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Dir(gce_MakeDir theMaker) => theMaker.Operator();
}
