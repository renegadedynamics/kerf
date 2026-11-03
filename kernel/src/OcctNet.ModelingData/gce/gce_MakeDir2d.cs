// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeDir2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeDir2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>Constructions of a gp_Dir2d: from a vector, coordinates or two points.</summary>
public sealed class gce_MakeDir2d : gce_Root
{
    private Direction2D TheDir2d = new();

    /// <summary>A direction from two points; fails with gce_ConfusedPoints if they are coincident.</summary>
    public gce_MakeDir2d(in Point2D P1, in Point2D P2)
    {
        if (P1.Distance(P2) <= Constants.Resolution())
        {
            TheError = gce_ConfusedPoints;
        }
        else
        {
            TheDir2d = new Direction2D(P2.XY() - P1.XY());
            TheError = gce_Done;
        }
    }

    /// <summary>A direction from a coordinate vector; fails with gce_NullVector if its modulus is too small.</summary>
    public gce_MakeDir2d(in Xy Coord)
    {
        if (Coord.Modulus() <= Constants.Resolution())
        {
            TheError = gce_NullVector;
        }
        else
        {
            TheDir2d = new Direction2D(Coord);
            TheError = gce_Done;
        }
    }

    /// <summary>The normalised vector V; fails with gce_NullVector if its magnitude is too small.</summary>
    public gce_MakeDir2d(in Vector2D V)
    {
        if (V.Magnitude() <= Constants.Resolution())
        {
            TheError = gce_NullVector;
        }
        else
        {
            TheDir2d = new Direction2D(V);
            TheError = gce_Done;
        }
    }

    /// <summary>A direction from its coordinates; fails with gce_NullVector if Xv*Xv + Yv*Yv is too small.</summary>
    public gce_MakeDir2d(double Xv, double Yv)
    {
        if (Xv * Xv + Yv * Yv <= Constants.Resolution())
        {
            TheError = gce_NullVector;
        }
        else
        {
            TheDir2d = new Direction2D(Xv, Yv);
            TheError = gce_Done;
        }
    }

    /// <summary>The constructed direction.</summary>
    /// <exception cref="NotDoneException">No direction was constructed.</exception>
    public Direction2D Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeDir2d::Value() - no result");
        return TheDir2d;
    }

    /// <summary>Alias for Value().</summary>
    public Direction2D Operator() => Value();

    /// <summary>The constructed direction (OCCT's implicit conversion operator).</summary>
    public static explicit operator Direction2D(gce_MakeDir2d theMaker) => theMaker.Operator();
}
