// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeDir.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeDir.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>Constructions of a gp_Dir: from a vector, coordinates or two points.</summary>
public sealed class gce_MakeDir : gce_Root
{
    private Direction3D TheDir = new();

    /// <summary>A direction from two points; fails with gce_ConfusedPoints if they are coincident.</summary>
    public gce_MakeDir(in Point3D P1, in Point3D P2)
    {
        if (P1.Distance(P2) <= Constants.Resolution())
        {
            TheError = gce_ConfusedPoints;
        }
        else
        {
            TheDir = new Direction3D(P2.XYZ() - P1.XYZ());
            TheError = gce_Done;
        }
    }

    /// <summary>A direction from a coordinate vector; fails with gce_NullVector if its modulus is too small.</summary>
    public gce_MakeDir(in Xyz Coord)
    {
        if (Coord.Modulus() <= Constants.Resolution())
        {
            TheError = gce_NullVector;
        }
        else
        {
            TheDir = new Direction3D(Coord);
            TheError = gce_Done;
        }
    }

    /// <summary>The normalised vector V; fails with gce_NullVector if its magnitude is too small.</summary>
    public gce_MakeDir(in Vector3D V)
    {
        if (V.Magnitude() <= Constants.Resolution())
        {
            TheError = gce_NullVector;
        }
        else
        {
            TheDir = new Direction3D(V);
            TheError = gce_Done;
        }
    }

    /// <summary>A direction from its coordinates; fails with gce_NullVector if Xv*Xv + Yv*Yv + Zv*Zv is too small.</summary>
    public gce_MakeDir(double Xv, double Yv, double Zv)
    {
        if (Xv * Xv + Yv * Yv + Zv * Zv <= Constants.Resolution())
        {
            TheError = gce_NullVector;
        }
        else
        {
            TheDir = new Direction3D(Xv, Yv, Zv);
            TheError = gce_Done;
        }
    }

    /// <summary>The constructed direction.</summary>
    /// <exception cref="NotDoneException">No direction was constructed.</exception>
    public Direction3D Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeDir::Value() - no result");
        return TheDir;
    }

    /// <summary>Alias for Value().</summary>
    public Direction3D Operator() => Value();

    /// <summary>The constructed direction (OCCT's implicit conversion operator).</summary>
    public static explicit operator Direction3D(gce_MakeDir theMaker) => theMaker.Operator();
}
