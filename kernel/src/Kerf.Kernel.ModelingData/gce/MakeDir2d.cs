// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeDir2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeDir2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>Constructions of a gp_Dir2d: from a vector, coordinates or two points.</summary>
public sealed class MakeDir2d : Root
{
    private Direction2D _theDir2d = new();

    /// <summary>A direction from two points; fails with gce_ConfusedPoints if they are coincident.</summary>
    public MakeDir2d(in Point2D p1, in Point2D p2)
    {
        if (p1.Distance(p2) <= Constants.Resolution())
        {
            _theError = ConfusedPoints;
        }
        else
        {
            _theDir2d = new Direction2D(p2.XY() - p1.XY());
            _theError = Done;
        }
    }

    /// <summary>A direction from a coordinate vector; fails with gce_NullVector if its modulus is too small.</summary>
    public MakeDir2d(in Xy coord)
    {
        if (coord.Modulus() <= Constants.Resolution())
        {
            _theError = NullVector;
        }
        else
        {
            _theDir2d = new Direction2D(coord);
            _theError = Done;
        }
    }

    /// <summary>The normalised vector V; fails with gce_NullVector if its magnitude is too small.</summary>
    public MakeDir2d(in Vector2D v)
    {
        if (v.Magnitude() <= Constants.Resolution())
        {
            _theError = NullVector;
        }
        else
        {
            _theDir2d = new Direction2D(v);
            _theError = Done;
        }
    }

    /// <summary>A direction from its coordinates; fails with gce_NullVector if Xv*Xv + Yv*Yv is too small.</summary>
    public MakeDir2d(double xv, double yv)
    {
        if (xv * xv + yv * yv <= Constants.Resolution())
        {
            _theError = NullVector;
        }
        else
        {
            _theDir2d = new Direction2D(xv, yv);
            _theError = Done;
        }
    }

    /// <summary>The constructed direction.</summary>
    /// <exception cref="NotDoneException">No direction was constructed.</exception>
    public Direction2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeDir2d::Value() - no result");
        return _theDir2d;
    }

    /// <summary>Alias for Value().</summary>
    public Direction2D Operator() => Value();

    /// <summary>The constructed direction (OCCT's implicit conversion operator).</summary>
    public static explicit operator Direction2D(MakeDir2d maker) => maker.Operator();
}
