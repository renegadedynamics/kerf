// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeDir.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeDir.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>Constructions of a gp_Dir: from a vector, coordinates or two points.</summary>
public sealed class MakeDir : Root
{
    private Direction3D _theDir = new();

    /// <summary>A direction from two points; fails with gce_ConfusedPoints if they are coincident.</summary>
    public MakeDir(in Point3D p1, in Point3D p2)
    {
        if (p1.DistanceTo(p2) <= Constants.Resolution())
        {
            _theError = ConfusedPoints;
        }
        else
        {
            _theDir = new Direction3D(p2.XYZ() - p1.XYZ());
            _theError = Done;
        }
    }

    /// <summary>A direction from a coordinate vector; fails with gce_NullVector if its modulus is too small.</summary>
    public MakeDir(in Xyz coord)
    {
        if (coord.Modulus() <= Constants.Resolution())
        {
            _theError = NullVector;
        }
        else
        {
            _theDir = new Direction3D(coord);
            _theError = Done;
        }
    }

    /// <summary>The normalised vector V; fails with gce_NullVector if its magnitude is too small.</summary>
    public MakeDir(in Vector3D v)
    {
        if (v.Magnitude <= Constants.Resolution())
        {
            _theError = NullVector;
        }
        else
        {
            _theDir = new Direction3D(v);
            _theError = Done;
        }
    }

    /// <summary>A direction from its coordinates; fails with gce_NullVector if Xv*Xv + Yv*Yv + Zv*Zv is too small.</summary>
    public MakeDir(double xv, double yv, double zv)
    {
        if (xv * xv + yv * yv + zv * zv <= Constants.Resolution())
        {
            _theError = NullVector;
        }
        else
        {
            _theDir = new Direction3D(xv, yv, zv);
            _theError = Done;
        }
    }

    /// <summary>The constructed direction.</summary>
    /// <exception cref="NotDoneException">No direction was constructed.</exception>
    public Direction3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeDir::Value() - no result");
        return _theDir;
    }

    /// <summary>Alias for Value().</summary>
    public Direction3D Operator() => Value();

    /// <summary>The constructed direction (OCCT's implicit conversion operator).</summary>
    public static explicit operator Direction3D(MakeDir maker) => maker.Operator();
}
