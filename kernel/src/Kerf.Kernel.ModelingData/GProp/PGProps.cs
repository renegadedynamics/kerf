// Port of: src/ModelingData/TKGeomBase/GProp/GProp_PGProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_PGProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Measurement;

/// <summary>
/// Global properties (mass, barycentre, matrix of inertia) of a weighted set of 3D points, accumulated with
/// <see cref="AddPoint(in Point3D)"/> or from arrays. Each point has unit mass unless a density is given.
/// </summary>
/// <remarks>Inertia is accumulated at the absolute origin.</remarks>
internal class PGProps : Properties
{
    /// <summary>An empty point set, at the origin, with zero mass.</summary>
    public PGProps()
    {
        _g = Constants.Origin();
        _loc = Constants.Origin();
        _dim = 0.0;
        _inertia = new Matrix3x3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /// <summary>A point set from an array of points of unit mass.</summary>
    public PGProps(NCollection_Array1<Point3D> pnts)
        : this()
    {
        for (int i = pnts.Lower(); i <= pnts.Upper(); ++i)
        {
            AddPoint(pnts[i]);
        }
    }

    /// <summary>A point set from a 2D array of points of unit mass.</summary>
    public PGProps(NCollection_Array2<Point3D> pnts)
        : this()
    {
        for (int j = pnts.LowerCol(); j <= pnts.UpperCol(); ++j)
        {
            for (int i = pnts.LowerRow(); i <= pnts.UpperRow(); ++i)
            {
                AddPoint(pnts[i, j]);
            }
        }
    }

    /// <summary>A point set from points and their masses.</summary>
    /// <exception cref="DomainErrorException">The arrays differ in length, or a density is not positive.</exception>
    public PGProps(NCollection_Array1<Point3D> pnts, NCollection_Array1<double> density)
        : this()
    {
        if (pnts.Length() != density.Length())
        {
            throw new DomainErrorException("GProp_PGProps: points and density arrays have different lengths");
        }
        int pntIdx = pnts.Lower();
        int densIdx = density.Lower();
        for (; pntIdx <= pnts.Upper(); ++pntIdx, ++densIdx)
        {
            AddPoint(pnts[pntIdx], density[densIdx]);
        }
    }

    /// <summary>A point set from 2D arrays of points and their masses.</summary>
    /// <exception cref="DomainErrorException">The arrays differ in size, or a density is not positive.</exception>
    public PGProps(NCollection_Array2<Point3D> pnts, NCollection_Array2<double> density)
        : this()
    {
        if (pnts.ColLength() != density.ColLength()
            || pnts.RowLength() != density.RowLength())
        {
            throw new DomainErrorException(
              "GProp_PGProps: points and density arrays have different dimensions");
        }
        int pntCol = pnts.LowerCol();
        int densCol = density.LowerCol();
        for (; pntCol <= pnts.UpperCol(); ++pntCol, ++densCol)
        {
            int pntRow = pnts.LowerRow();
            int densRow = density.LowerRow();
            for (; pntRow <= pnts.UpperRow(); ++pntRow, ++densRow)
            {
                AddPoint(pnts[pntRow, pntCol], density[densRow, densCol]);
            }
        }
    }

    /// <summary>Adds a point of unit mass.</summary>
    public void AddPoint(in Point3D pnt)
    {
        double x = pnt.X;
        double y = pnt.Y;
        double z = pnt.Z;

        var mp = new Matrix3x3(new Xyz(y * y + z * z, -x * y, -x * z),
                             new Xyz(-x * y, x * x + z * z, -y * z),
                             new Xyz(-x * z, -y * z, x * x + y * y));

        if (_dim == 0.0)
        {
            _dim = 1.0;
            _g = pnt;
            _inertia = mp;
        }
        else
        {
            double newMass = _dim + 1.0;
            double newX = (_g.X * _dim + x) / newMass;
            double newY = (_g.Y * _dim + y) / newMass;
            double newZ = (_g.Z * _dim + z) / newMass;
            _g = _g.WithCoord(newX, newY, newZ);
            _dim = newMass;
            _inertia = _inertia + mp;
        }
    }

    /// <summary>Adds a point of mass <paramref name="density"/>.</summary>
    /// <exception cref="DomainErrorException">theDensity is not greater than gp::Resolution().</exception>
    public void AddPoint(in Point3D pnt, double density)
    {
        if (density <= Constants.Resolution())
        {
            throw new DomainErrorException("GProp_PGProps::AddPoint: density must be positive");
        }

        double x = pnt.X;
        double y = pnt.Y;
        double z = pnt.Z;

        var mp = new Matrix3x3(new Xyz(y * y + z * z, -x * y, -x * z),
                             new Xyz(-x * y, x * x + z * z, -y * z),
                             new Xyz(-x * z, -y * z, x * x + y * y));

        if (_dim == 0.0)
        {
            _dim = density;
            _g = pnt;
            _inertia = mp * density;
        }
        else
        {
            double newMass = _dim + density;
            double newX = (_g.X * _dim + x * density) / newMass;
            double newY = (_g.Y * _dim + y * density) / newMass;
            double newZ = (_g.Z * _dim + z * density) / newMass;
            _g = _g.WithCoord(newX, newY, newZ);
            _dim = newMass;
            _inertia = _inertia + mp * density;
        }
    }

    /// <summary>The barycentre of a set of points of unit mass.</summary>
    public static Point3D Barycentre(NCollection_Array1<Point3D> pnts)
    {
        Xyz sum = pnts[pnts.Lower()].XYZ();
        for (int i = pnts.Lower() + 1; i <= pnts.Upper(); ++i)
        {
            sum.Add(pnts[i].XYZ());
        }
        sum.Divide(pnts.Length());
        return new Point3D(sum);
    }

    /// <summary>The barycentre of a 2D array of points of unit mass.</summary>
    public static Point3D Barycentre(NCollection_Array2<Point3D> pnts)
    {
        var sum = new Xyz(0.0, 0.0, 0.0);
        for (int j = pnts.LowerCol(); j <= pnts.UpperCol(); ++j)
        {
            for (int i = pnts.LowerRow(); i <= pnts.UpperRow(); ++i)
            {
                sum.Add(pnts[i, j].XYZ());
            }
        }
        sum.Divide(pnts.RowLength() * pnts.ColLength());
        return new Point3D(sum);
    }

    /// <summary>The total mass and the weighted barycentre of a set of points.</summary>
    /// <exception cref="DimensionErrorException">The arrays differ in length.</exception>
    public static void Barycentre(NCollection_Array1<Point3D> pnts, NCollection_Array1<double> density,
                                  out double mass, out Point3D g)
    {
        if (pnts.Length() != density.Length())
        {
            throw new DimensionErrorException(
              "GProp_PGProps::Barycentre: points and density arrays have different lengths");
        }

        int pntIdx = pnts.Lower();
        int densIdx = density.Lower();
        mass = density[densIdx];
        Xyz sum = pnts[pntIdx].XYZ();
        sum.Multiply(mass);
        ++pntIdx;
        ++densIdx;
        for (; pntIdx <= pnts.Upper(); ++pntIdx, ++densIdx)
        {
            mass += density[densIdx];
            sum.Add(pnts[pntIdx].XYZ().Multiplied(density[densIdx]));
        }
        sum.Divide(mass);
        g = new Point3D(sum);
    }

    /// <summary>The total mass and the weighted barycentre of a 2D array of points.</summary>
    /// <exception cref="DimensionErrorException">The arrays differ in size.</exception>
    public static void Barycentre(NCollection_Array2<Point3D> pnts, NCollection_Array2<double> density,
                                  out double mass, out Point3D g)
    {
        if (pnts.RowLength() != density.RowLength()
            || pnts.ColLength() != density.ColLength())
        {
            throw new DimensionErrorException(
              "GProp_PGProps::Barycentre: points and density arrays have different dimensions");
        }

        mass = 0.0;
        var sum = new Xyz(0.0, 0.0, 0.0);
        int pntCol = pnts.LowerCol();
        int densCol = density.LowerCol();
        for (; pntCol <= pnts.UpperCol(); ++pntCol, ++densCol)
        {
            int pntRow = pnts.LowerRow();
            int densRow = density.LowerRow();
            for (; pntRow <= pnts.UpperRow(); ++pntRow, ++densRow)
            {
                mass += density[densRow, densCol];
                sum.Add(pnts[pntRow, pntCol].XYZ().Multiplied(density[densRow, densCol]));
            }
        }
        sum.Divide(mass);
        g = new Point3D(sum);
    }
}
