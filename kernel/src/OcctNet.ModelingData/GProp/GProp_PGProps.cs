// Port of: src/ModelingData/TKGeomBase/GProp/GProp_PGProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_PGProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Global properties (mass, barycentre, matrix of inertia) of a weighted set of 3D points, accumulated with
/// <see cref="AddPoint(in Point3D)"/> or from arrays. Each point has unit mass unless a density is given.
/// </summary>
/// <remarks>Inertia is accumulated at the absolute origin.</remarks>
public class GProp_PGProps : GProp_GProps
{
    /// <summary>An empty point set, at the origin, with zero mass.</summary>
    public GProp_PGProps()
    {
        g = Constants.Origin();
        loc = Constants.Origin();
        dim = 0.0;
        inertia = new Matrix3x3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /// <summary>A point set from an array of points of unit mass.</summary>
    public GProp_PGProps(NCollection_Array1<Point3D> thePnts)
        : this()
    {
        for (int i = thePnts.Lower(); i <= thePnts.Upper(); ++i)
        {
            AddPoint(thePnts[i]);
        }
    }

    /// <summary>A point set from a 2D array of points of unit mass.</summary>
    public GProp_PGProps(NCollection_Array2<Point3D> thePnts)
        : this()
    {
        for (int j = thePnts.LowerCol(); j <= thePnts.UpperCol(); ++j)
        {
            for (int i = thePnts.LowerRow(); i <= thePnts.UpperRow(); ++i)
            {
                AddPoint(thePnts[i, j]);
            }
        }
    }

    /// <summary>A point set from points and their masses.</summary>
    /// <exception cref="DomainErrorException">The arrays differ in length, or a density is not positive.</exception>
    public GProp_PGProps(NCollection_Array1<Point3D> thePnts, NCollection_Array1<double> theDensity)
        : this()
    {
        if (thePnts.Length() != theDensity.Length())
        {
            throw new DomainErrorException("GProp_PGProps: points and density arrays have different lengths");
        }
        int aPntIdx = thePnts.Lower();
        int aDensIdx = theDensity.Lower();
        for (; aPntIdx <= thePnts.Upper(); ++aPntIdx, ++aDensIdx)
        {
            AddPoint(thePnts[aPntIdx], theDensity[aDensIdx]);
        }
    }

    /// <summary>A point set from 2D arrays of points and their masses.</summary>
    /// <exception cref="DomainErrorException">The arrays differ in size, or a density is not positive.</exception>
    public GProp_PGProps(NCollection_Array2<Point3D> thePnts, NCollection_Array2<double> theDensity)
        : this()
    {
        if (thePnts.ColLength() != theDensity.ColLength()
            || thePnts.RowLength() != theDensity.RowLength())
        {
            throw new DomainErrorException(
              "GProp_PGProps: points and density arrays have different dimensions");
        }
        int aPntCol = thePnts.LowerCol();
        int aDensCol = theDensity.LowerCol();
        for (; aPntCol <= thePnts.UpperCol(); ++aPntCol, ++aDensCol)
        {
            int aPntRow = thePnts.LowerRow();
            int aDensRow = theDensity.LowerRow();
            for (; aPntRow <= thePnts.UpperRow(); ++aPntRow, ++aDensRow)
            {
                AddPoint(thePnts[aPntRow, aPntCol], theDensity[aDensRow, aDensCol]);
            }
        }
    }

    /// <summary>Adds a point of unit mass.</summary>
    public void AddPoint(in Point3D thePnt)
    {
        double aX = thePnt.X();
        double aY = thePnt.Y();
        double aZ = thePnt.Z();

        var aMp = new Matrix3x3(new Xyz(aY * aY + aZ * aZ, -aX * aY, -aX * aZ),
                             new Xyz(-aX * aY, aX * aX + aZ * aZ, -aY * aZ),
                             new Xyz(-aX * aZ, -aY * aZ, aX * aX + aY * aY));

        if (dim == 0.0)
        {
            dim = 1.0;
            g = thePnt;
            inertia = aMp;
        }
        else
        {
            double aNewMass = dim + 1.0;
            double aNewX = (g.X() * dim + aX) / aNewMass;
            double aNewY = (g.Y() * dim + aY) / aNewMass;
            double aNewZ = (g.Z() * dim + aZ) / aNewMass;
            g.SetCoord(aNewX, aNewY, aNewZ);
            dim = aNewMass;
            inertia = inertia + aMp;
        }
    }

    /// <summary>Adds a point of mass <paramref name="theDensity"/>.</summary>
    /// <exception cref="DomainErrorException">theDensity is not greater than gp::Resolution().</exception>
    public void AddPoint(in Point3D thePnt, double theDensity)
    {
        if (theDensity <= Constants.Resolution())
        {
            throw new DomainErrorException("GProp_PGProps::AddPoint: density must be positive");
        }

        double aX = thePnt.X();
        double aY = thePnt.Y();
        double aZ = thePnt.Z();

        var aMp = new Matrix3x3(new Xyz(aY * aY + aZ * aZ, -aX * aY, -aX * aZ),
                             new Xyz(-aX * aY, aX * aX + aZ * aZ, -aY * aZ),
                             new Xyz(-aX * aZ, -aY * aZ, aX * aX + aY * aY));

        if (dim == 0.0)
        {
            dim = theDensity;
            g = thePnt;
            inertia = aMp * theDensity;
        }
        else
        {
            double aNewMass = dim + theDensity;
            double aNewX = (g.X() * dim + aX * theDensity) / aNewMass;
            double aNewY = (g.Y() * dim + aY * theDensity) / aNewMass;
            double aNewZ = (g.Z() * dim + aZ * theDensity) / aNewMass;
            g.SetCoord(aNewX, aNewY, aNewZ);
            dim = aNewMass;
            inertia = inertia + aMp * theDensity;
        }
    }

    /// <summary>The barycentre of a set of points of unit mass.</summary>
    public static Point3D Barycentre(NCollection_Array1<Point3D> thePnts)
    {
        Xyz aSum = thePnts[thePnts.Lower()].XYZ();
        for (int i = thePnts.Lower() + 1; i <= thePnts.Upper(); ++i)
        {
            aSum.Add(thePnts[i].XYZ());
        }
        aSum.Divide(thePnts.Length());
        return new Point3D(aSum);
    }

    /// <summary>The barycentre of a 2D array of points of unit mass.</summary>
    public static Point3D Barycentre(NCollection_Array2<Point3D> thePnts)
    {
        var aSum = new Xyz(0.0, 0.0, 0.0);
        for (int j = thePnts.LowerCol(); j <= thePnts.UpperCol(); ++j)
        {
            for (int i = thePnts.LowerRow(); i <= thePnts.UpperRow(); ++i)
            {
                aSum.Add(thePnts[i, j].XYZ());
            }
        }
        aSum.Divide(thePnts.RowLength() * thePnts.ColLength());
        return new Point3D(aSum);
    }

    /// <summary>The total mass and the weighted barycentre of a set of points.</summary>
    /// <exception cref="DimensionErrorException">The arrays differ in length.</exception>
    public static void Barycentre(NCollection_Array1<Point3D> thePnts, NCollection_Array1<double> theDensity,
                                  out double theMass, out Point3D theG)
    {
        if (thePnts.Length() != theDensity.Length())
        {
            throw new DimensionErrorException(
              "GProp_PGProps::Barycentre: points and density arrays have different lengths");
        }

        int aPntIdx = thePnts.Lower();
        int aDensIdx = theDensity.Lower();
        theMass = theDensity[aDensIdx];
        Xyz aSum = thePnts[aPntIdx].XYZ();
        aSum.Multiply(theMass);
        ++aPntIdx;
        ++aDensIdx;
        for (; aPntIdx <= thePnts.Upper(); ++aPntIdx, ++aDensIdx)
        {
            theMass += theDensity[aDensIdx];
            aSum.Add(thePnts[aPntIdx].XYZ().Multiplied(theDensity[aDensIdx]));
        }
        aSum.Divide(theMass);
        theG = new Point3D(aSum);
    }

    /// <summary>The total mass and the weighted barycentre of a 2D array of points.</summary>
    /// <exception cref="DimensionErrorException">The arrays differ in size.</exception>
    public static void Barycentre(NCollection_Array2<Point3D> thePnts, NCollection_Array2<double> theDensity,
                                  out double theMass, out Point3D theG)
    {
        if (thePnts.RowLength() != theDensity.RowLength()
            || thePnts.ColLength() != theDensity.ColLength())
        {
            throw new DimensionErrorException(
              "GProp_PGProps::Barycentre: points and density arrays have different dimensions");
        }

        theMass = 0.0;
        var aSum = new Xyz(0.0, 0.0, 0.0);
        int aPntCol = thePnts.LowerCol();
        int aDensCol = theDensity.LowerCol();
        for (; aPntCol <= thePnts.UpperCol(); ++aPntCol, ++aDensCol)
        {
            int aPntRow = thePnts.LowerRow();
            int aDensRow = theDensity.LowerRow();
            for (; aPntRow <= thePnts.UpperRow(); ++aPntRow, ++aDensRow)
            {
                theMass += theDensity[aDensRow, aDensCol];
                aSum.Add(thePnts[aPntRow, aPntCol].XYZ().Multiplied(theDensity[aDensRow, aDensCol]));
            }
        }
        aSum.Divide(theMass);
        theG = new Point3D(aSum);
    }
}
