// Port of: src/ModelingData/TKGeomBase/GProp/GProp_PEquation.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_PEquation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Decides whether a set of 3D points is coincident, collinear, coplanar or spans space, within a tolerance, by
/// principal-axis analysis (eigendecomposition of the inertia matrix), and returns the fitted point, line, plane or
/// box. The barycentre, principal axes and extents are available whatever the result.
/// </summary>
/// <remarks>Computed in the constructor and immutable afterwards.</remarks>
public sealed class GProp_PEquation
{
    /// <summary>Type of geometric entity that best fits the cloud.</summary>
    public enum Type
    {
        /// <summary>Not computed (the eigendecomposition failed).</summary>
        None,

        /// <summary>All points are coincident within tolerance.</summary>
        Point,

        /// <summary>Points are collinear within tolerance.</summary>
        Line,

        /// <summary>Points are coplanar within tolerance.</summary>
        Plane,

        /// <summary>Points span 3D space.</summary>
        Space,
    }

    private readonly Type myType = Type.None;
    private readonly Point3D myG;  // Type-specific point (mean point / box corner)
    private readonly Vector3D myV1; // Plane normal / line direction / box edge 1
    private readonly Vector3D myV2; // Box edge 2 (Space type only)
    private readonly Vector3D myV3; // Box edge 3 (Space type only)

    private readonly Point3D myBarycentre;       // Centre of mass, always valid
    private readonly Vector3D[] myAxes = new Vector3D[3]; // Principal axis unit vectors
    private readonly double[] myExtents = new double[3]; // Extent along each principal axis

    /// <summary>Analyses <paramref name="thePnts"/> with the tolerance <paramref name="theTol"/>.</summary>
    public GProp_PEquation(NCollection_Array1<Point3D> thePnts, double theTol)
    {
        var aProps = new GProp_PGProps(thePnts);
        myBarycentre = aProps.CentreOfMass();
        myG = myBarycentre;

        Matrix3x3 anInertia = aProps.MatrixOfInertia();

        var aMat = new Matrix(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aMat[i, j] = anInertia.Value(i, j);
            }
        }

        global::Kerf.Engine.Numerics.Utilities.MathUtils.EigenResult anEigen = global::Kerf.Engine.Numerics.Linear.MathLin.Jacobi(aMat, true);
        if (!anEigen.IsDone)
        {
            myType = Type.None;
            myAxes[0] = new Vector3D(1.0, 0.0, 0.0);
            myAxes[1] = new Vector3D(0.0, 1.0, 0.0);
            myAxes[2] = new Vector3D(0.0, 0.0, 1.0);
            myExtents[0] = 0.0;
            myExtents[1] = 0.0;
            myExtents[2] = 0.0;
            return;
        }

        Matrix aVecs = anEigen.EigenVectors!;
        myAxes[0] = new Vector3D(aVecs[1, 1], aVecs[2, 1], aVecs[3, 1]);
        myAxes[1] = new Vector3D(aVecs[1, 2], aVecs[2, 2], aVecs[3, 2]);
        myAxes[2] = new Vector3D(aVecs[1, 3], aVecs[2, 3], aVecs[3, 3]);

        double aXg = myG.X();
        double aYg = myG.Y();
        double aZg = myG.Z();

        double aMax1 = RealFirst(), aMin1 = RealLast();
        double aMax2 = RealFirst(), aMin2 = RealLast();
        double aMax3 = RealFirst(), aMin3 = RealLast();

        for (int i = thePnts.Lower(); i <= thePnts.Upper(); ++i)
        {
            double aDX = thePnts[i].X() - aXg;
            double aDY = thePnts[i].Y() - aYg;
            double aDZ = thePnts[i].Z() - aZg;

            double aD1 = aDX * myAxes[0].X() + aDY * myAxes[0].Y() + aDZ * myAxes[0].Z();
            if (aD1 > aMax1)
            {
                aMax1 = aD1;
            }
            if (aD1 < aMin1)
            {
                aMin1 = aD1;
            }

            double aD2 = aDX * myAxes[1].X() + aDY * myAxes[1].Y() + aDZ * myAxes[1].Z();
            if (aD2 > aMax2)
            {
                aMax2 = aD2;
            }
            if (aD2 < aMin2)
            {
                aMin2 = aD2;
            }

            double aD3 = aDX * myAxes[2].X() + aDY * myAxes[2].Y() + aDZ * myAxes[2].Z();
            if (aD3 > aMax3)
            {
                aMax3 = aD3;
            }
            if (aD3 < aMin3)
            {
                aMin3 = aD3;
            }
        }

        myExtents[0] = aMax1 - aMin1;
        myExtents[1] = aMax2 - aMin2;
        myExtents[2] = aMax3 - aMin3;

        int aDimension = 3;
        int aDimCode = 0;
        if (Math.Abs(myExtents[0]) <= theTol)
        {
            --aDimension;
            aDimCode = 1;
        }
        if (Math.Abs(myExtents[1]) <= theTol)
        {
            --aDimension;
            aDimCode = 2 * (aDimCode + 1);
        }
        if (Math.Abs(myExtents[2]) <= theTol)
        {
            --aDimension;
            aDimCode = 3 * (aDimCode + 1);
        }

        switch (aDimension)
        {
            case 0:
                {
                    myType = Type.Point;
                    break;
                }
            case 1:
                {
                    myType = Type.Line;
                    if (aDimCode == 4)
                    {
                        myV1 = myAxes[2];
                    }
                    else if (aDimCode == 6)
                    {
                        myV1 = myAxes[1];
                    }
                    else
                    {
                        myV1 = myAxes[0];
                    }
                    break;
                }
            case 2:
                {
                    myType = Type.Plane;
                    if (aDimCode == 1)
                    {
                        myV1 = myAxes[0];
                    }
                    else if (aDimCode == 2)
                    {
                        myV1 = myAxes[1];
                    }
                    else
                    {
                        myV1 = myAxes[2];
                    }
                    break;
                }
            case 3:
                {
                    myType = Type.Space;
                    myG.SetXYZ(myG.XYZ() + aMin1 * myAxes[0].XYZ() + aMin2 * myAxes[1].XYZ()
                               + aMin3 * myAxes[2].XYZ());
                    myV1 = myExtents[0] * myAxes[0];
                    myV2 = myExtents[1] * myAxes[1];
                    myV3 = myExtents[2] * myAxes[2];
                    break;
                }
        }
    }

    /// <summary>The type of the fitted entity.</summary>
    /// <remarks>Hides <see cref="object.GetType"/> to keep OCCT's name.</remarks>
    public new Type GetType() => myType;

    /// <summary>True if the points are coplanar within tolerance.</summary>
    public bool IsPlanar() => myType == Type.Plane;

    /// <summary>True if the points are collinear within tolerance.</summary>
    public bool IsLinear() => myType == Type.Line;

    /// <summary>True if the points are coincident within tolerance.</summary>
    public bool IsPoint() => myType == Type.Point;

    /// <summary>True if the points span 3D space.</summary>
    public bool IsSpace() => myType == Type.Space;

    /// <summary>The mean plane.</summary>
    /// <exception cref="NoSuchObjectException">The points are not planar.</exception>
    public Plane3D Plane()
    {
        if (!IsPlanar())
        {
            throw new NoSuchObjectException("GProp_PEquation::Plane: result is not planar");
        }
        return new Plane3D(myG, new Direction3D(myV1));
    }

    /// <summary>The mean line.</summary>
    /// <exception cref="NoSuchObjectException">The points are not linear.</exception>
    public Line3D Line()
    {
        if (!IsLinear())
        {
            throw new NoSuchObjectException("GProp_PEquation::Line: result is not linear");
        }
        return new Line3D(myG, new Direction3D(myV1));
    }

    /// <summary>The mean point.</summary>
    /// <exception cref="NoSuchObjectException">The points are not coincident.</exception>
    public Point3D Point()
    {
        if (!IsPoint())
        {
            throw new NoSuchObjectException("GProp_PEquation::Point: result is not a point");
        }
        return myG;
    }

    /// <summary>
    /// A box aligned with the principal axes: its corner (minimum projections) and its three edge vectors.
    /// </summary>
    /// <exception cref="NoSuchObjectException">The points do not span space.</exception>
    public void Box(out Point3D theP, out Vector3D theV1, out Vector3D theV2, out Vector3D theV3)
    {
        if (!IsSpace())
        {
            throw new NoSuchObjectException("GProp_PEquation::Box: result is not a 3D space");
        }
        theP = myG;
        theV1 = myV1;
        theV2 = myV2;
        theV3 = myV3;
    }

    /// <summary>The centre of mass of the points (valid whatever the result).</summary>
    public Point3D Barycentre() => myBarycentre;

    /// <summary>The unit principal axis <paramref name="theIndex"/> (1, 2 or 3), ordered by eigenvalue.</summary>
    public Vector3D PrincipalAxis(int theIndex) => myAxes[theIndex - 1];

    /// <summary>The extent (max - min projection) along principal axis <paramref name="theIndex"/> (1, 2 or 3).</summary>
    public double Extent(int theIndex) => myExtents[theIndex - 1];
}
