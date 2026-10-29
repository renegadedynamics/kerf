// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Geom Library: basic computations on the geometry of Geom and Geom2d (conversions to 3D, range changes,
/// extensions, inertia axes, closure checks, ...).
/// </summary>
public static partial class GeomLib
{
    /// <summary>
    /// Fuses the sorted intervals Interval1 and Interval2 into Fusion. Values closer than Confusion are merged: to
    /// their mean, or to the value of Interval1 when IsAdjustToFirstInterval is true.
    /// </summary>
    public static void FuseIntervals(NCollection_Array1<double> I1,
                                     NCollection_Array1<double> I2,
                                     NCollection_Sequence<double> Seq,
                                     double Epspar = 1.0e-9,
                                     bool IsAdjustToFirstInterval = false)
    {
        int ind1 = 1, ind2 = 1;
        double v1, v2;
        // Initializations: IND1 and IND2 point to the 1st element
        // of each of the 2 tables to process. INDS points to the last
        // created element of TABSOR

        //--- Fill TABSOR by traversing TABLE1 and TABLE2 simultaneously ---
        //------------------ eliminating multiple occurrences ----------------

        while ((ind1 <= I1.Upper()) && (ind2 <= I2.Upper()))
        {
            v1 = I1[ind1];
            v2 = I2[ind2];
            if (Math.Abs(v1 - v2) <= Epspar)
            {
                // Here the elements of I1 and I2 match.
                if (IsAdjustToFirstInterval)
                {
                    Seq.Append(v1);
                }
                else
                {
                    Seq.Append((v1 + v2) / 2);
                }
                ind1++;
                ind2++;
            }
            else if (v1 < v2)
            {
                // Here the element of I1 is suitable.
                Seq.Append(v1);
                ind1++;
            }
            else
            {
                // Here the element of TABLE2 is suitable.
                Seq.Append(v2);
                ind2++;
            }
        }

        if (ind1 > I1.Upper())
        {
            //----- Here I1 is exhausted, complete with the end of TABLE2 -------

            for (; ind2 <= I2.Upper(); ind2++)
            {
                Seq.Append(I2[ind2]);
            }
        }

        if (ind2 > I2.Upper())
        {
            //----- Here I2 is exhausted, complete with the end of I1 -------
            for (; ind1 <= I1.Upper(); ind1++)
            {
                Seq.Append(I1[ind1]);
            }
        }
    }

    /// <summary>
    /// Computes the principal axes of inertia of some points and their dispersions: XDir is the axis of largest
    /// inertia, YDir the next, and Xgap, Ygap, Zgap the square roots of the three eigenvalues.
    /// </summary>
    public static void Inertia(NCollection_Array1<gp_Pnt> Points,
                               out gp_Pnt Bary,
                               out gp_Dir XDir,
                               out gp_Dir YDir,
                               out double Xgap,
                               out double Ygap,
                               out double Zgap)
    {
        gp_XYZ GB = new(0.0, 0.0, 0.0), Diff = new();
        //  gp_Vec A,B,C,D;

        int i, nb = Points.Length();
        GB.SetCoord(0.0, 0.0, 0.0);
        for (i = 1; i <= nb; i++)
        {
            GB += Points[i].XYZ();
        }

        GB /= nb;

        math_Matrix M = new(1, 3, 1, 3);
        M.Init(0.0);
        for (i = 1; i <= nb; i++)
        {
            Diff.SetLinearForm(-1, Points[i].XYZ(), GB);
            M[1, 1] += Diff.X() * Diff.X();
            M[2, 2] += Diff.Y() * Diff.Y();
            M[3, 3] += Diff.Z() * Diff.Z();
            M[1, 2] += Diff.X() * Diff.Y();
            M[1, 3] += Diff.X() * Diff.Z();
            M[2, 3] += Diff.Y() * Diff.Z();
        }

        M[2, 1] = M[1, 2];
        M[3, 1] = M[1, 3];
        M[3, 2] = M[2, 3];

        M /= nb;

        math_Jacobi J = new(M);
        // OCCT reports a failure of Jacobi in debug builds only.

        double n1, n2, n3;

        n1 = J.Value(1);
        n2 = J.Value(2);
        n3 = J.Value(3);

        double r1 = Min(Min(n1, n2), n3), r2;
        int m1, m2, m3;
        if (r1 == n1)
        {
            m1 = 1;
            r2 = Min(n2, n3);
            if (r2 == n2)
            {
                m2 = 2;
                m3 = 3;
            }
            else
            {
                m2 = 3;
                m3 = 2;
            }
        }
        else
        {
            if (r1 == n2)
            {
                m1 = 2;
                r2 = Min(n1, n3);
                if (r2 == n1)
                {
                    m2 = 1;
                    m3 = 3;
                }
                else
                {
                    m2 = 3;
                    m3 = 1;
                }
            }
            else
            {
                m1 = 3;
                r2 = Min(n1, n2);
                if (r2 == n1)
                {
                    m2 = 1;
                    m3 = 2;
                }
                else
                {
                    m2 = 2;
                    m3 = 1;
                }
            }
        }

        math_Vector V2 = new(1, 3), V3 = new(1, 3);
        J.Vector(m2, V2);
        J.Vector(m3, V3);

        Bary = new gp_Pnt();
        Bary.SetXYZ(GB);
        XDir = new gp_Dir();
        XDir.SetCoord(V3[1], V3[2], V3[3]);
        YDir = new gp_Dir();
        YDir.SetCoord(V2[1], V2[2], V2[3]);

        Zgap = Math.Sqrt(Math.Abs(J.Value(m1)));
        Ygap = Math.Sqrt(Math.Abs(J.Value(m2)));
        Xgap = Math.Sqrt(Math.Abs(J.Value(m3)));
    }

    /// <summary>
    /// Computes the axes of inertia of some points: the location is their barycentre, the X direction the axis of
    /// largest inertia, the main direction the normal to the mean plane. IsSingular is true if the points are on a
    /// line (within Tol).
    /// </summary>
    public static void AxeOfInertia(NCollection_Array1<gp_Pnt> Points,
                                    out gp_Ax2 Axe,
                                    out bool IsSingular,
                                    double Tol = 1.0e-7)
    {
        gp_Pnt Bary;
        gp_Dir OX, OY, OZ;
        double gx, gy, gz;

        GeomLib.Inertia(Points, out Bary, out OX, out OY, out gx, out gy, out gz);

        if (gy * Points.Length() <= Tol)
        {
            gp_Ax2 axe = new(Bary, OX);
            OY = axe.XDirection();
            IsSingular = true;
        }
        else
        {
            IsSingular = false;
        }

        OZ = OX ^ OY;
        gp_Ax2 TheAxe = new(Bary, OZ, OX);
        Axe = TheAxe;
    }
}
