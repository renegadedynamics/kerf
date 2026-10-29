#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests.Geom;

/// <summary>Random inputs of the Geom and Geom2d oracle tests, identical on both sides to the last bit.</summary>
/// <remarks>
/// Each function draws the same random numbers as the <see cref="Oracle"/> function it replaces (a point, then
/// direction coordinates until the vector is longer than 0.1), so switching a test to it keeps the test's cases.
/// </remarks>
internal static class GeomOracleInputs
{
    // Directions and axes are built on both sides from the same raw coordinates through the same
    // constructors: Oracle.ToNative(gp_Dir) and ToNative(gp_Ax3) hand native OCCT the port's normalised values,
    // which the native constructors normalise (and orthogonalise) again, a last-bit change of the input.

    internal static gp_XYZ RawDir(Oracle theOracle)
    {
        while (true)
        {
            var aV = new gp_XYZ(theOracle.Real(-1, 1), theOracle.Real(-1, 1), theOracle.Real(-1, 1));
            if (aV.Modulus() > 0.1)
            {
                return aV;
            }
        }
    }

    internal static (N.gp_Dir Native, gp_Dir Port) Dirs(Oracle theOracle)
    {
        gp_XYZ aV = RawDir(theOracle);
        return (new N.gp_Dir(aV.X(), aV.Y(), aV.Z()), new gp_Dir(aV.X(), aV.Y(), aV.Z()));
    }

    internal static (N.gp_Ax1 Native, gp_Ax1 Port) Ax1s(Oracle theOracle)
    {
        gp_Pnt aP = theOracle.Pnt();
        (N.gp_Dir aND, gp_Dir aD) = Dirs(theOracle);
        using (aND)
        using (N.gp_Pnt aNP = Oracle.ToNative(aP))
        {
            return (new N.gp_Ax1(aNP, aND), new gp_Ax1(aP, aD));
        }
    }

    internal static (N.gp_Ax2 Native, gp_Ax2 Port) Ax2s(Oracle theOracle)
    {
        gp_Pnt aP = theOracle.Pnt();
        (N.gp_Dir aNN, gp_Dir aN) = Dirs(theOracle);
        (N.gp_Dir aNX, gp_Dir aX) = Dirs(theOracle);
        using (aNN)
        using (aNX)
        using (N.gp_Pnt aNP = Oracle.ToNative(aP))
        {
            return (new N.gp_Ax2(aNP, aNN, aNX), new gp_Ax2(aP, aN, aX));
        }
    }

    /// <summary>An axis system from a point and its main direction (gp_Ax2(P, V) computes the X direction), as
    /// <see cref="Oracle.Ax2"/>.</summary>
    internal static (N.gp_Ax2 Native, gp_Ax2 Port) Ax2sFromDirection(Oracle theOracle)
    {
        gp_Pnt aP = theOracle.Pnt();
        (N.gp_Dir aND, gp_Dir aD) = Dirs(theOracle);
        using (aND)
        using (N.gp_Pnt aNP = Oracle.ToNative(aP))
        {
            return (new N.gp_Ax2(aNP, aND), new gp_Ax2(aP, aD));
        }
    }

    /// <summary>A coordinate system, left-handed half the time.</summary>
    internal static (N.gp_Ax3 Native, gp_Ax3 Port) Ax3s(Oracle theOracle)
    {
        gp_Pnt aP = theOracle.Pnt();
        (N.gp_Dir aNN, gp_Dir aN) = Dirs(theOracle);
        (N.gp_Dir aNX, gp_Dir aX) = Dirs(theOracle);
        using (aNN)
        using (aNX)
        using (N.gp_Pnt aNP = Oracle.ToNative(aP))
        {
            var aNative = new N.gp_Ax3(aNP, aNN, aNX);
            var aPort = new gp_Ax3(aP, aN, aX);
            if (theOracle.Int(0, 1) == 1)
            {
                aNative.YReverse();
                aPort.YReverse();
            }
            return (aNative, aPort);
        }
    }

    // 2D: the same, for Oracle.Dir2d (which settles the port's direction so that ToNative doesn't change it) and
    // Ax2d.

    internal static gp_XY RawDir2d(Oracle theOracle)
    {
        while (true)
        {
            var aV = new gp_XY(theOracle.Real(-1, 1), theOracle.Real(-1, 1));
            if (aV.Modulus() > 0.1)
            {
                return aV;
            }
        }
    }

    internal static (N.gp_Dir2d Native, gp_Dir2d Port) Dir2ds(Oracle theOracle)
    {
        gp_XY aV = RawDir2d(theOracle);
        return (new N.gp_Dir2d(aV.X(), aV.Y()), new gp_Dir2d(aV.X(), aV.Y()));
    }

    internal static (N.gp_Ax2d Native, gp_Ax2d Port) Ax2ds(Oracle theOracle)
    {
        gp_Pnt2d aP = theOracle.Pnt2d();
        (N.gp_Dir2d aND, gp_Dir2d aD) = Dir2ds(theOracle);
        using (aND)
        using (N.gp_Pnt2d aNP = Oracle.ToNative(aP))
        {
            return (new N.gp_Ax2d(aNP, aND), new gp_Ax2d(aP, aD));
        }
    }

    internal static (N.gp_Trsf Native, gp_Trsf Port) RandomTrsf(Oracle theOracle)
    {
        var aNative = new N.gp_Trsf();
        switch (theOracle.Int(0, 3))
        {
            case 0:
            {
                (N.gp_Ax1 aNAxis, gp_Ax1 anAxis) = Ax1s(theOracle);
                using var aNAxisOwner = aNAxis;
                double anAngle = theOracle.Angle();
                aNative.SetRotation(aNAxis, anAngle);
                return (aNative, gp_Trsf.Rotation(anAxis, anAngle));
            }
            case 1:
            {
                gp_Vec aV = theOracle.Vec();
                using var aNV = Oracle.ToNative(aV);
                aNative.SetTranslation(aNV);
                return (aNative, gp_Trsf.Translation(aV));
            }
            case 2:
            {
                gp_Pnt aP = theOracle.Pnt();
                double aS = theOracle.Int(0, 1) == 0 ? theOracle.Real(0.2, 3) : -theOracle.Real(0.2, 3);
                using var aNP = Oracle.ToNative(aP);
                aNative.SetScale(aNP, aS);
                return (aNative, gp_Trsf.Scale(aP, aS));
            }
            default:
            {
                (N.gp_Ax2 aNAxes, gp_Ax2 anAxes) = Ax2s(theOracle);
                using var aNAxesOwner = aNAxes;
                aNative.SetMirror(aNAxes);
                return (aNative, gp_Trsf.Mirror(anAxes));
            }
        }
    }
}
#endif
