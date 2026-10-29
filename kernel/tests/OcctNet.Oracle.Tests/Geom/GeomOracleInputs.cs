#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests.Geom;

/// <summary>Random inputs of the Geom oracle tests, identical on both sides to the last bit.</summary>
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
