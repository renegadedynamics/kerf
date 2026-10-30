// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// ElCLib.Parameter as GccAna's solvers get it from OCCT's release build. Several solvers ask for the parameter of a
/// point at the centre of a circle (an unset tangency point of a solution equal to its argument, or a tangency point
/// OCCT computes wrongly at the centre). ElCLib then normalises a null vector, which OCCT's release build (No_Exception)
/// does without a check, returning NaN, where the port's gp_Dir2d throws (decision 9). This returns NaN there, and
/// otherwise what ElCLib returns, so the solvers report what native OCCT reports.
/// </summary>
internal static class GccAna_ElCLib
{
    internal static double Parameter(in gp_Circ2d C, in gp_Pnt2d P)
    {
        gp_Vec2d aV = new(C.Location(), P);
        double aSqMod = aV.SquareMagnitude();
        if (aSqMod == 0.0)
        {
            return double.NaN; // OCCT: 0 / 0 in gp_Dir2d
        }
        if (aSqMod <= gp.Resolution() * gp.Resolution())
        {
            // OCCT's release build normalises vectors below gp's resolution too: give ElCLib the unit vector
            aV.Divide(Math.Sqrt(aSqMod));
            return ElCLib.Parameter(C, new gp_Pnt2d(C.Location().XY() + aV.XY()));
        }
        return ElCLib.Parameter(C, P);
    }

    internal static double Parameter(in gp_Lin2d L, in gp_Pnt2d P) => ElCLib.Parameter(L, P);
}
