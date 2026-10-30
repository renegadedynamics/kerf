// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_S.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_S.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An "S" evolution law: a cubic between two values, with given derivatives at the ends.</summary>
/// <remarks>A working object (decision 21): <see cref="Set(double, double, double, double)"/> is public and changes the law.</remarks>
public class Law_S : Law_BSpFunc
{
    /// <summary>An empty S law.</summary>
    public Law_S()
    {
    }

    /// <summary>Defines the law between (Pdeb, Valdeb) and (Pfin, Valfin) with zero derivatives at both ends.</summary>
    public void Set(double Pdeb, double Valdeb, double Pfin, double Valfin)
    {
        Set(Pdeb, Valdeb, 0.0, Pfin, Valfin, 0.0);
    }

    /// <summary>Defines the law between (Pdeb, Valdeb) and (Pfin, Valfin) with the derivatives Ddeb and Dfin there.</summary>
    public void Set(double Pdeb, double Valdeb, double Ddeb, double Pfin, double Valfin, double Dfin)
    {
        NCollection_Array1<double> poles = new NCollection_Array1<double>(1, 4);
        NCollection_Array1<double> knots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> mults = new NCollection_Array1<int>(1, 2);
        poles[1] = Valdeb;
        poles[4] = Valfin;
        double coe = (Pfin - Pdeb) / 3.0;
        poles[2] = Valdeb + coe * Ddeb;
        poles[3] = Valfin - coe * Dfin;
        knots[1] = Pdeb;
        knots[2] = Pfin;
        mults[1] = mults[2] = 4;
        SetCurve(new Law_BSpline(poles, knots, mults, 3));
    }
}
