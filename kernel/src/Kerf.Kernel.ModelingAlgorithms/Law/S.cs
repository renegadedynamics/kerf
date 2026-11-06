// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_S.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_S.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Construction.Laws;

/// <summary>An "S" evolution law: a cubic between two values, with given derivatives at the ends.</summary>
/// <remarks>A working object (decision 21): <see cref="Set(double, double, double, double)"/> is public and changes the law.</remarks>
internal class S : BSpFunc
{
    /// <summary>An empty S law.</summary>
    public S()
    {
    }

    /// <summary>Defines the law between (Pdeb, Valdeb) and (Pfin, Valfin) with zero derivatives at both ends.</summary>
    public void Set(double pdeb, double valdeb, double pfin, double valfin)
    {
        Set(pdeb, valdeb, 0.0, pfin, valfin, 0.0);
    }

    /// <summary>Defines the law between (Pdeb, Valdeb) and (Pfin, Valfin) with the derivatives Ddeb and Dfin there.</summary>
    public void Set(double pdeb, double valdeb, double ddeb, double pfin, double valfin, double dfin)
    {
        NCollection_Array1<double> poles = new NCollection_Array1<double>(1, 4);
        NCollection_Array1<double> knots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> mults = new NCollection_Array1<int>(1, 2);
        poles[1] = valdeb;
        poles[4] = valfin;
        double coe = (pfin - pdeb) / 3.0;
        poles[2] = valdeb + coe * ddeb;
        poles[3] = valfin - coe * dfin;
        knots[1] = pdeb;
        knots[2] = pfin;
        mults[1] = mults[2] = 4;
        SetCurve(new BSpline(poles, knots, mults, 3));
    }
}
