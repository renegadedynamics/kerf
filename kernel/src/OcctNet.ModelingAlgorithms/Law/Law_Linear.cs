// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Linear.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Linear.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>A linear evolution law.</summary>
/// <remarks>A working object (decision 21): <see cref="Set"/> is public and changes the law.</remarks>
public class Law_Linear : Law_Function
{
    private double valdeb;
    private double valfin;
    private double pdeb;
    private double pfin;

    /// <summary>An empty linear law.</summary>
    public Law_Linear()
    {
        valdeb = 0.0;
        valfin = 0.0;
        pdeb = 0.0;
        pfin = 0.0;
    }

    /// <summary>Defines the law by the parametric bounds Pdeb and Pfin and the values Valdeb and Valfin there.</summary>
    public void Set(double Pdeb, double Valdeb, double Pfin, double Valfin)
    {
        pdeb = Pdeb;
        pfin = Pfin;
        valdeb = Valdeb;
        valfin = Valfin;
    }

    /// <summary>GeomAbs_CN.</summary>
    public override Continuity Continuity() => global::Kerf.Geometry.Continuity.CN;

    /// <summary>1.</summary>
    public override int NbIntervals(Continuity S) => 1;

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        T.SetValue(T.Lower(), pdeb);
        T.SetValue(T.Upper(), pfin);
    }

    public override double Value(double X) => ((X - pdeb) * valfin + (pfin - X) * valdeb) / (pfin - pdeb);

    public override void D1(double X, out double F, out double D)
    {
        F = ((X - pdeb) * valfin + (pfin - X) * valdeb) / (pfin - pdeb);
        D = (valfin - valdeb) / (pfin - pdeb);
    }

    public override void D2(double X, out double F, out double D, out double D2)
    {
        F = ((X - pdeb) * valfin + (pfin - X) * valdeb) / (pfin - pdeb);
        D = (valfin - valdeb) / (pfin - pdeb);
        D2 = 0;
    }

    public override Law_Function Trim(double PFirst, double PLast, double Tol)
    {
        Law_Linear l = new Law_Linear();
        double Vdeb, Vfin;
        Vdeb = ((PFirst - pdeb) * valfin + (pfin - PFirst) * valdeb) / (pfin - pdeb);
        Vfin = ((PLast - pdeb) * valfin + (pfin - PLast) * valdeb) / (pfin - pdeb);
        l.Set(PFirst, Vdeb, PLast, Vfin);
        return l;
    }

    public override void Bounds(out double PFirst, out double PLast)
    {
        PFirst = pdeb;
        PLast = pfin;
    }
}
