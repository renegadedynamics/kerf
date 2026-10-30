// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Constant.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Constant.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A constant law.</summary>
/// <remarks>A working object (decision 21): <see cref="Set"/> is public and changes the law.</remarks>
public class Law_Constant : Law_Function
{
    private double radius;
    private double first;
    private double last;

    public Law_Constant()
    {
        radius = 0.0;
        first = 0.0;
        last = 0.0;
    }

    /// <summary>Sets the value and the range of the law.</summary>
    public void Set(double Radius, double PFirst, double PLast)
    {
        radius = Radius;
        first = PFirst;
        last = PLast;
    }

    /// <summary>GeomAbs_CN.</summary>
    public override GeomAbs_Shape Continuity() => GeomAbs_Shape.GeomAbs_CN;

    /// <summary>1.</summary>
    public override int NbIntervals(GeomAbs_Shape S) => 1;

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        T.SetValue(T.Lower(), first);
        T.SetValue(T.Upper(), last);
    }

    public override double Value(double X) => radius;

    public override void D1(double X, out double F, out double D)
    {
        F = radius;
        D = 0.0;
    }

    public override void D2(double X, out double F, out double D, out double D2)
    {
        F = radius;
        D = D2 = 0.0;
    }

    public override Law_Function Trim(double PFirst, double PLast, double Tol)
    {
        Law_Constant l = new Law_Constant();
        l.Set(radius, PFirst, PLast);
        return l;
    }

    public override void Bounds(out double PFirst, out double PLast)
    {
        PFirst = first;
        PLast = last;
    }
}
