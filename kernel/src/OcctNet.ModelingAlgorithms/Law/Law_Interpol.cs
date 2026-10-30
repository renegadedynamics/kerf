// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Interpol.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Interpol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An evolution law interpolating (parameter, value) pairs: the X of each 2D point is the parameter, Y the value.</summary>
/// <remarks>
/// A working object (decision 21): the <c>Set</c> methods are public and change the law. The X coordinates must
/// increase; for a periodic law the last value (and derivative) is taken to be the first.
/// </remarks>
public class Law_Interpol : Law_BSpFunc
{
    /// <summary>An empty law, defined later by Set.</summary>
    public Law_Interpol()
    {
    }

    /// <summary>Defines the law by interpolating the points ParAndRad (X the parameter, Y the value).</summary>
    public void Set(NCollection_Array1<gp_Pnt2d> ParAndRad, bool Periodic = false)
    {
        int l = ParAndRad.Lower();
        int nbp = ParAndRad.Length();

        NCollection_HArray1<double> par = new NCollection_HArray1<double>(1, nbp);
        NCollection_HArray1<double> rad;
        if (Periodic)
        {
            rad = new NCollection_HArray1<double>(1, nbp - 1);
        }
        else
        {
            rad = new NCollection_HArray1<double>(1, nbp);
        }
        double x, y;
        int i;
        for (i = 1; i <= nbp; i++)
        {
            ParAndRad[l + i - 1].Coord(out x, out y);
            par.SetValue(i, x);
            if (!Periodic || i != nbp)
            {
                rad.SetValue(i, y);
            }
        }
        Law_Interpolate inter = new Law_Interpolate(rad, par, Periodic, Precision.Confusion());
        inter.Perform();
        SetCurve(inter.InternalCurve());
    }

    /// <summary>As Set, with the parameters of ParAndRad mapped linearly onto [Ud, Uf].</summary>
    public void SetInRelative(NCollection_Array1<gp_Pnt2d> ParAndRad, double Ud, double Uf, bool Periodic = false)
    {
        int l = ParAndRad.Lower(), u = ParAndRad.Upper();
        double wd = ParAndRad[l].X(), wf = ParAndRad[u].X();
        int nbp = ParAndRad.Length();
        NCollection_HArray1<double> par = new NCollection_HArray1<double>(1, nbp);
        NCollection_HArray1<double> rad;
        if (Periodic)
        {
            rad = new NCollection_HArray1<double>(1, nbp - 1);
        }
        else
        {
            rad = new NCollection_HArray1<double>(1, nbp);
        }
        double x, y;
        int i;
        for (i = 1; i <= nbp; i++)
        {
            ParAndRad[l + i - 1].Coord(out x, out y);
            par.SetValue(i, (Uf * (x - wd) + Ud * (wf - x)) / (wf - wd));
            if (!Periodic || i != nbp)
            {
                rad.SetValue(i, y);
            }
        }
        Law_Interpolate inter = new Law_Interpolate(rad, par, Periodic, Precision.Confusion());
        inter.Perform();
        SetCurve(inter.InternalCurve());
    }

    /// <summary>Defines the law by interpolating ParAndRad, with the derivatives Dd and Df at the first and last points.</summary>
    public void Set(NCollection_Array1<gp_Pnt2d> ParAndRad, double Dd, double Df, bool Periodic = false)
    {
        int l = ParAndRad.Lower();
        int nbp = ParAndRad.Length();

        NCollection_HArray1<double> par = new NCollection_HArray1<double>(1, nbp);
        NCollection_HArray1<double> rad;
        if (Periodic)
        {
            rad = new NCollection_HArray1<double>(1, nbp - 1);
        }
        else
        {
            rad = new NCollection_HArray1<double>(1, nbp);
        }
        double x, y;
        int i;
        for (i = 1; i <= nbp; i++)
        {
            ParAndRad[l + i - 1].Coord(out x, out y);
            par.SetValue(i, x);
            if (!Periodic || i != nbp)
            {
                rad.SetValue(i, y);
            }
        }
        Law_Interpolate inter = new Law_Interpolate(rad, par, Periodic, Precision.Confusion());
        inter.Load(Dd, Df);
        inter.Perform();
        SetCurve(inter.InternalCurve());
    }

    /// <summary>As Set with derivatives, with the parameters of ParAndRad mapped linearly onto [Ud, Uf].</summary>
    public void SetInRelative(NCollection_Array1<gp_Pnt2d> ParAndRad, double Ud, double Uf, double Dd, double Df,
                              bool Periodic = false)
    {
        int l = ParAndRad.Lower(), u = ParAndRad.Upper();
        double wd = ParAndRad[l].X(), wf = ParAndRad[u].X();
        int nbp = ParAndRad.Length();
        NCollection_HArray1<double> par = new NCollection_HArray1<double>(1, nbp);
        NCollection_HArray1<double> rad;
        if (Periodic)
        {
            rad = new NCollection_HArray1<double>(1, nbp - 1);
        }
        else
        {
            rad = new NCollection_HArray1<double>(1, nbp);
        }
        double x, y;
        int i;
        for (i = 1; i <= nbp; i++)
        {
            ParAndRad[l + i - 1].Coord(out x, out y);
            par.SetValue(i, (Uf * (x - wd) + Ud * (wf - x)) / (wf - wd));
            if (!Periodic || i != nbp)
            {
                rad.SetValue(i, y);
            }
        }
        Law_Interpolate inter = new Law_Interpolate(rad, par, Periodic, Precision.Confusion());
        inter.Load(Dd, Df);
        inter.Perform();
        SetCurve(inter.InternalCurve());
    }
}
