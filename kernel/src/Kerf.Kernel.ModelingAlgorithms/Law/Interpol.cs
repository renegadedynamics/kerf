// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Interpol.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Interpol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Laws;

/// <summary>An evolution law interpolating (parameter, value) pairs: the X of each 2D point is the parameter, Y the value.</summary>
/// <remarks>
/// A working object (decision 21): the <c>Set</c> methods are public and change the law. The X coordinates must
/// increase; for a periodic law the last value (and derivative) is taken to be the first.
/// </remarks>
internal class Interpol : BSpFunc
{
    /// <summary>An empty law, defined later by Set.</summary>
    public Interpol()
    {
    }

    /// <summary>Defines the law by interpolating the points ParAndRad (X the parameter, Y the value).</summary>
    public void Set(NCollection_Array1<Point2D> parAndRad, bool periodic = false)
    {
        int l = parAndRad.Lower();
        int nbp = parAndRad.Length();

        NCollection_HArray1<double> par = new NCollection_HArray1<double>(1, nbp);
        NCollection_HArray1<double> rad;
        if (periodic)
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
            parAndRad[l + i - 1].Coord(out x, out y);
            par.SetValue(i, x);
            if (!periodic || i != nbp)
            {
                rad.SetValue(i, y);
            }
        }
        LawsInterpolate inter = new LawsInterpolate(rad, par, periodic, global::Kerf.Engine.Foundation.Precision.Confusion());
        inter.Perform();
        SetCurve(inter.InternalCurve());
    }

    /// <summary>As Set, with the parameters of ParAndRad mapped linearly onto [Ud, Uf].</summary>
    public void SetInRelative(NCollection_Array1<Point2D> parAndRad, double ud, double uf, bool periodic = false)
    {
        int l = parAndRad.Lower(), u = parAndRad.Upper();
        double wd = parAndRad[l].X, wf = parAndRad[u].X;
        int nbp = parAndRad.Length();
        NCollection_HArray1<double> par = new NCollection_HArray1<double>(1, nbp);
        NCollection_HArray1<double> rad;
        if (periodic)
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
            parAndRad[l + i - 1].Coord(out x, out y);
            par.SetValue(i, (uf * (x - wd) + ud * (wf - x)) / (wf - wd));
            if (!periodic || i != nbp)
            {
                rad.SetValue(i, y);
            }
        }
        LawsInterpolate inter = new LawsInterpolate(rad, par, periodic, global::Kerf.Engine.Foundation.Precision.Confusion());
        inter.Perform();
        SetCurve(inter.InternalCurve());
    }

    /// <summary>Defines the law by interpolating ParAndRad, with the derivatives Dd and Df at the first and last points.</summary>
    public void Set(NCollection_Array1<Point2D> parAndRad, double dd, double df, bool periodic = false)
    {
        int l = parAndRad.Lower();
        int nbp = parAndRad.Length();

        NCollection_HArray1<double> par = new NCollection_HArray1<double>(1, nbp);
        NCollection_HArray1<double> rad;
        if (periodic)
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
            parAndRad[l + i - 1].Coord(out x, out y);
            par.SetValue(i, x);
            if (!periodic || i != nbp)
            {
                rad.SetValue(i, y);
            }
        }
        LawsInterpolate inter = new LawsInterpolate(rad, par, periodic, global::Kerf.Engine.Foundation.Precision.Confusion());
        inter.Load(dd, df);
        inter.Perform();
        SetCurve(inter.InternalCurve());
    }

    /// <summary>As Set with derivatives, with the parameters of ParAndRad mapped linearly onto [Ud, Uf].</summary>
    public void SetInRelative(NCollection_Array1<Point2D> parAndRad, double ud, double uf, double dd, double df,
                              bool periodic = false)
    {
        int l = parAndRad.Lower(), u = parAndRad.Upper();
        double wd = parAndRad[l].X, wf = parAndRad[u].X;
        int nbp = parAndRad.Length();
        NCollection_HArray1<double> par = new NCollection_HArray1<double>(1, nbp);
        NCollection_HArray1<double> rad;
        if (periodic)
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
            parAndRad[l + i - 1].Coord(out x, out y);
            par.SetValue(i, (uf * (x - wd) + ud * (wf - x)) / (wf - wd));
            if (!periodic || i != nbp)
            {
                rad.SetValue(i, y);
            }
        }
        LawsInterpolate inter = new LawsInterpolate(rad, par, periodic, global::Kerf.Engine.Foundation.Precision.Confusion());
        inter.Load(dd, df);
        inter.Perform();
        SetCurve(inter.InternalCurve());
    }
}
