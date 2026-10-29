// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// TODO(4A): a temporary stand-in, to be DELETED when unit 4A (surface extrema) is merged. Unit 4A ports
// Extrema_ExtFlag, Extrema_ExtAlgo, Extrema_ExtPS and Extrema_GenExtCS with exactly these signatures (and more
// members); Extrema_ExtCS (unit 4B) calls only the members declared here. Until the merge, the paths of
// Extrema_ExtCS that need them (a line against a non-parallel plane or a non-elementary surface, and every
// non-analytic curve/surface pair) throw NotImplementedException. When both units are merged, the duplicate type
// definitions (CS0101) point here: delete this file.

namespace Occt;

/// <summary>Stand-in for unit 4A's Extrema_ExtFlag (TODO(4A): delete at the merge).</summary>
public enum Extrema_ExtFlag
{
    Extrema_ExtFlag_MIN,
    Extrema_ExtFlag_MAX,
    Extrema_ExtFlag_MINMAX,
}

/// <summary>Stand-in for unit 4A's Extrema_ExtAlgo (TODO(4A): delete at the merge).</summary>
public enum Extrema_ExtAlgo
{
    Extrema_ExtAlgo_Grad,
    Extrema_ExtAlgo_Tree,
}

/// <summary>Stand-in for unit 4A's Extrema_ExtPS (TODO(4A): delete at the merge). Every member throws.</summary>
public class Extrema_ExtPS
{
    public Extrema_ExtPS()
    {
    }

    public Extrema_ExtPS(in gp_Pnt P, Adaptor3d_Surface S, double Uinf, double Usup, double Vinf, double Vsup,
        double TolU, double TolV, Extrema_ExtFlag F = Extrema_ExtFlag.Extrema_ExtFlag_MINMAX,
        Extrema_ExtAlgo A = Extrema_ExtAlgo.Extrema_ExtAlgo_Grad)
        => throw new NotImplementedException("TODO(4A)");

    public void Initialize(Adaptor3d_Surface S, double Uinf, double Usup, double Vinf, double Vsup, double TolU,
        double TolV) => throw new NotImplementedException("TODO(4A)");

    public void Perform(in gp_Pnt P) => throw new NotImplementedException("TODO(4A)");

    public bool IsDone() => throw new NotImplementedException("TODO(4A)");

    public int NbExt() => throw new NotImplementedException("TODO(4A)");

    public double SquareDistance(int N) => throw new NotImplementedException("TODO(4A)");

    public Extrema_POnSurf Point(int N) => throw new NotImplementedException("TODO(4A)");
}

/// <summary>Stand-in for unit 4A's Extrema_GenExtCS (TODO(4A): delete at the merge). Every member throws.</summary>
public class Extrema_GenExtCS
{
    public Extrema_GenExtCS()
    {
    }

    public Extrema_GenExtCS(Adaptor3d_Curve C, Adaptor3d_Surface S, int NbT, int NbU, int NbV, double tmin,
        double tsup, double Umin, double Usup, double Vmin, double Vsup, double Tol1, double Tol2)
        => throw new NotImplementedException("TODO(4A)");

    public void Initialize(Adaptor3d_Surface S, int NbU, int NbV, double Tol2)
        => throw new NotImplementedException("TODO(4A)");

    public void Perform(Adaptor3d_Curve C, int NbT, double Tol1) => throw new NotImplementedException("TODO(4A)");

    public void Perform(Adaptor3d_Curve C, int NbT, double tmin, double tsup, double Tol1)
        => throw new NotImplementedException("TODO(4A)");

    public bool IsDone() => throw new NotImplementedException("TODO(4A)");

    public int NbExt() => throw new NotImplementedException("TODO(4A)");

    public double SquareDistance(int N) => throw new NotImplementedException("TODO(4A)");

    public Extrema_POnCurv PointOnCurve(int N) => throw new NotImplementedException("TODO(4A)");

    public Extrema_POnSurf PointOnSurface(int N) => throw new NotImplementedException("TODO(4A)");
}
