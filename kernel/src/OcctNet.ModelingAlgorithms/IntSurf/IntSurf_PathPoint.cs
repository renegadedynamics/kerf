// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PathPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PathPoint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PathPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A starting point of a walking line on the boundary of a parametric surface (IntWalk_IWalking): the 3D point, its
/// parameters (several when the point is multiple), the tangent directions in 3D and in the parametric space, and
/// the tangency and passing flags.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. As in OCCT, the parameters are a handle to a sequence that copies share:
/// <see cref="AddUV"/> on a copy is seen by the original, and <see cref="SetValue"/> gives the point a new sequence.
/// <c>new()</c> is OCCT's empty constructor (passing, tangent, no parameters); <c>default</c> has the flags false,
/// so ported code writes <c>new()</c>. The mutators are internal (translation rules, section 3); the constructor is
/// <see cref="SetValue"/>'s twin.
/// </remarks>
public partial struct IntSurf_PathPoint : IEquatable<IntSurf_PathPoint>
{
    private gp_Pnt pt;
    private bool ispass;
    private bool istgt;
    private gp_Vec vectg;
    private gp_Dir2d dirtg;
    private NCollection_HSequence<gp_XY>? sequv;

    public IntSurf_PathPoint()
    {
        pt = new gp_Pnt();
        ispass = true;
        istgt = true;
        vectg = new gp_Vec();
        dirtg = new gp_Dir2d();
        sequv = null;
    }

    public IntSurf_PathPoint(in gp_Pnt P, double U, double V)
    {
        pt = P;
        ispass = true;
        istgt = true;
        vectg = new gp_Vec();
        dirtg = new gp_Dir2d();
        sequv = new NCollection_HSequence<gp_XY>();
        sequv.Append(new gp_XY(U, V));
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValue(in gp_Pnt P, double U, double V)
    {
        pt = P;
        sequv = new NCollection_HSequence<gp_XY>();
        sequv.Append(new gp_XY(U, V));
    }

    /// <summary>Adds a pair of parameters (the point is then multiple); shared with the copies of the point.</summary>
    internal readonly void AddUV(double U, double V) => sequv!.Append(new gp_XY(U, V));

    internal void SetDirections(in gp_Vec V, in gp_Dir2d D)
    {
        istgt = false;
        vectg = V;
        dirtg = D;
    }

    internal void SetTangency(bool Tang) => istgt = Tang;

    internal void SetPassing(bool Pass) => ispass = Pass;

    // Queries

    public readonly gp_Pnt Value() => pt;

    public readonly void Value2d(out double U, out double V)
    {
        gp_XY uv = sequv!.Sequence().First();
        U = uv.X();
        V = uv.Y();
    }

    public readonly bool IsPassingPnt() => ispass;

    public readonly bool IsTangent() => istgt;

    /// <summary>The tangent in 3D. Throws StdFail_UndefinedDerivative at a tangent point.</summary>
    public readonly gp_Vec Direction3d()
    {
        if (istgt)
        {
            throw new StdFail_UndefinedDerivative();
        }
        return vectg;
    }

    /// <summary>The tangent in the parametric space. Throws StdFail_UndefinedDerivative at a tangent point.</summary>
    public readonly gp_Dir2d Direction2d()
    {
        if (istgt)
        {
            throw new StdFail_UndefinedDerivative();
        }
        return dirtg;
    }

    /// <summary>The number of parameter pairs after the first.</summary>
    public readonly int Multiplicity() => (sequv!.Length() - 1);

    /// <summary>The parameters of rank <paramref name="Index"/> (0 is the first pair).</summary>
    public readonly void Parameters(int Index, out double U, out double V)
    {
        gp_XY uv = sequv!.Value(Index + 1);
        U = uv.X();
        V = uv.Y();
    }

    /// <summary>Field equality, the parameter sequences compared by reference (OCCT's handle).</summary>
    public readonly bool Equals(IntSurf_PathPoint other)
        => pt.Equals(other.pt) && ispass == other.ispass && istgt == other.istgt && vectg.Equals(other.vectg)
           && dirtg.Equals(other.dirtg) && ReferenceEquals(sequv, other.sequv);

    public override readonly bool Equals(object? obj) => obj is IntSurf_PathPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pt, ispass, istgt, vectg, dirtg, sequv);

    public static bool operator ==(in IntSurf_PathPoint theLeft, in IntSurf_PathPoint theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntSurf_PathPoint theLeft, in IntSurf_PathPoint theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"{gp.Format("IntSurf_PathPoint", pt.X(), pt.Y(), pt.Z())} passing {ispass} tangent {istgt} uv {sequv?.Length() ?? 0}";
}
