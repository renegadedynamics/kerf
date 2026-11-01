// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Extremity.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Extremity.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Extremity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An extremity of a walking line on one of its supports: the point, its parameters, the guide's parameter, the
/// tolerance, the tangent if known, the vertex if it is one, and the arcs of restriction it lies on.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct, copied by value as OCCT copies it (decision 9801): its sequence of points on
/// arcs is copied on write (<c>AddArc</c> appends to a new sequence, <c>SetValue</c> drops it), so two copies never
/// share one. <c>new BRepBlend_Extremity()</c> is OCCT's default extremity; <c>default</c> is the same.
/// The mutators are internal, with public <c>With*</c> twins.
/// </remarks>
public partial struct BRepBlend_Extremity : IEquatable<BRepBlend_Extremity>
{
    private Adaptor3d_HVertex? vtx;
    private NCollection_Sequence<BRepBlend_PointOnRst>? seqpt;
    private gp_Pnt pt;
    private gp_Vec tang;
    private double param;
    private double u;
    private double v;
    private double tol;
    private bool isvtx;
    private bool hastang;

    public BRepBlend_Extremity()
    {
        pt = new gp_Pnt(0, 0, 0);
        tang = new gp_Vec(0, 0, 0);
        param = 0.0;
        u = 0.0;
        v = 0.0;
        tol = 0.0;
        isvtx = false;
        hastang = false;
    }

    /// <summary>An extremity at <paramref name="P"/>, (<paramref name="U"/>, <paramref name="V"/>) on a surface.</summary>
    public BRepBlend_Extremity(in gp_Pnt P, double U, double V, double Param, double Tol)
    {
        pt = P;
        tang = new gp_Vec(0, 0, 0);
        param = Param;
        u = U;
        v = V;
        tol = Tol;
        isvtx = false;
        hastang = false;
    }

    /// <summary>An extremity on the vertex <paramref name="Vtx"/>.</summary>
    public BRepBlend_Extremity(in gp_Pnt P, double U, double V, double Param, double Tol, Adaptor3d_HVertex Vtx)
    {
        vtx = Vtx;
        pt = P;
        tang = new gp_Vec(0, 0, 0);
        param = Param;
        u = U;
        v = V;
        tol = Tol;
        isvtx = true;
        hastang = false;
    }

    /// <summary>An extremity at parameter <paramref name="W"/> on a curve.</summary>
    public BRepBlend_Extremity(in gp_Pnt P, double W, double Param, double Tol)
    {
        pt = P;
        tang = new gp_Vec(0, 0, 0);
        param = Param;
        u = W;
        v = 0.0;
        tol = Tol;
        isvtx = false;
        hastang = false;
    }

    internal void SetValue(in gp_Pnt P, double U, double V, double Param, double Tol)
    {
        pt = P;
        u = U;
        v = V;
        param = Param;
        tol = Tol;
        isvtx = false;
        seqpt = null; // seqpt.Clear()
    }

    internal void SetValue(in gp_Pnt P, double U, double V, double Param, double Tol, Adaptor3d_HVertex Vtx)
    {
        pt = P;
        u = U;
        v = V;
        param = Param;
        tol = Tol;
        isvtx = true;
        vtx = Vtx;
        seqpt = null; // seqpt.Clear()
    }

    internal void SetValue(in gp_Pnt P, double W, double Param, double Tol)
    {
        pt = P;
        u = W;
        param = Param;
        tol = Tol;
        isvtx = false;
        seqpt = null; // seqpt.Clear()
    }

    public readonly gp_Pnt Value() => pt;

    internal void SetTangent(in gp_Vec Tangent)
    {
        hastang = true;
        tang = Tangent;
    }

    public readonly bool HasTangent() => hastang;

    public readonly gp_Vec Tangent()
    {
        if (!hastang)
        {
            throw new Standard_DomainError();
        }

        return tang;
    }

    public readonly double Tolerance() => tol;

    internal void SetVertex(Adaptor3d_HVertex V)
    {
        isvtx = true;
        vtx = V;
    }

    /// <summary>Adds the arc <paramref name="A"/> through the extremity, at parameter <paramref name="Param"/>.</summary>
    internal void AddArc(Adaptor2d_Curve2d A, double Param, in IntSurf_Transition TLine, in IntSurf_Transition TArc)
    {
        NCollection_Sequence<BRepBlend_PointOnRst> aSeq = seqpt is null ? new() : new(seqpt);
        aSeq.Append(new BRepBlend_PointOnRst(A, Param, TLine, TArc));
        seqpt = aSeq;
    }

    public readonly void Parameters(out double U, out double V)
    {
        U = u;
        V = v;
    }

    public readonly bool IsVertex() => isvtx;

    public readonly Adaptor3d_HVertex Vertex()
    {
        if (!isvtx)
        {
            throw new Standard_DomainError();
        }

        return vtx!;
    }

    public readonly int NbPointOnRst() => seqpt is null ? 0 : seqpt.Length();

    public readonly BRepBlend_PointOnRst PointOnRst(int Index)
    {
        if (seqpt is null)
        {
            throw new Standard_OutOfRange("NCollection_Sequence::Value");
        }

        return seqpt.Value(Index);
    }

    public readonly double Parameter() => u;

    public readonly double ParameterOnGuide() => param;

    public readonly BRepBlend_Extremity WithValue(in gp_Pnt P, double U, double V, double Param, double Tol)
    {
        BRepBlend_Extremity aCopy = this;
        aCopy.SetValue(P, U, V, Param, Tol);
        return aCopy;
    }

    public readonly BRepBlend_Extremity WithValue(in gp_Pnt P, double U, double V, double Param, double Tol,
                                                  Adaptor3d_HVertex Vtx)
    {
        BRepBlend_Extremity aCopy = this;
        aCopy.SetValue(P, U, V, Param, Tol, Vtx);
        return aCopy;
    }

    public readonly BRepBlend_Extremity WithValue(in gp_Pnt P, double W, double Param, double Tol)
    {
        BRepBlend_Extremity aCopy = this;
        aCopy.SetValue(P, W, Param, Tol);
        return aCopy;
    }

    public readonly BRepBlend_Extremity WithTangent(in gp_Vec Tangent)
    {
        BRepBlend_Extremity aCopy = this;
        aCopy.SetTangent(Tangent);
        return aCopy;
    }

    public readonly BRepBlend_Extremity WithVertex(Adaptor3d_HVertex V)
    {
        BRepBlend_Extremity aCopy = this;
        aCopy.SetVertex(V);
        return aCopy;
    }

    public readonly BRepBlend_Extremity WithArc(Adaptor2d_Curve2d A, double Param, in IntSurf_Transition TLine,
                                                in IntSurf_Transition TArc)
    {
        BRepBlend_Extremity aCopy = this;
        aCopy.AddArc(A, Param, TLine, TArc);
        return aCopy;
    }

    public readonly bool Equals(BRepBlend_Extremity other)
    {
        if (!(ReferenceEquals(vtx, other.vtx) && pt.Equals(other.pt) && tang.Equals(other.tang)
              && param.Equals(other.param) && u.Equals(other.u) && v.Equals(other.v) && tol.Equals(other.tol)
              && isvtx == other.isvtx && hastang == other.hastang && NbPointOnRst() == other.NbPointOnRst()))
        {
            return false;
        }

        for (int i = 1; i <= NbPointOnRst(); i++)
        {
            if (!PointOnRst(i).Equals(other.PointOnRst(i)))
            {
                return false;
            }
        }

        return true;
    }

    public override readonly bool Equals(object? obj) => obj is BRepBlend_Extremity other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pt, param, u, v, tol, isvtx, NbPointOnRst());

    public static bool operator ==(in BRepBlend_Extremity theLeft, in BRepBlend_Extremity theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in BRepBlend_Extremity theLeft, in BRepBlend_Extremity theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => gp.Format("BRepBlend_Extremity", pt.X(), pt.Y(), pt.Z(), u, v, param, tol);
}
