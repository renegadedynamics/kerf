// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Point.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Point.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A vertex of a contour line: most of the time the meeting of the contour and a restriction of the surface (then
/// <see cref="IsOnArc"/>), with its point, its parameters on the surface and on the line, and its logical
/// information (vertex of the domain, multiple, internal).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct (decision 609): lines keep copies, and <see cref="Contap_Line.Vertex"/>
/// returns a reference to the stored one. <c>new()</c> and <c>default</c> differ only in the transitions
/// (<c>new()</c> has OCCT's undecided ones), so ported code writes <c>new()</c>. The mutators are internal. The arcs
/// and vertices are adaptors, held and compared by reference, as OCCT's handles.
/// </remarks>
public partial struct Contap_Point : IEquatable<Contap_Point>
{
    private gp_Pnt pt;
    private double uparam;
    private double vparam;
    private double paraline;
    private bool onarc;
    private Adaptor2d_Curve2d? arc;
    private IntSurf_Transition traline;
    private IntSurf_Transition traarc;
    private double prmarc;
    private bool isvtx;
    private Adaptor3d_HVertex? vtx;
    private bool ismult;
    private bool myInternal;

    /// <summary>Empty constructor.</summary>
    public Contap_Point()
    {
        pt = new gp_Pnt();
        uparam = 0.0;
        vparam = 0.0;
        paraline = 0.0;
        onarc = false;
        arc = null;
        traline = new IntSurf_Transition();
        traarc = new IntSurf_Transition();
        prmarc = 0.0;
        isvtx = false;
        vtx = null;
        ismult = false;
        myInternal = false;
    }

    /// <summary>A point of the surface at (U, V).</summary>
    public Contap_Point(in gp_Pnt Pt, double U, double V)
        : this()
    {
        pt = Pt;
        uparam = U;
        vparam = V;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the point and its parameters on the surface (and clears the logical information but the arc's).</summary>
    internal void SetValue(in gp_Pnt Pt, double U, double V)
    {
        pt = Pt;
        uparam = U;
        vparam = V;
        onarc = false;
        isvtx = false;
        ismult = false;
        myInternal = false;
    }

    /// <summary>Sets the parameter on the contour line.</summary>
    internal void SetParameter(double Para)
    {
        paraline = Para;
    }

    /// <summary>Makes the point a vertex of the domain.</summary>
    internal void SetVertex(Adaptor3d_HVertex V)
    {
        isvtx = true;
        vtx = V;
    }

    /// <summary>Sets the arc, the parameter on it and the transitions of the line and the arc.</summary>
    internal void SetArc(Adaptor2d_Curve2d A, double Param, in IntSurf_Transition TLine, in IntSurf_Transition TArc)
    {
        onarc = true;
        arc = A;
        prmarc = Param;
        traline = TLine;
        traarc = TArc;
    }

    internal void SetMultiple()
    {
        ismult = true;
    }

    internal void SetInternal()
    {
        myInternal = true;
    }

    // Queries

    /// <summary>The point.</summary>
    public readonly gp_Pnt Value() => pt;

    /// <summary>The parameter on the contour line (meaningless when the point is on none).</summary>
    public readonly double ParameterOnLine() => paraline;

    /// <summary>The parameters on the surface.</summary>
    public readonly void Parameters(out double U1, out double V1)
    {
        U1 = uparam;
        V1 = vparam;
    }

    /// <summary>True when the point is the meeting of the contour and a restriction.</summary>
    public readonly bool IsOnArc() => onarc;

    /// <summary>The arc of restriction; throws Standard_DomainError if the point is on none.</summary>
    public readonly Adaptor2d_Curve2d Arc()
    {
        if (!onarc)
        {
            throw new Standard_DomainError();
        }
        return arc!;
    }

    /// <summary>The parameter on <see cref="Arc"/>; throws Standard_DomainError if the point is on no arc.</summary>
    public readonly double ParameterOnArc()
    {
        if (!onarc)
        {
            throw new Standard_DomainError();
        }
        return prmarc;
    }

    /// <summary>The transition of the point on the contour; throws Standard_DomainError if the point is on no arc.</summary>
    public readonly IntSurf_Transition TransitionOnLine()
    {
        if (!onarc)
        {
            throw new Standard_DomainError();
        }
        return traline;
    }

    /// <summary>The transition of the point on the arc; throws Standard_DomainError if the point is on no arc.</summary>
    public readonly IntSurf_Transition TransitionOnArc()
    {
        if (!onarc)
        {
            throw new Standard_DomainError();
        }
        return traarc;
    }

    /// <summary>True if the point is a vertex of the domain.</summary>
    public readonly bool IsVertex() => isvtx;

    /// <summary>The vertex; throws Standard_DomainError if the point is not one.</summary>
    public readonly Adaptor3d_HVertex Vertex()
    {
        if (!isvtx)
        {
            throw new Standard_DomainError();
        }
        return vtx!;
    }

    /// <summary>True if the point belongs to several lines.</summary>
    public readonly bool IsMultiple() => ismult;

    /// <summary>True if the tangent to the line at the point and the eye direction are parallel.</summary>
    public readonly bool IsInternal() => myInternal;

    public readonly bool Equals(Contap_Point other)
        => pt.Equals(other.pt) && uparam.Equals(other.uparam) && vparam.Equals(other.vparam)
           && paraline.Equals(other.paraline) && onarc == other.onarc && ReferenceEquals(arc, other.arc)
           && traline.Equals(other.traline) && traarc.Equals(other.traarc) && prmarc.Equals(other.prmarc)
           && isvtx == other.isvtx && ReferenceEquals(vtx, other.vtx) && ismult == other.ismult
           && myInternal == other.myInternal;

    public override readonly bool Equals(object? obj) => obj is Contap_Point other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pt, uparam, vparam, paraline, onarc, prmarc, isvtx);

    public static bool operator ==(in Contap_Point theLeft, in Contap_Point theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Contap_Point theLeft, in Contap_Point theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"{gp.Format("Contap_Point", pt.X(), pt.Y(), pt.Z(), uparam, vparam, paraline)} onarc {onarc} vertex {isvtx}";
}
