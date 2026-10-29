// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Point.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Point.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Runtime.CompilerServices;
using static Occt.IntSurf_Situation;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

/// <summary>
/// An intersection point between two surfaces, a vertex of an intersection line: the point and its parameters on
/// both surfaces (<see cref="PntOn2S"/>), its parameter on the line, its tolerance, and where it lies on the
/// restrictions of each surface (a vertex, an arc with its parameter and the transitions of the line and the arc).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct (decision 5103): lines keep copies, and <see cref="IntPatch_PointLine.ChangeVertex"/>
/// returns a reference to the stored one. <c>new()</c> and <c>default</c> differ only in the transitions
/// (<c>new()</c> has OCCT's undecided ones), so ported code writes <c>new()</c>. The mutators are internal
/// (translation rules, section 3): only ported code builds vertices. The arcs and vertices are adaptors, held and
/// compared by reference, as OCCT's handles. <see cref="Dump(TextWriter)"/> is OCCT's <c>Dump</c> (to stdout).
/// </remarks>
public partial struct IntPatch_Point : IEquatable<IntPatch_Point>
{
    private IntSurf_PntOn2S pt;
    private double para;
    private double tol;
    private bool tgt;
    // TRUE if the point is in several intersection lines
    private bool mult;
    private bool onS1;
    private bool vtxonS1;
    private Adaptor3d_HVertex? vS1;
    private Adaptor2d_Curve2d? arcS1;
    private IntSurf_Transition traline1;
    private IntSurf_Transition tra1;
    private double prm1;
    private bool onS2;
    private bool vtxonS2;
    private Adaptor3d_HVertex? vS2;
    private Adaptor2d_Curve2d? arcS2;
    private IntSurf_Transition traline2;
    private IntSurf_Transition tra2;
    private double prm2;

    /// <summary>Empty constructor.</summary>
    public IntPatch_Point()
    {
        pt = new IntSurf_PntOn2S();
        para = 0.0;
        tol = 0.0;
        tgt = false;
        mult = false;
        onS1 = false;
        vtxonS1 = false;
        vS1 = null;
        arcS1 = null;
        traline1 = new IntSurf_Transition();
        tra1 = new IntSurf_Transition();
        prm1 = 0.0;
        onS2 = false;
        vtxonS2 = false;
        vS2 = null;
        arcS2 = null;
        traline2 = new IntSurf_Transition();
        tra2 = new IntSurf_Transition();
        prm2 = 0.0;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>
    /// Sets a point on no domain, when both surfaces are implicit; <paramref name="Tangent"/> if it is a point of
    /// tangency between the surfaces.
    /// </summary>
    internal void SetValue(in gp_Pnt Pt, double Tol, bool Tangent)
    {
        onS1 = false;
        onS2 = false;
        vtxonS1 = false;
        vtxonS2 = false;
        mult = false;
        tgt = Tangent;
        pt.SetValue(Pt);
        tol = Tol;
    }

    internal void SetValue(in gp_Pnt Pt) => pt.SetValue(Pt);

    /// <summary>Sets the point and its parameters.</summary>
    internal void SetValue(in IntSurf_PntOn2S thePOn2S) => pt = thePOn2S;

    internal void SetTolerance(double Tol) => tol = Tol;

    /// <summary>Sets the parameters on each surface.</summary>
    internal void SetParameters(double U1, double V1, double U2, double V2) => pt.SetValue(U1, V1, U2, V2);

    /// <summary>Sets the parameter on the intersection line.</summary>
    internal void SetParameter(double Para) => para = Para;

    /// <summary>
    /// Makes the point a vertex of the restrictions of the first surface (<paramref name="OnFirst"/>) or of the second.
    /// </summary>
    internal void SetVertex(bool OnFirst, Adaptor3d_HVertex V)
    {
        if (OnFirst)
        {
            onS1 = true;
            vtxonS1 = true;
            vS1 = V;
        }
        else
        {
            onS2 = true;
            vtxonS2 = true;
            vS2 = V;
        }
    }

    /// <summary>
    /// Puts the point on an arc of the restrictions of the first surface (<paramref name="OnFirst"/>) or of the second,
    /// at <paramref name="Param"/>, with the transitions of the line (<paramref name="TLine"/>) and of the arc
    /// (<paramref name="TArc"/>).
    /// </summary>
    internal void SetArc(bool OnFirst, Adaptor2d_Curve2d A, double Param, in IntSurf_Transition TLine,
                         in IntSurf_Transition TArc)
    {
        if (OnFirst)
        {
            onS1 = true;
            arcS1 = A;
            traline1 = TLine;
            tra1 = TArc;
            prm1 = Param;
        }
        else
        {
            onS2 = true;
            arcS2 = A;
            traline2 = TLine;
            tra2 = TArc;
            prm2 = Param;
        }
    }

    /// <summary>Sets (or unsets) the point as a point on several intersection lines.</summary>
    internal void SetMultiple(bool IsMult) => mult = IsMult;

    /// <summary>Swaps in and out in the transitions on each arc (others become undecided).</summary>
    internal void ReverseTransition()
    {
        if (onS1)
        {
            IntSurf_Transition TLine = new();
            switch (traline1.TransitionType())
            {
                case IntSurf_In:
                    TLine.SetValue(false, IntSurf_Out);
                    break;
                case IntSurf_Out:
                    TLine.SetValue(false, IntSurf_In);
                    break;
                default:
                    break;
            }
            traline1 = TLine;
            IntSurf_Transition TArc = new();
            switch (tra1.TransitionType())
            {
                case IntSurf_In:
                    TArc.SetValue(false, IntSurf_Out);
                    break;
                case IntSurf_Out:
                    TArc.SetValue(false, IntSurf_In);
                    break;
                default:
                    break;
            }
            tra1 = TArc;
        }
        if (onS2)
        {
            IntSurf_Transition TLine = new();
            switch (traline2.TransitionType())
            {
                case IntSurf_In:
                    TLine.SetValue(false, IntSurf_Out);
                    break;
                case IntSurf_Out:
                    TLine.SetValue(false, IntSurf_In);
                    break;
                default:
                    break;
            }
            traline2 = TLine;
            IntSurf_Transition TArc = new();
            switch (tra2.TransitionType())
            {
                case IntSurf_In:
                    TArc.SetValue(false, IntSurf_Out);
                    break;
                case IntSurf_Out:
                    TArc.SetValue(false, IntSurf_In);
                    break;
                default:
                    break;
            }
            tra2 = TArc;
        }
    }

    // Queries

    /// <summary>The intersection point.</summary>
    public readonly gp_Pnt Value() => pt.Value();

    /// <summary>The parameter on the intersection line (meaningless if the point is on no line).</summary>
    public readonly double ParameterOnLine() => para;

    /// <summary>The fuzziness of the point.</summary>
    public readonly double Tolerance() => tol;

    /// <summary>True if the point is a tangency point between the surfaces.</summary>
    public readonly bool IsTangencyPoint() => tgt;

    /// <summary>The parameters on the first surface.</summary>
    public readonly void ParametersOnS1(out double U1, out double V1) => pt.ParametersOnS1(out U1, out V1);

    /// <summary>The parameters on the second surface.</summary>
    public readonly void ParametersOnS2(out double U2, out double V2) => pt.ParametersOnS2(out U2, out V2);

    /// <summary>True if the point belongs to several intersection lines.</summary>
    public readonly bool IsMultiple() => mult;

    /// <summary>True if the point is on a boundary of the domain of the first surface.</summary>
    public readonly bool IsOnDomS1() => onS1;

    /// <summary>True if the point is a vertex of the restrictions of the first surface.</summary>
    public readonly bool IsVertexOnS1() => vtxonS1;

    /// <summary>The vertex on the first surface; throws Standard_DomainError unless <see cref="IsVertexOnS1"/>.</summary>
    public readonly Adaptor3d_HVertex VertexOnS1()
    {
        if (!vtxonS1)
        {
            throw new Standard_DomainError();
        }
        return vS1!;
    }

    /// <summary>The arc of restriction of the first surface; throws Standard_DomainError unless <see cref="IsOnDomS1"/>.</summary>
    public readonly Adaptor2d_Curve2d ArcOnS1()
    {
        if (!onS1)
        {
            throw new Standard_DomainError();
        }
        return arcS1!;
    }

    /// <summary>The transition of the line with the arc on the first surface.</summary>
    public readonly IntSurf_Transition TransitionLineArc1()
    {
        if (!onS1)
        {
            throw new Standard_DomainError();
        }
        return traline1;
    }

    /// <summary>The transition of the arc on the first surface with the line.</summary>
    public readonly IntSurf_Transition TransitionOnS1()
    {
        if (!onS1)
        {
            throw new Standard_DomainError();
        }
        return tra1;
    }

    /// <summary>The parameter on the arc of the first surface.</summary>
    public readonly double ParameterOnArc1()
    {
        if (!onS1)
        {
            throw new Standard_DomainError();
        }
        return prm1;
    }

    /// <summary>True if the point is on a boundary of the domain of the second surface.</summary>
    public readonly bool IsOnDomS2() => onS2;

    /// <summary>True if the point is a vertex of the restrictions of the second surface.</summary>
    public readonly bool IsVertexOnS2() => vtxonS2;

    /// <summary>The vertex on the second surface; throws Standard_DomainError unless <see cref="IsVertexOnS2"/>.</summary>
    public readonly Adaptor3d_HVertex VertexOnS2()
    {
        if (!vtxonS2)
        {
            throw new Standard_DomainError();
        }
        return vS2!;
    }

    /// <summary>The arc of restriction of the second surface; throws Standard_DomainError unless <see cref="IsOnDomS2"/>.</summary>
    public readonly Adaptor2d_Curve2d ArcOnS2()
    {
        if (!onS2)
        {
            throw new Standard_DomainError();
        }
        return arcS2!;
    }

    /// <summary>The transition of the line with the arc on the second surface.</summary>
    public readonly IntSurf_Transition TransitionLineArc2()
    {
        if (!onS2)
        {
            throw new Standard_DomainError();
        }
        return traline2;
    }

    /// <summary>The transition of the arc on the second surface with the line.</summary>
    public readonly IntSurf_Transition TransitionOnS2()
    {
        if (!onS2)
        {
            throw new Standard_DomainError();
        }
        return tra2;
    }

    /// <summary>The parameter on the arc of the second surface.</summary>
    public readonly double ParameterOnArc2()
    {
        if (!onS2)
        {
            throw new Standard_DomainError();
        }
        return prm2;
    }

    /// <summary>The point and its parameters on both surfaces.</summary>
    public readonly IntSurf_PntOn2S PntOn2S() => pt;

    /// <summary>The parameters on both surfaces.</summary>
    public readonly void Parameters(out double U1, out double V1, out double U2, out double V2) =>
        pt.Parameters(out U1, out V1, out U2, out V2);

    /// <summary>
    /// Writes the point as OCCT's <c>Dump</c> prints it (printf's <c>%+10.20f</c> for the numbers; the arcs' and
    /// vertices' addresses are their identity hash codes).
    /// </summary>
    public readonly void Dump(TextWriter theOut)
    {
        theOut.Write("----------- IntPatch_Point : " + '\n');
        pt.Parameters(out double u1, out double v1, out double u2, out double v2);
        theOut.Write($"P({F(pt.Value().X())},{F(pt.Value().Y())},{F(pt.Value().Z())}) UV1({F(u1)},{F(v1)})  "
                     + $"UV2({F(u2)},{F(v2)}) (Para:{F(para)})\n");
        if (onS1)
        {
            theOut.Write($"*OnS1*  par={F(prm1)} arc1={Address(arcS1)}");
        }
        if (vtxonS1)
        {
            theOut.Write($" *Vtx1*  vtx1={Address(vS1)}");
        }
        if (onS1 || vtxonS1)
        {
            theOut.Write("\n");
        }
        if (onS2)
        {
            theOut.Write($"*OnS2*  par={F(prm2)} arc2={Address(arcS2)}");
        }
        if (vtxonS2)
        {
            theOut.Write($" *Vtx2*  vtx2={Address(vS2)}");
        }
        if (onS2 || vtxonS2)
        {
            theOut.Write("\n");
        }

        if (onS1 || onS2)
        {
            theOut.Write(" tgt:" + ((tgt) ? 1 : 0) + "   mult:" + ((mult) ? 1 : 0));
            if (onS1)
            {
                theOut.Write("\n-traline1  : ");
                DumpTransition(theOut, traline1);
                theOut.Write("  -tra1  : ");
                DumpTransition(theOut, tra1);
            }
            if (onS2)
            {
                theOut.Write("\n-traline2  : ");
                DumpTransition(theOut, traline2);
                theOut.Write("  -tra2  : ");
                DumpTransition(theOut, tra2);
            }
            theOut.Write('\n');
        }
        theOut.Write("\nToler = " + Standard_Stream.FormatReal(Tolerance(), 6)); // std::cout << double
        theOut.Write('\n');
    }

    // printf's "%+10.20f": a sign always, twenty decimals.
    private static string F(double theValue)
    {
        string aStr = theValue.ToString("F20", CultureInfo.InvariantCulture);
        return aStr.StartsWith('-') ? aStr : "+" + aStr;
    }

    private static string Address(object? theObject)
        => theObject is null ? "0000000000000000" : RuntimeHelpers.GetHashCode(theObject).ToString("X16", CultureInfo.InvariantCulture);

    private static void DumpTransition(TextWriter theOut, in IntSurf_Transition T)
    {
        IntSurf_TypeTrans typetrans = T.TransitionType();
        if (typetrans == IntSurf_In)
        {
            theOut.Write(" In  ");
        }
        else if (typetrans == IntSurf_Out)
        {
            theOut.Write(" Out ");
        }
        else if (typetrans == IntSurf_Undecided)
        {
            theOut.Write(" Und.");
        }
        else if (typetrans == IntSurf_Touch)
        {
            theOut.Write(" Touch ");
            if (T.Situation() == IntSurf_Inside)
            {
                theOut.Write(" Inside ");
            }
            else if (T.Situation() == IntSurf_Outside)
            {
                theOut.Write(" Outside");
            }
            else
            {
                theOut.Write(" Unknown");
            }
            if (T.IsOpposite())
            {
                theOut.Write("     Opposite ");
            }
            else
            {
                theOut.Write(" Non_Opposite ");
            }
        }
        if (typetrans != IntSurf_Undecided)
        {
            if (T.IsTangent())
            {
                theOut.Write(" Tangent    ");
            }
            else
            {
                theOut.Write(" Non_Tangent");
            }
        }
    }

    /// <summary>Field equality; the arcs and vertices compared by reference (OCCT's handles).</summary>
    public readonly bool Equals(IntPatch_Point other)
        => pt.Equals(other.pt) && para.Equals(other.para) && tol.Equals(other.tol) && tgt == other.tgt
           && mult == other.mult && onS1 == other.onS1 && vtxonS1 == other.vtxonS1
           && ReferenceEquals(vS1, other.vS1) && ReferenceEquals(arcS1, other.arcS1)
           && traline1.Equals(other.traline1) && tra1.Equals(other.tra1) && prm1.Equals(other.prm1)
           && onS2 == other.onS2 && vtxonS2 == other.vtxonS2 && ReferenceEquals(vS2, other.vS2)
           && ReferenceEquals(arcS2, other.arcS2) && traline2.Equals(other.traline2) && tra2.Equals(other.tra2)
           && prm2.Equals(other.prm2);

    public override readonly bool Equals(object? obj) => obj is IntPatch_Point other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pt, para, tol, onS1, onS2, prm1, prm2);

    public static bool operator ==(in IntPatch_Point theLeft, in IntPatch_Point theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntPatch_Point theLeft, in IntPatch_Point theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
    {
        StringWriter aWriter = new(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
