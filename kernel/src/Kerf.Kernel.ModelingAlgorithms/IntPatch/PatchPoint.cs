// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Point.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Point.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using System.Globalization;
using System.Runtime.CompilerServices;
using static Kerf.Engine.Intersection.Walking.WalkingSituation;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// An intersection point between two surfaces, a vertex of an intersection line: the point and its parameters on
/// both surfaces (<see cref="PntOn2S"/>), its parameter on the line, its tolerance, and where it lies on the
/// restrictions of each surface (a vertex, an arc with its parameter and the transitions of the line and the arc).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct (decision 609): lines keep copies, and <see cref="PointLine.ChangeVertex"/>
/// returns a reference to the stored one. <c>new()</c> and <c>default</c> differ only in the transitions
/// (<c>new()</c> has OCCT's undecided ones), so ported code writes <c>new()</c>. The mutators are internal
/// (translation rules, section 3): only ported code builds vertices. The arcs and vertices are adaptors, held and
/// compared by reference, as OCCT's handles. <see cref="Dump(TextWriter)"/> is OCCT's <c>Dump</c> (to stdout).
/// </remarks>
internal partial struct PatchPoint : IEquatable<PatchPoint>
{
    private PntOn2S _pt;
    private double _para;
    private double _tol;
    private bool _tgt;
    // TRUE if the point is in several intersection lines
    private bool _mult;
    private bool _onS1;
    private bool _vtxonS1;
    private HVertex? _vS1;
    private Curve2DAdaptorBase? _arcS1;
    private WalkingTransition _traline1;
    private WalkingTransition _tra1;
    private double _prm1;
    private bool _onS2;
    private bool _vtxonS2;
    private HVertex? _vS2;
    private Curve2DAdaptorBase? _arcS2;
    private WalkingTransition _traline2;
    private WalkingTransition _tra2;
    private double _prm2;

    /// <summary>Empty constructor.</summary>
    public PatchPoint()
    {
        _pt = new PntOn2S();
        _para = 0.0;
        _tol = 0.0;
        _tgt = false;
        _mult = false;
        _onS1 = false;
        _vtxonS1 = false;
        _vS1 = null;
        _arcS1 = null;
        _traline1 = new WalkingTransition();
        _tra1 = new WalkingTransition();
        _prm1 = 0.0;
        _onS2 = false;
        _vtxonS2 = false;
        _vS2 = null;
        _arcS2 = null;
        _traline2 = new WalkingTransition();
        _tra2 = new WalkingTransition();
        _prm2 = 0.0;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>
    /// Sets a point on no domain, when both surfaces are implicit; <paramref name="tangent"/> if it is a point of
    /// tangency between the surfaces.
    /// </summary>
    internal void SetValue(in Point3D pt, double tol, bool tangent)
    {
        _onS1 = false;
        _onS2 = false;
        _vtxonS1 = false;
        _vtxonS2 = false;
        _mult = false;
        _tgt = tangent;
        _pt.SetValue(pt);
        _tol = tol;
    }

    internal void SetValue(in Point3D pt) => _pt.SetValue(pt);

    /// <summary>Sets the point and its parameters.</summary>
    internal void SetValue(in PntOn2S pOn2S) => _pt = pOn2S;

    internal void SetTolerance(double tol) => _tol = tol;

    /// <summary>Sets the parameters on each surface.</summary>
    internal void SetParameters(double u1, double v1, double u2, double v2) => _pt.SetValue(u1, v1, u2, v2);

    /// <summary>Sets the parameter on the intersection line.</summary>
    internal void SetParameter(double para) => _para = para;

    /// <summary>
    /// Makes the point a vertex of the restrictions of the first surface (<paramref name="onFirst"/>) or of the second.
    /// </summary>
    internal void SetVertex(bool onFirst, HVertex v)
    {
        if (onFirst)
        {
            _onS1 = true;
            _vtxonS1 = true;
            _vS1 = v;
        }
        else
        {
            _onS2 = true;
            _vtxonS2 = true;
            _vS2 = v;
        }
    }

    /// <summary>
    /// Puts the point on an arc of the restrictions of the first surface (<paramref name="onFirst"/>) or of the second,
    /// at <paramref name="param"/>, with the transitions of the line (<paramref name="tLine"/>) and of the arc
    /// (<paramref name="tArc"/>).
    /// </summary>
    internal void SetArc(bool onFirst, Curve2DAdaptorBase a, double param, in WalkingTransition tLine,
                         in WalkingTransition tArc)
    {
        if (onFirst)
        {
            _onS1 = true;
            _arcS1 = a;
            _traline1 = tLine;
            _tra1 = tArc;
            _prm1 = param;
        }
        else
        {
            _onS2 = true;
            _arcS2 = a;
            _traline2 = tLine;
            _tra2 = tArc;
            _prm2 = param;
        }
    }

    /// <summary>Sets (or unsets) the point as a point on several intersection lines.</summary>
    internal void SetMultiple(bool isMult) => _mult = isMult;

    /// <summary>Swaps in and out in the transitions on each arc (others become undecided).</summary>
    internal void ReverseTransition()
    {
        if (_onS1)
        {
            WalkingTransition tLine = new();
            switch (_traline1.TransitionType())
            {
                case WalkingTypeTrans.In:
                    tLine.SetValue(false, WalkingTypeTrans.Out);
                    break;
                case WalkingTypeTrans.Out:
                    tLine.SetValue(false, WalkingTypeTrans.In);
                    break;
                default:
                    break;
            }
            _traline1 = tLine;
            WalkingTransition tArc = new();
            switch (_tra1.TransitionType())
            {
                case WalkingTypeTrans.In:
                    tArc.SetValue(false, WalkingTypeTrans.Out);
                    break;
                case WalkingTypeTrans.Out:
                    tArc.SetValue(false, WalkingTypeTrans.In);
                    break;
                default:
                    break;
            }
            _tra1 = tArc;
        }
        if (_onS2)
        {
            WalkingTransition tLine = new();
            switch (_traline2.TransitionType())
            {
                case WalkingTypeTrans.In:
                    tLine.SetValue(false, WalkingTypeTrans.Out);
                    break;
                case WalkingTypeTrans.Out:
                    tLine.SetValue(false, WalkingTypeTrans.In);
                    break;
                default:
                    break;
            }
            _traline2 = tLine;
            WalkingTransition tArc = new();
            switch (_tra2.TransitionType())
            {
                case WalkingTypeTrans.In:
                    tArc.SetValue(false, WalkingTypeTrans.Out);
                    break;
                case WalkingTypeTrans.Out:
                    tArc.SetValue(false, WalkingTypeTrans.In);
                    break;
                default:
                    break;
            }
            _tra2 = tArc;
        }
    }

    // Queries

    /// <summary>The intersection point.</summary>
    public readonly Point3D Value() => _pt.Value();

    /// <summary>The parameter on the intersection line (meaningless if the point is on no line).</summary>
    public readonly double ParameterOnLine() => _para;

    /// <summary>The fuzziness of the point.</summary>
    public readonly double Tolerance() => _tol;

    /// <summary>True if the point is a tangency point between the surfaces.</summary>
    public readonly bool IsTangencyPoint => _tgt;

    /// <summary>The parameters on the first surface.</summary>
    public readonly void ParametersOnS1(out double u1, out double v1) => _pt.ParametersOnS1(out u1, out v1);

    /// <summary>The parameters on the second surface.</summary>
    public readonly void ParametersOnS2(out double u2, out double v2) => _pt.ParametersOnS2(out u2, out v2);

    /// <summary>True if the point belongs to several intersection lines.</summary>
    public readonly bool IsMultiple => _mult;

    /// <summary>True if the point is on a boundary of the domain of the first surface.</summary>
    public readonly bool IsOnDomS1 => _onS1;

    /// <summary>True if the point is a vertex of the restrictions of the first surface.</summary>
    public readonly bool IsVertexOnS1 => _vtxonS1;

    /// <summary>The vertex on the first surface; throws Standard_DomainError unless <see cref="IsVertexOnS1"/>.</summary>
    public readonly HVertex VertexOnS1()
    {
        if (!_vtxonS1)
        {
            throw new DomainErrorException();
        }
        return _vS1!;
    }

    /// <summary>The arc of restriction of the first surface; throws Standard_DomainError unless <see cref="IsOnDomS1"/>.</summary>
    public readonly Curve2DAdaptorBase ArcOnS1()
    {
        if (!_onS1)
        {
            throw new DomainErrorException();
        }
        return _arcS1!;
    }

    /// <summary>The transition of the line with the arc on the first surface.</summary>
    public readonly WalkingTransition TransitionLineArc1()
    {
        if (!_onS1)
        {
            throw new DomainErrorException();
        }
        return _traline1;
    }

    /// <summary>The transition of the arc on the first surface with the line.</summary>
    public readonly WalkingTransition TransitionOnS1()
    {
        if (!_onS1)
        {
            throw new DomainErrorException();
        }
        return _tra1;
    }

    /// <summary>The parameter on the arc of the first surface.</summary>
    public readonly double ParameterOnArc1()
    {
        if (!_onS1)
        {
            throw new DomainErrorException();
        }
        return _prm1;
    }

    /// <summary>True if the point is on a boundary of the domain of the second surface.</summary>
    public readonly bool IsOnDomS2 => _onS2;

    /// <summary>True if the point is a vertex of the restrictions of the second surface.</summary>
    public readonly bool IsVertexOnS2 => _vtxonS2;

    /// <summary>The vertex on the second surface; throws Standard_DomainError unless <see cref="IsVertexOnS2"/>.</summary>
    public readonly HVertex VertexOnS2()
    {
        if (!_vtxonS2)
        {
            throw new DomainErrorException();
        }
        return _vS2!;
    }

    /// <summary>The arc of restriction of the second surface; throws Standard_DomainError unless <see cref="IsOnDomS2"/>.</summary>
    public readonly Curve2DAdaptorBase ArcOnS2()
    {
        if (!_onS2)
        {
            throw new DomainErrorException();
        }
        return _arcS2!;
    }

    /// <summary>The transition of the line with the arc on the second surface.</summary>
    public readonly WalkingTransition TransitionLineArc2()
    {
        if (!_onS2)
        {
            throw new DomainErrorException();
        }
        return _traline2;
    }

    /// <summary>The transition of the arc on the second surface with the line.</summary>
    public readonly WalkingTransition TransitionOnS2()
    {
        if (!_onS2)
        {
            throw new DomainErrorException();
        }
        return _tra2;
    }

    /// <summary>The parameter on the arc of the second surface.</summary>
    public readonly double ParameterOnArc2()
    {
        if (!_onS2)
        {
            throw new DomainErrorException();
        }
        return _prm2;
    }

    /// <summary>The point and its parameters on both surfaces.</summary>
    public readonly PntOn2S PntOn2S() => _pt;

    /// <summary>The parameters on both surfaces.</summary>
    public readonly void Parameters(out double u1, out double v1, out double u2, out double v2) =>
        _pt.Parameters(out u1, out v1, out u2, out v2);

    /// <summary>
    /// Writes the point as OCCT's <c>Dump</c> prints it (printf's <c>%+10.20f</c> for the numbers; the arcs' and
    /// vertices' addresses are their identity hash codes).
    /// </summary>
    public readonly void Dump(TextWriter theOut)
    {
        theOut.Write("----------- IntPatch_Point : " + '\n');
        _pt.Parameters(out double u1, out double v1, out double u2, out double v2);
        theOut.Write($"P({F(_pt.Value().X)},{F(_pt.Value().Y)},{F(_pt.Value().Z)}) UV1({F(u1)},{F(v1)})  "
                     + $"UV2({F(u2)},{F(v2)}) (Para:{F(_para)})\n");
        if (_onS1)
        {
            theOut.Write($"*OnS1*  par={F(_prm1)} arc1={Address(_arcS1)}");
        }
        if (_vtxonS1)
        {
            theOut.Write($" *Vtx1*  vtx1={Address(_vS1)}");
        }
        if (_onS1 || _vtxonS1)
        {
            theOut.Write("\n");
        }
        if (_onS2)
        {
            theOut.Write($"*OnS2*  par={F(_prm2)} arc2={Address(_arcS2)}");
        }
        if (_vtxonS2)
        {
            theOut.Write($" *Vtx2*  vtx2={Address(_vS2)}");
        }
        if (_onS2 || _vtxonS2)
        {
            theOut.Write("\n");
        }

        if (_onS1 || _onS2)
        {
            theOut.Write(" tgt:" + ((_tgt) ? 1 : 0) + "   mult:" + ((_mult) ? 1 : 0));
            if (_onS1)
            {
                theOut.Write("\n-traline1  : ");
                DumpTransition(theOut, _traline1);
                theOut.Write("  -tra1  : ");
                DumpTransition(theOut, _tra1);
            }
            if (_onS2)
            {
                theOut.Write("\n-traline2  : ");
                DumpTransition(theOut, _traline2);
                theOut.Write("  -tra2  : ");
                DumpTransition(theOut, _tra2);
            }
            theOut.Write('\n');
        }
        theOut.Write("\nToler = " + StreamUtilities.FormatReal(Tolerance(), 6)); // std::cout << double
        theOut.Write('\n');
    }

    // printf's "%+10.20f": a sign always, twenty decimals.
    private static string F(double value)
    {
        string str = value.ToString("F20", CultureInfo.InvariantCulture);
        return str.StartsWith('-') ? str : "+" + str;
    }

    private static string Address(object? theObject)
        => theObject is null ? "0000000000000000" : RuntimeHelpers.GetHashCode(theObject).ToString("X16", CultureInfo.InvariantCulture);

    private static void DumpTransition(TextWriter theOut, in WalkingTransition t)
    {
        WalkingTypeTrans typetrans = t.TransitionType();
        if (typetrans == WalkingTypeTrans.In)
        {
            theOut.Write(" In  ");
        }
        else if (typetrans == WalkingTypeTrans.Out)
        {
            theOut.Write(" Out ");
        }
        else if (typetrans == WalkingTypeTrans.Undecided)
        {
            theOut.Write(" Und.");
        }
        else if (typetrans == WalkingTypeTrans.Touch)
        {
            theOut.Write(" Touch ");
            if (t.Situation() == WalkingSituation.Inside)
            {
                theOut.Write(" Inside ");
            }
            else if (t.Situation() == WalkingSituation.Outside)
            {
                theOut.Write(" Outside");
            }
            else
            {
                theOut.Write(" Unknown");
            }
            if (t.IsOpposite)
            {
                theOut.Write("     Opposite ");
            }
            else
            {
                theOut.Write(" Non_Opposite ");
            }
        }
        if (typetrans != WalkingTypeTrans.Undecided)
        {
            if (t.IsTangent)
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
    public readonly bool Equals(PatchPoint other)
        => _pt.Equals(other._pt) && _para.Equals(other._para) && _tol.Equals(other._tol) && _tgt == other._tgt
           && _mult == other._mult && _onS1 == other._onS1 && _vtxonS1 == other._vtxonS1
           && ReferenceEquals(_vS1, other._vS1) && ReferenceEquals(_arcS1, other._arcS1)
           && _traline1.Equals(other._traline1) && _tra1.Equals(other._tra1) && _prm1.Equals(other._prm1)
           && _onS2 == other._onS2 && _vtxonS2 == other._vtxonS2 && ReferenceEquals(_vS2, other._vS2)
           && ReferenceEquals(_arcS2, other._arcS2) && _traline2.Equals(other._traline2) && _tra2.Equals(other._tra2)
           && _prm2.Equals(other._prm2);

    public override readonly bool Equals(object? obj) => obj is PatchPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pt, _para, _tol, _onS1, _onS2, _prm1, _prm2);

    public static bool operator ==(in PatchPoint left, in PatchPoint right) => left.Equals(right);

    public static bool operator !=(in PatchPoint left, in PatchPoint right) => !left.Equals(right);

    public override readonly string ToString()
    {
        StringWriter writer = new(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
