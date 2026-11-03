// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Utils.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Utils.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// This class provides various useful utility routines, to facilitate handling of most common data
/// structures : transients (type, type name ...), strings (ascii or extended, pointed or handled or ...),
/// shapes (reading, writing, testing ...), sequences &amp; arrays (of strings, of transients, of shapes ...),
/// ... Also it gives some helps on some data structures from XSTEP, such as printing on standard trace file,
/// recognizing most currently used auxiliary types (Binder, Mapper ...)
/// </summary>
/// <remarks>
/// OCCT.NET: the extended
/// strings members (<c>ToEString</c>, <c>ToXString</c>, <c>AsciiToExtended</c>, <c>IsAscii</c>,
/// <c>ExtendedToAscii</c>, <c>EStrValue</c>, <c>NewSeqEStr</c>, <c>AppendEStr</c> and the extended strings cases of
/// the others) are left out: the port has no TCollection_ExtendedString, .NET strings are UTF-16 (decision 1221).
/// The shapes it builds are left as built (a tool's results, as OCCT's: the caller publishes them).
/// </remarks>
public class Utils
{
    /// <summary>
    /// the only use of this, is to allow a frontal to get one distinct "Utils" set per separate engine
    /// </summary>
    public Utils()
    {
    }

    /// <summary>
    /// Just prints a line into the current Trace File. This allows to better characterise the various trace
    /// outputs, as desired.
    /// </summary>
    public void TraceLine(string line)
    {
        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
        sout.Append(line).Append('\n');
    }

    /// <summary>
    /// Just prints a line or a set of lines into the current Trace File. &lt;lines&gt; can be a HAscii String
    /// (produces a print without ending line) or a HSequence (one new line per item)
    /// </summary>
    public void TraceLines(Transient? lines)
    {
        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
        int i, nb;
        if (lines is NCollection_HSequence<TCollection_HAsciiString> linha)
        {
            nb = linha.Length();
            for (i = 1; i <= nb; i++)
            {
                if (linha.Value(i) is not null)
                {
                    sout.Append(linha.Value(i).ToCString()).Append('\n');
                }
            }
            return;
        }
        if (lines is NCollection_HSequence<string> lina)
        {
            nb = lina.Length();
            for (i = 1; i <= nb; i++)
            {
                sout.Append(lina.Value(i)).Append('\n');
            }
            return;
        }
        if (lines is TCollection_HAsciiString lin1a)
        {
            sout.Append(lin1a.String());
        }
    }

    public bool IsKind(Transient? item, Type? what)
    {
        if (item is null)
        {
            return false;
        }
        if (what is null)
        {
            return false;
        }
        return item.IsKind(what);
    }

    /// <summary>
    /// Returns the name of the dynamic type of an object, i.e. : If it is a Type, its Name If it is a object
    /// not a type, the Name of its DynamicType If it is Null, an empty string If &lt;nopk&gt; is False (D),
    /// gives complete name If &lt;nopk&gt; is True, returns class name without package
    /// </summary>
    public string TypeName(Transient? item, bool nopk = false)
    {
        if (item is null)
        {
            return "";
        }
        // OCCT first takes the item as a Standard_Type itself: a System.Type is never a Standard_Transient
        Type atype = item.DynamicType();
        string tn = global::Kerf.Engine.Foundation.OcctTypeNames.Of(atype);
        if (!nopk)
        {
            return tn;
        }
        for (int i = 0; i < tn.Length; i++)
        {
            if (tn[i] == '_')
            {
                return tn.Substring(i + 1);
            }
        }
        return tn;
    }

    public Transient? TraValue(Transient? seqval, int num)
    {
        Transient val = null;
        if (num < 1)
        {
            return val;
        }
        if (seqval is null)
        {
            return val;
        }
        if (seqval is NCollection_HSequence<TCollection_HAsciiString> seqs)
        {
            if (num <= seqs.Length())
            {
                val = seqs.Value(num);
            }
            return val;
        }
        if (seqval is NCollection_HSequence<Transient> seqt)
        {
            if (num <= seqt.Length())
            {
                val = seqt.Value(num);
            }
            return val;
        }
        //  throw Standard_TypeMismatch("XSControl_Utils::SeqTraValue");
        return val;
    }

    public NCollection_HSequence<Transient> NewSeqTra()
    {
        return new NCollection_HSequence<Transient>();
    }

    public void AppendTra(NCollection_HSequence<Transient>? seqval, Transient? traval)
    {
        seqval.Append(traval);
    }

    public string DateString(int yy, int mm, int dd, int hh, int mn, int ss)
    {
        FrameworkMsg.TDate(out string ladate, yy, mm, dd, hh, mn, ss);
        return ladate;
    }

    public void DateValues(string text, ref int yy, ref int mm, ref int dd, ref int hh, ref int mn, ref int ss)
    {
        FrameworkMsg.NDate(text, ref yy, ref mm, ref dd, ref hh, ref mn, ref ss);
    }

    public string ToCString(TCollection_HAsciiString? strval)
    {
        // JR/Hp
        return strval is null ? "" : strval.ToCString();
    }

    public string ToCString(string strval)
    {
        return strval;
    }

    public TCollection_HAsciiString ToHString(string strcon)
    {
        return new TCollection_HAsciiString(strcon);
    }

    public string ToAString(string strcon)
    {
        return strcon;
    }

    public string CStrValue(Transient? list, int num)
    {
        if (list is NCollection_HSequence<TCollection_HAsciiString> linha)
        {
            // JR/Hp
            return num > linha.Length() ? "" : linha.Value(num).ToCString();
        }

        if (list is NCollection_HSequence<string> lina)
        {
            // JR/Hp
            return num > lina.Length() ? "" : lina.Value(num);
        }

        if (list is TCollection_HAsciiString lin1a)
        {
            return lin1a.ToCString();
        }
        return "";
    }

    public NCollection_HSequence<TCollection_HAsciiString> NewSeqCStr()
    {
        return new NCollection_HSequence<TCollection_HAsciiString>();
    }

    public void AppendCStr(NCollection_HSequence<TCollection_HAsciiString>? seqval, string strval)
    {
        seqval.Append(new TCollection_HAsciiString(strval));
    }

    /// <summary>
    /// Converts a list of Shapes to a Compound (a kind of Shape)
    /// </summary>
    public Shape CompoundFromSeq(NCollection_HSequence<Shape>? seqval)
    {
        BrepBuilder b = new BrepBuilder();
        Compound c = new Compound();
        b.MakeCompound(ref c);
        int i, n = seqval.Length();
        for (i = 1; i <= n; i++)
        {
            b.Add(c, seqval.Value(i));
        }
        return c;
    }

    /// <summary>
    /// Returns the type of a Shape : true type if &lt;compound&gt; is False If &lt;compound&gt; is True and
    /// &lt;shape&gt; is a Compound, iterates on its items. If all are of the same type, returns this type. Else,
    /// returns COMPOUND. If it is empty, returns SHAPE For a Null Shape, returns SHAPE
    /// </summary>
    public ShapeType ShapeType(Shape shape, bool compound)
    {
        if (shape.IsNull)
        {
            return global::Kerf.Topology.ShapeType.Shape;
        }
        ShapeType res = shape.ShapeType();
        if (!compound || res != global::Kerf.Topology.ShapeType.Compound)
        {
            return res;
        }
        res = global::Kerf.Topology.ShapeType.Shape;
        for (Iterator iter = new Iterator(shape); iter.More(); iter.Next())
        {
            Shape sh = iter.Value();
            if (sh.IsNull)
            {
                continue;
            }
            ShapeType typ = sh.ShapeType();
            if (typ == global::Kerf.Topology.ShapeType.Compound)
            {
                typ = ShapeType(sh, compound);
            }
            if (res == global::Kerf.Topology.ShapeType.Shape)
            {
                res = typ;
                //   Equality: OK;  Pseudo-Equality: EDGE/WIRE or FACE/SHELL
            }
            else if (res == global::Kerf.Topology.ShapeType.Edge && typ == global::Kerf.Topology.ShapeType.Wire)
            {
                res = typ;
            }
            else if (res == global::Kerf.Topology.ShapeType.Wire && typ == global::Kerf.Topology.ShapeType.Edge)
            {
                continue;
            }
            else if (res == global::Kerf.Topology.ShapeType.Face && typ == global::Kerf.Topology.ShapeType.Shell)
            {
                res = typ;
            }
            else if (res == global::Kerf.Topology.ShapeType.Shell && typ == global::Kerf.Topology.ShapeType.Face)
            {
                continue;
            }
            else if (res != typ)
            {
                return global::Kerf.Topology.ShapeType.Compound;
            }
        }
        return res;
    }

    /// <summary>
    /// From a Shape, builds a Compound as follows : explores it level by level If &lt;explore&gt; is False, only
    /// COMPOUND items. Else, all items Adds to the result, shapes which comply to &lt;type&gt; + if &lt;type&gt;
    /// is WIRE, considers free edges (and makes wires) + if &lt;type&gt; is SHELL, considers free faces (and
    /// makes shells) If &lt;compound&gt; is True, gathers items in compounds which correspond to starting
    /// COMPOUND,SOLID or SHELL containers, or items directly contained in a Compound
    /// </summary>
    public Shape SortedCompound(Shape shape, ShapeType type, bool explore, bool compound)
    {
        if (shape.IsNull)
        {
            return shape;
        }
        ShapeType typ = shape.ShapeType();
        Shape sh = default, sh0;
        int nb = 0;

        //  Compound: we take it, either as is, or its content
        if (typ == global::Kerf.Topology.ShapeType.Compound || typ == global::Kerf.Topology.ShapeType.CompSolid)
        {
            Compound c = new Compound();
            BrepBuilder b = new BrepBuilder();
            b.MakeCompound(ref c);
            for (Iterator it = new Iterator(shape); it.More(); it.Next())
            {
                sh0 = SortedCompound(it.Value(), type, explore, compound);
                if (sh0.IsNull)
                {
                    continue;
                }
                sh = sh0;
                typ = sh.ShapeType();
                if (typ == global::Kerf.Topology.ShapeType.Compound && !compound)
                {
                    for (Iterator it2 = new Iterator(sh); it2.More(); it2.Next())
                    {
                        nb++;
                        sh = it2.Value();
                        b.Add(c, sh);
                    }
                }
                else
                {
                    nb++;
                    b.Add(c, sh);
                }
            }
            if (nb == 0)
            {
                c.Nullify();
            }
            else if (nb == 1)
            {
                return sh;
            }
            return c;
        }

        //   Egalite : OK;  Pseudo-Egalite : EDGE/WIRE ou FACE/SHELL
        if (typ == type)
        {
            return shape;
        }
        if (typ == global::Kerf.Topology.ShapeType.Edge && type == global::Kerf.Topology.ShapeType.Wire)
        {
            BrepBuilder b = new BrepBuilder();
            Wire w = new Wire();
            b.MakeWire(ref w);
            b.Add(w, shape); // ne passe pas ! : TopoDS::Edge(shape)
            return w;
        }
        if (typ == global::Kerf.Topology.ShapeType.Face && type == global::Kerf.Topology.ShapeType.Shell)
        {
            BrepBuilder b = new BrepBuilder();
            Shell s = new Shell();
            b.MakeShell(ref s);
            b.Add(s, shape); // ne passe pas ! : TopoDS::Face(shape));
            s.Closed(Tool.IsClosed(s));
            return s;
        }

        //   Le reste : selon exploration
        if (!explore)
        {
            Shape nulsh = default;
            return nulsh;
        }

        //  Ici, on doit explorer
        //  SOLID + mode COMPOUND : reconduire les SHELLs
        if (typ == global::Kerf.Topology.ShapeType.Solid && compound)
        {
            Compound c = new Compound();
            BrepBuilder b = new BrepBuilder();
            b.MakeCompound(ref c);
            for (Iterator it = new Iterator(shape); it.More(); it.Next())
            {
                sh0 = SortedCompound(it.Value(), type, explore, compound);
                if (sh0.IsNull)
                {
                    continue;
                }
                sh = sh0;
                nb++;
                b.Add(c, sh);
            }
            if (nb == 0)
            {
                c.Nullify();
            }
            else if (nb == 1)
            {
                return sh;
            }
            return c;
        }

        //  Exploration classique
        Compound cc = new Compound();
        BrepBuilder bb = new BrepBuilder();
        bb.MakeCompound(ref cc);
        for (Explorer exp = new Explorer(shape, type); exp.More(); exp.Next())
        {
            nb++;
            sh = exp.Current();
            bb.Add(cc, sh);
        }
        if (nb == 0)
        {
            cc.Nullify();
        }
        else if (nb == 1)
        {
            return sh;
        }
        return cc;
    }

    public Shape ShapeValue(NCollection_HSequence<Shape>? seqval, int num)
    {
        Shape shape = default;
        if (seqval is null)
        {
            return shape;
        }
        if (num > 0 && num <= seqval.Length())
        {
            shape = seqval.Value(num);
        }
        return shape;
    }

    public NCollection_HSequence<Shape> NewSeqShape()
    {
        return new NCollection_HSequence<Shape>();
    }

    public void AppendShape(NCollection_HSequence<Shape>? seqval, Shape shape)
    {
        seqval.Append(shape);
    }

    /// <summary>
    /// Creates a Transient Object from a Shape : it is either a Binder (used by functions which require a
    /// Transient but can process a Shape, such as viewing functions) or a HShape (according to hs) Default is a
    /// HShape
    /// </summary>
    public Transient ShapeBinder(Shape shape, bool hs = true)
    {
        if (hs)
        {
            return new TopoDS_HShape(shape);
        }
        else
        {
            return new ShapeBinder(shape);
        }
    }

    /// <summary>
    /// From a Transient, returns a Shape. In fact, recognizes ShapeBinder ShapeMapper and HShape
    /// </summary>
    public Shape BinderShape(Transient? tr)
    {
        Shape sh = default;
        if (tr is Binder sb)
        {
            return global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(sb);
        }
        if (tr is ShapeMapper sm)
        {
            return sm.Value();
        }
        if (tr is TopoDS_HShape hs)
        {
            return hs.Shape();
        }
        return sh;
    }

    public int SeqLength(Transient? seqval)
    {
        if (seqval is null)
        {
            return 0;
        }
        if (seqval is NCollection_HSequence<TCollection_HAsciiString> seqs)
        {
            return seqs.Length();
        }
        if (seqval is NCollection_HSequence<string> seqa)
        {
            return seqa.Length();
        }
        if (seqval is NCollection_HSequence<Transient> seqt)
        {
            return seqt.Length();
        }
        if (seqval is NCollection_HSequence<Shape> seqh)
        {
            return seqh.Length();
        }
        if (seqval is NCollection_HSequence<int> seqi)
        {
            return seqi.Length();
        }
        //  throw Standard_TypeMismatch("XSControl_Utils::SeqLength");
        return 0;
    }

    public Transient? SeqToArr(Transient? seqval, int first = 1)
    {
        int i, lng;
        Transient val = null;
        if (seqval is null)
        {
            return val;
        }
        if (seqval is NCollection_HSequence<TCollection_HAsciiString> seqs)
        {
            lng = seqs.Length();
            NCollection_HArray1<TCollection_HAsciiString> arrs =
                new NCollection_HArray1<TCollection_HAsciiString>(first, lng - first + 1);
            for (i = 1; i <= lng; i++)
            {
                arrs.SetValue(i - first + 1, seqs.Value(i));
            }
            return arrs;
        }
        if (seqval is NCollection_HSequence<Transient> seqt)
        {
            lng = seqt.Length();
            NCollection_HArray1<Transient> arrt =
                new NCollection_HArray1<Transient>(first, lng - first + 1);
            for (i = 1; i <= lng; i++)
            {
                arrt.SetValue(i - first + 1, seqt.Value(i));
            }
            return arrt;
        }
        throw new TypeMismatchException("XSControl_Utils::SeqToArr");
    }

    public Transient? ArrToSeq(Transient? arrval)
    {
        int i, first, last;
        Transient val = null;
        if (arrval is null)
        {
            return val;
        }
        if (arrval is NCollection_HArray1<TCollection_HAsciiString> arrs)
        {
            first = arrs.Lower();
            last = arrs.Upper();
            NCollection_HSequence<TCollection_HAsciiString> seqs =
                new NCollection_HSequence<TCollection_HAsciiString>();
            for (i = first; i <= last; i++)
            {
                seqs.Append(arrs.Value(i));
            }
            return seqs;
        }
        if (arrval is NCollection_HArray1<Transient> arrt)
        {
            first = arrt.Lower();
            last = arrt.Upper();
            NCollection_HSequence<Transient> seqt =
                new NCollection_HSequence<Transient>();
            for (i = first; i <= last; i++)
            {
                seqt.Append(arrt.Value(i));
            }
            return seqt;
        }
        throw new TypeMismatchException("XSControl_Utils::ArrToSeq");
    }

    public int SeqIntValue(NCollection_HSequence<int>? seqval, int num)
    {
        if (seqval is null)
        {
            return 0;
        }
        return seqval.Value(num);
    }
}
