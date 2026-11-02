// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Utils.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Utils.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

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
public class XSControl_Utils
{
    /// <summary>
    /// the only use of this, is to allow a frontal to get one distinct "Utils" set per separate engine
    /// </summary>
    public XSControl_Utils()
    {
    }

    /// <summary>
    /// Just prints a line into the current Trace File. This allows to better characterise the various trace
    /// outputs, as desired.
    /// </summary>
    public void TraceLine(string line)
    {
        using Message_Messenger.StreamBuffer sout = Message.SendInfo();
        sout.Append(line).Append('\n');
    }

    /// <summary>
    /// Just prints a line or a set of lines into the current Trace File. &lt;lines&gt; can be a HAscii String
    /// (produces a print without ending line) or a HSequence (one new line per item)
    /// </summary>
    public void TraceLines(Standard_Transient? lines)
    {
        using Message_Messenger.StreamBuffer sout = Message.SendInfo();
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

    public bool IsKind(Standard_Transient? item, Type? what)
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
    public string TypeName(Standard_Transient? item, bool nopk = false)
    {
        if (item is null)
        {
            return "";
        }
        // OCCT first takes the item as a Standard_Type itself: a System.Type is never a Standard_Transient
        Type atype = item.DynamicType();
        string tn = atype.Name;
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

    public Standard_Transient? TraValue(Standard_Transient? seqval, int num)
    {
        Standard_Transient val = null;
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
        if (seqval is NCollection_HSequence<Standard_Transient> seqt)
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

    public NCollection_HSequence<Standard_Transient> NewSeqTra()
    {
        return new NCollection_HSequence<Standard_Transient>();
    }

    public void AppendTra(NCollection_HSequence<Standard_Transient>? seqval, Standard_Transient? traval)
    {
        seqval.Append(traval);
    }

    public string DateString(int yy, int mm, int dd, int hh, int mn, int ss)
    {
        Interface_MSG.TDate(out string ladate, yy, mm, dd, hh, mn, ss);
        return ladate;
    }

    public void DateValues(string text, ref int yy, ref int mm, ref int dd, ref int hh, ref int mn, ref int ss)
    {
        Interface_MSG.NDate(text, ref yy, ref mm, ref dd, ref hh, ref mn, ref ss);
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

    public string CStrValue(Standard_Transient? list, int num)
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
    public TopoDS_Shape CompoundFromSeq(NCollection_HSequence<TopoDS_Shape>? seqval)
    {
        BRep_Builder B = new BRep_Builder();
        TopoDS_Compound C = new TopoDS_Compound();
        B.MakeCompound(ref C);
        int i, n = seqval.Length();
        for (i = 1; i <= n; i++)
        {
            B.Add(C, seqval.Value(i));
        }
        return C;
    }

    /// <summary>
    /// Returns the type of a Shape : true type if &lt;compound&gt; is False If &lt;compound&gt; is True and
    /// &lt;shape&gt; is a Compound, iterates on its items. If all are of the same type, returns this type. Else,
    /// returns COMPOUND. If it is empty, returns SHAPE For a Null Shape, returns SHAPE
    /// </summary>
    public TopAbs_ShapeEnum ShapeType(TopoDS_Shape shape, bool compound)
    {
        if (shape.IsNull())
        {
            return TopAbs_SHAPE;
        }
        TopAbs_ShapeEnum res = shape.ShapeType();
        if (!compound || res != TopAbs_COMPOUND)
        {
            return res;
        }
        res = TopAbs_SHAPE;
        for (TopoDS_Iterator iter = new TopoDS_Iterator(shape); iter.More(); iter.Next())
        {
            TopoDS_Shape sh = iter.Value();
            if (sh.IsNull())
            {
                continue;
            }
            TopAbs_ShapeEnum typ = sh.ShapeType();
            if (typ == TopAbs_COMPOUND)
            {
                typ = ShapeType(sh, compound);
            }
            if (res == TopAbs_SHAPE)
            {
                res = typ;
                //   Equality: OK;  Pseudo-Equality: EDGE/WIRE or FACE/SHELL
            }
            else if (res == TopAbs_EDGE && typ == TopAbs_WIRE)
            {
                res = typ;
            }
            else if (res == TopAbs_WIRE && typ == TopAbs_EDGE)
            {
                continue;
            }
            else if (res == TopAbs_FACE && typ == TopAbs_SHELL)
            {
                res = typ;
            }
            else if (res == TopAbs_SHELL && typ == TopAbs_FACE)
            {
                continue;
            }
            else if (res != typ)
            {
                return TopAbs_COMPOUND;
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
    public TopoDS_Shape SortedCompound(TopoDS_Shape shape, TopAbs_ShapeEnum type, bool explore, bool compound)
    {
        if (shape.IsNull())
        {
            return shape;
        }
        TopAbs_ShapeEnum typ = shape.ShapeType();
        TopoDS_Shape sh = default, sh0;
        int nb = 0;

        //  Compound: we take it, either as is, or its content
        if (typ == TopAbs_COMPOUND || typ == TopAbs_COMPSOLID)
        {
            TopoDS_Compound C = new TopoDS_Compound();
            BRep_Builder B = new BRep_Builder();
            B.MakeCompound(ref C);
            for (TopoDS_Iterator it = new TopoDS_Iterator(shape); it.More(); it.Next())
            {
                sh0 = SortedCompound(it.Value(), type, explore, compound);
                if (sh0.IsNull())
                {
                    continue;
                }
                sh = sh0;
                typ = sh.ShapeType();
                if (typ == TopAbs_COMPOUND && !compound)
                {
                    for (TopoDS_Iterator it2 = new TopoDS_Iterator(sh); it2.More(); it2.Next())
                    {
                        nb++;
                        sh = it2.Value();
                        B.Add(C, sh);
                    }
                }
                else
                {
                    nb++;
                    B.Add(C, sh);
                }
            }
            if (nb == 0)
            {
                C.Nullify();
            }
            else if (nb == 1)
            {
                return sh;
            }
            return C;
        }

        //   Egalite : OK;  Pseudo-Egalite : EDGE/WIRE ou FACE/SHELL
        if (typ == type)
        {
            return shape;
        }
        if (typ == TopAbs_EDGE && type == TopAbs_WIRE)
        {
            BRep_Builder B = new BRep_Builder();
            TopoDS_Wire W = new TopoDS_Wire();
            B.MakeWire(ref W);
            B.Add(W, shape); // ne passe pas ! : TopoDS::Edge(shape)
            return W;
        }
        if (typ == TopAbs_FACE && type == TopAbs_SHELL)
        {
            BRep_Builder B = new BRep_Builder();
            TopoDS_Shell S = new TopoDS_Shell();
            B.MakeShell(ref S);
            B.Add(S, shape); // ne passe pas ! : TopoDS::Face(shape));
            S.Closed(BRep_Tool.IsClosed(S));
            return S;
        }

        //   Le reste : selon exploration
        if (!explore)
        {
            TopoDS_Shape nulsh = default;
            return nulsh;
        }

        //  Ici, on doit explorer
        //  SOLID + mode COMPOUND : reconduire les SHELLs
        if (typ == TopAbs_SOLID && compound)
        {
            TopoDS_Compound C = new TopoDS_Compound();
            BRep_Builder B = new BRep_Builder();
            B.MakeCompound(ref C);
            for (TopoDS_Iterator it = new TopoDS_Iterator(shape); it.More(); it.Next())
            {
                sh0 = SortedCompound(it.Value(), type, explore, compound);
                if (sh0.IsNull())
                {
                    continue;
                }
                sh = sh0;
                nb++;
                B.Add(C, sh);
            }
            if (nb == 0)
            {
                C.Nullify();
            }
            else if (nb == 1)
            {
                return sh;
            }
            return C;
        }

        //  Exploration classique
        TopoDS_Compound CC = new TopoDS_Compound();
        BRep_Builder BB = new BRep_Builder();
        BB.MakeCompound(ref CC);
        for (TopExp_Explorer aExp = new TopExp_Explorer(shape, type); aExp.More(); aExp.Next())
        {
            nb++;
            sh = aExp.Current();
            BB.Add(CC, sh);
        }
        if (nb == 0)
        {
            CC.Nullify();
        }
        else if (nb == 1)
        {
            return sh;
        }
        return CC;
    }

    public TopoDS_Shape ShapeValue(NCollection_HSequence<TopoDS_Shape>? seqval, int num)
    {
        TopoDS_Shape shape = default;
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

    public NCollection_HSequence<TopoDS_Shape> NewSeqShape()
    {
        return new NCollection_HSequence<TopoDS_Shape>();
    }

    public void AppendShape(NCollection_HSequence<TopoDS_Shape>? seqval, TopoDS_Shape shape)
    {
        seqval.Append(shape);
    }

    /// <summary>
    /// Creates a Transient Object from a Shape : it is either a Binder (used by functions which require a
    /// Transient but can process a Shape, such as viewing functions) or a HShape (according to hs) Default is a
    /// HShape
    /// </summary>
    public Standard_Transient ShapeBinder(TopoDS_Shape shape, bool hs = true)
    {
        if (hs)
        {
            return new TopoDS_HShape(shape);
        }
        else
        {
            return new TransferBRep_ShapeBinder(shape);
        }
    }

    /// <summary>
    /// From a Transient, returns a Shape. In fact, recognizes ShapeBinder ShapeMapper and HShape
    /// </summary>
    public TopoDS_Shape BinderShape(Standard_Transient? tr)
    {
        TopoDS_Shape sh = default;
        if (tr is Transfer_Binder sb)
        {
            return TransferBRep.ShapeResult(sb);
        }
        if (tr is TransferBRep_ShapeMapper sm)
        {
            return sm.Value();
        }
        if (tr is TopoDS_HShape hs)
        {
            return hs.Shape();
        }
        return sh;
    }

    public int SeqLength(Standard_Transient? seqval)
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
        if (seqval is NCollection_HSequence<Standard_Transient> seqt)
        {
            return seqt.Length();
        }
        if (seqval is NCollection_HSequence<TopoDS_Shape> seqh)
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

    public Standard_Transient? SeqToArr(Standard_Transient? seqval, int first = 1)
    {
        int i, lng;
        Standard_Transient val = null;
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
        if (seqval is NCollection_HSequence<Standard_Transient> seqt)
        {
            lng = seqt.Length();
            NCollection_HArray1<Standard_Transient> arrt =
                new NCollection_HArray1<Standard_Transient>(first, lng - first + 1);
            for (i = 1; i <= lng; i++)
            {
                arrt.SetValue(i - first + 1, seqt.Value(i));
            }
            return arrt;
        }
        throw new Standard_TypeMismatch("XSControl_Utils::SeqToArr");
    }

    public Standard_Transient? ArrToSeq(Standard_Transient? arrval)
    {
        int i, first, last;
        Standard_Transient val = null;
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
        if (arrval is NCollection_HArray1<Standard_Transient> arrt)
        {
            first = arrt.Lower();
            last = arrt.Upper();
            NCollection_HSequence<Standard_Transient> seqt =
                new NCollection_HSequence<Standard_Transient>();
            for (i = first; i <= last; i++)
            {
                seqt.Append(arrt.Value(i));
            }
            return seqt;
        }
        throw new Standard_TypeMismatch("XSControl_Utils::ArrToSeq");
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
