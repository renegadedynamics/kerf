// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepWriter.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepWriter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;
using System.Globalization;
using System.Text;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// Manages atomic file writing, under control of StepModel (for general organisation of file) and each class of
/// Transient (for its own parameters): prepares text to be written then writes it. A stream cannot be used because
/// Step limits line length at 72. In more, a specific object offers more appropriate functions.
/// </summary>
/// <remarks>
/// <para>
/// The lines are built as the file's bytes, one char per byte (decision 1174), so that their 72-character limit
/// counts bytes as OCCT's does: the texts of the model (<see cref="Send(string)"/>, comments), which OCCT holds in
/// UTF-8 and the port as .NET strings (decision 1192), are written as their UTF-8 bytes; the others (types, enums,
/// <see cref="SendString(string)"/>, the raw parameters of undefined entities) are taken as they are.
/// <see cref="Print(Stream)"/> writes those bytes; <see cref="Line"/> returns them one char per byte.
/// </para>
/// <para>A C++ value class, a C# class here (decision 1185).</para>
/// </remarks>
internal class StepWriter
{
    private const int StepLong = 72;
    // Enough for "#", an int and " = "
    private const int IdentSize = 16;
    // StepLong: maximum length of a Step file line

    // Literal constants (useful for performance AND MEMORY)
    private const string Textscope = " &SCOPE";
    private const string Textendscope = "        ENDSCOPE";
    private const string Textcomm = "  /*  ";
    private const string Textendcomm = "  */";
    private const string Textlist = "(";
    private const string Textendlist = ")";
    private const string Textendent = ");";
    private const string Textparam = ",";
    private const string Textundef = "$";
    private const string Textderived = "*";
    private const string Texttrue = ".T.";
    private const string Textfalse = ".F.";
    private const string Textunknown = ".U.";

    private readonly StepModel _themodel;
    private readonly NCollection_HSequence<TCollection_HAsciiString> _thefile;
    private readonly LineBuffer _thecurr;
    private bool _thesect;
    private bool _thecomm;
    private bool _thefirst;
    private bool _themult;
    private int _thelevel;
    private bool _theindent;
    private int _theindval;
    private int _thetypmode;
    private readonly FloatWriter _thefloatw;
    private readonly CheckIterator _thechecks = new();
    private int _thenum;
    private int _thelabmode;
    private NCollection_HArray1<int> _thescopebeg;
    private NCollection_HArray1<int> _thescopeend;
    private NCollection_HArray1<int> _thescopenext;

    /// <summary>Creates an empty StepWriter from a StepModel. The StepModel provides the Number of Entities, as identifiers for File.</summary>
    public StepWriter(StepModel amodel)
    {
        _thecurr = new LineBuffer(StepLong);
        _thefloatw = new FloatWriter(12);
        _themodel = amodel;
        _thelabmode = _thetypmode = 0;
        _thefile = new NCollection_HSequence<TCollection_HAsciiString>();
        _thesect = false;
        _thefirst = true;
        _themult = false;
        _thecomm = false;
        _thelevel = _theindval = 0;
        _theindent = false;
        // Floating point format: delegated to FloatWriter
    }

    //  ....                Float Sending Control                ....

    /// <summary>
    /// Returns the embedded FloatWriter, which controls sending Reals. Use this method to access FloatWriter in order
    /// to consult or change its options (MainFormat, FormatForRange, ZeroSuppress).
    /// </summary>
    public FloatWriter FloatWriter()
    {
        return _thefloatw;
    } // refer to it

    /// <summary>
    /// ModeLabel controls how to display entity ids: 0 (D) gives entity number in the model, 1 gives the already
    /// recorded label (else, its number). Warning: conflicts are not controlled.
    /// </summary>
    public ref int LabelMode()
    {
        return ref _thelabmode;
    }

    /// <summary>
    /// TypeMode controls the type form to use: 0 (D) for normal long form, 1 for short form (if a type name has no
    /// short form, normal long form is then used).
    /// </summary>
    public ref int TypeMode()
    {
        return ref _thetypmode;
    }

    //  ....                Scope Description (BEFORE Sending)               ....

    /// <summary>
    /// Declares the Entity Number numscope to correspond to a Scope which contains the Entity Number numin. Several
    /// calls to the same numscope add Entities in this Scope, in this order. Error if numin is already declared in
    /// the Scope.
    /// </summary>
    public void SetScope(int numscope, int numin)
    {
        int nb = _themodel.EntityCount;
        if (numscope <= 0 || numscope > nb || numin <= 0 || numin > nb)
        {
            throw new InterfaceMismatchException("StepWriter : SetScope, out of range");
        }
        if (_thescopenext is null)
        {
            _thescopebeg = new NCollection_HArray1<int>(1, nb);
            _thescopebeg.Init(0);
            _thescopeend = new NCollection_HArray1<int>(1, nb);
            _thescopeend.Init(0);
            _thescopenext = new NCollection_HArray1<int>(1, nb);
            _thescopenext.Init(0);
        }
        else if (_thescopenext.Value(numin) != 0)
        {
            throw new InterfaceMismatchException("StepWriter : SetScope, already set");
        }
        _thescopenext.SetValue(numin, -1); // new end of scope
        if (_thescopebeg.Value(numscope) == 0)
        {
            _thescopebeg.SetValue(numscope, numin);
        }
        int lastin = _thescopeend.Value(numscope);
        if (lastin > 0)
        {
            _thescopenext.SetValue(lastin, numin);
        }
        _thescopeend.SetValue(numscope, numin);
    }

    /// <summary>Returns True if an Entity identified by its Number is in a Scope.</summary>
    public bool IsInScope(int num)
    {
        if (_thescopenext is null)
        {
            return false;
        }
        return (_thescopenext.Value(num) != 0);
    }

    //  ###########################################################################
    //  ##    ##    ##    ##        SENDING SECTIONS        ##    ##    ##    ##

    /// <summary>
    /// Sends the complete Model, included HEADER and DATA Sections. Works with a WriterLib defined through a Protocol.
    /// If headeronly is given True, only the HEADER Section is sent (used to Dump the Header of a StepModel).
    /// </summary>
    public void SendModel(StepProtocol protocol, bool headeronly = false)
    {
        WriterLib lib = new WriterLib(protocol);
        if (!headeronly)
        {
            _thefile.Append(new TCollection_HAsciiString("ISO-10303-21;"));
        }
        SendHeader();

        //  ....                Header: sequence of entities without Ident                ....

        EntityIterator header = _themodel.Header();
        _thenum = 0;
        for (header.Start(); header.More(); header.Next())
        {
            Transient anent = header.Value();

            //   Write Entity via Lib  (similaire a SendEntity)
            if (lib.Select(anent, out ReadWriteModule module, out int cn))
            {
                if (module.IsComplex(cn))
                {
                    StartComplex();
                }
                else
                {
                    string styp = "";
                    if (_thetypmode > 0)
                    {
                        styp = module.ShortType(cn);
                    }
                    if (styp.Length == 0)
                    {
                        styp = module.StepType(cn);
                    }
                    StartEntity(styp);
                }
                module.WriteStep(cn, this, anent);
                if (module.IsComplex(cn))
                {
                    EndComplex();
                }
            }
            else
            {
                //    Not found above ... try UndefinedEntity
                if (anent is not UndefinedEntity und)
                {
                    continue;
                }
                if (und.IsComplex)
                {
                    StartComplex();
                }
                und.WriteParams(this);
                if (und.IsComplex)
                {
                    EndComplex();
                }
            }
            EndEntity();
        }
        EndSec();
        if (headeronly)
        {
            return;
        }

        //  Data: Like Header but with Idents ... otherwise the code is the same
        SendData();

        // ....                    Global Errors (if any)                    ....

        FrameworkCheck achglob = _themodel.GlobalCheck();
        int nbfails = achglob.FailCount;
        if (nbfails > 0)
        {
            Comment(true);
            SendComment("GLOBAL FAIL MESSAGES,  recorded at Read time :");
            for (int ifail = 1; ifail <= nbfails; ifail++)
            {
                SendComment(achglob.Fail(ifail));
            }
            Comment(false);
            NewLine(false);
        }

        //  ....                Output Entities one by one                ....

        int nb = _themodel.EntityCount;
        for (int i = 1; i <= nb; i++)
        {
            // Kerf: sending a large model has no progress range (decision 1855)
            if ((i & 255) == 0)
            {
                Cancellation.ThrowIfRequested();
            }

            //    Main list: we don't send Entities that are in a Scope
            //    They will be sent through the Scope that contains them
            if (_thescopebeg is not null)
            {
                if (_thescopenext.Value(i) != 0)
                {
                    continue;
                }
            }
            SendEntity(i, lib);
        }

        EndSec();
        EndFile();
    }

    //  ....                FILE DIVISION INTO SECTIONS                ....

    /// <summary>Begins model header.</summary>
    public void SendHeader()
    {
        NewLine(false);
        _thefile.Append(new TCollection_HAsciiString("HEADER;"));
        _thesect = true;
    }

    /// <summary>Begins data section; error if EndSec was not set.</summary>
    public void SendData()
    {
        if (_thesect)
        {
            throw new InterfaceMismatchException("StepWriter : Data section");
        }
        NewLine(false);
        _thefile.Append(new TCollection_HAsciiString("DATA;"));
        _thesect = true;
    }

    /// <summary>Sets end of section; to be done before passing to next one.</summary>
    public void EndSec()
    {
        _thefile.Append(new TCollection_HAsciiString("ENDSEC;"));
        _thesect = false;
    }

    /// <summary>Sets end of file; error is EndSec was not set.</summary>
    public void EndFile()
    {
        if (_thesect)
        {
            throw new InterfaceMismatchException("StepWriter : EndFile");
        }
        NewLine(false);
        _thefile.Append(new TCollection_HAsciiString("END-ISO-10303-21;"));
        _thesect = false;
    }

    //  ....                        SENDING AN ENTITY                        ....

    /// <summary>Send an Entity of the Data Section. If it corresponds to a Scope, also Sends the Scope information and contained Items.</summary>
    public void SendEntity(int num, WriterLib lib)
    {
        string lident;
        Transient anent = _themodel.Entity(num);
        int idnum = num, idtrue = 0;

        //   themodel->Number(anent) and/or IdentLabel(anent)
        if (_thelabmode > 0)
        {
            idtrue = _themodel.IdentLabel(anent);
        }
        if (_thelabmode == 1)
        {
            idnum = idtrue;
        }
        if (idnum == 0)
        {
            idnum = num;
        }
        if (_thelabmode < 2 || idnum == idtrue)
        {
            //  skl 29.01.2003. Kerf: written in place, not through Sprintf (decision 1728)
            _thecurr.Clear();
            Span<char> aText = stackalloc char[IdentSize];
            aText[0] = '#';
            idnum.TryFormat(aText[1..], out int aLength, default, CultureInfo.InvariantCulture);
            " = ".CopyTo(aText[(1 + aLength)..]);
            _thecurr.Add(aText[..(1 + aLength + 3)]);
        }
        else
        {
            lident = global::Kerf.Engine.Foundation.Standard.Sprintf("%d:#%d = ", idnum, idtrue); // skl 29.01.2003
            //  SendIdent reused, lident has just been calculated
            _thecurr.Clear();
            _thecurr.Add(lident);
        }
        _themult = false;

        //  ....        Processing of Potential Scope
        if (_thescopebeg is not null)
        {
            int numin = _thescopebeg.Value(num);
            if (numin != 0)
            {
                SendScope();
                for (int nument = numin; numin > 0; nument = numin)
                {
                    SendEntity(nument, lib);
                    numin = _thescopenext.Value(nument);
                }
                SendEndscope();
            }
        }

        //  ....        Sending the Entity proper

        //   Write Entity via Lib
        _thenum = num;
        if (_themodel.IsRedefinedContent(num))
        {
            //    Error Entity: Write the Content + Errors as Comments
            ReportEntity rep = _themodel.ReportEntity(num);
            UndefinedEntity und = rep.Content() as UndefinedEntity;
            if (und is null)
            {
                _thechecks.CCheck(num).AddFail("Erroneous Entity, Content lost");
                StartEntity("!?LOST_DATA");
            }
            else
            {
                _thechecks.CCheck(num).AddWarning("Erroneous Entity, equivalent content");
                if (und.IsComplex)
                {
                    AddCString(" (", 2);
                }
                und.WriteParams(this);
                if (und.IsComplex)
                {
                    AddCString(") ", 2);
                } // thelevel --; }
            }
            EndEntity(); // BEFORE Comments
            NewLine(false);
            Comment(true);
            if (und is null)
            {
                SendComment("   ERRONEOUS ENTITY, DATA LOST");
            }
            SendComment("On Entity above, Fail Messages recorded at Read time :");
            FrameworkCheck ach = rep.Check();
            int nbfails = ach.FailCount;
            for (int ifail = 1; ifail <= nbfails; ifail++)
            {
                SendComment(ach.Fail(ifail));
            }
            Comment(false);
            NewLine(false);

            //    Normal case
        }
        else if (lib.Select(anent, out ReadWriteModule module, out int cn))
        {
            if (module.IsComplex(cn))
            {
                StartComplex();
            }
            else
            {
                string styp = "";
                if (_thetypmode > 0)
                {
                    styp = module.ShortType(cn);
                }
                if (styp.Length == 0)
                {
                    styp = module.StepType(cn);
                }
                StartEntity(styp);
            }
            module.WriteStep(cn, this, anent);
            if (module.IsComplex(cn))
            {
                EndComplex();
            }
            EndEntity();
        }
        else
        {
            //    Not found above ... try UndefinedEntity
            if (anent is not UndefinedEntity und)
            {
                return;
            }
            if (und.IsComplex)
            {
                StartComplex();
            }
            und.WriteParams(this);
            if (und.IsComplex)
            {
                EndComplex();
            }
            EndEntity();
        }
    }

    //  ###########################################################################
    //  ##    ##    ##        TEXT CONSTITUTION FOR SENDING        ##    ##    ##

    //  Go to next line. Empty line not counted unless evenempty == true

    /// <summary>Flushes current line; if empty, flushes it (defines a new empty line) if evenempty is True; else, skips it.</summary>
    public void NewLine(bool evenempty)
    {
        if (evenempty || _thecurr.Length() > 0)
        {
            _thefile.Append(_thecurr.Moved());
        }
        int indst = _thelevel * 2;
        if (_theindent)
        {
            indst += _theindval;
        }
        _thecurr.SetInitial(indst);
        _thecurr.Clear();
    }

    //  Group current line with previous; remains current except if newline
    //  == true, in which case we start a new line
    //  Does nothing if: corresponding total > StepLong or start or end of entity

    /// <summary>
    /// Joins current line to last one, only if new length is 72 max. If newline is True, a new current line begins;
    /// else, current line is set to the last line (once joined) itself an can be completed.
    /// </summary>
    public void JoinLast(bool newline)
    {
        _thecurr.SetKeep();
    }

    /// <summary>
    /// Asks that further indentations will begin at position of entity first opening bracket; else they begin at zero
    /// (def). For each sublist level, two more blanks are added at beginning (except for text continuation, which must
    /// begin at true zero).
    /// </summary>
    public void Indent(bool onent)
    {
        _theindent = onent;
    }

    /// <summary>
    /// Begins an entity with an ident plus '=' (at beginning of line); entity ident is its Number given by the
    /// containing Model. Warning: ident must be, either Number or Label, according LabelMode.
    /// </summary>
    public void SendIdent(int ident)
    {
        string lident = global::Kerf.Engine.Foundation.Standard.Sprintf("#%d =", ident);
        _thecurr.Clear();
        _thecurr.Add(lident);
        _themult = false;
    }

    /// <summary>Sets a begin of Scope (ends this line).</summary>
    public void SendScope()
    {
        AddString(Textscope);
    }

    /// <summary>Sets an end of Scope (on a separate line).</summary>
    public void SendEndscope()
    {
        NewLine(false);
        _thefile.Append(new TCollection_HAsciiString(Textendscope));
    }

    /// <summary>Sets a comment mark: if mode is True, begins Comment zone, if mode is False, ends Comment zone (if one is begun).</summary>
    public void Comment(bool mode)
    {
        if (mode && !_thecomm)
        {
            AddString(Textcomm, 20);
        }
        if (!mode && _thecomm)
        {
            AddString(Textendcomm);
        }
        _thecomm = mode;
    }

    /// <summary>Sends a comment. Error if we are not inside a comment zone.</summary>
    public void SendComment(TCollection_HAsciiString text)
    {
        if (!_thecomm)
        {
            throw new InterfaceMismatchException("StepWriter : Comment");
        }
        string aText = ToFileText(text.ToCString());
        AddCString(aText, aText.Length);
    }

    /// <summary>Same as above but accepts a CString (ex.: "..." directly).</summary>
    public void SendComment(string text)
    {
        if (!_thecomm)
        {
            throw new InterfaceMismatchException("StepWriter : Comment");
        }
        string aText = ToFileText(text);
        AddCString(aText, aText.Length);
    }

    /// <summary>
    /// Sets entity's StepType, opens brackets, starts param no to 0; params are separated by comma. Remark: for a
    /// Multiple Type Entity (see Express ANDOR clause) StartComplex must be called before sending components, then
    /// each "Component" must be sent separately (one call to StartEntity for each one): the Type which precedes is then
    /// automatically closed. Once all the components have been sent, EndComplex must be called, then and only then
    /// EndEntity.
    /// </summary>
    public void StartEntity(string atype)
    {
        if (atype.Length == 0)
        {
            return;
        }
        if (_themult)
        {
            if (_thelevel != 1)
            {
                throw new InterfaceMismatchException("StepWriter : StartEntity"); // bad parentheses count...
            }
            AddString(Textendlist);
            AddCString(" ", 1); // skl 29.01.2003
        }
        _themult = true;
        AddString(atype);
        _thelevel = 0;
        _theindval = _thecurr.Length();
        _thecurr.SetInitial(0);
        _thefirst = true;
        OpenSub();
    }

    /// <summary>
    /// Sends the start of a complex entity, which is a simple open bracket (without increasing bracket level). It must
    /// be called JUST AFTER SendEntity and BEFORE sending components, each one begins by StartEntity.
    /// </summary>
    public void StartComplex()
    {
        AddCString("( ", 2); // skl 29.01.2003
    } // thelevel unchanged

    /// <summary>
    /// Sends the end of a complex entity: a simple closed bracket. It must be called AFTER sending all the components
    /// and BEFORE the final call to EndEntity.
    /// </summary>
    public void EndComplex()
    {
        AddCString(") ", 2);
    } // thelevel unchanged

    //  ....                SendField and what goes with it

    /// <summary>
    /// Sends the content of a field, controlled by its descriptor. If the descriptor is not defined, follows the
    /// description detained by the field itself.
    /// </summary>
    public void SendField(Field fild, PDescr? descr)
    {
        bool done = true;
        int kind = fild.Kind(false); // valeur interne
        if (kind == 16)
        {
            SelectMember sm = fild.Transient() as SelectMember;
            SendSelect(sm, descr);
            return;
        }
        switch (kind)
        {
            //   here the simple cases; then we cast and see
            case 0:
                SendUndef();
                break;
            case 1:
                Send(fild.Integer());
                break;
            case 2:
                SendBoolean(fild.Boolean());
                break;
            case 3:
                SendLogical(fild.Logical());
                break;
            case 4:
                SendEnum(fild.EnumText());
                break; // enum : descr ?
            case 5:
                Send(fild.Real());
                break;
            case 6:
                Send(fild.String());
                break;
            case 7:
                Send(fild.Entity());
                break;
            case 8:
                done = false;
                break;
            case 9:
                SendDerived();
                break;
            default:
                done = false;
                break;
        }
        if (done)
        {
            return;
        }

        //  What remains: the arrays ...
        int arity = fild.Arity();
        if (arity == 0)
        {
            SendUndef();
            return;
        } // PAS NORMAL
        if (arity == 1)
        {
            OpenSub();
            int i, low = fild.Lower(), up = low + fild.Length() - 1;
            for (i = low; i <= up; i++)
            {
                kind = fild.ItemKind(i);
                done = true;
                switch (kind)
                {
                    case 0:
                        SendUndef();
                        break;
                    case 1:
                        Send(fild.Integer(i));
                        break;
                    case 2:
                        SendBoolean(fild.Boolean(i));
                        break;
                    case 3:
                        SendLogical(fild.Logical(i));
                        break;
                    case 4:
                        SendEnum(fild.EnumText(i));
                        break;
                    case 5:
                        Send(fild.Real(i));
                        break;
                    case 6:
                        Send(fild.String(i));
                        break;
                    case 7:
                        Send(fild.Entity(i));
                        break;
                    default:
                        SendUndef();
                        done = false;
                        break; // ANORMAL
                }
            }
            CloseSub();
            return;
        }
        if (arity == 2)
        {
            OpenSub();
            int j, low1 = fild.Lower(1), up1 = low1 + fild.Length(1) - 1;
            for (j = low1; j <= up1; j++)
            {
                int i = 0, low2 = fild.Lower(2), up2 = low2 + fild.Length(2) - 1;
                OpenSub();
                for (i = low2; i <= up2; i++)
                {
                    kind = fild.ItemKind(i, j);
                    done = true;
                    switch (kind)
                    {
                        case 0:
                            SendUndef();
                            break;
                        case 1:
                            Send(fild.Integer(i, j));
                            break;
                        case 2:
                            SendBoolean(fild.Boolean(i, j));
                            break;
                        case 3:
                            SendLogical(fild.Logical(i, j));
                            break;
                        case 4:
                            SendEnum(fild.EnumText(i, j));
                            break;
                        case 5:
                            Send(fild.Real(i, j));
                            break;
                        case 6:
                            Send(fild.String(i, j));
                            break;
                        case 7:
                            Send(fild.Entity(i, j));
                            break;
                        default:
                            SendUndef();
                            done = false;
                            break; // ANORMAL
                    }
                }
                CloseSub();
            }
            CloseSub();
            return;
        }
    }

    /// <summary>Sends a SelectMember, which can be named or not.</summary>
    public void SendSelect(SelectMember? sm, PDescr? descr)
    {
        //    SelectMember case. Process the Select then the value
        //    NB: current processing non-recursive (no SELNAME(SELNAME(..)) )
        bool selname = false;
        if (sm is null)
        {
            return; // ??
        }
        if (sm.HasName)
        {
            selname = true;
            OpenTypedSub(sm.Name());
        }
        int kind = sm.Kind();
        switch (kind)
        {
            case 0:
                SendUndef();
                break;
            case 1:
                Send(sm.Integer());
                break;
            case 2:
                SendBoolean(sm.Boolean());
                break;
            case 3:
                SendLogical(sm.Logical());
                break;
            case 4:
                SendEnum(sm.EnumText());
                break; // enum : descr ?
            case 5:
                Send(sm.Real());
                break;
            case 6:
                Send(sm.String());
                break;
            case 8:
                SendArrReal((sm as SelectArrReal).ArrReal());
                break;
            default:
                break; // ??
        }
        if (selname)
        {
            CloseSub();
        }
    }

    /// <summary>
    /// Send the content of an entity as being a FieldList controlled by its descriptor. This includes start and end
    /// brackets but not the entity type.
    /// </summary>
    public void SendList(FieldList list, ESDescr? descr)
    {
        // start entity  ?
        int i, nb = list.FieldCount;
        for (i = 1; i <= nb; i++)
        {
            PDescr pde = null;
            if (descr is not null)
            {
                pde = descr.Field(i);
            }
            Field fild = list.Field(i);
            SendField(fild, pde);
        }
        // end entity  ?
    }

    //  ....                Basic Send* methods

    /// <summary>Open a sublist by a '('.</summary>
    public void OpenSub()
    {
        AddParam();
        AddString(Textlist);
        _thefirst = true;
        _thelevel++;
    }

    /// <summary>Open a sublist with its type then a '('.</summary>
    public void OpenTypedSub(string subtype)
    {
        AddParam();
        if (!string.IsNullOrEmpty(subtype))
        {
            AddCString(subtype, subtype.Length);
        }
        AddString(Textlist);
        _thefirst = true;
        _thelevel++;
    }

    /// <summary>Closes a sublist by a ')'.</summary>
    public void CloseSub()
    {
        AddString(Textendlist);
        _thefirst = false; // the parameter following a sub-list is therefore not first
        _thelevel--;
    }

    /// <summary>
    /// Prepares adding a parameter (that is, adds ',' except for first one); normally for internal use; can be used to
    /// send a totally empty parameter (with no literal value).
    /// </summary>
    public void AddParam()
    {
        if (!_thefirst)
        {
            AddString(Textparam);
        }
        _thefirst = false;
    }

    /// <summary>Sends an integer parameter.</summary>
    public void Send(int val)
    {
        AddParam();
        Span<char> aText = stackalloc char[IdentSize];
        val.TryFormat(aText, out int aLength, default, CultureInfo.InvariantCulture);
        AddShort(aText[..aLength]);
    }

    /// <summary>Sends a real parameter (works with FloatWriter).</summary>
    public void Send(double val)
    {
        //    Floating point value, cleaned of trailing "0000" and "E+00"
        int lng = _thefloatw.Write(val, out string lval);
        AddParam();
        AddCString(lval, lng); // handles specific format: if needed
    }

    //  Send(String): note, we send a Text ... so between '  '

    /// <summary>Sends a text given as string (it will be set between '...').</summary>
    public void Send(string val)
    {
        AddParam();
        // Use helper function to clean text while preserving control directives
        // Kerf: OCCT's text is UTF-8, so the line is filled with its bytes (decision 1192); the public writer asks for ISO
        // 10303-21's directives instead (OCCT #1686; decision 1920)
        string aClean = CleanTextForSend(val);
        string aVal = TextsAsDirectives.Value ? WithDirectives(aClean) : ToFileText(aClean);
        int nn = aVal.Length;
        // Add surrounding quotes
        aVal = nn == 0 ? "''" : "'" + aVal + "'";
        nn += 2;

        //: i2 abv 31 Aug 98: ProSTEP TR9: avoid wrapping text or do it at spaces

        //    Watch out for exceeding 72 characters
        if (_thecurr.CanGet(nn))
        {
            AddString(aVal, 0);
            //: i2
        }
        else
        {
            _thefile.Append(_thecurr.Moved());
            int indst = _thelevel * 2;
            if (_theindent)
            {
                indst += _theindval;
            }
            if (indst + nn <= StepLong)
            {
                _thecurr.SetInitial(indst);
            }
            else
            {
                _thecurr.SetInitial(0);
            }
            if (_thecurr.CanGet(nn))
            {
                AddString(aVal, 0);
            }
            else
            {
                while (nn > 0)
                {
                    if (nn <= StepLong)
                    {
                        _thecurr.Add(aVal); // That's it, we've exhausted everything
                        _thecurr.FreezeInitial();
                        break;
                    }
                    int stop = StepLong; // position of last separator
                    for (; stop > 0 && aVal[stop - 1] != ' '; stop--)
                    {
                    }
                    if (stop == 0)
                    {
                        stop = StepLong;
                        for (; stop > 0 && aVal[stop - 1] != '\\'; stop--)
                        {
                        }
                        if (stop == 0)
                        {
                            stop = StepLong;
                            for (; stop > 0 && aVal[stop - 1] != '_'; stop--)
                            {
                            }
                            if (stop == 0)
                            {
                                stop = StepLong;
                            }
                        }
                    }
                    string bval = aVal.Substring(stop); // TCollection_AsciiString::Split
                    aVal = aVal.Substring(0, stop);
                    _thefile.Append(new TCollection_HAsciiString(aVal));
                    aVal = bval;
                    nn -= stop;
                }
            }
        }
    }

    /// <summary>
    /// Sends a reference to an entity (its identifier with '#'). REMARK 1: a Null val is interpreted as "Undefined".
    /// REMARK 2: for an HAsciiString which is not recorded in the Model, it is send as its String Content, between
    /// quotes.
    /// </summary>
    public void Send(Transient? val)
    {
        string lident;
        //  Undefined ?
        if (val is null)
        {
            //   throw Interface_InterfaceMismatch("StepWriter : Sending Null Reference");
            _thechecks.CCheck(_thenum).AddFail("Null Reference");
            SendUndef();
            Comment(true);
            SendComment(" NUL REF ");
            Comment(false);
            return;
        }
        // Kerf: nearly every entity starts with a name, and asking the model about each gives each its first hash code,
        // which is slow in .NET. A model that never held a string can't have this one (decision 1728)
        if (val is TCollection_HAsciiString aName && !_themodel.HoldsStrings)
        {
            Send(aName.ToCString());
            return;
        }
        int num = _themodel.Number(val);
        //  String? (if not listed in the Model)
        if (num == 0)
        {
            if (val is TCollection_HAsciiString strval)
            {
                Send(strval.ToCString());
                return;
            }
            //  SelectMember? (always, if not listed)
            //  but beware, no description attached
            else if (val is SelectMember sm)
            {
                PDescr descr = null; // null
                SendSelect(sm, descr);
            }
            //  Otherwise, NOT NORMAL!
            else
            {
                _thechecks.CCheck(_thenum).AddFail("UnknownReference");
                SendUndef();
                Comment(true);
                SendComment(" UNKNOWN REF ");
                Comment(false);
                //      throw Interface_InterfaceMismatch("StepWriter : Sending Unknown Reference");
            }
        }
        //  Normal case: a good Entity, we send its Ident.
        else
        {
            int idnum = num, idtrue = 0;
            if (_thelabmode > 0)
            {
                idtrue = _themodel.IdentLabel(val);
            }
            if (_thelabmode == 1)
            {
                idnum = idtrue;
            }
            if (idnum == 0)
            {
                idnum = num;
            }
            if (_thelabmode < 2 || idnum == idtrue)
            {
                AddParam();
                Span<char> aRef = stackalloc char[IdentSize];
                aRef[0] = '#';
                idnum.TryFormat(aRef[1..], out int aLength, default, CultureInfo.InvariantCulture);
                AddShort(aRef[..(1 + aLength)]);
                return;
            }
            else
            {
                lident = global::Kerf.Engine.Foundation.Standard.Sprintf("%d:#%d", idnum, idtrue);
            }
            AddParam();
            AddCString(lident, lident.Length);
        }
    }

    /// <summary>Sends a Boolean as .T. for True or .F. for False (it is an useful case of Enum, which is built-in).</summary>
    public void SendBoolean(bool val)
    {
        if (val)
        {
            SendString(Texttrue);
        }
        else
        {
            SendString(Textfalse);
        }
    }

    /// <summary>Sends a Logical as .T. or .F. or .U. according its Value (it is a standard case of Enum for Step, and is built-in).</summary>
    public void SendLogical(Logical val)
    {
        if (val == Logical.LTrue)
        {
            SendString(Texttrue);
        }
        else if (val == Logical.LFalse)
        {
            SendString(Textfalse);
        }
        else
        {
            SendString(Textunknown);
        }
    }

    //  SendString: note, we give the exact label

    /// <summary>Sends a string exactly as it is given.</summary>
    public void SendString(string val)
    {
        AddParam();
        AddCString(val, CLength(val));
    }

    //  SendEnum: note, we send an Enum label ... so between .  .

    /// <summary>Sends an enum given by String (literal expression), adds '.' around it if not done.</summary>
    /// <remarks>OCCT's <c>TCollection_AsciiString</c> and <c>const char*</c> forms are one method here.</remarks>
    public void SendEnum(string val)
    {
        if (global::Kerf.Engine.Foundation.Standard.CStringChar(val, 0) == '$' && global::Kerf.Engine.Foundation.Standard.CStringChar(val, 1) == '\0')
        {
            SendUndef();
            return;
        }
        AddParam();
        string value = val;
        if (value.Length == 0)
        {
            throw new OutOfRangeException("TCollection_AsciiString::Value : parameter where"); // OCCT: an empty enum
        }
        if (value[0] != '.')
        {
            value = "." + value;
        }
        if (value[value.Length - 1] != '.')
        {
            value += '.';
        }
        AddString(value, 2);
    }

    /// <summary>Sends an array of real.</summary>
    public void SendArrReal(NCollection_HArray1<double> anArr)
    {
        AddString(Textlist);
        if (anArr.Length() > 0)
        {
            // add real
            Send(anArr.Value(1));
            for (int i = 2; i <= anArr.Length(); i++)
            {
                // add real
                Send(anArr.Value(i));
            }
        }
        AddString(Textendlist);
    }

    /// <summary>Sends an undefined (optional absent) parameter (by '$').</summary>
    public void SendUndef()
    {
        AddParam();
        AddString(Textundef);
    }

    /// <summary>
    /// Sends a "Derived" parameter (by '*'). A Derived Parameter has been inherited from a Super-Type then redefined
    /// as being computed by a function. Hence its value in file is senseless.
    /// </summary>
    public void SendDerived()
    {
        AddParam();
        AddString(Textderived);
    }

    // EndEntity: if we need to put ; on the line, align it with start of entity ...

    /// <summary>Sends end of entity (closing bracket plus ';'). Error if count of opened-closed brackets is not null.</summary>
    public void EndEntity()
    {
        if (_thelevel != 1)
        {
            throw new InterfaceMismatchException("StepWriter : EndEntity"); // parentheses count is wrong ...
        }
        AddString(Textendent);
        _thelevel = 0; // on garde theindval : sera traite au prochain NewLine
        bool indent = _theindent;
        _theindent = false;
        NewLine(false);
        _theindent = indent;
        _themult = false;
        // to force indentation if necessary
    }

    //  current line management (see also NewLine/JoinLine)

    /// <summary>
    /// Adds a string to current line; first flushes it if full (72 char); more allows to ask a reserve at end of line:
    /// flush is done if remaining length (to 72) is less than more.
    /// </summary>
    private void AddString(string astr, int more = 0)
    {
        AddCString(astr, astr.Length, more);
    }

    // AddCString's short case, for a text that is never cut (an identifier, an integer)
    private void AddShort(ReadOnlySpan<char> text)
    {
        while (!_thecurr.CanGet(text.Length))
        {
            _thefile.Append(_thecurr.Moved());
            int indst = _thelevel * 2;
            if (_theindent)
            {
                indst += _theindval;
            }
            _thecurr.SetInitial(indst < StepLong && indst + text.Length <= StepLong ? indst : 0);
        }
        _thecurr.Add(text);
    }

    /// <summary>Same as above, but the string is given by CString + Length (OCCT's overload on <c>const char*</c>).</summary>
    private void AddCString(string astr, int lnstr, int more = 0)
    {
        void FlushLine(int required)
        {
            _thefile.Append(_thecurr.Moved());
            int indst = _thelevel * 2;
            if (_theindent)
            {
                indst += _theindval;
            }
            _thecurr.SetInitial(indst < StepLong && indst + required <= StepLong ? indst : 0);
        }

        if (lnstr + more <= StepLong)
        {
            while (!_thecurr.CanGet(lnstr + more))
            {
                FlushLine(lnstr + more);
            }
            _thecurr.Add(astr, lnstr);
            return;
        }

        int start = 0;
        int rest = lnstr;
        while (rest > 0)
        {
            if (_thecurr.CanGet(rest + more))
            {
                _thecurr.Add(astr.Substring(start), rest);
                return;
            }

            int room = StepLong - _thecurr.Length();
            if (room <= 0 || (rest + more <= StepLong && room < rest + more))
            {
                FlushLine(rest + more <= StepLong ? rest + more : 0);
                continue;
            }

            int chunk = (rest < room ? rest : room);
            _thecurr.Add(astr.Substring(start), chunk);
            start += chunk;
            rest -= chunk;
            if (rest > 0)
            {
                FlushLine(rest + more <= StepLong ? rest + more : 0);
            }
        }
    }

    //   FINAL SENDING

    /// <summary>
    /// Returns the check-list, which has received possible checks: for unknown entities, badly loaded ones, null or
    /// unknown references.
    /// </summary>
    public CheckIterator CheckList()
    {
        return _thechecks;
    }

    /// <summary>Returns count of Lines.</summary>
    public int LineCount
    {
        get
        {
            return _thefile.Length();
        }
    }

    /// <summary>Returns a Line given its rank in the File (its bytes, one char per byte).</summary>
    public TCollection_HAsciiString Line(int num)
    {
        return _thefile.Value(num);
    }

    /// <summary>
    /// Writes the result on a stream, each line followed by a line feed (the file's bytes); returns False if the
    /// stream fails. Unlike OCCT's, the lines are kept.
    /// </summary>
    public bool Print(Stream s)
    {
        try
        {
            int nb = _thefile.Length();
            var bytes = new byte[256];
            for (int i = 1; i <= nb; i++)
            {
                string line = _thefile.Value(i).ToCString();
                if (bytes.Length <= line.Length)
                {
                    bytes = new byte[line.Length + 1];
                }
                for (int j = 0; j < line.Length; j++)
                {
                    bytes[j] = unchecked((byte)line[j]);
                }
                bytes[line.Length] = (byte)'\n';
                s.Write(bytes, 0, line.Length + 1);
            }
            s.Flush();
            return true;
        }
        catch (IOException)
        {
            return false;
        }
    }

    /// <summary>Writes the result on a text writer, each line decoded from UTF-8 (for dumps).</summary>
    public bool Print(TextWriter s)
    {
        try
        {
            int nb = _thefile.Length();
            for (int i = 1; i <= nb; i++)
            {
                s.Write(FromFileText(_thefile.Value(i).ToCString()));
                s.Write('\n');
            }
            s.Flush();
            return true;
        }
        catch (IOException)
        {
            return false;
        }
    }

    /// <summary>
    /// Static helper function to prepare text for STEP file output while preserving existing ISO 10303-21 control
    /// directives (<c>\X{HH}\</c>, <c>\X2\...\X0\</c>, <c>\X4\...\X0\</c>, <c>\S\</c>, <c>\P{A-I}\</c>, <c>\N\</c>,
    /// <c>\T\</c>). On non-directive content: a single quote is doubled, a backslash is doubled, a newline becomes
    /// <c>\N\</c>, a tab <c>\T\</c>.
    /// </summary>
    public static string CleanTextForSend(string text)
    {
        // Kerf: nothing to clean in nearly all of them
        if (text.AsSpan().IndexOfAny("'\\\n\t") < 0)
        {
            return text;
        }
        var result = new StringBuilder();
        int nb = text.Length;
        // Process characters from beginning to end
        for (int i = 1; i <= nb; i++)
        {
            char uncar = text[i - 1];
            // Check if we're at the start of a control directive
            bool isDirective = false;
            int directiveLength = 0;
            if (uncar == '\\' && i <= nb)
            {
                // Check for \X2\ and \X4\ patterns first (need exactly 4 characters: \X2\)
                if (i + 3 <= nb && text[i] == 'X' && text[i + 2] == '\\')
                {
                    char thirdChar = text[i + 1];
                    // \X2, \X4, \X0 patterns - special control sequences
                    if (thirdChar == '2' || thirdChar == '4' || thirdChar == '0')
                    {
                        isDirective = true;
                        directiveLength = 4; // Basic directive length: \X2\, \X4\, \X0\ (4 chars)
                        // For \X2 and \X4, find the terminating \X0 sequence
                        if (thirdChar == '2' || thirdChar == '4')
                        {
                            int j = i + 4;
                            while (j <= nb - 3)
                            {
                                if (text[j - 1] == '\\' && text[j] == 'X' && text[j + 1] == '0' && text[j + 2] == '\\')
                                {
                                    directiveLength = (j + 4) - i; // Include the \X0 sequence
                                    break;
                                }
                                j++;
                            }
                        }
                    }
                }
                // Check for \X{HH}\ pattern (need exactly 5 characters: \X{HH}\)
                else if (i + 4 <= nb && text[i] == 'X' && text[i + 3] == '\\')
                {
                    char thirdChar = text[i + 1];
                    char fourthChar = text[i + 2];
                    // Regular \X{HH}\ pattern - check for two hex characters
                    if (IsXDigit(thirdChar) && IsXDigit(fourthChar))
                    {
                        isDirective = true;
                        directiveLength = 5; // Control directive with two hex chars
                    }
                }
                // Check for \S, \N, \T patterns (need exactly 3 characters: \S\)
                else if (i + 2 <= nb && text[i + 1] == '\\')
                {
                    char secondChar = text[i];
                    if (secondChar == 'S' || secondChar == 'N' || secondChar == 'T')
                    {
                        isDirective = true;
                        directiveLength = 3; // Simple directive pattern
                    }
                }
                // Check for \P{char}\ patterns (need exactly 4 characters: \P{char}\)
                else if (i + 3 <= nb && text[i] == 'P' && text[i + 2] == '\\')
                {
                    char secondChar = text[i + 1];
                    if (IsAlpha(secondChar))
                    {
                        isDirective = true;
                        directiveLength = 4; // P directive with parameter
                    }
                }
            }

            if (isDirective)
            {
                // Copy the entire directive as-is
                for (int j = 0; j < directiveLength; j++)
                {
                    result.Append(text[i - 1 + j]);
                }
                i += directiveLength - 1; // Move past directive (loop will increment by 1)
            }
            else
            {
                // Process non-directive characters
                if (uncar == '\'')
                {
                    result.Append("''"); // Double the quote
                }
                else if (uncar == '\\')
                {
                    result.Append("\\\\"); // Double the backslash
                }
                else if (uncar == '\n')
                {
                    result.Append("\\N\\"); // Convert to directive
                }
                else if (uncar == '\t')
                {
                    result.Append("\\T\\"); // Convert to directive
                }
                else
                {
                    result.Append(uncar); // Copy as-is
                }
            }
        }
        return result.ToString();
    }

    // C's isxdigit and isalpha in the "C" locale (OCCT tests the UTF-8 bytes, which are never ASCII above 0x7F)
    private static bool IsXDigit(char theChar) => char.IsAsciiHexDigit(theChar);

    private static bool IsAlpha(char theChar) => char.IsAsciiLetter(theChar);

    // strlen: a C string's length, up to its first null character
    private static int CLength(string text)
    {
        int end = text.IndexOf('\0');
        return end < 0 ? text.Length : end;
    }

    /// <summary>Whether <see cref="Send(string)"/> writes what isn't ASCII as directives, not as OCCT writes it.</summary>
    internal static readonly ContextKey<bool> TextsAsDirectives = new("StepWriter.TextsAsDirectives", false);

    /// <summary>
    /// A text with what isn't ASCII as ISO 10303-21's directives: <c>\X2\</c> for the basic plane and <c>\X4\</c> beyond it, each
    /// up to <c>\X0\</c>. The format allows nothing else in a text, and the reader decodes both (decision 1920).
    /// </summary>
    internal static string WithDirectives(string text)
    {
        if (text.AsSpan().IndexOfAnyExceptInRange('\0', '\x7f') < 0)
        {
            return text;
        }

        var result = new StringBuilder(text.Length + 16);
        for (int i = 0; i < text.Length;)
        {
            if (text[i] < 0x80)
            {
                result.Append(text[i++]);
                continue;
            }

            int wide = i;
            while (wide < text.Length && text[wide] >= 0x80 && !char.IsSurrogate(text[wide]))
            {
                wide++;
            }

            if (wide > i)
            {
                result.Append("\\X2\\");
                for (; i < wide; i++)
                {
                    result.Append(((int)text[i]).ToString("X4", CultureInfo.InvariantCulture));
                }

                result.Append("\\X0\\");
                continue;
            }

            result.Append("\\X4\\");
            while (i + 1 < text.Length && char.IsSurrogatePair(text[i], text[i + 1]))
            {
                result.Append(char.ConvertToUtf32(text[i], text[i + 1]).ToString("X8", CultureInfo.InvariantCulture));
                i += 2;
            }

            if (i < text.Length && char.IsSurrogate(text[i]))
            {
                result.Append(0xFFFD.ToString("X8", CultureInfo.InvariantCulture)); // half a pair isn't a character
                i++;
            }

            result.Append("\\X0\\");
        }

        return result.ToString();
    }

    /// <summary>A text of the model (OCCT's UTF-8) as the file's bytes, one char per byte (decision 1192).</summary>
    internal static string ToFileText(string text)
    {
        foreach (char aChar in text)
        {
            if (aChar >= 0x80)
            {
                byte[] bytes = Encoding.UTF8.GetBytes(text);
                return Encoding.Latin1.GetString(bytes);
            }
        }
        return text;
    }

    /// <summary>The file's bytes (one char per byte) read back as UTF-8.</summary>
    internal static string FromFileText(string text)
    {
        foreach (char aChar in text)
        {
            if (aChar >= 0x80)
            {
                return Encoding.UTF8.GetString(Encoding.Latin1.GetBytes(text));
            }
        }
        return text;
    }
}
