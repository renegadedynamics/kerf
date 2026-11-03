// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepWriter.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepWriter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using System.Text;
using static Occt.StepData_Logical;

namespace Occt;

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
public class StepData_StepWriter
{
    private const int StepLong = 72;
    // StepLong: maximum length of a Step file line

    // Literal constants (useful for performance AND MEMORY)
    private const string textscope = " &SCOPE";
    private const string textendscope = "        ENDSCOPE";
    private const string textcomm = "  /*  ";
    private const string textendcomm = "  */";
    private const string textlist = "(";
    private const string textendlist = ")";
    private const string textendent = ");";
    private const string textparam = ",";
    private const string textundef = "$";
    private const string textderived = "*";
    private const string texttrue = ".T.";
    private const string textfalse = ".F.";
    private const string textunknown = ".U.";

    private readonly StepData_StepModel themodel;
    private readonly NCollection_HSequence<TCollection_HAsciiString> thefile;
    private readonly Interface_LineBuffer thecurr;
    private bool thesect;
    private bool thecomm;
    private bool thefirst;
    private bool themult;
    private int thelevel;
    private bool theindent;
    private int theindval;
    private int thetypmode;
    private readonly Interface_FloatWriter thefloatw;
    private readonly Interface_CheckIterator thechecks = new();
    private int thenum;
    private int thelabmode;
    private NCollection_HArray1<int> thescopebeg;
    private NCollection_HArray1<int> thescopeend;
    private NCollection_HArray1<int> thescopenext;

    /// <summary>Creates an empty StepWriter from a StepModel. The StepModel provides the Number of Entities, as identifiers for File.</summary>
    public StepData_StepWriter(StepData_StepModel amodel)
    {
        thecurr = new Interface_LineBuffer(StepLong);
        thefloatw = new Interface_FloatWriter(12);
        themodel = amodel;
        thelabmode = thetypmode = 0;
        thefile = new NCollection_HSequence<TCollection_HAsciiString>();
        thesect = false;
        thefirst = true;
        themult = false;
        thecomm = false;
        thelevel = theindval = 0;
        theindent = false;
        // Floating point format: delegated to FloatWriter
    }

    //  ....                Float Sending Control                ....

    /// <summary>
    /// Returns the embedded FloatWriter, which controls sending Reals. Use this method to access FloatWriter in order
    /// to consult or change its options (MainFormat, FormatForRange, ZeroSuppress).
    /// </summary>
    public Interface_FloatWriter FloatWriter()
    {
        return thefloatw;
    } // refer to it

    /// <summary>
    /// ModeLabel controls how to display entity ids: 0 (D) gives entity number in the model, 1 gives the already
    /// recorded label (else, its number). Warning: conflicts are not controlled.
    /// </summary>
    public ref int LabelMode()
    {
        return ref thelabmode;
    }

    /// <summary>
    /// TypeMode controls the type form to use: 0 (D) for normal long form, 1 for short form (if a type name has no
    /// short form, normal long form is then used).
    /// </summary>
    public ref int TypeMode()
    {
        return ref thetypmode;
    }

    //  ....                Scope Description (BEFORE Sending)               ....

    /// <summary>
    /// Declares the Entity Number numscope to correspond to a Scope which contains the Entity Number numin. Several
    /// calls to the same numscope add Entities in this Scope, in this order. Error if numin is already declared in
    /// the Scope.
    /// </summary>
    public void SetScope(int numscope, int numin)
    {
        int nb = themodel.NbEntities();
        if (numscope <= 0 || numscope > nb || numin <= 0 || numin > nb)
        {
            throw new Interface_InterfaceMismatch("StepWriter : SetScope, out of range");
        }
        if (thescopenext is null)
        {
            thescopebeg = new NCollection_HArray1<int>(1, nb);
            thescopebeg.Init(0);
            thescopeend = new NCollection_HArray1<int>(1, nb);
            thescopeend.Init(0);
            thescopenext = new NCollection_HArray1<int>(1, nb);
            thescopenext.Init(0);
        }
        else if (thescopenext.Value(numin) != 0)
        {
            throw new Interface_InterfaceMismatch("StepWriter : SetScope, already set");
        }
        thescopenext.SetValue(numin, -1); // new end of scope
        if (thescopebeg.Value(numscope) == 0)
        {
            thescopebeg.SetValue(numscope, numin);
        }
        int lastin = thescopeend.Value(numscope);
        if (lastin > 0)
        {
            thescopenext.SetValue(lastin, numin);
        }
        thescopeend.SetValue(numscope, numin);
    }

    /// <summary>Returns True if an Entity identified by its Number is in a Scope.</summary>
    public bool IsInScope(int num)
    {
        if (thescopenext is null)
        {
            return false;
        }
        return (thescopenext.Value(num) != 0);
    }

    //  ###########################################################################
    //  ##    ##    ##    ##        SENDING SECTIONS        ##    ##    ##    ##

    /// <summary>
    /// Sends the complete Model, included HEADER and DATA Sections. Works with a WriterLib defined through a Protocol.
    /// If headeronly is given True, only the HEADER Section is sent (used to Dump the Header of a StepModel).
    /// </summary>
    public void SendModel(StepData_Protocol protocol, bool headeronly = false)
    {
        StepData_WriterLib lib = new StepData_WriterLib(protocol);
        if (!headeronly)
        {
            thefile.Append(new TCollection_HAsciiString("ISO-10303-21;"));
        }
        SendHeader();

        //  ....                Header: sequence of entities without Ident                ....

        Interface_EntityIterator header = themodel.Header();
        thenum = 0;
        for (header.Start(); header.More(); header.Next())
        {
            Transient anent = header.Value();

            //   Write Entity via Lib  (similaire a SendEntity)
            if (lib.Select(anent, out StepData_ReadWriteModule module, out int CN))
            {
                if (module.IsComplex(CN))
                {
                    StartComplex();
                }
                else
                {
                    string styp = "";
                    if (thetypmode > 0)
                    {
                        styp = module.ShortType(CN);
                    }
                    if (styp.Length == 0)
                    {
                        styp = module.StepType(CN);
                    }
                    StartEntity(styp);
                }
                module.WriteStep(CN, this, anent);
                if (module.IsComplex(CN))
                {
                    EndComplex();
                }
            }
            else
            {
                //    Not found above ... try UndefinedEntity
                if (anent is not StepData_UndefinedEntity und)
                {
                    continue;
                }
                if (und.IsComplex())
                {
                    StartComplex();
                }
                und.WriteParams(this);
                if (und.IsComplex())
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

        Interface_Check achglob = themodel.GlobalCheck();
        int nbfails = achglob.NbFails();
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

        int nb = themodel.NbEntities();
        for (int i = 1; i <= nb; i++)
        {
            //    Main list: we don't send Entities that are in a Scope
            //    They will be sent through the Scope that contains them
            if (thescopebeg is not null)
            {
                if (thescopenext.Value(i) != 0)
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
        thefile.Append(new TCollection_HAsciiString("HEADER;"));
        thesect = true;
    }

    /// <summary>Begins data section; error if EndSec was not set.</summary>
    public void SendData()
    {
        if (thesect)
        {
            throw new Interface_InterfaceMismatch("StepWriter : Data section");
        }
        NewLine(false);
        thefile.Append(new TCollection_HAsciiString("DATA;"));
        thesect = true;
    }

    /// <summary>Sets end of section; to be done before passing to next one.</summary>
    public void EndSec()
    {
        thefile.Append(new TCollection_HAsciiString("ENDSEC;"));
        thesect = false;
    }

    /// <summary>Sets end of file; error is EndSec was not set.</summary>
    public void EndFile()
    {
        if (thesect)
        {
            throw new Interface_InterfaceMismatch("StepWriter : EndFile");
        }
        NewLine(false);
        thefile.Append(new TCollection_HAsciiString("END-ISO-10303-21;"));
        thesect = false;
    }

    //  ....                        SENDING AN ENTITY                        ....

    /// <summary>Send an Entity of the Data Section. If it corresponds to a Scope, also Sends the Scope information and contained Items.</summary>
    public void SendEntity(int num, StepData_WriterLib lib)
    {
        string lident;
        Transient anent = themodel.Entity(num);
        int idnum = num, idtrue = 0;

        //   themodel->Number(anent) and/or IdentLabel(anent)
        if (thelabmode > 0)
        {
            idtrue = themodel.IdentLabel(anent);
        }
        if (thelabmode == 1)
        {
            idnum = idtrue;
        }
        if (idnum == 0)
        {
            idnum = num;
        }
        if (thelabmode < 2 || idnum == idtrue)
        {
            lident = global::Kerf.Engine.Foundation.Standard.Sprintf("#%d = ", idnum); // skl 29.01.2003
        }
        else
        {
            lident = global::Kerf.Engine.Foundation.Standard.Sprintf("%d:#%d = ", idnum, idtrue); // skl 29.01.2003
        }

        //  SendIdent reused, lident has just been calculated
        thecurr.Clear();
        thecurr.Add(lident);
        themult = false;

        //  ....        Processing of Potential Scope
        if (thescopebeg is not null)
        {
            int numin = thescopebeg.Value(num);
            if (numin != 0)
            {
                SendScope();
                for (int nument = numin; numin > 0; nument = numin)
                {
                    SendEntity(nument, lib);
                    numin = thescopenext.Value(nument);
                }
                SendEndscope();
            }
        }

        //  ....        Sending the Entity proper

        //   Write Entity via Lib
        thenum = num;
        if (themodel.IsRedefinedContent(num))
        {
            //    Error Entity: Write the Content + Errors as Comments
            Interface_ReportEntity rep = themodel.ReportEntity(num);
            StepData_UndefinedEntity und = rep.Content() as StepData_UndefinedEntity;
            if (und is null)
            {
                thechecks.CCheck(num).AddFail("Erroneous Entity, Content lost");
                StartEntity("!?LOST_DATA");
            }
            else
            {
                thechecks.CCheck(num).AddWarning("Erroneous Entity, equivalent content");
                if (und.IsComplex())
                {
                    AddCString(" (", 2);
                }
                und.WriteParams(this);
                if (und.IsComplex())
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
            Interface_Check ach = rep.Check();
            int nbfails = ach.NbFails();
            for (int ifail = 1; ifail <= nbfails; ifail++)
            {
                SendComment(ach.Fail(ifail));
            }
            Comment(false);
            NewLine(false);

            //    Normal case
        }
        else if (lib.Select(anent, out StepData_ReadWriteModule module, out int CN))
        {
            if (module.IsComplex(CN))
            {
                StartComplex();
            }
            else
            {
                string styp = "";
                if (thetypmode > 0)
                {
                    styp = module.ShortType(CN);
                }
                if (styp.Length == 0)
                {
                    styp = module.StepType(CN);
                }
                StartEntity(styp);
            }
            module.WriteStep(CN, this, anent);
            if (module.IsComplex(CN))
            {
                EndComplex();
            }
            EndEntity();
        }
        else
        {
            //    Not found above ... try UndefinedEntity
            if (anent is not StepData_UndefinedEntity und)
            {
                return;
            }
            if (und.IsComplex())
            {
                StartComplex();
            }
            und.WriteParams(this);
            if (und.IsComplex())
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
        if (evenempty || thecurr.Length() > 0)
        {
            thefile.Append(thecurr.Moved());
        }
        int indst = thelevel * 2;
        if (theindent)
        {
            indst += theindval;
        }
        thecurr.SetInitial(indst);
        thecurr.Clear();
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
        thecurr.SetKeep();
    }

    /// <summary>
    /// Asks that further indentations will begin at position of entity first opening bracket; else they begin at zero
    /// (def). For each sublist level, two more blanks are added at beginning (except for text continuation, which must
    /// begin at true zero).
    /// </summary>
    public void Indent(bool onent)
    {
        theindent = onent;
    }

    /// <summary>
    /// Begins an entity with an ident plus '=' (at beginning of line); entity ident is its Number given by the
    /// containing Model. Warning: ident must be, either Number or Label, according LabelMode.
    /// </summary>
    public void SendIdent(int ident)
    {
        string lident = global::Kerf.Engine.Foundation.Standard.Sprintf("#%d =", ident);
        thecurr.Clear();
        thecurr.Add(lident);
        themult = false;
    }

    /// <summary>Sets a begin of Scope (ends this line).</summary>
    public void SendScope()
    {
        AddString(textscope);
    }

    /// <summary>Sets an end of Scope (on a separate line).</summary>
    public void SendEndscope()
    {
        NewLine(false);
        thefile.Append(new TCollection_HAsciiString(textendscope));
    }

    /// <summary>Sets a comment mark: if mode is True, begins Comment zone, if mode is False, ends Comment zone (if one is begun).</summary>
    public void Comment(bool mode)
    {
        if (mode && !thecomm)
        {
            AddString(textcomm, 20);
        }
        if (!mode && thecomm)
        {
            AddString(textendcomm);
        }
        thecomm = mode;
    }

    /// <summary>Sends a comment. Error if we are not inside a comment zone.</summary>
    public void SendComment(TCollection_HAsciiString text)
    {
        if (!thecomm)
        {
            throw new Interface_InterfaceMismatch("StepWriter : Comment");
        }
        string aText = ToFileText(text.ToCString());
        AddCString(aText, aText.Length);
    }

    /// <summary>Same as above but accepts a CString (ex.: "..." directly).</summary>
    public void SendComment(string text)
    {
        if (!thecomm)
        {
            throw new Interface_InterfaceMismatch("StepWriter : Comment");
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
        if (themult)
        {
            if (thelevel != 1)
            {
                throw new Interface_InterfaceMismatch("StepWriter : StartEntity"); // bad parentheses count...
            }
            AddString(textendlist);
            AddCString(" ", 1); // skl 29.01.2003
        }
        themult = true;
        AddString(atype);
        thelevel = 0;
        theindval = thecurr.Length();
        thecurr.SetInitial(0);
        thefirst = true;
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
    public void SendField(StepData_Field fild, StepData_PDescr? descr)
    {
        bool done = true;
        int kind = fild.Kind(false); // valeur interne
        if (kind == 16)
        {
            StepData_SelectMember sm = fild.Transient() as StepData_SelectMember;
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
    public void SendSelect(StepData_SelectMember? sm, StepData_PDescr? descr)
    {
        //    SelectMember case. Process the Select then the value
        //    NB: current processing non-recursive (no SELNAME(SELNAME(..)) )
        bool selname = false;
        if (sm is null)
        {
            return; // ??
        }
        if (sm.HasName())
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
                SendArrReal((sm as StepData_SelectArrReal).ArrReal());
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
    public void SendList(StepData_FieldList list, StepData_ESDescr? descr)
    {
        // start entity  ?
        int i, nb = list.NbFields();
        for (i = 1; i <= nb; i++)
        {
            StepData_PDescr pde = null;
            if (descr is not null)
            {
                pde = descr.Field(i);
            }
            StepData_Field fild = list.Field(i);
            SendField(fild, pde);
        }
        // end entity  ?
    }

    //  ....                Basic Send* methods

    /// <summary>Open a sublist by a '('.</summary>
    public void OpenSub()
    {
        AddParam();
        AddString(textlist);
        thefirst = true;
        thelevel++;
    }

    /// <summary>Open a sublist with its type then a '('.</summary>
    public void OpenTypedSub(string subtype)
    {
        AddParam();
        if (!string.IsNullOrEmpty(subtype))
        {
            AddCString(subtype, subtype.Length);
        }
        AddString(textlist);
        thefirst = true;
        thelevel++;
    }

    /// <summary>Closes a sublist by a ')'.</summary>
    public void CloseSub()
    {
        AddString(textendlist);
        thefirst = false; // the parameter following a sub-list is therefore not first
        thelevel--;
    }

    /// <summary>
    /// Prepares adding a parameter (that is, adds ',' except for first one); normally for internal use; can be used to
    /// send a totally empty parameter (with no literal value).
    /// </summary>
    public void AddParam()
    {
        if (!thefirst)
        {
            AddString(textparam);
        }
        thefirst = false;
    }

    /// <summary>Sends an integer parameter.</summary>
    public void Send(int val)
    {
        AddParam();
        string lval = global::Kerf.Engine.Foundation.Standard.Sprintf("%d", val);
        AddCString(lval, lval.Length);
    }

    /// <summary>Sends a real parameter (works with FloatWriter).</summary>
    public void Send(double val)
    {
        //    Floating point value, cleaned of trailing "0000" and "E+00"
        int lng = thefloatw.Write(val, out string lval);
        AddParam();
        AddCString(lval, lng); // handles specific format: if needed
    }

    //  Send(String): note, we send a Text ... so between '  '

    /// <summary>Sends a text given as string (it will be set between '...').</summary>
    public void Send(string val)
    {
        AddParam();
        // Use helper function to clean text while preserving control directives
        // OCCT.NET: OCCT's text is UTF-8, so the line is filled with its bytes (decision 1192)
        string aVal = ToFileText(CleanTextForSend(val));
        int aNn = aVal.Length;
        // Add surrounding quotes
        aVal = "'" + aVal + "'";
        aNn += 2;

        //: i2 abv 31 Aug 98: ProSTEP TR9: avoid wrapping text or do it at spaces

        //    Watch out for exceeding 72 characters
        if (thecurr.CanGet(aNn))
        {
            AddString(aVal, 0);
            //: i2
        }
        else
        {
            thefile.Append(thecurr.Moved());
            int anIndst = thelevel * 2;
            if (theindent)
            {
                anIndst += theindval;
            }
            if (anIndst + aNn <= StepLong)
            {
                thecurr.SetInitial(anIndst);
            }
            else
            {
                thecurr.SetInitial(0);
            }
            if (thecurr.CanGet(aNn))
            {
                AddString(aVal, 0);
            }
            else
            {
                while (aNn > 0)
                {
                    if (aNn <= StepLong)
                    {
                        thecurr.Add(aVal); // That's it, we've exhausted everything
                        thecurr.FreezeInitial();
                        break;
                    }
                    int aStop = StepLong; // position of last separator
                    for (; aStop > 0 && aVal[aStop - 1] != ' '; aStop--)
                    {
                    }
                    if (aStop == 0)
                    {
                        aStop = StepLong;
                        for (; aStop > 0 && aVal[aStop - 1] != '\\'; aStop--)
                        {
                        }
                        if (aStop == 0)
                        {
                            aStop = StepLong;
                            for (; aStop > 0 && aVal[aStop - 1] != '_'; aStop--)
                            {
                            }
                            if (aStop == 0)
                            {
                                aStop = StepLong;
                            }
                        }
                    }
                    string aBval = aVal.Substring(aStop); // TCollection_AsciiString::Split
                    aVal = aVal.Substring(0, aStop);
                    thefile.Append(new TCollection_HAsciiString(aVal));
                    aVal = aBval;
                    aNn -= aStop;
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
            thechecks.CCheck(thenum).AddFail("Null Reference");
            SendUndef();
            Comment(true);
            SendComment(" NUL REF ");
            Comment(false);
            return;
        }
        int num = themodel.Number(val);
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
            else if (val is StepData_SelectMember sm)
            {
                StepData_PDescr descr = null; // null
                SendSelect(sm, descr);
            }
            //  Otherwise, NOT NORMAL!
            else
            {
                thechecks.CCheck(thenum).AddFail("UnknownReference");
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
            if (thelabmode > 0)
            {
                idtrue = themodel.IdentLabel(val);
            }
            if (thelabmode == 1)
            {
                idnum = idtrue;
            }
            if (idnum == 0)
            {
                idnum = num;
            }
            if (thelabmode < 2 || idnum == idtrue)
            {
                lident = global::Kerf.Engine.Foundation.Standard.Sprintf("#%d", idnum);
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
            SendString(texttrue);
        }
        else
        {
            SendString(textfalse);
        }
    }

    /// <summary>Sends a Logical as .T. or .F. or .U. according its Value (it is a standard case of Enum for Step, and is built-in).</summary>
    public void SendLogical(StepData_Logical val)
    {
        if (val == StepData_Logical.StepData_LTrue)
        {
            SendString(texttrue);
        }
        else if (val == StepData_Logical.StepData_LFalse)
        {
            SendString(textfalse);
        }
        else
        {
            SendString(textunknown);
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
        string aValue = val;
        if (aValue.Length == 0)
        {
            throw new OutOfRangeException("TCollection_AsciiString::Value : parameter where"); // OCCT: an empty enum
        }
        if (aValue[0] != '.')
        {
            aValue = "." + aValue;
        }
        if (aValue[aValue.Length - 1] != '.')
        {
            aValue += '.';
        }
        AddString(aValue, 2);
    }

    /// <summary>Sends an array of real.</summary>
    public void SendArrReal(NCollection_HArray1<double> anArr)
    {
        AddString(textlist);
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
        AddString(textendlist);
    }

    /// <summary>Sends an undefined (optional absent) parameter (by '$').</summary>
    public void SendUndef()
    {
        AddParam();
        AddString(textundef);
    }

    /// <summary>
    /// Sends a "Derived" parameter (by '*'). A Derived Parameter has been inherited from a Super-Type then redefined
    /// as being computed by a function. Hence its value in file is senseless.
    /// </summary>
    public void SendDerived()
    {
        AddParam();
        AddString(textderived);
    }

    // EndEntity: if we need to put ; on the line, align it with start of entity ...

    /// <summary>Sends end of entity (closing bracket plus ';'). Error if count of opened-closed brackets is not null.</summary>
    public void EndEntity()
    {
        if (thelevel != 1)
        {
            throw new Interface_InterfaceMismatch("StepWriter : EndEntity"); // parentheses count is wrong ...
        }
        AddString(textendent);
        thelevel = 0; // on garde theindval : sera traite au prochain NewLine
        bool indent = theindent;
        theindent = false;
        NewLine(false);
        theindent = indent;
        themult = false;
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

    /// <summary>Same as above, but the string is given by CString + Length (OCCT's overload on <c>const char*</c>).</summary>
    private void AddCString(string astr, int lnstr, int more = 0)
    {
        void flushLine(int theRequired)
        {
            thefile.Append(thecurr.Moved());
            int indst = thelevel * 2;
            if (theindent)
            {
                indst += theindval;
            }
            thecurr.SetInitial(indst < StepLong && indst + theRequired <= StepLong ? indst : 0);
        }

        if (lnstr + more <= StepLong)
        {
            while (!thecurr.CanGet(lnstr + more))
            {
                flushLine(lnstr + more);
            }
            thecurr.Add(astr, lnstr);
            return;
        }

        int aStart = 0;
        int aRest = lnstr;
        while (aRest > 0)
        {
            if (thecurr.CanGet(aRest + more))
            {
                thecurr.Add(astr.Substring(aStart), aRest);
                return;
            }

            int aRoom = StepLong - thecurr.Length();
            if (aRoom <= 0 || (aRest + more <= StepLong && aRoom < aRest + more))
            {
                flushLine(aRest + more <= StepLong ? aRest + more : 0);
                continue;
            }

            int aChunk = (aRest < aRoom ? aRest : aRoom);
            thecurr.Add(astr.Substring(aStart), aChunk);
            aStart += aChunk;
            aRest -= aChunk;
            if (aRest > 0)
            {
                flushLine(aRest + more <= StepLong ? aRest + more : 0);
            }
        }
    }

    //   FINAL SENDING

    /// <summary>
    /// Returns the check-list, which has received possible checks: for unknown entities, badly loaded ones, null or
    /// unknown references.
    /// </summary>
    public Interface_CheckIterator CheckList()
    {
        return thechecks;
    }

    /// <summary>Returns count of Lines.</summary>
    public int NbLines()
    {
        return thefile.Length();
    }

    /// <summary>Returns a Line given its rank in the File (its bytes, one char per byte).</summary>
    public TCollection_HAsciiString Line(int num)
    {
        return thefile.Value(num);
    }

    /// <summary>
    /// Writes the result on a stream, each line followed by a line feed (the file's bytes); returns False if the
    /// stream fails. Unlike OCCT's, the lines are kept.
    /// </summary>
    public bool Print(Stream S)
    {
        try
        {
            int nb = thefile.Length();
            for (int i = 1; i <= nb; i++)
            {
                string aLine = thefile.Value(i).ToCString();
                var aBytes = new byte[aLine.Length + 1];
                for (int j = 0; j < aLine.Length; j++)
                {
                    aBytes[j] = unchecked((byte)aLine[j]);
                }
                aBytes[aLine.Length] = (byte)'\n';
                S.Write(aBytes, 0, aBytes.Length);
            }
            S.Flush();
            return true;
        }
        catch (IOException)
        {
            return false;
        }
    }

    /// <summary>Writes the result on a text writer, each line decoded from UTF-8 (for dumps).</summary>
    public bool Print(TextWriter S)
    {
        try
        {
            int nb = thefile.Length();
            for (int i = 1; i <= nb; i++)
            {
                S.Write(FromFileText(thefile.Value(i).ToCString()));
                S.Write('\n');
            }
            S.Flush();
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
    public static string CleanTextForSend(string theText)
    {
        var aResult = new StringBuilder();
        int aNb = theText.Length;
        // Process characters from beginning to end
        for (int anI = 1; anI <= aNb; anI++)
        {
            char anUncar = theText[anI - 1];
            // Check if we're at the start of a control directive
            bool anIsDirective = false;
            int aDirectiveLength = 0;
            if (anUncar == '\\' && anI <= aNb)
            {
                // Check for \X2\ and \X4\ patterns first (need exactly 4 characters: \X2\)
                if (anI + 3 <= aNb && theText[anI] == 'X' && theText[anI + 2] == '\\')
                {
                    char aThirdChar = theText[anI + 1];
                    // \X2, \X4, \X0 patterns - special control sequences
                    if (aThirdChar == '2' || aThirdChar == '4' || aThirdChar == '0')
                    {
                        anIsDirective = true;
                        aDirectiveLength = 4; // Basic directive length: \X2\, \X4\, \X0\ (4 chars)
                        // For \X2 and \X4, find the terminating \X0 sequence
                        if (aThirdChar == '2' || aThirdChar == '4')
                        {
                            int aJ = anI + 4;
                            while (aJ <= aNb - 3)
                            {
                                if (theText[aJ - 1] == '\\' && theText[aJ] == 'X' && theText[aJ + 1] == '0' && theText[aJ + 2] == '\\')
                                {
                                    aDirectiveLength = (aJ + 4) - anI; // Include the \X0 sequence
                                    break;
                                }
                                aJ++;
                            }
                        }
                    }
                }
                // Check for \X{HH}\ pattern (need exactly 5 characters: \X{HH}\)
                else if (anI + 4 <= aNb && theText[anI] == 'X' && theText[anI + 3] == '\\')
                {
                    char aThirdChar = theText[anI + 1];
                    char aFourthChar = theText[anI + 2];
                    // Regular \X{HH}\ pattern - check for two hex characters
                    if (IsXDigit(aThirdChar) && IsXDigit(aFourthChar))
                    {
                        anIsDirective = true;
                        aDirectiveLength = 5; // Control directive with two hex chars
                    }
                }
                // Check for \S, \N, \T patterns (need exactly 3 characters: \S\)
                else if (anI + 2 <= aNb && theText[anI + 1] == '\\')
                {
                    char aSecondChar = theText[anI];
                    if (aSecondChar == 'S' || aSecondChar == 'N' || aSecondChar == 'T')
                    {
                        anIsDirective = true;
                        aDirectiveLength = 3; // Simple directive pattern
                    }
                }
                // Check for \P{char}\ patterns (need exactly 4 characters: \P{char}\)
                else if (anI + 3 <= aNb && theText[anI] == 'P' && theText[anI + 2] == '\\')
                {
                    char aSecondChar = theText[anI + 1];
                    if (IsAlpha(aSecondChar))
                    {
                        anIsDirective = true;
                        aDirectiveLength = 4; // P directive with parameter
                    }
                }
            }

            if (anIsDirective)
            {
                // Copy the entire directive as-is
                for (int aJ = 0; aJ < aDirectiveLength; aJ++)
                {
                    aResult.Append(theText[anI - 1 + aJ]);
                }
                anI += aDirectiveLength - 1; // Move past directive (loop will increment by 1)
            }
            else
            {
                // Process non-directive characters
                if (anUncar == '\'')
                {
                    aResult.Append("''"); // Double the quote
                }
                else if (anUncar == '\\')
                {
                    aResult.Append("\\\\"); // Double the backslash
                }
                else if (anUncar == '\n')
                {
                    aResult.Append("\\N\\"); // Convert to directive
                }
                else if (anUncar == '\t')
                {
                    aResult.Append("\\T\\"); // Convert to directive
                }
                else
                {
                    aResult.Append(anUncar); // Copy as-is
                }
            }
        }
        return aResult.ToString();
    }

    // C's isxdigit and isalpha in the "C" locale (OCCT tests the UTF-8 bytes, which are never ASCII above 0x7F)
    private static bool IsXDigit(char theChar) => char.IsAsciiHexDigit(theChar);

    private static bool IsAlpha(char theChar) => char.IsAsciiLetter(theChar);

    // strlen: a C string's length, up to its first null character
    private static int CLength(string theText)
    {
        int anEnd = theText.IndexOf('\0');
        return anEnd < 0 ? theText.Length : anEnd;
    }

    /// <summary>A text of the model (OCCT's UTF-8) as the file's bytes, one char per byte (decision 1192).</summary>
    internal static string ToFileText(string theText)
    {
        foreach (char aChar in theText)
        {
            if (aChar >= 0x80)
            {
                byte[] aBytes = Encoding.UTF8.GetBytes(theText);
                return Encoding.Latin1.GetString(aBytes);
            }
        }
        return theText;
    }

    /// <summary>The file's bytes (one char per byte) read back as UTF-8.</summary>
    internal static string FromFileText(string theText)
    {
        foreach (char aChar in theText)
        {
            if (aChar >= 0x80)
            {
                return Encoding.UTF8.GetString(Encoding.Latin1.GetBytes(theText));
            }
        }
        return theText;
    }
}
