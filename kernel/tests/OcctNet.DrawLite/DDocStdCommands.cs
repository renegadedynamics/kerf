// Port of: src/Draw/TKDCAF/DDocStd/DDocStd.cxx, DDocStd_ApplicationCommands.cxx, DDocStd_DocumentCommands.cxx (parts),
//          src/Draw/TKDCAF/DDataStd/DDataStd_BasicCommands.cxx (SetName, GetName), src/Draw/TKDraw/Draw/Draw.cxx (ParseColor)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Runtime.CompilerServices;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// A Draw-lite document (DDocStd_DrawDocument): an <see cref="XdeDoc_Document"/> under a Draw variable, in place of an
/// OCAF document with XCAF's tools (decision 1166, plan 2.6). Its labels are XCAF's; it has no undo and no storage.
/// </summary>
public sealed class DrawDocument(XdeDoc_Document theDocument) : DrawDrawable
{
    /// <summary>The document.</summary>
    public XdeDoc_Document Document { get; } = theDocument;

    public override string TypeName => "DDocStd_DrawDocument";

    /// <summary>DDocStd_DrawDocument::Whatis: the document's type name.</summary>
    public override void Whatis(DrawResult theResult) => theResult.Append("TDocStd_Document");

    public override void Dump(TextWriter theStream) => theStream.Write("TDocStd_Document\n");

    public override void Save(TextWriter theStream) => throw new DrawException("a document can't be saved by bsave");

    /// <summary>DDocStd_DrawDocument::Copy: a new drawable on the same document.</summary>
    public override DrawDrawable Copy() => new DrawDocument(Document);

    /// <summary>The undo limit UndoLimit set (Draw-lite keeps no undo: decision 13304).</summary>
    internal int UndoLimit { get; set; }
}

/// <summary>DDocStd's and DDataStd's helpers, and the document commands the XDE grids use (unit 8G1).</summary>
/// <remarks>
/// <c>std::cout</c> lines (DDocStd::GetDocument's complaint) and <c>Message::SendFail</c> lines go to the interpreter's
/// output (decision 405). The commands of OCAF storage (<c>Open</c>, <c>Save</c>, <c>SaveAs</c>) and undo
/// (<c>Undo</c>, <c>Redo</c>) are not registered (decision 13304).
/// </remarks>
internal static class DDocStdCommands
{
    private sealed class Session
    {
        // the documents created, in order; Close * takes those still bound to their variable (Draw::Drawables)
        public readonly List<DrawDocument> Documents = [];

        // XCAFDoc_ShapeTool::SetAutoNaming: process-wide in OCCT, per interpreter here (decision 13302)
        public bool AutoNaming = true;
    }

    private static readonly ConditionalWeakTable<DrawInterpreter, Session> s_sessions = new();

    private static Session session(DrawInterpreter theDI) => s_sessions.GetValue(theDI, static _ => new Session());

    /// <summary>XCAFDoc_ShapeTool::AutoNaming for the interpreter's documents.</summary>
    internal static bool AutoNaming(DrawInterpreter theDI) => session(theDI).AutoNaming;

    /// <summary>XCAFDoc_ShapeTool::SetAutoNaming: for the interpreter's documents, existing and new.</summary>
    internal static void SetAutoNaming(DrawInterpreter theDI, bool theValue)
    {
        Session aSession = session(theDI);
        aSession.AutoNaming = theValue;
        foreach (DrawDocument aDoc in aSession.Documents)
        {
            aDoc.Document.AutoNaming = theValue;
        }
    }

    /// <summary>A new document (TDocStd_Application::NewDocument "BinXCAF") bound to the variable (Draw::Set).</summary>
    internal static XdeDoc_Document NewDocument(DrawInterpreter theDI, string theName, XdeDoc_Document? theDocument = null)
    {
        Session aSession = session(theDI);
        XdeDoc_Document aDoc = theDocument ?? new XdeDoc_Document();
        aDoc.AutoNaming = aSession.AutoNaming;
        var aDrawDoc = new DrawDocument(aDoc);
        aSession.Documents.RemoveAll(d => theDI.GetDrawable(d.Name) != d);
        aSession.Documents.Add(aDrawDoc);
        theDI.SetDrawable(theName, aDrawDoc);
        return aDoc;
    }

    /// <summary>DDocStd::GetDocument: the document of the variable, or null (with a line on the output if asked).</summary>
    internal static XdeDoc_Document? GetDocument(DrawInterpreter theDI, string theName, bool theToComplain = true)
    {
        if (theDI.GetDrawable(theName) is DrawDocument aDoc)
        {
            return aDoc.Document;
        }
        if (theToComplain)
        {
            theDI.Output.Write(theName + " is not a Document\n");
        }
        return null;
    }

    /// <summary>
    /// TDF_Tool::Label(data, entry, label): the label of the entry ("0:1:1:2"), or the null label when the entry isn't
    /// one (any character but digits and ':') or names no label. The tags after the first are followed from the root;
    /// the root itself (an entry of one tag), which Draw-lite's documents don't have, gives the null label.
    /// </summary>
    internal static XdeDoc_Label Label(XdeDoc_Document theDoc, string theEntry)
    {
        var aTags = new List<int>();
        int n = 0;
        int i = 0;
        while (i < theEntry.Length)
        {
            while (i < theEntry.Length && theEntry[i] >= '0' && theEntry[i] <= '9')
            {
                n = unchecked(10 * n + (theEntry[i] - '0'));
                ++i;
            }
            if (i == theEntry.Length || theEntry[i] == ':')
            {
                aTags.Add(n);
                n = 0;
                if (i < theEntry.Length)
                {
                    ++i;
                }
            }
            else
            {
                aTags.Clear();
                break;
            }
        }
        if (aTags.Count < 2)
        {
            return default;
        }
        return theDoc.FindLabel("0:" + string.Join(':', aTags.Skip(1).Select(t => t.ToString(CultureInfo.InvariantCulture))));
    }

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "NewDocument", NewDocumentCommand);
        DrawCxx.Add(theInterpreter, "Close", Close);
        DrawCxx.Add(theInterpreter, "UndoLimit", UndoLimitCommand);
        DrawCxx.Add(theInterpreter, "NewCommand", TransactionCommand);
        DrawCxx.Add(theInterpreter, "OpenCommand", TransactionCommand);
        DrawCxx.Add(theInterpreter, "AbortCommand", TransactionCommand);
        DrawCxx.Add(theInterpreter, "CommitCommand", TransactionCommand);
        DrawCxx.Add(theInterpreter, "SetName", SetName);
        DrawCxx.Add(theInterpreter, "GetName", GetName);
        DrawCxx.Add(theInterpreter, "ForgetAll", ForgetAll);
    }

    // NewDocument docname [format]
    private static int NewDocumentCommand(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int nb = a.Count;
        if (nb == 2 || nb == 3)
        {
            if (GetDocument(theDI, a[1], false) is null)
            {
                XdeDoc_Document aDoc = NewDocument(theDI, a[1]);
                di.Append(nb == 2 ? "document (not handled by application)  " : "document ").Append(a[1]).Append(" created\n");
                // DDocStd::ReturnLabel (D->Main())
                di.Append(aDoc.Main().Entry());
            }
            else
            {
                di.Append(a[1]).Append(" is already a document\n");
            }
            return 0;
        }
        di.Append("DDocStd_NewDocument : Error\n");
        return 1;
    }

    // Close {Doc|*|-all} [-silent]
    private static int Close(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> a)
    {
        bool toComplain = true;
        var aDocNames = new List<string>();
        for (int anArgIt = 1; anArgIt < a.Count; ++anArgIt)
        {
            string anArgCase = a[anArgIt].ToLowerInvariant();
            if (anArgCase is "*" or "-all" or "all")
            {
                Session aSession = session(theDI);
                aSession.Documents.RemoveAll(d => theDI.GetDrawable(d.Name) != d);
                aDocNames.AddRange(aSession.Documents.Select(d => d.Name));
                if (aDocNames.Count == 0)
                {
                    return 0;
                }
            }
            else if (anArgCase == "-silent")
            {
                toComplain = false;
            }
            else
            {
                aDocNames.Add(a[anArgIt]);
            }
        }

        if (aDocNames.Count == 0)
        {
            theResult.Append("Syntax error: wrong number of arguments");
            return 1;
        }

        foreach (string aDocName in aDocNames)
        {
            if (GetDocument(theDI, aDocName, toComplain) is null && toComplain)
            {
                return 1;
            }
            // TDocStd_Application::Close; Draw::Set (name, null) unsets the variable
            theDI.SetDrawable(aDocName, null);
        }
        return 0;
    }

    // UndoLimit DOC (limit): Draw-lite keeps no undo, so no undo or redo is ever available (decision 13304)
    private static int UndoLimitCommand(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }
        if (GetDocument(theDI, a[1]) is null || theDI.GetDrawable(a[1]) is not DrawDocument aDrawDoc)
        {
            return 1;
        }
        if (a.Count > 2)
        {
            aDrawDoc.UndoLimit = theDI.Atoi(a[2]);
        }
        di.Append(aDrawDoc.UndoLimit).Append(" ").Append(0).Append(" ").Append(0);
        return 0;
    }

    // NewCommand, OpenCommand, AbortCommand, CommitCommand DOC: transactions, which Draw-lite's documents don't keep
    private static int TransactionCommand(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }
        return GetDocument(theDI, a[1]) is null ? 1 : 0;
    }

    // ForgetAll dfname Label (DDF_BasicCommands.cxx): the label's attributes and its children's
    private static int ForgetAll(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 3)
        {
            return 1;
        }
        XdeDoc_Document? aDoc = getDF(theDI, a[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = Label(aDoc, a[2]);
        if (aLabel.IsNull())
        {
            return 1;
        }
        aDoc.ForgetAllAttributes(aLabel);
        return 0;
    }

    // DDF::GetDF: the document's data framework, with DDF's complaint
    private static XdeDoc_Document? getDF(DrawInterpreter theDI, string theName)
    {
        XdeDoc_Document? aDoc = GetDocument(theDI, theName, false);
        if (aDoc is null)
        {
            theDI.Output.Write("framework " + theName + " not found \n");
        }
        return aDoc;
    }

    // SetName (DF, entry, name [,guid]): DDataStd's, on the labels of a Draw-lite document
    private static int SetName(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count == 4 || a.Count == 5)
        {
            XdeDoc_Document? aDoc = getDF(theDI, a[1]);
            if (aDoc is null)
            {
                return 1;
            }
            // DDF::AddLabel creates a missing label (TDF_Tool::Label with create), which Draw-lite's documents can't:
            // the command then fails (decision 13304)
            XdeDoc_Label aLabel = Label(aDoc, a[2]);
            if (aLabel.IsNull())
            {
                di.Append("Label is not found\n");
                return 1;
            }
            // a GUID (fifth argument) names a user name attribute, which Draw-lite's documents don't have: the name is set
            aDoc.SetName(aLabel, a[3]);
            return 0;
        }
        di.Append("DDataStd_SetName : Error\n");
        return 1;
    }

    // GetName (DF, entry [,guid])
    private static int GetName(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count == 3 || a.Count == 4)
        {
            XdeDoc_Document? aDoc = getDF(theDI, a[1]);
            if (aDoc is null)
            {
                return 1;
            }
            XdeDoc_Label aLabel = Label(aDoc, a[2]);
            if (aLabel.IsNull())
            {
                di.Append("Label is not found\n");
            }
            if (aLabel.IsNull() || !aDoc.GetName(aLabel, out string? aName))
            {
                di.Append("Name attribute is not found or not set");
                return 1;
            }
            di.Append(aName ?? "");
            return 0;
        }
        di.Append("DDataStd_SetName : Error\n");
        return 1;
    }

    /// <summary>
    /// Draw::ParseColor: a colour from the arguments, as a name (optionally followed by an alpha), a hex code, or three
    /// or four components (integers 0..255 with one above 1, as sRGB; else reals 0..1, as linear RGB). Returns the
    /// number of arguments used, 0 on a syntax error.
    /// </summary>
    internal static int ParseColor(DrawInterpreter theDI, IReadOnlyList<string> theArgs, int theFirst, out Quantity_ColorRGBA theColor, bool theToParseAlpha = true)
    {
        theColor = new Quantity_ColorRGBA();
        int theArgNb = theArgs.Count - theFirst;
        if (theArgNb >= 1 && Quantity_ColorRGBA.ColorFromHex(theArgs[theFirst], ref theColor, !theToParseAlpha))
        {
            return 1;
        }
        if (theArgNb >= 1 && Quantity_ColorRGBA.ColorFromName(theArgs[theFirst], ref theColor))
        {
            if (theArgNb >= 2 && theToParseAlpha && parseColorComponent(theDI, theArgs[theFirst + 1], out float anAlpha))
            {
                theColor = theColor.WithAlpha(anAlpha);
                return 2;
            }
            return 1;
        }
        if (theArgNb >= 3)
        {
            int aToParse = Math.Min(theArgNb, theToParseAlpha ? 4 : 3);
            int aParsed = aToParse;
            if (parseIntegerColor(theDI, ref aParsed, theArgs, theFirst, ref theColor))
            {
                return aParsed;
            }
            aParsed = aToParse;
            if (parseRealColor(theDI, ref aParsed, theArgs, theFirst, ref theColor))
            {
                return aParsed;
            }
        }
        return 0;
    }

    // Draw::ParseInteger: a real expression with an integer value
    internal static bool ParseInteger(DrawInterpreter theDI, string theText, out int theValue)
    {
        theValue = 0;
        if (!theDI.ParseReal(theText, out double aReal))
        {
            return false;
        }
        int anInt = (int)aReal;
        if (anInt != aReal)
        {
            return false;
        }
        theValue = anInt;
        return true;
    }

    private static bool parseIntegerComponent(DrawInterpreter theDI, string theText, out int theValue)
        => ParseInteger(theDI, theText, out theValue) && theValue >= 0 && theValue <= 255;

    private static bool parseRealComponent(DrawInterpreter theDI, string theText, out float theValue)
    {
        theValue = 0f;
        if (!theDI.ParseReal(theText, out double aReal))
        {
            return false;
        }
        float aShort = (float)aReal;
        if (aShort < 0.0f || aShort > 1.0f)
        {
            return false;
        }
        theValue = aShort;
        return true;
    }

    private static bool parseColorComponent(DrawInterpreter theDI, string theText, out float theValue)
    {
        if (parseIntegerComponent(theDI, theText, out int anInt))
        {
            theValue = anInt == 1 ? 1.0f : anInt * 1.0f / 255;
            return true;
        }
        return parseRealComponent(theDI, theText, out theValue);
    }

    private static bool parseIntegerColor(DrawInterpreter theDI, ref int theNb, IReadOnlyList<string> theArgs, int theFirst, ref Quantity_ColorRGBA theColor)
    {
        int[] aColor = [-1, -1, -1, -1];
        for (int i = 0; i < theNb; ++i)
        {
            if (parseIntegerComponent(theDI, theArgs[theFirst + i], out int aValue))
            {
                aColor[i] = aValue;
            }
            else if (i == 3)
            {
                theNb = 3;
            }
            else
            {
                return false;
            }
        }
        if (aColor.Max() <= 1)
        {
            return false;
        }
        if (aColor[3] == -1)
        {
            aColor[3] = 255;
        }
        var aReal = new NCollection_Vec4<float>(aColor[0] / 255f, aColor[1] / 255f, aColor[2] / 255f, aColor[3] / 255f);
        NCollection_Vec4<float> aLinear = Quantity_ColorRGBA.Convert_sRGB_To_LinearRGB(aReal);
        theColor = new Quantity_ColorRGBA(aLinear.x(), aLinear.y(), aLinear.z(), aLinear.w());
        return true;
    }

    private static bool parseRealColor(DrawInterpreter theDI, ref int theNb, IReadOnlyList<string> theArgs, int theFirst, ref Quantity_ColorRGBA theColor)
    {
        float[] aColor = [1f, 1f, 1f, 1f];
        for (int i = 0; i < theNb; ++i)
        {
            if (parseRealComponent(theDI, theArgs[theFirst + i], out float aValue))
            {
                aColor[i] = aValue;
            }
            else if (i == 3)
            {
                theNb = 3;
            }
            else
            {
                return false;
            }
        }
        theColor = new Quantity_ColorRGBA(aColor[0], aColor[1], aColor[2], aColor[3]);
        return true;
    }
}
