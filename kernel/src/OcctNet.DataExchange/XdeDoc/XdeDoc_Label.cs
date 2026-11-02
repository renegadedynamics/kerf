// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A label of an <see cref="XdeDoc_Document"/>, in place of OCAF's <c>TDF_Label</c> in code ported from XCAF's users:
/// a handle on a shape, component, sub-shape or layer of the document. The default value is the null label.
/// </summary>
public readonly struct XdeDoc_Label : IEquatable<XdeDoc_Label>
{
    private readonly XdeDoc_Document? myDocument;
    private readonly int myIndex;

    internal XdeDoc_Label(XdeDoc_Document theDocument, int theIndex)
    {
        myDocument = theDocument;
        myIndex = theIndex;
    }

    internal int Index => myIndex;

    /// <summary>The document, or null for the null label.</summary>
    public XdeDoc_Document? Document => myDocument;

    /// <summary>True for the null label.</summary>
    public bool IsNull() => myDocument is null;

    /// <summary>The father label (null for the document's main label).</summary>
    public XdeDoc_Label Father() => myDocument is null ? default : myDocument.FatherOf(this);

    /// <summary>The label's tag: its number among its father's children, from 1.</summary>
    public int Tag() => myDocument is null ? 0 : myDocument.TagOf(this);

    /// <summary>The label's entry, as OCAF writes it ("0:1:1:2"); empty for the null label.</summary>
    public string Entry() => myDocument is null ? "" : myDocument.EntryOf(this);

    /// <summary>True if both are the same label of the same document.</summary>
    public bool IsEqual(in XdeDoc_Label theOther) => Equals(theOther);

    /// <inheritdoc/>
    public bool Equals(XdeDoc_Label other) => ReferenceEquals(myDocument, other.myDocument) && myIndex == other.myIndex;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is XdeDoc_Label other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode() => myDocument is null ? 0 : HashCode.Combine(myDocument.GetHashCode(), myIndex);

    /// <summary>Same label.</summary>
    public static bool operator ==(XdeDoc_Label theLeft, XdeDoc_Label theRight) => theLeft.Equals(theRight);

    /// <summary>Different labels.</summary>
    public static bool operator !=(XdeDoc_Label theLeft, XdeDoc_Label theRight) => !theLeft.Equals(theRight);

    /// <inheritdoc/>
    public override string ToString() => IsNull() ? "(null label)" : Entry();
}
