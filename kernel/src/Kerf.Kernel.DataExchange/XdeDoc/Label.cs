// New in Kerf (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Documents;

/// <summary>
/// A label of an <see cref="ExchangeDocument"/>, in place of OCAF's <c>TDF_Label</c> in code ported from XCAF's users:
/// a handle on a shape, component, sub-shape or layer of the document. The default value is the null label.
/// </summary>
internal readonly struct Label : IEquatable<Label>
{
    private readonly ExchangeDocument? _document;
    private readonly int _index;

    internal Label(ExchangeDocument document, int index)
    {
        _document = document;
        _index = index;
    }

    internal int Index => _index;

    /// <summary>The document, or null for the null label.</summary>
    public ExchangeDocument? Document => _document;

    /// <summary>True for the null label.</summary>
    public bool IsNull => _document is null;

    /// <summary>The father label (null for the document's main label).</summary>
    public Label Father() => _document is null ? default : _document.FatherOf(this);

    /// <summary>The label's tag: its number among its father's children, from 1.</summary>
    public int Tag() => _document is null ? 0 : _document.TagOf(this);

    /// <summary>The label's entry, as OCAF writes it ("0:1:1:2"); empty for the null label.</summary>
    public string Entry() => _document is null ? "" : _document.EntryOf(this);

    /// <summary>True if both are the same label of the same document.</summary>
    public bool IsEqual(in Label other) => Equals(other);

    /// <inheritdoc/>
    public bool Equals(Label other) => ReferenceEquals(_document, other._document) && _index == other._index;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Label other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode() => _document is null ? 0 : HashCode.Combine(_document.GetHashCode(), _index);

    /// <summary>Same label.</summary>
    public static bool operator ==(Label left, Label right) => left.Equals(right);

    /// <summary>Different labels.</summary>
    public static bool operator !=(Label left, Label right) => !left.Equals(right);

    /// <inheritdoc/>
    public override string ToString() => IsNull ? "(null label)" : Entry();
}
