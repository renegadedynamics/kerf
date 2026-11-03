// Port of: src/FoundationClasses/TKernel/Message/Message_AttributeStream.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_AttributeStream.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation;

/// <summary>An alert attribute holding the text of a stream.</summary>
/// <remarks>OCCT's <c>Standard_SStream</c> is a <see cref="string"/>: the attribute keeps a copy of the stream's text.</remarks>
public class AttributeStream : MessageAttribute
{
    private string _stream = ""; //!< container of values

    /// <summary>An attribute holding the stream's text, with the name.</summary>
    public AttributeStream(string stream, string name = "")
        : base(name)
    {
        SetStream(stream);
    }

    /// <summary>The stream's text.</summary>
    public string Stream() => _stream;

    /// <summary>Replaces the stream's text.</summary>
    public void SetStream(string stream)
    {
        _stream = stream;
    }
}
