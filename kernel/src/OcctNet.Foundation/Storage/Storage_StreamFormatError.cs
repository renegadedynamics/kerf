// Port of: src/FoundationClasses/TKernel/Storage/Storage_StreamFormatError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Thrown when a stream doesn't hold what its format says. The BRep readers (text and binary) throw it for input
/// they can't read safely: counts larger than the rest of the input could hold, words longer than OCCT's buffers,
/// nesting beyond the caps, references to entries that don't exist, and input that ends where OCCT would loop.
/// </summary>
[OcctName("Storage_StreamFormatError")]
public class Storage_StreamFormatError : Standard_Failure
{
    public Storage_StreamFormatError() { }
    public Storage_StreamFormatError(string? message) : base(message) { }
    public Storage_StreamFormatError(string? message, Exception? innerException) : base(message, innerException) { }
}
