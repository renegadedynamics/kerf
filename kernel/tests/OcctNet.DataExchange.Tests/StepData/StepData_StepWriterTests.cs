// Port of: src/DataExchange/TKDESTEP/GTests/StepData_StepWriter_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.DataExchange.Tests;

public class StepData_StepWriterTests
{
    // Test CleanTextForSend with basic character escaping
    [Fact]
    public void CleanTextForSend_BasicEscaping()
    {
        // Test single quote escaping
        Assert.Equal("text with ''single quotes''", StepData_StepWriter.CleanTextForSend("text with 'single quotes'"));

        // Test backslash escaping
        Assert.Equal("path\\\\with\\\\backslashes", StepData_StepWriter.CleanTextForSend("path\\with\\backslashes"));

        // Test newline escaping
        Assert.Equal("line1\\N\\line2", StepData_StepWriter.CleanTextForSend("line1\nline2"));

        // Test tab escaping
        Assert.Equal("text\\T\\with\\T\\tabs", StepData_StepWriter.CleanTextForSend("text\twith\ttabs"));
    }

    // Test CleanTextForSend with control directives preservation
    [Fact]
    public void CleanTextForSend_ControlDirectivePreservation()
    {
        // Test \X\ control directive preservation
        Assert.Equal("text with \\XA7\\ section sign", StepData_StepWriter.CleanTextForSend("text with \\XA7\\ section sign"));

        // Test \X2\ control directive preservation
        Assert.Equal("\\X2\\03C0\\X0\\ is pi", StepData_StepWriter.CleanTextForSend("\\X2\\03C0\\X0\\ is pi"));

        // Test \X4\ control directive preservation
        Assert.Equal("emoji \\X4\\001F600\\X0\\ face", StepData_StepWriter.CleanTextForSend("emoji \\X4\\001F600\\X0\\ face"));

        // Test \S\ control directive preservation
        Assert.Equal("text with \\S\\ directive", StepData_StepWriter.CleanTextForSend("text with \\S\\ directive"));

        // Test \P\ control directive preservation
        Assert.Equal("\\PA\\ code page setting", StepData_StepWriter.CleanTextForSend("\\PA\\ code page setting"));
    }

    // Test CleanTextForSend with existing \N\ and \T\ directive preservation
    [Fact]
    public void CleanTextForSend_ExistingDirectivePreservation()
    {
        // Test existing \N\ directive preservation
        Assert.Equal("line1\\N\\line2", StepData_StepWriter.CleanTextForSend("line1\\N\\line2"));

        // Test existing \T\ directive preservation
        Assert.Equal("text\\T\\with\\T\\tab", StepData_StepWriter.CleanTextForSend("text\\T\\with\\T\\tab"));
    }

    // Test CleanTextForSend with mixed content
    [Fact]
    public void CleanTextForSend_MixedContent()
    {
        // Test quotes outside control directives
        Assert.Equal("see \\XA7\\ section and ''quotes''", StepData_StepWriter.CleanTextForSend("see \\XA7\\ section and 'quotes'"));

        // Test backslashes outside control directives
        Assert.Equal("\\XA7\\ and path\\\\file", StepData_StepWriter.CleanTextForSend("\\XA7\\ and path\\file"));

        // Test complex mixture
        Assert.Equal("prefix \\X2\\03B103B2\\X0\\ ''text'' with\\T\\tab",
                     StepData_StepWriter.CleanTextForSend("prefix \\X2\\03B103B2\\X0\\ 'text' with\ttab"));
    }

    // Test CleanTextForSend with edge cases
    [Fact]
    public void CleanTextForSend_EdgeCases()
    {
        // Test empty string
        Assert.Equal("", StepData_StepWriter.CleanTextForSend(""));

        // Test string with only quotes
        Assert.Equal("''''", StepData_StepWriter.CleanTextForSend("''"));

        // Test string with only control directive
        Assert.Equal("\\XA7\\", StepData_StepWriter.CleanTextForSend("\\XA7\\"));

        // Test consecutive control directives
        Assert.Equal("\\XA7\\\\XB6\\", StepData_StepWriter.CleanTextForSend("\\XA7\\\\XB6\\"));
    }

    // Test CleanTextForSend with malformed but safe input
    [Fact]
    public void CleanTextForSend_MalformedInput()
    {
        // Test incomplete control directive (should be treated as regular text)
        Assert.Equal("incomplete \\\\X and ''quotes''", StepData_StepWriter.CleanTextForSend("incomplete \\X and 'quotes'"));

        // Test partial control directive
        Assert.Equal("partial \\\\XA and more", StepData_StepWriter.CleanTextForSend("partial \\XA and more"));
    }

    // Test CleanTextForSend hex sequence detection
    [Fact]
    public void CleanTextForSend_HexSequenceDetection()
    {
        // Test valid hex sequences in \X2\ directive
        Assert.Equal("\\X2\\03B103B203B3\\X0\\", StepData_StepWriter.CleanTextForSend("\\X2\\03B103B203B3\\X0\\"));

        // Test valid hex sequences in \X4\ directive
        Assert.Equal("\\X4\\001F600001F638\\X0\\", StepData_StepWriter.CleanTextForSend("\\X4\\001F600001F638\\X0\\"));

        // Test text around hex sequences
        Assert.Equal("start \\X2\\03C0\\X0\\ end", StepData_StepWriter.CleanTextForSend("start \\X2\\03C0\\X0\\ end"));
    }
}
