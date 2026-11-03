// Port of: src/FoundationClasses/TKernel/GTests/Message_Messenger_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>Tests that change or read the process-wide messenger run one at a time.</summary>
[CollectionDefinition(Name, DisableParallelization = true)]
public sealed class DefaultMessengerCollection
{
    public const string Name = "Message.DefaultMessenger";
}

/// <summary>Custom message printer that collects messages into a string builder.</summary>
internal sealed class TestMessagePrinter(StringBuilder theStream) : global::Kerf.Engine.Foundation.Printer
{
    protected override void SendCore(string theString, global::Kerf.Engine.Foundation.Gravity theGravity)
    {
        if (theGravity >= Info)
        {
            theStream.Append(theString).Append('\n');
        }
    }
}

/// <summary>A printer recording each message with its gravity, whatever the gravity.</summary>
internal sealed class RecordingPrinter : global::Kerf.Engine.Foundation.Printer
{
    public readonly List<(string Text, global::Kerf.Engine.Foundation.Gravity Gravity)> Messages = [];

    public RecordingPrinter()
    {
        SetTraceLevel(Trace);
    }

    protected override void SendCore(string theString, global::Kerf.Engine.Foundation.Gravity theGravity)
    {
        lock (Messages)
        {
            Messages.Add((theString, theGravity));
        }
    }
}

[Collection(DefaultMessengerCollection.Name)]
public class Message_Messenger_Test
{
    // The C++ stream buffers send on destruction at the end of their scope; here at Dispose (a using declaration) or Flush.
    [Fact]
    public void OCC31189_StreamBufferMessageOrdering()
    {
        // Bug OCC31189: Test consistency of messages output using stream buffer interface
        // Verify that messages sent via stream buffers and directly to messenger don't intermix

        StringBuilder anOutput = new();
        TestMessagePrinter aPrinter = new(anOutput);
        global::Kerf.Engine.Foundation.Messenger aMsgMgr = global::Kerf.Engine.Foundation.Message.DefaultMessenger();

        // Save original printers
        NCollection_Sequence<global::Kerf.Engine.Foundation.Printer> anOriginalPrinters = new();
        anOriginalPrinters.Append(aMsgMgr.ChangePrinters());

        // Replace with our test printer
        aMsgMgr.ChangePrinters().Clear();
        aMsgMgr.AddPrinter(aPrinter);

        // Scope block to test output of message on destruction of a stream buffer
        {
            using global::Kerf.Engine.Foundation.Messenger.StreamBuffer aSender = global::Kerf.Engine.Foundation.Message.SendInfo();

            // Check that messages output to sender and directly to messenger do not intermix
            aSender.Append("Sender message 1: start ...");
            aMsgMgr.Send("Direct message 1");
            aSender.Append("... end");
            aSender.Flush(true);

            // Check that empty stream buffer does not produce output on destruction
            global::Kerf.Engine.Foundation.Message.SendInfo().Dispose();

            // Additional message to check that they go in expected order
            aMsgMgr.Send("Direct message 2");

            // Check that empty stream buffer does produce empty line if explicit flush is requested
            using global::Kerf.Engine.Foundation.Messenger.StreamBuffer anEmptySender = global::Kerf.Engine.Foundation.Message.SendInfo();
            anEmptySender.Flush(true);

            // Last message should be sent on destruction of a sender
            aSender.Append("Sender message 2");
        }

        // Restore original printers
        aMsgMgr.ChangePrinters().Clear();
        aMsgMgr.ChangePrinters().Append(anOriginalPrinters);

        // Verify the output matches expected format
        string anOutputStr = anOutput.ToString();

        Assert.Contains("Direct message 1", anOutputStr);
        Assert.Contains("Sender message 1: start ...... end", anOutputStr);
        Assert.Contains("Direct message 2", anOutputStr);
        Assert.Contains("Sender message 2", anOutputStr);

        int aDirect1Pos = anOutputStr.IndexOf("Direct message 1", StringComparison.Ordinal);
        int aSender1Pos = anOutputStr.IndexOf("Sender message 1", StringComparison.Ordinal);
        int aDirect2Pos = anOutputStr.IndexOf("Direct message 2", StringComparison.Ordinal);
        int aSender2Pos = anOutputStr.IndexOf("Sender message 2", StringComparison.Ordinal);

        Assert.True(aDirect1Pos < aSender1Pos, "Direct message 1 should appear before Sender message 1");
        Assert.True(aSender1Pos < aDirect2Pos, "Sender message 1 should appear before Direct message 2");
        Assert.True(aDirect2Pos < aSender2Pos, "Direct message 2 should appear before Sender message 2");

        // Addition: the exact output, with the empty line of the forced flush
        Assert.Equal("Direct message 1\nSender message 1: start ...... end\nDirect message 2\n\nSender message 2\n", anOutputStr);
    }

    [Fact]
    public void StreamBufferBasicUsage()
    {
        // Test basic stream buffer functionality
        StringBuilder anOutput = new();
        TestMessagePrinter aPrinter = new(anOutput);
        global::Kerf.Engine.Foundation.Messenger aMsgMgr = global::Kerf.Engine.Foundation.Message.DefaultMessenger();

        NCollection_Sequence<global::Kerf.Engine.Foundation.Printer> anOriginalPrinters = new();
        anOriginalPrinters.Append(aMsgMgr.ChangePrinters());

        aMsgMgr.ChangePrinters().Clear();
        aMsgMgr.AddPrinter(aPrinter);
        {
            using global::Kerf.Engine.Foundation.Messenger.StreamBuffer aBuffer = global::Kerf.Engine.Foundation.Message.SendInfo();
            aBuffer.Append("Test message").Append('\n');
        }

        aMsgMgr.ChangePrinters().Clear();
        aMsgMgr.ChangePrinters().Append(anOriginalPrinters);

        string anOutputStr = anOutput.ToString();
        Assert.Contains("Test message", anOutputStr);
    }
}

/// <summary>Additions: the default messenger, messengers and printers.</summary>
[Collection(DefaultMessengerCollection.Name)]
public class Message_MessengerAdditionsTest
{
    [Fact]
    public void DefaultMessenger_SendsNothing()
    {
        global::Kerf.Engine.Foundation.Messenger aDefault = global::Kerf.Engine.Foundation.Message.DefaultMessenger();
        Assert.Equal(0, aDefault.Printers().Length());

        // Nothing reaches the console: capture it while sending every kind of message
        TextWriter anOut = Console.Out, anErr = Console.Error;
        StringWriter aCapture = new();
        try
        {
            Console.SetOut(aCapture);
            Console.SetError(aCapture);
            global::Kerf.Engine.Foundation.Message.SendFail("fail");
            global::Kerf.Engine.Foundation.Message.SendAlarm("alarm");
            global::Kerf.Engine.Foundation.Message.SendWarning("warning");
            global::Kerf.Engine.Foundation.Message.SendInfo("info");
            global::Kerf.Engine.Foundation.Message.SendTrace("trace");
            global::Kerf.Engine.Foundation.Message.Send("text", Fail);
            global::Kerf.Engine.Foundation.Message.SendFail().Append("buffer ").Append(1).Append(2.5).Flush();
            global::Kerf.Engine.Foundation.Message.SendWarning().Endl();
            new global::Kerf.Engine.Foundation.Algorithm().SendMessages(Trace);
        }
        finally
        {
            Console.SetOut(anOut);
            Console.SetError(anErr);
        }
        Assert.Equal("", aCapture.ToString());
    }

    [Fact]
    public void SetDefaultMessenger_InstallsAndRestores()
    {
        RecordingPrinter aPrinter = new();
        global::Kerf.Engine.Foundation.Message.SetDefaultMessenger(new global::Kerf.Engine.Foundation.Messenger(aPrinter));
        try
        {
            global::Kerf.Engine.Foundation.Message.SendWarning("w");
            global::Kerf.Engine.Foundation.Message.SendFail().Append("f ").Append(42).Append(' ').Append(0.1).Flush();
            global::Kerf.Engine.Foundation.Message.Send(Info).Append(true).Dispose();
            Assert.Equal([("w", Warning), ("f 42 0.1", Fail), ("1", Info)], aPrinter.Messages);
        }
        finally
        {
            global::Kerf.Engine.Foundation.Message.SetDefaultMessenger(null);
        }
        Assert.Equal(0, global::Kerf.Engine.Foundation.Message.DefaultMessenger().Printers().Length());
    }

    [Fact]
    public void Messenger_PrinterList()
    {
        RecordingPrinter aFirst = new(), aSecond = new();
        global::Kerf.Engine.Foundation.Messenger aMessenger = new(aFirst);
        Assert.False(aMessenger.AddPrinter(aFirst));
        Assert.True(aMessenger.AddPrinter(aSecond));
        aMessenger.SendInfo("both");
        Assert.True(aMessenger.RemovePrinter(aFirst));
        Assert.False(aMessenger.RemovePrinter(aFirst));
        aMessenger.SendInfo("second");
        Assert.Equal(1, aMessenger.RemovePrinters(typeof(RecordingPrinter)));
        Assert.Equal(0, aMessenger.Printers().Length());
        Assert.Equal([("both", Info)], aFirst.Messages);
        Assert.Equal([("both", Info), ("second", Info)], aSecond.Messages);

        // Printers() is a copy
        global::Kerf.Engine.Foundation.Messenger anOther = new(aFirst);
        anOther.Printers().Clear();
        Assert.Equal(1, anOther.Printers().Length());
    }

    [Fact]
    public void Printer_TraceLevelFilters()
    {
        StringBuilder anOutput = new();
        global::Kerf.Engine.Foundation.PrinterOStream aPrinter = new(new StringWriter(anOutput), Warning);
        global::Kerf.Engine.Foundation.Messenger aMessenger = new(aPrinter);
        aMessenger.SendInfo("info");
        aMessenger.SendWarning("warning");
        aMessenger.SendFail("fail");
        aMessenger.Send(new StringBuilder("stream"), Alarm);
        aMessenger.Send(aMessenger, Fail);
        aPrinter.Close();
        string[] aLines = anOutput.ToString().Split('\n');
        Assert.Equal("warning", aLines[0]);
        Assert.Equal("fail", aLines[1]);
        Assert.Equal("stream", aLines[2]);
        Assert.StartsWith("Message_Messenger: 0x", aLines[3]);
        Assert.Equal("", aLines[4]);
        Assert.Equal(Warning, aPrinter.GetTraceLevel());
        Assert.Null(aPrinter.GetStream());
    }

    [Fact]
    public void FillTime()
    {
        Assert.Equal("05h:12m:26.33s", global::Kerf.Engine.Foundation.Message.FillTime(5, 12, 26.3345));
        Assert.Equal("06m:34.50s", global::Kerf.Engine.Foundation.Message.FillTime(0, 6, 34.496));
        Assert.Equal("4.50s", global::Kerf.Engine.Foundation.Message.FillTime(0, 0, 4.5));
    }
}
