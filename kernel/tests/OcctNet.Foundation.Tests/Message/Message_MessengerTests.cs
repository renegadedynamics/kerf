// Port of: src/FoundationClasses/TKernel/GTests/Message_Messenger_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using static Occt.Message_Gravity;

namespace OcctNet.Foundation.Tests;

/// <summary>Tests that change or read the process-wide messenger run one at a time.</summary>
[CollectionDefinition(Name, DisableParallelization = true)]
public sealed class DefaultMessengerCollection
{
    public const string Name = "Message.DefaultMessenger";
}

/// <summary>Custom message printer that collects messages into a string builder.</summary>
internal sealed class TestMessagePrinter(StringBuilder theStream) : Message_Printer
{
    protected override void send(string theString, Message_Gravity theGravity)
    {
        if (theGravity >= Message_Info)
        {
            theStream.Append(theString).Append('\n');
        }
    }
}

/// <summary>A printer recording each message with its gravity, whatever the gravity.</summary>
internal sealed class RecordingPrinter : Message_Printer
{
    public readonly List<(string Text, Message_Gravity Gravity)> Messages = [];

    public RecordingPrinter()
    {
        SetTraceLevel(Message_Trace);
    }

    protected override void send(string theString, Message_Gravity theGravity)
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
        Message_Messenger aMsgMgr = Message.DefaultMessenger();

        // Save original printers
        NCollection_Sequence<Message_Printer> anOriginalPrinters = new();
        anOriginalPrinters.Append(aMsgMgr.ChangePrinters());

        // Replace with our test printer
        aMsgMgr.ChangePrinters().Clear();
        aMsgMgr.AddPrinter(aPrinter);

        // Scope block to test output of message on destruction of a stream buffer
        {
            using Message_Messenger.StreamBuffer aSender = Message.SendInfo();

            // Check that messages output to sender and directly to messenger do not intermix
            aSender.Append("Sender message 1: start ...");
            aMsgMgr.Send("Direct message 1");
            aSender.Append("... end");
            aSender.Flush(true);

            // Check that empty stream buffer does not produce output on destruction
            Message.SendInfo().Dispose();

            // Additional message to check that they go in expected order
            aMsgMgr.Send("Direct message 2");

            // Check that empty stream buffer does produce empty line if explicit flush is requested
            using Message_Messenger.StreamBuffer anEmptySender = Message.SendInfo();
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
        Message_Messenger aMsgMgr = Message.DefaultMessenger();

        NCollection_Sequence<Message_Printer> anOriginalPrinters = new();
        anOriginalPrinters.Append(aMsgMgr.ChangePrinters());

        aMsgMgr.ChangePrinters().Clear();
        aMsgMgr.AddPrinter(aPrinter);
        {
            using Message_Messenger.StreamBuffer aBuffer = Message.SendInfo();
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
        Message_Messenger aDefault = Message.DefaultMessenger();
        Assert.Equal(0, aDefault.Printers().Length());

        // Nothing reaches the console: capture it while sending every kind of message
        TextWriter anOut = Console.Out, anErr = Console.Error;
        StringWriter aCapture = new();
        try
        {
            Console.SetOut(aCapture);
            Console.SetError(aCapture);
            Message.SendFail("fail");
            Message.SendAlarm("alarm");
            Message.SendWarning("warning");
            Message.SendInfo("info");
            Message.SendTrace("trace");
            Message.Send("text", Message_Fail);
            Message.SendFail().Append("buffer ").Append(1).Append(2.5).Flush();
            Message.SendWarning().Endl();
            new Message_Algorithm().SendMessages(Message_Trace);
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
        Message.SetDefaultMessenger(new Message_Messenger(aPrinter));
        try
        {
            Message.SendWarning("w");
            Message.SendFail().Append("f ").Append(42).Append(' ').Append(0.1).Flush();
            Message.Send(Message_Info).Append(true).Dispose();
            Assert.Equal([("w", Message_Warning), ("f 42 0.1", Message_Fail), ("1", Message_Info)], aPrinter.Messages);
        }
        finally
        {
            Message.SetDefaultMessenger(null);
        }
        Assert.Equal(0, Message.DefaultMessenger().Printers().Length());
    }

    [Fact]
    public void Messenger_PrinterList()
    {
        RecordingPrinter aFirst = new(), aSecond = new();
        Message_Messenger aMessenger = new(aFirst);
        Assert.False(aMessenger.AddPrinter(aFirst));
        Assert.True(aMessenger.AddPrinter(aSecond));
        aMessenger.SendInfo("both");
        Assert.True(aMessenger.RemovePrinter(aFirst));
        Assert.False(aMessenger.RemovePrinter(aFirst));
        aMessenger.SendInfo("second");
        Assert.Equal(1, aMessenger.RemovePrinters(typeof(RecordingPrinter)));
        Assert.Equal(0, aMessenger.Printers().Length());
        Assert.Equal([("both", Message_Info)], aFirst.Messages);
        Assert.Equal([("both", Message_Info), ("second", Message_Info)], aSecond.Messages);

        // Printers() is a copy
        Message_Messenger anOther = new(aFirst);
        anOther.Printers().Clear();
        Assert.Equal(1, anOther.Printers().Length());
    }

    [Fact]
    public void Printer_TraceLevelFilters()
    {
        StringBuilder anOutput = new();
        Message_PrinterOStream aPrinter = new(new StringWriter(anOutput), Message_Warning);
        Message_Messenger aMessenger = new(aPrinter);
        aMessenger.SendInfo("info");
        aMessenger.SendWarning("warning");
        aMessenger.SendFail("fail");
        aMessenger.Send(new StringBuilder("stream"), Message_Alarm);
        aMessenger.Send(aMessenger, Message_Fail);
        aPrinter.Close();
        string[] aLines = anOutput.ToString().Split('\n');
        Assert.Equal("warning", aLines[0]);
        Assert.Equal("fail", aLines[1]);
        Assert.Equal("stream", aLines[2]);
        Assert.StartsWith("Message_Messenger: 0x", aLines[3]);
        Assert.Equal("", aLines[4]);
        Assert.Equal(Message_Warning, aPrinter.GetTraceLevel());
        Assert.Null(aPrinter.GetStream());
    }

    [Fact]
    public void FillTime()
    {
        Assert.Equal("05h:12m:26.33s", Message.FillTime(5, 12, 26.3345));
        Assert.Equal("06m:34.50s", Message.FillTime(0, 6, 34.496));
        Assert.Equal("4.50s", Message.FillTime(0, 0, 4.5));
    }
}
