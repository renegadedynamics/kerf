// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of Message_Algorithm, Message_ExecStatus and Message_Msg (no GTests in OCCT).

using static Occt.Message_Gravity;
using static Occt.Message_Status;
using static Occt.Message_StatusType;

namespace OcctNet.Foundation.Tests;

/// <summary>An algorithm subclass, for the messages looked up by class name.</summary>
internal sealed class TestAlgorithm : Message_Algorithm;

public class Message_ExecStatusTest
{
    [Fact]
    public void SetIsSetClear()
    {
        Message_ExecStatus aStatus = new();
        Assert.False(aStatus.IsDone() || aStatus.IsWarn() || aStatus.IsAlarm() || aStatus.IsFail());
        aStatus.Set(Message_Done1);
        aStatus.Set(Message_Warn32);
        aStatus.Set(Message_Fail5);
        Assert.True(aStatus.IsSet(Message_Done1));
        Assert.True(aStatus.IsSet(Message_Warn32));
        Assert.True(aStatus.IsSet(Message_Fail5));
        Assert.False(aStatus.IsSet(Message_Fail4));
        Assert.False(aStatus.IsSet(Message_Alarm1));
        Assert.False(aStatus.IsSet(Message_None));
        Assert.True(aStatus.IsDone() && aStatus.IsWarn() && aStatus.IsFail());
        Assert.False(aStatus.IsAlarm());
        aStatus.Clear(Message_Fail5);
        Assert.False(aStatus.IsFail());
        aStatus.Clear();
        Assert.Equal(new Message_ExecStatus(), aStatus);
    }

    [Fact]
    public void Indices()
    {
        Assert.Equal(1, Message_ExecStatus.StatusIndex(Message_Done1));
        Assert.Equal(33, Message_ExecStatus.StatusIndex(Message_Warn1));
        Assert.Equal(64, Message_ExecStatus.StatusIndex(Message_Warn32));
        Assert.Equal(65, Message_ExecStatus.StatusIndex(Message_Alarm1));
        Assert.Equal(128, Message_ExecStatus.StatusIndex(Message_Fail32));
        Assert.Equal(0, Message_ExecStatus.StatusIndex(Message_None));
        Assert.Equal(7, Message_ExecStatus.LocalStatusIndex(Message_Fail7));
        Assert.Equal(Message_ALARM, Message_ExecStatus.TypeOfStatus(Message_Alarm3));
        for (int i = Message_ExecStatus.FirstStatus; i < Message_ExecStatus.LastStatus; ++i)
        {
            Assert.Equal(i, Message_ExecStatus.StatusIndex(Message_ExecStatus.StatusByIndex(i)));
        }
        Assert.Equal(Message_None, Message_ExecStatus.StatusByIndex(Message_ExecStatus.LastStatus));
    }

    [Fact]
    public void GroupOperationsAndTwins()
    {
        Message_ExecStatus aStatus = new(Message_Warn2);
        Message_ExecStatus anOther = new Message_ExecStatus(Message_Warn3).With(Message_Fail1);
        Message_ExecStatus aUnion = aStatus | anOther;
        Assert.True(aUnion.IsSet(Message_Warn2) && aUnion.IsSet(Message_Warn3) && aUnion.IsSet(Message_Fail1));
        Assert.False(aStatus.IsSet(Message_Warn3)); // the twins don't change the original
        Message_ExecStatus aCommon = aUnion & new Message_ExecStatus(Message_Warn3);
        Assert.Equal(new Message_ExecStatus(Message_Warn3), aCommon);
        Assert.Equal(aUnion.Without(Message_Fail1), aStatus.Added(new Message_ExecStatus(Message_Warn3)));
        Assert.True(aStatus.WithAllAlarm().IsSet(Message_Alarm17));
        Assert.False(aStatus.WithAllWarn().WithoutAllWarn().IsWarn());
        Assert.Equal(new Message_ExecStatus(), aUnion.Cleared());

        Message_ExecStatus aMutable = new();
        aMutable.SetAllFail();
        Assert.True(aMutable.IsSet(Message_Fail32));
        aMutable.ClearAllFail();
        Assert.False(aMutable.IsFail());
        aMutable.Add(aUnion);
        aMutable.And(new Message_ExecStatus(Message_Fail1));
        Assert.Equal(new Message_ExecStatus(Message_Fail1), aMutable);
    }
}

[Collection(DefaultMessengerCollection.Name)]
public class Message_AlgorithmTest
{
    [Fact]
    public void Statuses()
    {
        Message_Algorithm anAlgo = new();
        Assert.Same(Message.DefaultMessenger(), anAlgo.GetMessenger());
        anAlgo.SetStatus(Message_Done1);
        anAlgo.SetStatus(Message_Fail3);
        Assert.True(anAlgo.GetStatus().IsSet(Message_Done1));
        Assert.True(anAlgo.GetStatus().IsSet(Message_Fail3));
        Assert.False(anAlgo.GetStatus().IsSet(Message_Fail1));

        anAlgo.ChangeStatus().Clear(Message_Done1);
        Assert.False(anAlgo.GetStatus().IsDone());

        anAlgo.SetStatus(Message_Warn1, 5);
        anAlgo.SetStatus(Message_Warn1, 2);
        anAlgo.SetStatus(Message_Warn1, 5);
        NCollection_PackedMapOfInteger? aNumbers = anAlgo.GetMessageNumbers(Message_Warn1);
        Assert.NotNull(aNumbers);
        Assert.Equal(2, aNumbers.Extent());
        Assert.True(aNumbers.Contains(2) && aNumbers.Contains(5));
        Assert.Null(anAlgo.GetMessageNumbers(Message_Warn2));
        Assert.Null(anAlgo.GetMessageStrings(Message_Warn1));

        anAlgo.SetStatus(Message_Alarm2, "a");
        anAlgo.SetStatus(Message_Alarm2, "b");
        anAlgo.SetStatus(Message_Alarm2, "a"); // no repetition by default
        anAlgo.SetStatus(Message_Alarm2, "a", false);
        anAlgo.SetStatus(Message_Alarm3, (string?)null);
        Assert.Equal(["a", "b", "a"], anAlgo.GetMessageStrings(Message_Alarm2)!.ToArray());
        Assert.True(anAlgo.GetStatus().IsSet(Message_Alarm3));
        Assert.Null(anAlgo.GetMessageStrings(Message_Alarm3));

        // Message_None sets nothing and stores nothing
        anAlgo.SetStatus(Message_None, 1);
        Assert.Null(anAlgo.GetMessageNumbers(Message_None));

        anAlgo.ClearStatus();
        Assert.Equal(new Message_ExecStatus(), anAlgo.GetStatus());
        Assert.Null(anAlgo.GetMessageNumbers(Message_Warn1));
        Assert.Null(anAlgo.GetMessageStrings(Message_Alarm2));
    }

    [Fact]
    public void AddStatus_CopiesFilteredStatusesAndParameters()
    {
        Message_Algorithm anOther = new();
        anOther.SetStatus(Message_Fail1, 7);
        anOther.SetStatus(Message_Fail1, 8);
        anOther.SetStatus(Message_Warn4, "x");
        anOther.SetStatus(Message_Warn4, "y");
        anOther.SetStatus(Message_Warn4, "z");
        anOther.SetStatus(Message_Done2);

        Message_Algorithm anAlgo = new();
        anAlgo.SetStatus(Message_Fail1, 1);
        anAlgo.AddStatus(new Message_ExecStatus(Message_Fail1).With(Message_Warn4), anOther);
        Assert.False(anAlgo.GetStatus().IsSet(Message_Done2));
        Assert.True(anAlgo.GetStatus().IsSet(Message_Warn4));
        NCollection_PackedMapOfInteger aNumbers = anAlgo.GetMessageNumbers(Message_Fail1)!;
        Assert.Equal(3, aNumbers.Extent());
        // OCCT: AddStatus copies the strings but the last
        Assert.Equal(["x", "y"], anAlgo.GetMessageStrings(Message_Warn4)!.ToArray());

        Message_Algorithm anAll = new();
        anAll.AddStatus(anOther);
        Assert.True(anAll.GetStatus().IsSet(Message_Done2));
        Assert.Equal(2, anAll.GetMessageNumbers(Message_Fail1)!.Extent());
    }

    [Fact]
    public void PrepareReport()
    {
        NCollection_PackedMapOfInteger aMap = new();
        aMap.Add(3);
        Assert.Equal("3", Message_Algorithm.PrepareReport(aMap, 20));
        for (int i = 1; i <= 5; ++i)
        {
            aMap.Add(i);
        }
        string aReport = Message_Algorithm.PrepareReport(aMap, 2);
        Assert.EndsWith(" ... (total 5)", aReport);
        Assert.Equal(2, aReport.Substring(0, aReport.IndexOf(" ...", StringComparison.Ordinal)).Split(' ').Length);

        NCollection_Sequence<string> aSeq = new();
        aSeq.Append("a");
        aSeq.Append("b");
        aSeq.Append("c");
        Assert.Equal("'a', 'b', 'c'", Message_Algorithm.PrepareReport(aSeq, 20));
        Assert.Equal("'a', 'b' ... (total 3) ", Message_Algorithm.PrepareReport(aSeq, 2));
    }

    [Fact]
    public void SendMessages_ThroughMessenger()
    {
        RecordingPrinter aPrinter = new();
        TestAlgorithm anAlgo = new();
        anAlgo.SetMessenger(new Message_Messenger(aPrinter));

        anAlgo.SetStatus(Message_Done1); // Done statuses are not sent by SendMessages
        anAlgo.SetStatus(Message_Warn2, 4);
        anAlgo.SetStatus(Message_Warn2, 9);
        anAlgo.SetStatus(Message_Fail3, "edge");
        Message_Msg aCustom = new();
        aCustom.Set("custom %s and %d");
        aCustom.Arg("text");
        anAlgo.SetStatus(Message_Alarm1, aCustom);
        Message_MsgFile.AddMsg("TestAlgorithm.Fail3", "Failed on %s");
        anAlgo.SendMessages(Message_Alarm);

        Assert.Equal(3, aPrinter.Messages.Count);
        Assert.All(aPrinter.Messages, theMessage => Assert.Equal(Message_Alarm, theMessage.Gravity));
        // The Warn message isn't in the table: OCCT's surrogate text, under the last class tried (Standard_Transient)
        Assert.Equal("Unknown message invoked with the keyword Standard_Transient.Warn2", aPrinter.Messages[0].Text);
        Assert.Equal("custom text and UNKNOWN", aPrinter.Messages[1].Text);
        Assert.Equal("Failed on 'edge'", aPrinter.Messages[2].Text);

        // A message for a base class is found from the subclass
        Message_MsgFile.AddMsg("Message_Algorithm.Warn2", "Bad items: %s");
        aPrinter.Messages.Clear();
        anAlgo.SendStatusMessages(new Message_ExecStatus(Message_Warn2), Message_Info);
        Assert.Equal([("Bad items: 4 9", Message_Info)], aPrinter.Messages);

        // SetMessenger(null) restores the default messenger
        anAlgo.SetMessenger(null);
        Assert.Same(Message.DefaultMessenger(), anAlgo.GetMessenger());
    }

    /// <summary>
    /// OCCT: sending a stored custom message twice garbles it, because Get() fills the unfilled placeholders in place
    /// and remembers them.
    /// </summary>
    [Fact]
    public void SendMessages_CustomMessageTwice_IsGarbledAsInOcct()
    {
        RecordingPrinter aPrinter = new();
        Message_Algorithm anAlgo = new();
        anAlgo.SetMessenger(new Message_Messenger(aPrinter));
        Message_Msg aCustom = new();
        aCustom.Set("a %s b");
        anAlgo.SetStatus(Message_Fail1, aCustom);
        anAlgo.SendMessages();
        anAlgo.SendMessages();
        Assert.Equal("a UNKNOWN b", aPrinter.Messages[0].Text);
        Assert.Equal("a UNKNOWNKNOWN b", aPrinter.Messages[1].Text);
        // the message given to SetStatus was copied
        Assert.Equal("a %s b", aCustom.Value());
    }
}

public class Message_MsgTest
{
    [Fact]
    public void ArgFillsPlaceholdersInOrderOfType()
    {
        Message_Msg aMsg = new();
        aMsg.Set("int %d, real %f, string %s, hex %x, again %5.2f");
        Assert.False(aMsg.IsEdited());
        aMsg.Arg("str").Arg(42).Arg(1.5).Arg(255).Arg(3.14159);
        Assert.True(aMsg.IsEdited());
        Assert.Equal("int 42, real 1.500000, string str, hex ff, again  3.14", aMsg.Get());
        Assert.Equal("int %d, real %f, string %s, hex %x, again %5.2f", aMsg.Original());

        // No placeholder of the type left: nothing changes
        aMsg.Arg(1);
        Assert.Equal("int 42, real 1.500000, string str, hex ff, again  3.14", aMsg.Value());
    }

    [Fact]
    public void FormatsAsSprintf()
    {
        Assert.Equal("|  abc|", fill("|%5s|", "abc"));
        Assert.Equal("|abc  |", fill("|%-5s|", "abc"));
        Assert.Equal("|ab|", fill("|%.2s|", "abc"));
        Assert.Equal("|-0042|", fill("|%05d|", -42));
        Assert.Equal("|+7|", fill("|%+d|", 7));
        Assert.Equal("|0x1f|", fill("|%#x|", 31));
        Assert.Equal("|1.500000e+00|", fill("|%e|", 1.5));
        Assert.Equal("|1.2E-05|", fill("|%.1E|", 1.2e-5));
        Assert.Equal("|0.1|", fill("|%g|", 0.1));
        Assert.Equal("|1e+20|", fill("|%g|", 1e20));
        Assert.Equal("|12|", fill("|%ld|", 12));
    }

    [Fact]
    public void ExtendedArgIgnoresWidth()
    {
        Message_Msg aMsg = new();
        aMsg.Set("[%10s]");
        aMsg.ArgExtended("x");
        Assert.Equal("[x]", aMsg.Get());
    }

    [Fact]
    public void Get_ReplacesUnfilledByUnknown()
    {
        Message_Msg aMsg = new();
        aMsg.Set("%s and %d and %s");
        aMsg.Arg("first");
        Assert.Equal("first and UNKNOWN and UNKNOWN", aMsg.Get());
    }

    [Fact]
    public void PercentPercent()
    {
        Message_Msg aMsg = new();
        aMsg.Set("100%%");
        Assert.Equal("100%", aMsg.Get());
    }

    /// <summary>OCCT: the character after "%%" is read as the start of a format, so "%% d" becomes a placeholder.</summary>
    [Fact]
    public void PercentPercent_FollowedByConversion_IsAPlaceholderAsInOcct()
    {
        Message_Msg aMsg = new();
        aMsg.Set("100%% done");
        Assert.Equal("100% done", aMsg.Value());
        aMsg.Arg(7);
        Assert.Equal("100 7one", aMsg.Get());
    }

    [Fact]
    public void CopyIsIndependent()
    {
        Message_Msg aMsg = new();
        aMsg.Set("%s-%s");
        aMsg.Arg("a");
        Message_Msg aCopy = new(aMsg);
        aCopy.Arg("b");
        Assert.Equal("a-UNKNOWN", aMsg.Get());
        Assert.Equal("a-b", aCopy.Get());
    }

    [Fact]
    public void MsgFile_UnknownKeyword()
    {
        string aKey = "OcctNet.Tests.NoSuchKeyword." + Guid.NewGuid().ToString("N");
        Assert.False(Message_MsgFile.HasMsg(aKey));
        Message_Msg aMsg = new(aKey);
        Assert.Equal("Unknown message invoked with the keyword " + aKey, aMsg.Get());
        // OCCT stores the surrogate under the keyword
        Assert.True(Message_MsgFile.HasMsg(aKey));
        Assert.True(Message_MsgFile.AddMsg(aKey, "known %d"));
        Assert.Equal("known 3", new Message_Msg(aKey).Arg(3).Get());
    }

    private static string fill(string theFormat, object theValue)
    {
        Message_Msg aMsg = new();
        aMsg.Set(theFormat);
        switch (theValue)
        {
            case string aString:
                aMsg.Arg(aString);
                break;
            case int anInt:
                aMsg.Arg(anInt);
                break;
            default:
                aMsg.Arg((double)theValue);
                break;
        }
        return aMsg.Get();
    }
}
