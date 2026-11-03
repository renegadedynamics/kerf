// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of Message_Algorithm, Message_ExecStatus and Message_Msg (no GTests in OCCT).

using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Engine.Foundation.Status;
using static Kerf.Engine.Foundation.StatusType;

namespace OcctNet.Foundation.Tests;

/// <summary>An algorithm subclass, for the messages looked up by class name.</summary>
internal sealed class TestAlgorithm : global::Kerf.Engine.Foundation.Algorithm;

public class Message_ExecStatusTest
{
    [Fact]
    public void SetIsSetClear()
    {
        global::Kerf.Engine.Foundation.ExecStatus aStatus = new();
        Assert.False(aStatus.IsDone || aStatus.IsWarn || aStatus.IsAlarm || aStatus.IsFail);
        aStatus.Set(Done1);
        aStatus.Set(Warn32);
        aStatus.Set(Fail5);
        Assert.True(aStatus.IsSet(Done1));
        Assert.True(aStatus.IsSet(Warn32));
        Assert.True(aStatus.IsSet(Fail5));
        Assert.False(aStatus.IsSet(Fail4));
        Assert.False(aStatus.IsSet(Alarm1));
        Assert.False(aStatus.IsSet(None));
        Assert.True(aStatus.IsDone && aStatus.IsWarn && aStatus.IsFail);
        Assert.False(aStatus.IsAlarm);
        aStatus.Clear(Fail5);
        Assert.False(aStatus.IsFail);
        aStatus.Clear();
        Assert.Equal(new global::Kerf.Engine.Foundation.ExecStatus(), aStatus);
    }

    [Fact]
    public void Indices()
    {
        Assert.Equal(1, global::Kerf.Engine.Foundation.ExecStatus.StatusIndex(Done1));
        Assert.Equal(33, global::Kerf.Engine.Foundation.ExecStatus.StatusIndex(Warn1));
        Assert.Equal(64, global::Kerf.Engine.Foundation.ExecStatus.StatusIndex(Warn32));
        Assert.Equal(65, global::Kerf.Engine.Foundation.ExecStatus.StatusIndex(Alarm1));
        Assert.Equal(128, global::Kerf.Engine.Foundation.ExecStatus.StatusIndex(Fail32));
        Assert.Equal(0, global::Kerf.Engine.Foundation.ExecStatus.StatusIndex(None));
        Assert.Equal(7, global::Kerf.Engine.Foundation.ExecStatus.LocalStatusIndex(Fail7));
        Assert.Equal(Kerf.Engine.Foundation.StatusType.Alarm, global::Kerf.Engine.Foundation.ExecStatus.TypeOfStatus(Alarm3));
        for (int i = global::Kerf.Engine.Foundation.ExecStatus.FirstStatus; i < global::Kerf.Engine.Foundation.ExecStatus.LastStatus; ++i)
        {
            Assert.Equal(i, global::Kerf.Engine.Foundation.ExecStatus.StatusIndex(global::Kerf.Engine.Foundation.ExecStatus.StatusByIndex(i)));
        }
        Assert.Equal(None, global::Kerf.Engine.Foundation.ExecStatus.StatusByIndex(global::Kerf.Engine.Foundation.ExecStatus.LastStatus));
    }

    [Fact]
    public void GroupOperationsAndTwins()
    {
        global::Kerf.Engine.Foundation.ExecStatus aStatus = new(Warn2);
        global::Kerf.Engine.Foundation.ExecStatus anOther = new global::Kerf.Engine.Foundation.ExecStatus(Warn3).With(Fail1);
        global::Kerf.Engine.Foundation.ExecStatus aUnion = aStatus | anOther;
        Assert.True(aUnion.IsSet(Warn2) && aUnion.IsSet(Warn3) && aUnion.IsSet(Fail1));
        Assert.False(aStatus.IsSet(Warn3)); // the twins don't change the original
        global::Kerf.Engine.Foundation.ExecStatus aCommon = aUnion & new global::Kerf.Engine.Foundation.ExecStatus(Warn3);
        Assert.Equal(new global::Kerf.Engine.Foundation.ExecStatus(Warn3), aCommon);
        Assert.Equal(aUnion.Without(Fail1), aStatus.Added(new global::Kerf.Engine.Foundation.ExecStatus(Warn3)));
        Assert.True(aStatus.WithAllAlarm().IsSet(Alarm17));
        Assert.False(aStatus.WithAllWarn().WithoutAllWarn().IsWarn);
        Assert.Equal(new global::Kerf.Engine.Foundation.ExecStatus(), aUnion.Cleared());

        global::Kerf.Engine.Foundation.ExecStatus aMutable = new();
        aMutable.SetAllFail();
        Assert.True(aMutable.IsSet(Fail32));
        aMutable.ClearAllFail();
        Assert.False(aMutable.IsFail);
        aMutable.Add(aUnion);
        aMutable.And(new global::Kerf.Engine.Foundation.ExecStatus(Fail1));
        Assert.Equal(new global::Kerf.Engine.Foundation.ExecStatus(Fail1), aMutable);
    }
}

[Collection(DefaultMessengerCollection.Name)]
public class Message_AlgorithmTest
{
    [Fact]
    public void Statuses()
    {
        global::Kerf.Engine.Foundation.Algorithm anAlgo = new();
        Assert.Same(global::Kerf.Engine.Foundation.Message.DefaultMessenger(), anAlgo.GetMessenger());
        anAlgo.SetStatus(Done1);
        anAlgo.SetStatus(Fail3);
        Assert.True(anAlgo.GetStatus().IsSet(Done1));
        Assert.True(anAlgo.GetStatus().IsSet(Fail3));
        Assert.False(anAlgo.GetStatus().IsSet(Fail1));

        anAlgo.ChangeStatus().Clear(Done1);
        Assert.False(anAlgo.GetStatus().IsDone);

        anAlgo.SetStatus(Warn1, 5);
        anAlgo.SetStatus(Warn1, 2);
        anAlgo.SetStatus(Warn1, 5);
        NCollection_PackedMapOfInteger? aNumbers = anAlgo.GetMessageNumbers(Warn1);
        Assert.NotNull(aNumbers);
        Assert.Equal(2, aNumbers.Extent());
        Assert.True(aNumbers.Contains(2) && aNumbers.Contains(5));
        Assert.Null(anAlgo.GetMessageNumbers(Warn2));
        Assert.Null(anAlgo.GetMessageStrings(Warn1));

        anAlgo.SetStatus(Alarm2, "a");
        anAlgo.SetStatus(Alarm2, "b");
        anAlgo.SetStatus(Alarm2, "a"); // no repetition by default
        anAlgo.SetStatus(Alarm2, "a", false);
        anAlgo.SetStatus(Alarm3, (string?)null);
        Assert.Equal(["a", "b", "a"], anAlgo.GetMessageStrings(Alarm2)!.ToArray());
        Assert.True(anAlgo.GetStatus().IsSet(Alarm3));
        Assert.Null(anAlgo.GetMessageStrings(Alarm3));

        // Message_None sets nothing and stores nothing
        anAlgo.SetStatus(None, 1);
        Assert.Null(anAlgo.GetMessageNumbers(None));

        anAlgo.ClearStatus();
        Assert.Equal(new global::Kerf.Engine.Foundation.ExecStatus(), anAlgo.GetStatus());
        Assert.Null(anAlgo.GetMessageNumbers(Warn1));
        Assert.Null(anAlgo.GetMessageStrings(Alarm2));
    }

    [Fact]
    public void AddStatus_CopiesFilteredStatusesAndParameters()
    {
        global::Kerf.Engine.Foundation.Algorithm anOther = new();
        anOther.SetStatus(Fail1, 7);
        anOther.SetStatus(Fail1, 8);
        anOther.SetStatus(Warn4, "x");
        anOther.SetStatus(Warn4, "y");
        anOther.SetStatus(Warn4, "z");
        anOther.SetStatus(Done2);

        global::Kerf.Engine.Foundation.Algorithm anAlgo = new();
        anAlgo.SetStatus(Fail1, 1);
        anAlgo.AddStatus(new global::Kerf.Engine.Foundation.ExecStatus(Fail1).With(Warn4), anOther);
        Assert.False(anAlgo.GetStatus().IsSet(Done2));
        Assert.True(anAlgo.GetStatus().IsSet(Warn4));
        NCollection_PackedMapOfInteger aNumbers = anAlgo.GetMessageNumbers(Fail1)!;
        Assert.Equal(3, aNumbers.Extent());
        // OCCT: AddStatus copies the strings but the last
        Assert.Equal(["x", "y"], anAlgo.GetMessageStrings(Warn4)!.ToArray());

        global::Kerf.Engine.Foundation.Algorithm anAll = new();
        anAll.AddStatus(anOther);
        Assert.True(anAll.GetStatus().IsSet(Done2));
        Assert.Equal(2, anAll.GetMessageNumbers(Fail1)!.Extent());
    }

    [Fact]
    public void PrepareReport()
    {
        NCollection_PackedMapOfInteger aMap = new();
        aMap.Add(3);
        Assert.Equal("3", global::Kerf.Engine.Foundation.Algorithm.PrepareReport(aMap, 20));
        for (int i = 1; i <= 5; ++i)
        {
            aMap.Add(i);
        }
        string aReport = global::Kerf.Engine.Foundation.Algorithm.PrepareReport(aMap, 2);
        Assert.EndsWith(" ... (total 5)", aReport);
        Assert.Equal(2, aReport.Substring(0, aReport.IndexOf(" ...", StringComparison.Ordinal)).Split(' ').Length);

        NCollection_Sequence<string> aSeq = new();
        aSeq.Append("a");
        aSeq.Append("b");
        aSeq.Append("c");
        Assert.Equal("'a', 'b', 'c'", global::Kerf.Engine.Foundation.Algorithm.PrepareReport(aSeq, 20));
        Assert.Equal("'a', 'b' ... (total 3) ", global::Kerf.Engine.Foundation.Algorithm.PrepareReport(aSeq, 2));
    }

    [Fact]
    public void SendMessages_ThroughMessenger()
    {
        RecordingPrinter aPrinter = new();
        TestAlgorithm anAlgo = new();
        anAlgo.SetMessenger(new global::Kerf.Engine.Foundation.Messenger(aPrinter));

        anAlgo.SetStatus(Done1); // Done statuses are not sent by SendMessages
        anAlgo.SetStatus(Warn2, 4);
        anAlgo.SetStatus(Warn2, 9);
        anAlgo.SetStatus(Fail3, "edge");
        global::Kerf.Engine.Foundation.Msg aCustom = new();
        aCustom.Set("custom %s and %d");
        aCustom.Arg("text");
        anAlgo.SetStatus(Alarm1, aCustom);
        global::Kerf.Engine.Foundation.MsgFile.AddMsg("TestAlgorithm.Fail3", "Failed on %s");
        anAlgo.SendMessages(Kerf.Engine.Foundation.Gravity.Alarm);

        Assert.Equal(3, aPrinter.Messages.Count);
        Assert.All(aPrinter.Messages, theMessage => Assert.Equal(Kerf.Engine.Foundation.Gravity.Alarm, theMessage.Gravity));
        // The Warn message isn't in the table: OCCT's surrogate text, under the last class tried (Standard_Transient)
        Assert.Equal("Unknown message invoked with the keyword Standard_Transient.Warn2", aPrinter.Messages[0].Text);
        Assert.Equal("custom text and UNKNOWN", aPrinter.Messages[1].Text);
        Assert.Equal("Failed on 'edge'", aPrinter.Messages[2].Text);

        // A message for a base class is found from the subclass
        global::Kerf.Engine.Foundation.MsgFile.AddMsg("Message_Algorithm.Warn2", "Bad items: %s");
        aPrinter.Messages.Clear();
        anAlgo.SendStatusMessages(new global::Kerf.Engine.Foundation.ExecStatus(Warn2), Info);
        Assert.Equal([("Bad items: 4 9", Info)], aPrinter.Messages);

        // SetMessenger(null) restores the default messenger
        anAlgo.SetMessenger(null);
        Assert.Same(global::Kerf.Engine.Foundation.Message.DefaultMessenger(), anAlgo.GetMessenger());
    }

    /// <summary>
    /// OCCT: sending a stored custom message twice garbles it, because Get() fills the unfilled placeholders in place
    /// and remembers them.
    /// </summary>
    [Fact]
    public void SendMessages_CustomMessageTwice_IsGarbledAsInOcct()
    {
        RecordingPrinter aPrinter = new();
        global::Kerf.Engine.Foundation.Algorithm anAlgo = new();
        anAlgo.SetMessenger(new global::Kerf.Engine.Foundation.Messenger(aPrinter));
        global::Kerf.Engine.Foundation.Msg aCustom = new();
        aCustom.Set("a %s b");
        anAlgo.SetStatus(Fail1, aCustom);
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
        global::Kerf.Engine.Foundation.Msg aMsg = new();
        aMsg.Set("int %d, real %f, string %s, hex %x, again %5.2f");
        Assert.False(aMsg.IsEdited);
        aMsg.Arg("str").Arg(42).Arg(1.5).Arg(255).Arg(3.14159);
        Assert.True(aMsg.IsEdited);
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
        global::Kerf.Engine.Foundation.Msg aMsg = new();
        aMsg.Set("[%10s]");
        aMsg.ArgExtended("x");
        Assert.Equal("[x]", aMsg.Get());
    }

    [Fact]
    public void Get_ReplacesUnfilledByUnknown()
    {
        global::Kerf.Engine.Foundation.Msg aMsg = new();
        aMsg.Set("%s and %d and %s");
        aMsg.Arg("first");
        Assert.Equal("first and UNKNOWN and UNKNOWN", aMsg.Get());
    }

    [Fact]
    public void PercentPercent()
    {
        global::Kerf.Engine.Foundation.Msg aMsg = new();
        aMsg.Set("100%%");
        Assert.Equal("100%", aMsg.Get());
    }

    /// <summary>OCCT: the character after "%%" is read as the start of a format, so "%% d" becomes a placeholder.</summary>
    [Fact]
    public void PercentPercent_FollowedByConversion_IsAPlaceholderAsInOcct()
    {
        global::Kerf.Engine.Foundation.Msg aMsg = new();
        aMsg.Set("100%% done");
        Assert.Equal("100% done", aMsg.Value());
        aMsg.Arg(7);
        Assert.Equal("100 7one", aMsg.Get());
    }

    [Fact]
    public void CopyIsIndependent()
    {
        global::Kerf.Engine.Foundation.Msg aMsg = new();
        aMsg.Set("%s-%s");
        aMsg.Arg("a");
        global::Kerf.Engine.Foundation.Msg aCopy = new(aMsg);
        aCopy.Arg("b");
        Assert.Equal("a-UNKNOWN", aMsg.Get());
        Assert.Equal("a-b", aCopy.Get());
    }

    [Fact]
    public void MsgFile_UnknownKeyword()
    {
        string aKey = "OcctNet.Tests.NoSuchKeyword." + Guid.NewGuid().ToString("N");
        Assert.False(global::Kerf.Engine.Foundation.MsgFile.HasMsg(aKey));
        global::Kerf.Engine.Foundation.Msg aMsg = new(aKey);
        Assert.Equal("Unknown message invoked with the keyword " + aKey, aMsg.Get());
        // OCCT stores the surrogate under the keyword
        Assert.True(global::Kerf.Engine.Foundation.MsgFile.HasMsg(aKey));
        Assert.True(global::Kerf.Engine.Foundation.MsgFile.AddMsg(aKey, "known %d"));
        Assert.Equal("known 3", new global::Kerf.Engine.Foundation.Msg(aKey).Arg(3).Get());
    }

    private static string fill(string theFormat, object theValue)
    {
        global::Kerf.Engine.Foundation.Msg aMsg = new();
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
