// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;
using System.IO.Compression;
using System.Text;

namespace OcctNet.DataExchange.Tests.Interface;

/// <summary>
/// TKXSBase's foundation against native OCCT's recording (tools/xsbaserec, unit 8A1): the formatting of reals
/// (Interface_FloatWriter, TCollection_HAsciiString), the line buffer of the file writers, the static parameters
/// (Standards, edition, the library of types), Interface_MSG's utilities, the categories, bit maps and entity lists.
/// The port runs the recorder's cases, from the same 64-bit LCG, and writes the same lines; they must be equal, but
/// for the order of an enumeration's alphabetic values in a definition, which the port keeps in declaration order
/// where OCCT's map hashes them (decision 1181): both sides' lists are sorted before comparing.
/// </summary>
[Collection(ProcessWideMessages.Name)]
public class XSBaseNativeTests
{
    private ulong mySeed = 0x8A1UL;

    private uint Next()
    {
        mySeed = unchecked(mySeed * 6364136223846793005UL + 1442695040888963407UL);
        return (uint)(mySeed >> 33);
    }

    private int NextInt(int theMax) => (int)(Next() % (uint)theMax);

    private double NextReal()
    {
        double aMant = 1.0 + NextInt(9000000) / 1000000.0;
        int anExp = NextInt(29) - 14;
        double aVal = aMant * Math.Pow(10.0, anExp);
        if (NextInt(2) == 1)
        {
            aVal = -aVal;
        }
        return aVal;
    }

    private string Text(int theLength)
    {
        const string aChars = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJ0123456789_,.()'#=";
        var aText = new StringBuilder();
        for (int i = 0; i < theLength; i++)
        {
            aText.Append(aChars[NextInt(aChars.Length)]);
        }
        return aText.ToString();
    }

    private static string B(bool theValue) => theValue ? "1" : "0";

    private static string G(double theValue) => StreamUtilities.FormatReal(theValue, 6);

    private void FloatWriters(List<string> theOut)
    {
        double[] aFixed = [0.0, 1.0, -1.0, 0.1, 0.09999999, 999.999, 1000.0, 1.0e-7, 1.0e-300, 1.0e300,
                           123.456, -0.5, 3.14159265358979, 1.5e-5, 2.5, 1.0e20, 100.0, 0.25, 1.0e-3, -1000.0];
        var aValues = new List<double>(aFixed);
        for (int i = 0; i < 3000; i++)
        {
            aValues.Add(NextReal());
        }
        Interface_FloatWriter[] aWriters = [new(), new(12), new(6), new()];
        aWriters[3].SetFormat("%.10g");
        for (int w = 0; w < 4; w++)
        {
            aWriters[w].Options(out bool zs, out bool rg, out _, out _);
            theOut.Add($"FW {w} options {B(zs)} {B(rg)} {aWriters[w].MainFormat()} {aWriters[w].FormatForRange()}");
            foreach (double aVal in aValues)
            {
                int aLen = aWriters[w].Write(aVal, out string aText);
                theOut.Add($"FW {w} {aLen} {aText}");
            }
        }
        for (int i = 0; i < 400; i++)
        {
            double aVal = i < 20 ? aFixed[i] : NextReal();
            theOut.Add("HS " + new TCollection_HAsciiString(aVal).ToCString());
        }
    }

    private void LineBuffers(List<string> theOut)
    {
        for (int aCase = 0; aCase < 60; aCase++)
        {
            int aSize = 20 + NextInt(60);
            var aBuf = new Interface_LineBuffer(aSize);
            theOut.Add($"LB {aCase} size {aSize}");
            for (int anOp = 0; anOp < 80; anOp++)
            {
                int aKind = NextInt(10);
                switch (aKind)
                {
                    case 0:
                    case 1:
                    case 2:
                    {
                        string aText = Text(1 + NextInt(15));
                        bool aCan = aBuf.CanGet(aText.Length);
                        string aLine = $"LB add {aText} can {B(aCan)}";
                        if (aCan || NextInt(3) == 0)
                        {
                            aBuf.Add(aText);
                        }
                        theOut.Add(aLine + $" len {aBuf.Length()}");
                        break;
                    }
                    case 3:
                        if (aBuf.CanGet(1))
                        {
                            aBuf.Add('x');
                            theOut.Add($"LB char len {aBuf.Length()}");
                        }
                        break;
                    case 4:
                    {
                        int anInit = NextInt(8);
                        try
                        {
                            aBuf.SetInitial(anInit);
                            theOut.Add($"LB init {anInit}");
                        }
                        catch (FailureException)
                        {
                            theOut.Add($"LB init {anInit} failed");
                        }
                        break;
                    }
                    case 5:
                        aBuf.SetKeep();
                        theOut.Add("LB keep");
                        break;
                    case 6:
                        aBuf.FreezeInitial();
                        theOut.Add("LB freeze");
                        break;
                    case 7:
                    case 8:
                    {
                        TCollection_HAsciiString aLine = aBuf.Moved();
                        theOut.Add($"LB moved [{aLine.ToCString()}] len {aBuf.Length()} content [{aBuf.Content()}]");
                        break;
                    }
                    default:
                        aBuf.Clear();
                        theOut.Add("LB clear");
                        break;
                }
            }
        }
    }

    private static void PrintStatic(Interface_StaticSet theSet, List<string> theOut, string theName)
    {
        Interface_Static? aStat = theSet.Static(theName);
        if (aStat is null)
        {
            theOut.Add($"ST {theName} absent");
            return;
        }
        theOut.Add($"ST {theName} family {theSet.CDef(theName, "family")} type {theSet.CDef(theName, "type")} label [{theSet.CDef(theName, "label")}] imin {theSet.CDef(theName, "imin")} imax {theSet.CDef(theName, "imax")} rmin {theSet.CDef(theName, "rmin")} rmax {theSet.CDef(theName, "rmax")} unit [{theSet.CDef(theName, "unit")}]");
        // OCCT's IDef reads uninitialised values for the enumeration's parts of a static that isn't one
        bool isEnum = theSet.CDef(theName, "type") == "enum";
        int aStart = isEnum ? theSet.IDef(theName, "estart") : 0;
        int aCount = isEnum ? theSet.IDef(theName, "ecount") : -2;
        if (isEnum)
        {
            theOut.Add($"ST {theName} estart {aStart} ecount {aCount} ematch {theSet.IDef(theName, "ematch")}");
        }
        for (int i = aStart - 1; i <= aStart + aCount; i++)
        {
            string aVal = theSet.CDef(theName, "enum " + i.ToString(CultureInfo.InvariantCulture));
            theOut.Add($"ST {theName} enum {i} [{aVal}] case {theSet.IDef(theName, "eval " + aVal)}");
        }
        theOut.Add($"ST {theName} value [{theSet.CVal(theName)}] ival {theSet.IVal(theName)} rval {G(theSet.RVal(theName))} set {B(theSet.IsSet(theName))} def [{aStat.Definition()}]");
        var aPrint = new StringWriter();
        aStat.PrintStatic(aPrint);
        theOut.Add($"ST {theName} print {aPrint.ToString().Replace('\n', '|')}");
    }

    // The ", alpha: " parts of a line (an enumeration's definition), their items sorted (OCCT lists them in its map's
    // order, the port in declaration order)
    private static string SortAlpha(string theText)
    {
        const string aKey = " , alpha: ";
        int aFrom = 0;
        while (true)
        {
            int aStart = theText.IndexOf(aKey, aFrom, StringComparison.Ordinal);
            if (aStart < 0)
            {
                return theText;
            }
            aStart += aKey.Length;
            int anEnd = theText.IndexOfAny(['|', ']'], aStart);
            if (anEnd < 0)
            {
                anEnd = theText.Length;
            }
            string[] anItems = theText[aStart..anEnd].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            Array.Sort(anItems, StringComparer.Ordinal);
            string aSorted = string.Join(" ", anItems) + " ";
            theText = theText[..aStart] + aSorted + theText[anEnd..];
            aFrom = aStart + aSorted.Length;
        }
    }

    private static void Statics(List<string> theOut)
    {
        var aSet = new Interface_StaticSet();
        aSet.Standards();
        var aNames = new List<string>();
        NCollection_HSequence<TCollection_HAsciiString> anItems = aSet.Items();
        for (int i = 1; i <= anItems.Length(); i++)
        {
            aNames.Add(anItems.Value(i).ToCString());
        }
        aNames.Sort(StringComparer.Ordinal);
        foreach (string aName in aNames)
        {
            PrintStatic(aSet, theOut, aName);
        }

        string[][] aSets =
        [
            ["read.precision.mode", "User"], ["read.precision.mode", "File"], ["read.precision.mode", "1"],
            ["read.precision.mode", "bogus"], ["read.precision.val", "0.05"], ["read.precision.val", "abc"],
            ["read.surfacecurve.mode", "-3"], ["read.surfacecurve.mode", "?"], ["read.surfacecurve.mode", "3DUse_Preferred"],
            ["write.precision.mode", "Max"], ["write.surfacecurve.mode", "Off"], ["read.maxprecision.val", "1e-7"],
            ["absent.param", "1"],
        ];
        foreach (string[] aPair in aSets)
        {
            bool anOk = aSet.SetCVal(aPair[0], aPair[1]);
            theOut.Add($"SET {aPair[0]} {aPair[1]} {B(anOk)} -> [{aSet.CVal(aPair[0])}] {aSet.IVal(aPair[0])} {G(aSet.RVal(aPair[0]))}");
        }
        foreach (int aVal in new[] { -4, -3, -1, 0, 1, 2, 3, 4, 5, 6 })
        {
            bool anOk = aSet.SetIVal("read.surfacecurve.mode", aVal);
            theOut.Add($"SETI read.surfacecurve.mode {aVal} {B(anOk)} -> [{aSet.CVal("read.surfacecurve.mode")}] {aSet.IVal("read.surfacecurve.mode")}");
        }
        foreach (double aVal in new[] { 0.001, 1.5, -2.0, 1.0e-9, 123456.789 })
        {
            bool anOk = aSet.SetRVal("read.precision.val", aVal);
            theOut.Add($"SETR read.precision.val {G(aVal)} {B(anOk)} -> [{aSet.CVal("read.precision.val")}] {G(aSet.RVal("read.precision.val"))}");
        }

        aSet.Init("XSTEP8A1", "t8a1.int", 'i', "5");
        aSet.Init("XSTEP8A1", "t8a1.int", '&', "imin 1");
        aSet.Init("XSTEP8A1", "t8a1.int", '&', "imax 10");
        aSet.Init("XSTEP8A1", "t8a1.real", 'r', "2.5");
        aSet.Init("XSTEP8A1", "t8a1.real", '&', "rmin 0.5");
        aSet.Init("XSTEP8A1", "t8a1.real", '&', "unit mm");
        aSet.Init("XSTEP8A1", "t8a1.text", 't', "hello");
        aSet.Init("XSTEP8A1", "t8a1.path", 'p', "dir/file");
        aSet.Init("XSTEP8A1", "t8a1.copy", '=', "read.surfacecurve.mode");
        aSet.Init("XSTEP8A1", "t8a1.enum", 'e', "");
        aSet.Init("XSTEP8A1", "t8a1.enum", '&', "enum 2");
        aSet.Init("XSTEP8A1", "t8a1.enum", '&', "eval One");
        aSet.Init("XSTEP8A1", "t8a1.enum", '&', "eval Two");
        aSet.Init("XSTEP8A1", "t8a1.enum", '&', "eval Three");
        theOut.Add($"INIT again {B(aSet.Init("XSTEP8A1", "t8a1.int", 'i', "7"))} bad {B(aSet.Init("XSTEP8A1", "t8a1.bad", 'z', ""))} badedit {B(aSet.Init("XSTEP8A1", "t8a1.int", '&', "zzz 1"))}");
        foreach (string aName in new[] { "t8a1.int", "t8a1.real", "t8a1.text", "t8a1.path", "t8a1.copy", "t8a1.enum" })
        {
            PrintStatic(aSet, theOut, aName);
        }
        string[][] aNewSets =
        [
            ["t8a1.int", "0"], ["t8a1.int", "10"], ["t8a1.int", "11"], ["t8a1.int", "3.5"], ["t8a1.real", "0.4"],
            ["t8a1.real", "7"], ["t8a1.enum", "Two"], ["t8a1.enum", "5"], ["t8a1.enum", "One"],
        ];
        foreach (string[] aPair in aNewSets)
        {
            bool anOk = aSet.SetCVal(aPair[0], aPair[1]);
            theOut.Add($"SET {aPair[0]} {aPair[1]} {B(anOk)} -> [{aSet.CVal(aPair[0])}] {aSet.IVal(aPair[0])} {G(aSet.RVal(aPair[0]))}");
        }
        theOut.Add($"ITEMS XSTEP8A1 {aSet.Items(0, "XSTEP8A1").Length()}");

        foreach (string aName in new[] { "Integer", "Real", "Text", "Transient", "Boolean", "Logical", "Unknown" })
        {
            MoniTool_TypedValue? aTV = MoniTool_TypedValue.FromLib(aName);
            if (aTV is null)
            {
                theOut.Add($"LIB {aName} null");
                continue;
            }
            var aPrint = new StringWriter();
            aTV.Print(aPrint);
            theOut.Add($"LIB {aName} {aPrint.ToString().Replace('\n', '|')}");
        }
        theOut.Add($"MSG XSTEP_1 [{MsgFile.Msg("XSTEP_1")}]");
    }

    private void Msgs(List<string> theOut)
    {
        foreach (int aVal in new[] { 0, 5, 9, 10, 99, 100, 12345, 999999999, 1000000000, -7, -1234 })
        {
            for (int aMax = 0; aMax <= 12; aMax += 3)
            {
                theOut.Add($"BL {aVal} {aMax} [{Interface_MSG.Blanks(aVal, aMax)}]");
            }
        }
        foreach (string aText in new[] { "", "a", "abcdef", "abcdefghijklmnop" })
        {
            for (int aMax = 0; aMax <= 12; aMax += 4)
            {
                theOut.Add($"BS [{aText}] {aMax} [{Interface_MSG.Blanks(aText, aMax)}]");
            }
        }
        for (int aCount = -1; aCount <= 80; aCount += 9)
        {
            theOut.Add($"BC {aCount} [{Interface_MSG.Blanks(aCount)}]");
        }
        for (int i = 0; i < 300; i++)
        {
            double aVal = i % 10 == 0 ? 0.0 : NextReal();
            int anOrd = NextInt(12);
            bool anUp = NextInt(2) == 1;
            double aRes = Interface_MSG.Intervalled(aVal, anOrd, anUp);
            theOut.Add("IV " + Sprintf("%.17g %d %d %.17g", aVal, anOrd, anUp ? 1 : 0, aRes));
        }
        string[] aDates = ["2026-11-02:10-20-30", "1999-01-01:00-00-00", "2026-11-02:10-20-31", "2026-11-03", "abc",
                           "12/7/2020 8h5", "2026-11-02:10-20-30"];
        foreach (string aD1 in aDates)
        {
            int yy = 0, mm = 0, dd = 0, hh = 0, mn = 0, ss = 0;
            bool anOk = Interface_MSG.NDate(aD1, ref yy, ref mm, ref dd, ref hh, ref mn, ref ss);
            var aLine = new StringBuilder($"ND [{aD1}] {B(anOk)} {yy} {mm} {dd} {hh} {mn} {ss}");
            foreach (string aD2 in aDates)
            {
                aLine.Append(' ').Append(Interface_MSG.CDate(aD1, aD2));
            }
            theOut.Add(aLine.ToString());
        }
        Interface_MSG.TDate(out string aDate, 2026, 11, 2, 9, 8, 7, "");
        theOut.Add($"TD [{aDate}]");
        Interface_MSG.TDate(out aDate, 2026, 11, 2, 9, 8, 7, "C:%4.4d-%2.2d-%2.2dT%2.2d:%2.2d:%2.2d");
        theOut.Add($"TD [{aDate}]");
        for (int i = -1; i <= 10; i++)
        {
            theOut.Add($"CAT {i} {Interface_Category.Name(i)}");
        }
        foreach (string aCat in new[] { "Shape", "Drawing", "Structure", "Description", "Auxiliary", "Professional", "FEA", "Kinematics", "Piping", "Other" })
        {
            theOut.Add($"CATN {aCat} {Interface_Category.Number(aCat)}");
        }
        theOut.Add($"CATS {Interface_Category.NbCategories()}");
    }

    private void BitMaps(List<string> theOut)
    {
        for (int aCase = 0; aCase < 20; aCase++)
        {
            int aNb = 1 + NextInt(100);
            var aMap = new Interface_BitMap(aNb, NextInt(3));
            theOut.Add($"BM {aCase} {aNb}");
            for (int anOp = 0; anOp < 120; anOp++)
            {
                int aKind = NextInt(8);
                int aNum = 1 + NextInt(aMap.Length());
                int aFlag = NextInt(aMap.NbFlags() + 1);
                switch (aKind)
                {
                    case 0:
                        aMap.SetTrue(aNum, aFlag);
                        break;
                    case 1:
                        aMap.SetFalse(aNum, aFlag);
                        break;
                    case 2:
                        theOut.Add($"BM ctrue {aNum} {aFlag} {B(aMap.CTrue(aNum, aFlag))}");
                        break;
                    case 3:
                        theOut.Add($"BM cfalse {aNum} {aFlag} {B(aMap.CFalse(aNum, aFlag))}");
                        break;
                    case 4:
                    {
                        string aName = "f" + NextInt(5).ToString(CultureInfo.InvariantCulture);
                        theOut.Add($"BM addflag {aName} {aMap.AddFlag(aName)} nb {aMap.NbFlags()}");
                        break;
                    }
                    case 5:
                        theOut.Add($"BM flagnumber {aMap.FlagNumber("f2")}");
                        break;
                    case 6:
                        aMap.SetValue(aNum, NextInt(2) == 1, aFlag);
                        break;
                    default:
                    {
                        var aBits = new StringBuilder();
                        for (int i = 1; i <= aMap.Length(); i++)
                        {
                            aBits.Append(aMap.Value(i, aFlag) ? '1' : '0');
                        }
                        theOut.Add($"BM bits {aFlag} {aBits}");
                        break;
                    }
                }
            }
        }
    }

    private void EntityLists(List<string> theOut)
    {
        for (int aCase = 0; aCase < 20; aCase++)
        {
            var aList = new Interface_EntityList();
            int aNb = 1 + NextInt(30);
            for (int i = 0; i < aNb; i++)
            {
                var anItem = new Interface_IntVal();
                anItem.CValue() = i + 1;
                if (NextInt(3) == 0)
                {
                    aList.Add(anItem);
                }
                else
                {
                    aList.Append(anItem);
                }
            }
            int aRemoves = NextInt(3);
            for (int r = 0; r < aRemoves && aList.NbEntities() > 1; r++)
            {
                aList.Remove(aList.NbEntities());
            }
            var aVals = new StringBuilder();
            for (int i = 1; i <= aList.NbEntities(); i++)
            {
                try
                {
                    int aValue = ((Interface_IntVal)aList.Value(i)!).Value();
                    aVals.Append(' ').Append(aValue);
                }
                catch (FailureException)
                {
                    aVals.Append(" !");
                }
            }
            theOut.Add($"EL {aCase} {aList.NbEntities()}:{aVals}");
        }
    }

    private static List<string> Recording()
    {
        using Stream aRes = typeof(XSBaseNativeTests).Assembly.GetManifestResourceStream("Interface.xsbaserec.txt.gz")
                            ?? throw new InvalidOperationException("Interface.xsbaserec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.Latin1);
        var aLines = new List<string>();
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            aLines.Add(aLine);
        }
        return aLines;
    }

    [Fact]
    public void The_port_writes_what_native_OCCT_recorded()
    {
        var aPort = new List<string>();
        FloatWriters(aPort);
        LineBuffers(aPort);
        Statics(aPort);
        Msgs(aPort);
        BitMaps(aPort);
        EntityLists(aPort);

        List<string> aNative = Recording();
        // the native definitions list an enumeration's alphabetic values in hash order: sorted as the port's
        for (int i = 0; i < aNative.Count; i++)
        {
            aNative[i] = SortAlpha(aNative[i]);
        }
        for (int i = 0; i < aPort.Count; i++)
        {
            aPort[i] = SortAlpha(aPort[i]);
        }
        int aCount = Math.Min(aPort.Count, aNative.Count);
        var aDiffs = new List<string>();
        for (int i = 0; i < aCount && aDiffs.Count < 20; i++)
        {
            if (aPort[i] != aNative[i])
            {
                aDiffs.Add($"line {i + 1}:\n native [{aNative[i]}]\n port   [{aPort[i]}]");
            }
        }
        Assert.True(aDiffs.Count == 0, string.Join("\n", aDiffs));
        Assert.Equal(aNative.Count, aPort.Count);
    }
}
