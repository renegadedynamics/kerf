// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// The <c>.brep</c> files of OCCT's data (data/occ) through both formats: the port reads each file (text), writes it
/// in the binary format (BinTools), reads that back and writes the text again, which must be the text it wrote from
/// the file, word for word, with reals equal to the last bits (binary reading normalises directions and axes again).
/// Byte-for-byte agreement of the text with native OCCT is checked by the oracle tests.
/// </summary>
public class BRepToolsCorpusTests
{
    private static string[] Words(string theText) => theText.Split([' ', '\n'], StringSplitOptions.RemoveEmptyEntries);

    private static bool SameWord(string theA, string theB, ref int theNbIdentical)
    {
        if (theA == theB)
        {
            theNbIdentical++;
            return true;
        }

        if (!double.TryParse(theA, NumberStyles.Float, CultureInfo.InvariantCulture, out double a)
            || !double.TryParse(theB, NumberStyles.Float, CultureInfo.InvariantCulture, out double b))
        {
            return false;
        }

        // within four units of the last place, or of 1e-15 for the near-zero terms of rotation matrices and directions
        return Math.Abs(a - b) <= 4 * (Math.BitIncrement(Math.Abs(a)) - Math.Abs(a)) || Math.Abs(a - b) <= 1e-15;
    }

    [Fact]
    public void Corpus_shapes_survive_the_binary_format()
    {
        int aNbFiles = 0, aNbWords = 0, aNbIdentical = 0;
        foreach (string aFile in Directory.GetFiles(GeomTools_CorpusTests.DataDirectory, "*.brep").Order(StringComparer.Ordinal))
        {
            Assert.True(BRepTools.Read(out TopoDS_Shape aShape, aFile), aFile);
            string aText = BRepToolsIOTests.Write(aShape);

            var aBinary = new MemoryStream();
            BinTools.Write(aShape, aBinary);
            aBinary.Position = 0;
            BinTools.Read(out TopoDS_Shape aFromBinary, aBinary);
            Assert.True(aFromBinary.TShape()!.IsFrozen());
            string aText2 = BRepToolsIOTests.Write(aFromBinary);

            string[] aWords = Words(aText), aWords2 = Words(aText2);
            Assert.True(aWords.Length == aWords2.Length, aFile);
            for (int i = 0; i < aWords.Length; i++)
            {
                Assert.True(SameWord(aWords[i], aWords2[i], ref aNbIdentical), $"{Path.GetFileName(aFile)}, word {i}: {aWords[i]} {aWords2[i]}");
            }

            aNbWords += aWords.Length;
            aNbFiles++;
        }

        Assert.Equal(37, aNbFiles);
        // most words are identical; the others are reals within four units of the last place
        Assert.True(aNbIdentical > aNbWords * 0.95, $"{aNbIdentical} of {aNbWords}");
    }

    [Fact]
    public void Corpus_files_read_from_streams_and_strings_alike()
    {
        string aFile = Path.Combine(GeomTools_CorpusTests.DataDirectory, "Ball.brep");
        string aText = File.ReadAllText(aFile, Encoding.Latin1);
        Assert.True(BRepTools.Read(out TopoDS_Shape aFromFile, aFile));
        TopoDS_Shape aFromString = BRepToolsIOTests.Read(aText);
        BRepTools.Read(out TopoDS_Shape aFromStream, new StreamReader(new MemoryStream(Encoding.Latin1.GetBytes(aText)), Encoding.Latin1));
        string aWritten = BRepToolsIOTests.Write(aFromFile);
        Assert.Equal(aWritten, BRepToolsIOTests.Write(aFromString));
        Assert.Equal(aWritten, BRepToolsIOTests.Write(aFromStream));
    }
}
