// Port of: src/FoundationClasses/TKernel/GTests/Resource_Manager_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Foundation.Resources;

namespace OcctNet.Foundation.Tests;

/// <summary>
/// Resource_Manager: OCCT's GTests, with the directories given to the constructor instead of the CSF_ environment
/// variables the port doesn't read (decision 1023), and the file syntax, the values, the user resources and the
/// compiled-in defaults.
/// </summary>
public sealed class Resource_ManagerTests : IDisposable
{
    private readonly string myTempBase = Path.Combine(Path.GetTempPath(), "occtnet_resource_manager_test_" + Guid.NewGuid().ToString("N"));

    public void Dispose()
    {
        if (Directory.Exists(myTempBase))
        {
            Directory.Delete(myTempBase, true);
        }
    }

    // Writes a minimal resource file containing one entry:  key : value
    private static void writeResourceFile(string theDir, string theFileName, string theKey, string theValue)
    {
        Directory.CreateDirectory(theDir);
        File.WriteAllText(Path.Combine(theDir, theFileName), theKey + " : " + theValue + "\n");
    }

    // Test OCC27849: Resource_Manager correctly reads resource files located in
    // directories whose names contain dots, spaces, or multiple path segments.
    [Fact]
    public void OCC27849_PathsWithSpecialChars()
    {
        const string aResourceName = "TestResource";
        const string aKey = "test.resource";
        const string aExpected = "ok";

        string[] aPaths = ["path", "path.with.dots", "path with spaces", "nested/dirs/path with spaces"];

        foreach (string aRelPath in aPaths)
        {
            string aDir = Path.Combine(myTempBase, aRelPath);
            writeResourceFile(aDir, aResourceName, aKey, aExpected);

            // OCCT points CSF_TestResourceDefaults to the directory
            var aManager = new Manager(aResourceName, aDir, null);

            Assert.True(aManager.Find(aKey), "Resource key not found in path: " + aDir);
            Assert.Equal(aExpected, aManager.Value(aKey));
        }
    }

    // Test OCC181 (case 1): Resource_Manager::Save() succeeds when the target user-defaults
    // directory already exists (flat path).
    [Fact]
    public void OCC181_SaveToExistingDirectory()
    {
        const string aFileName = "OCC181";
        string aSourceDir = Path.Combine(myTempBase, "source");
        string aSaveDir = Path.Combine(myTempBase, "save_flat");

        writeResourceFile(aSourceDir, aFileName, "test.key", "test_value");
        Directory.CreateDirectory(aSaveDir);

        var aManager = new Manager(aFileName, null, aSourceDir);

        // Redirect Save() to a different directory
        Assert.True(aManager.Save(aSaveDir));
        Assert.True(File.Exists(Path.Combine(aSaveDir, aFileName)), "Saved resource file not found in " + aSaveDir);
    }

    // Test OCC181 (case 2): Resource_Manager::Save() succeeds and creates intermediate
    // directories when the target user-defaults path does not yet exist (nested path).
    [Fact]
    public void OCC181_SaveToNestedNonExistentDirectory()
    {
        const string aFileName = "OCC181";
        string aSourceDir = Path.Combine(myTempBase, "source2");
        string aSaveDir = Path.Combine(myTempBase, "nested", "deep", "dir");

        writeResourceFile(aSourceDir, aFileName, "test.key", "test_value");

        var aManager = new Manager(aFileName, null, aSourceDir);

        Assert.True(aManager.Save(aSaveDir));
        Assert.True(File.Exists(Path.Combine(aSaveDir, aFileName)), "Saved resource file not found in nested path " + aSaveDir);
    }

    [Fact]
    public void Files_are_parsed_as_OCCT_parses_them()
    {
        string aDir = Path.Combine(myTempBase, "syntax");
        Directory.CreateDirectory(aDir);
        File.WriteAllText(Path.Combine(aDir, "Syntax"),
                          "! a comment : not a resource\r\n"
                          + "\r\n"
                          + "   \t\n"
                          + "  a.b\t :  value with spaces  \r\n"
                          + "c:d\n"
                          + "empty:\n"
                          + "escaped : \\  leading\n"
                          + "backslash : \\\\x\n"
                          + "no colon here\n"
                          + ": no name\n"
                          + "int : 42\n"
                          + "real : 1.5e-3\n"
                          + "notint : 4.5\n"
                          + "c : redefined\n"
                          + "last : no newline");

        var aManager = new Manager("Syntax", aDir, "");
        Assert.True(aManager.IsInitialized);
        Assert.Equal("value with spaces  ", aManager.Value("a.b"));
        Assert.Equal("redefined", aManager.Value("c"));
        Assert.Equal("", aManager.Value("empty"));
        Assert.Equal("  leading", aManager.Value("escaped"));
        Assert.Equal("\\x", aManager.Value("backslash"));
        Assert.Equal("no newline", aManager.Value("last"));
        Assert.False(aManager.Find("no colon here"));
        Assert.Equal(42, aManager.Integer("int"));
        Assert.Equal(1.5e-3, aManager.Real("real"));
        Assert.Equal(4.5, aManager.Real("notint"));
        Assert.ThrowsAny<TypeMismatchException>(() => aManager.Integer("notint"));
        Assert.ThrowsAny<TypeMismatchException>(() => aManager.Real("a.b"));
        Assert.ThrowsAny<NoSuchResourceException>(() => aManager.Value("missing"));
        Assert.True(aManager.Find("int", out string aValue));
        Assert.Equal("42", aValue);

        // a missing file: not initialized
        Assert.False(new Manager("Missing", aDir, null).IsInitialized);
    }

    [Fact]
    public void User_resources_override_and_are_saved_sorted()
    {
        var aManager = new Manager();
        Assert.False(aManager.Find("x"));
        aManager.SetResource("z", 3);
        aManager.SetResource("a", 0.5);
        aManager.SetResource("m", " spaced");
        Assert.Equal("3", aManager.Value("z"));
        Assert.Equal("0.5", aManager.Value("a"));
        Assert.Equal(3, aManager.GetMap(false).Extent());
        Assert.Equal(0, aManager.GetMap().Extent());

        // the copy is independent
        var aCopy = new Manager(aManager);
        aCopy.SetResource("z", "4");
        Assert.Equal("3", aManager.Value("z"));
        Assert.Equal("4", aCopy.Value("z"));

        string aDir = Path.Combine(myTempBase, "save");
        var aNamed = new Manager("Saved", null, null);
        aNamed.SetResource("z", 3);
        aNamed.SetResource("a", 0.5);
        aNamed.SetResource("m", " spaced");
        Assert.True(aNamed.Save(aDir));
        Assert.Equal("a:\t0.5\nm:\t\\ spaced\nz:\t3\n", File.ReadAllText(Path.Combine(aDir, "Saved")));
        // read back: the escaped leading space is kept
        Assert.Equal(" spaced", new Manager("Saved", aDir, null).Value("m"));
        Assert.False(aNamed.Save(null));
    }

    [Fact]
    public void OCCT_default_resource_files_are_compiled_in()
    {
        var anIges = new Manager("IGES");
        Assert.True(anIges.IsInitialized);
        Assert.Equal("DirectFaces", anIges.Value("ToIGES.exec.op"));
        Assert.Equal("FixShape", anIges.Value("FromIGES.exec.op"));
        Assert.Equal("&Runtime.Tolerance", anIges.Value("FromIGES.FixShape.Tolerance3d"));
        Assert.Equal(-1, anIges.Integer("FromIGES.FixShape.FixFreeShellMode"));

        var aStep = new Manager("STEP");
        Assert.Equal("SplitCommonVertex,DirectFaces", aStep.Value("ToSTEP.exec.op"));

        // no environment variable is read: an unknown name has no resources
        Assert.False(new Manager("SHAPEProcess").IsInitialized);
        Assert.Null(Manager.EmbeddedDefaults("data"));
    }

    [Fact]
    public void AsciiString_values_follow_strtol_and_strtod()
    {
        Assert.True(TCollection_AsciiString.IsIntegerValue(" 12abc"));
        Assert.Equal(12, TCollection_AsciiString.IntegerValue(" 12abc"));
        Assert.False(TCollection_AsciiString.IsIntegerValue("1.5"));
        Assert.False(TCollection_AsciiString.IsIntegerValue("1 x."));
        Assert.True(TCollection_AsciiString.IsIntegerValue("-7"));
        Assert.False(TCollection_AsciiString.IsIntegerValue("&Runtime.Tolerance"));
        Assert.ThrowsAny<NumericErrorException>(() => TCollection_AsciiString.IntegerValue("x"));
        Assert.Equal(int.MaxValue, TCollection_AsciiString.IntegerValue("99999999999"));

        Assert.True(TCollection_AsciiString.IsRealValue("1.e-7 "));
        Assert.False(TCollection_AsciiString.IsRealValue("1.e-7 ", true));
        Assert.Equal(1e-7, TCollection_AsciiString.RealValue("1.e-7"));
        Assert.False(TCollection_AsciiString.IsRealValue("C1"));
        Assert.ThrowsAny<NumericErrorException>(() => TCollection_AsciiString.RealValue("x"));

        const string aSeq = " DirectFaces,ElementaryToRevolution;\tSplitAngle , ";
        Assert.Equal("DirectFaces", TCollection_AsciiString.Token(aSeq, " \t,;", 1));
        Assert.Equal("ElementaryToRevolution", TCollection_AsciiString.Token(aSeq, " \t,;", 2));
        Assert.Equal("SplitAngle", TCollection_AsciiString.Token(aSeq, " \t,;", 3));
        Assert.Equal("", TCollection_AsciiString.Token(aSeq, " \t,;", 4));
        Assert.Equal("", TCollection_AsciiString.Token("", " ", 1));
    }
}
