// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>Unit 8G1's XDE commands over Draw-lite's documents, on hand-made shapes, with OCCT's output text.</summary>
[Collection(MeshDefaultsCollection.Name)]
public class DrawXdeCommandsTests
{
    private static (DrawInterpreter, StringWriter) NewInterpreter()
    {
        var anOutput = new StringWriter();
        var anInterpreter = new DrawInterpreter(anOutput);
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return (anInterpreter, anOutput);
    }

    // two boxes in a compound, added as an assembly, and a third box
    private static DrawInterpreter Assembly()
    {
        (DrawInterpreter aDI, _) = NewInterpreter();
        Assert.Equal("document D created\n", aDI.Eval("XNewDoc D"));
        aDI.Eval("box b1 10 10 10; box b2 10 10 10 20 20 20; compound b1 b2 c; box b3 30 30 30 30 30 30");
        Assert.Equal("0:1:1:1", aDI.Eval("XAddShape D c"));
        Assert.Equal("0:1:1:4", aDI.Eval("XAddShape D b3"));
        return aDI;
    }

    [Fact]
    public void A_compound_is_added_as_an_assembly_with_XCAFs_names_and_dump()
    {
        DrawInterpreter aDI = Assembly();
        Assert.Equal("0:1:1:1", aDI.Eval("XFindShape D c"));
        Assert.Equal("0:1:1:2", aDI.Eval("XFindShape D b1"));
        Assert.Equal("0:1:1:1 0:1:1:2 0:1:1:3 0:1:1:4 ", aDI.Eval("XGetTopLevelShapes D"));
        Assert.Equal("0:1:1:1 0:1:1:4 ", aDI.Eval("XGetFreeShapes D"));
        Assert.Equal("There are a TopLevelShapeThere are an AssemblyThis Shape don't used", aDI.Eval("XLabelInfo D 0:1:1:1"));
        Assert.Equal("There are a TopLevelShapeThere are a ComponentThis Shape don't used", aDI.Eval("XLabelInfo D 0:1:1:1:1"));
        Assert.Equal("0:1:1:2", aDI.Eval("XGetReferredShape D 0:1:1:1:1"));
        Assert.Equal("1", aDI.Eval("XGetUsers D 0:1:1:2"));
        Assert.Equal("2", aDI.Eval("XNbComponents D 0:1:1:1"));
        Assert.Equal("ASSEMBLY", aDI.Eval("GetName D 0:1:1:1"));
        Assert.Equal("=>[0:1:1:2]", aDI.Eval("GetName D 0:1:1:1:1"));
        Assert.Equal("mm", aDI.Eval("XGetLengthUnit D"));
        Assert.Equal("0.001", aDI.Eval("XGetLengthUnit D -scale"));

        Assert.Equal("\nASSEMBLY COMPOUND 0:1:1:1 \"ASSEMBLY\" \n"
                     + "\tINSTANCE SOLID 0:1:1:1:1 (refers to 0:1:1:2) \"=>[0:1:1:2]\" \n"
                     + "\tINSTANCE SOLID 0:1:1:1:2 (refers to 0:1:1:3) \"=>[0:1:1:3]\" \n\n"
                     + "PART SOLID 0:1:1:2 \"SOLID\" \n\n"
                     + "PART SOLID 0:1:1:3 \"SOLID\" \n\n"
                     + "PART SOLID 0:1:1:4 \"SOLID\" \n\n"
                     + "\nFree Shapes: 2\n"
                     + "ASSEMBLY COMPOUND  0:1:1:1 \"ASSEMBLY\" \n"
                     + "PART SOLID  0:1:1:4 \"SOLID\" \n",
                     aDI.Eval("Xdump D"));

        // XGetOneShape: the free shapes' compound
        Assert.Equal("s", aDI.Eval("XGetOneShape s D"));
        Assert.Contains("SOLID     : 3", aDI.Eval("nbshapes s"), StringComparison.Ordinal);

        // auto-naming off: no names
        Assert.Equal("1", aDI.Eval("XAutoNaming D"));
        aDI.Eval("XAutoNaming D 0; box b4 1 1 1");
        Assert.Equal("0:1:1:5", aDI.Eval("XAddShape D b4"));
        Assert.Throws<DrawException>(() => aDI.Eval("GetName D 0:1:1:5"));
    }

    [Fact]
    public void XStat_counts_the_labels_names_colours_and_layers()
    {
        DrawInterpreter aDI = Assembly();
        aDI.Eval("XSetColor D b1 1 0 0 s; XSetColor D c 0 0 1 c; XSetLayer D b3 L1; XSetProps D b3");
        string aStat = aDI.Eval("XStat D");
        Assert.Equal("\nStatistis of shapes in the document:\n"
                     + "level N 0 : 4\nlevel N 1 : 2\n"
                     + "Total number of labels for shapes in the document = 6\n"
                     + "Number of labels with name = 6\n"
                     + "Number of labels with color link = 2\n"
                     + "Number of labels with layer link = 1\n"
                     + "Number of labels with vis material link = 0\n"
                     + "\nStatistis of Props in the document:\n"
                     + "Number of Centroid Props = 1\nNumber of Volume Props = 1\nNumber of Area Props = 1\n"
                     + "\nNumber of colors = 2\nRED BLUE \n"
                     + "\nNumber of layers = 1\n\"L1\" \n"
                     + "\nNumber of vis materials = 0\n\n",
                     aStat);
        Assert.Contains("0:1:1:4 SOLID  has attributes: Centroid Volume(27000", aDI.Eval("XStat D 1"), StringComparison.Ordinal);
    }

    [Fact]
    public void Colours_have_labels_that_can_be_found_and_removed()
    {
        DrawInterpreter aDI = Assembly();
        Assert.Equal("0:1:2:1", aDI.Eval("XAddColor D 1 0 0"));
        Assert.Equal("0:1:2:2", aDI.Eval("XAddColor D GREEN"));
        Assert.Equal("0:1:2:1", aDI.Eval("XFindColor D 1 0 0"));
        Assert.Equal("", aDI.Eval("XFindColor D 0 0 1"));
        aDI.Eval("XSetColor D b1 1 0 0 s; XSetColor D b1 0 1 1 c; XSetColor D 0:1:1:4 YELLOW 0.5");
        Assert.Equal("RED GREEN CYAN YELLOW (0.5) ", aDI.Eval("XGetAllColors D"));
        Assert.Equal("RED", aDI.Eval("XGetShapeColor D 0:1:1:2 s"));
        Assert.Equal("CYAN", aDI.Eval("XGetShapeColor D 0:1:1:2 c"));
        Assert.Equal("YELLOW (0.5)", aDI.Eval("XGetShapeColor D 0:1:1:4"));
        Assert.Equal("RED", aDI.Eval("XGetColor D 0:1:2:1"));

        // removing a colour removes it from its shapes; its label stays empty
        aDI.Eval("XRemoveColor D [XFindColor D 0 1 1]");
        Assert.Equal("", aDI.Eval("XGetShapeColor D 0:1:1:2 c"));
        Assert.Equal("RED GREEN YELLOW (0.5) ", aDI.Eval("XGetAllColors D"));
        Assert.Equal("0:1:2:5", aDI.Eval("XAddColor D 0 1 1"));
        aDI.Eval("XUnsetColor D b1 s");
        Assert.Equal("", aDI.Eval("XGetShapeColor D 0:1:1:2 s"));

        // the errors' messages on the output (Message::SendFail)
        (DrawInterpreter aDI2, StringWriter anOutput) = NewInterpreter();
        Assert.Throws<DrawException>(() => aDI2.Eval("XSetColor Z b1 1 0 0"));
        Assert.Contains("Z is not a Document", anOutput.ToString(), StringComparison.Ordinal);
        Assert.Contains("Syntax error: Z is not a document", anOutput.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void Layers_are_set_found_and_removed()
    {
        DrawInterpreter aDI = Assembly();
        Assert.Equal("0:1:3:1", aDI.Eval("XAddLayer D L1"));
        Assert.Equal("0:1:3:2", aDI.Eval("XAddLayer D L2"));
        Assert.Equal("0:1:3:1", aDI.Eval("XAddLayer D L1"));
        aDI.Eval("XSetLayer D b1 L1; XSetLayer D 0:1:1:4 L2; XSetLayer D 0:1:1:4 L3");
        Assert.Equal("\"L1\" \"L2\" \"L3\" ", aDI.Eval("XGetAllLayers D"));
        Assert.Equal("0:1:3:1 0:1:3:2 0:1:3:3 ", aDI.Eval("XGetLayerLabels D"));
        Assert.Equal("\"L2\" \"L3\" ", aDI.Eval("XGetLayers D b3"));
        Assert.Equal("\"L1\"", aDI.Eval("XGetOneLayer D 0:1:3:1"));
        aDI.Eval("XUnSetLayer D b3 L2");
        Assert.Equal("\"L3\" ", aDI.Eval("XGetLayers D b3"));
        aDI.Eval("XRemoveLayer D L3");
        Assert.Equal("", aDI.Eval("XGetLayers D b3"));
        Assert.Equal("\"L1\" \"L2\" ", aDI.Eval("XGetAllLayers D"));
        Assert.Equal("", aDI.Eval("XFindLayer D L3"));
        Assert.Equal("1", aDI.Eval("XIsVisible D L1"));
        aDI.Eval("XSetVisibility D L1 0");
        Assert.Equal("0", aDI.Eval("XIsVisible D L1"));
    }

    [Fact]
    public void Shapes_and_components_are_removed_and_assemblies_updated()
    {
        (DrawInterpreter aDI, _) = NewInterpreter();
        aDI.Eval("box b 1 1 1; box bb 2 0 0 1 1 1; compound b bb c; XNewDoc D; XAddShape D c");
        aDI.Eval("XRemoveComponent D 0:1:1:1:2; XRemoveShape D 0:1:1:3; XUpdateAssemblies D; XGetOneShape r D");
        Assert.Contains("SOLID     : 1", aDI.Eval("nbshapes r"), StringComparison.Ordinal);
        Assert.Equal("0:1:1:1 0:1:1:2 ", aDI.Eval("XGetTopLevelShapes D"));

        // a free shape removed completely: its reference and its referred shape
        aDI.Eval("box t 1 1 1; ttranslate t 5 0 0");
        Assert.Equal("0:1:1:4", aDI.Eval("XAddShape D t"));
        Assert.Equal("Transformation (3 rows * 4 columns matrix): (1,0,0,5) (0,1,0,0) (0,0,1,0)", aDI.Eval("XDumpLocation D 0:1:1:4"));
        aDI.Eval("XRemoveShape D 0:1:1:4");
        Assert.Equal("0:1:1:1 0:1:1:2 ", aDI.Eval("XGetTopLevelShapes D"));

        // sub-shapes: added, found, forgotten
        aDI.Eval("XAddShape D bb 0; explode bb f");
        Assert.Equal("0:1:1:6:1", aDI.Eval("XAddSubShape D bb_1 0:1:1:6"));
        Assert.Equal("0:1:1:6:1", aDI.Eval("XFindSubShape D bb_1 0:1:1:6"));
        Assert.Equal("0:1:1:6", aDI.Eval("XFindMainShape D bb_1"));
        aDI.Eval("ForgetAll D 0:1:1:6:1");
        Assert.Equal("", aDI.Eval("XFindSubShape D bb_1 0:1:1:6"));
    }

    [Fact]
    public void Documents_are_closed_and_the_undo_commands_are_stubs()
    {
        (DrawInterpreter aDI, StringWriter anOutput) = NewInterpreter();
        Assert.Equal("document D created\n0:1", aDI.Eval("NewDocument D BinXCAF"));
        Assert.Equal("D is already a document\n", aDI.Eval("XNewDoc D"));
        Assert.Equal("100 0 0", aDI.Eval("UndoLimit D 100"));
        Assert.Equal("", aDI.Eval("NewCommand D; CommitCommand D"));
        aDI.Eval("XNewDoc E");
        aDI.Eval("Close *");
        Assert.Equal("0", aDI.Eval("info exists D"));
        Assert.Equal("0", aDI.Eval("info exists E"));
        Assert.Throws<DrawException>(() => aDI.Eval("Close D"));
        Assert.Contains("D is not a Document", anOutput.ToString(), StringComparison.Ordinal);
        Assert.Equal("", aDI.Eval("Close D -silent"));
    }

    [Fact]
    public void A_document_written_to_glTF_is_read_back()
    {
        (DrawInterpreter aDI, _) = NewInterpreter();
        aDI.Eval("box b 1 2 3; incmesh b 0.1; compound b c; compound c cc; XNewDoc D1; XAddShape D1 cc");
        aDI.Eval("SetName D1 [XFindShape D1 b] box; SetName D1 [XFindShape D1 c]:1 box_inst");
        aDI.Eval("WriteGltf D1 /tmp/p8g1/t.glb");
        Assert.False(File.Exists("/tmp/p8g1/t.glb"));
        aDI.Eval("ReadGltf D /tmp/p8g1/t.glb; XGetOneShape s D");
        // a face per primitive, as OCCT's writer writes them; with -mergeFaces one face
        Assert.Contains("FACE      : 6", aDI.Eval("nbshapes s"), StringComparison.Ordinal);
        Assert.Contains("INSTANCE COMPOUND 0:1:1:2:1 (refers to 0:1:1:3) \"box_inst\"", aDI.Eval("Xdump D"), StringComparison.Ordinal);
        Assert.Contains("PART COMPOUND 0:1:1:3 \"box\"", aDI.Eval("Xdump D"), StringComparison.Ordinal);
        aDI.Eval("WriteGltf D1 /tmp/p8g1/m.gltf -mergeFaces; ReadGltf Dm /tmp/p8g1/m.gltf; XGetOneShape sm Dm");
        Assert.Contains("FACE      : 1", aDI.Eval("nbshapes sm"), StringComparison.Ordinal);
        Assert.Equal("1", aDI.Eval("file exists /tmp/p8g1/m.bin"));
        // a shape: through a document of its own
        aDI.Eval("writegltf b /tmp/p8g1/b.glb; readgltf rb /tmp/p8g1/b.glb");
        Assert.Contains("FACE      : 6", aDI.Eval("nbshapes rb"), StringComparison.Ordinal);
        // OBJ and its materials
        aDI.Eval("WriteObj D1 /tmp/p8g1/t.obj; writeobj b /tmp/p8g1/b.obj");
        Assert.Equal("1", aDI.Eval("file exists /tmp/p8g1/t.mtl"));
        Assert.Contains("o ", aDI.Eval("set f [open /tmp/p8g1/b.obj]; set t [read $f]; close $f; set t"), StringComparison.Ordinal);
    }
}
