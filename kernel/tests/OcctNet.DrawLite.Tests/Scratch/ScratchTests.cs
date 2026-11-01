using Occt;
using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

public class ScratchTests(ITestOutputHelper output)
{
    [Fact]
    public void Scratch()
    {
        var aDI = new DrawInterpreter();
        aDI.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        string aScript = File.ReadAllText(Environment.GetEnvironmentVariable("SCRATCH_SCRIPT")!);
        try
        {
            output.WriteLine(aDI.Eval(aScript));
        }
        catch (Exception e)
        {
            output.WriteLine("!! " + e.Message);
        }
        output.WriteLine(aDI.Output.ToString() ?? "");
    }

    [Fact]
    public void Scratch2()
    {
        var aDI = new DrawInterpreter();
        aDI.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        aDI.Eval("beziercurve bc1 5 5 5 5 5 -5 5 -5 -5 5 -5 5 5 5 5 5; mkedge e1 bc1; wire UpCover e1; copy e1 e2; ttranslate e2 0 0 -5; wire BottomCover e2; thrusections s 1 1 UpCover BottomCover");
        TopoDS_Shape s = aDI.GetShape("s");
        var aMO = new BRepOffset_MakeOffset();
        aMO.Initialize(s, -0.001, 1e-7, BRepOffset_Mode.BRepOffset_Skin, false, false, GeomAbs_JoinType.GeomAbs_Intersection, false, false);
        aMO.MakeOffsetShape();
        output.WriteLine(aMO.IsDone() + " " + aMO.Error());
        try
        {
            var aExp = new TopExp_Explorer(s, TopAbs_ShapeEnum.TopAbs_FACE);
            output.WriteLine(aMO.Generated(aExp.Current()).Count + " " + aMO.Modified(aExp.Current()).Count + " " + aMO.IsDeleted(aExp.Current()));
        }
        catch (Exception e)
        {
            output.WriteLine(e.ToString());
        }
    }
}
