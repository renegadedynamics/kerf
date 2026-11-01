// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeInterferenceTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeInterferenceTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeInterferenceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The tool HLRBRep_EdgeIList and HLRBRep_VertexList use to read the current edge of the hidden-line data: its two
/// vertices (as intersections), its 2D geometry at a parameter, and the geometry of an interference's boundary.
/// </summary>
/// <remarks>
/// OCCT's value class (HLRBRep_VertexList copies it); a class here with OCCT's implicit copy as
/// <see cref="HLRBRep_EdgeInterferenceTool(HLRBRep_EdgeInterferenceTool)"/> (the data shared, the vertices and the
/// current index copied). A working object.
/// </remarks>
public sealed class HLRBRep_EdgeInterferenceTool
{
    private readonly HLRBRep_Data myDS;
    private readonly HLRAlgo_Intersection[] inter = new HLRAlgo_Intersection[2];
    private int cur;

    public HLRBRep_EdgeInterferenceTool(HLRBRep_Data DS)
    {
        myDS = DS;
    }

    /// <summary>OCCT's implicit copy.</summary>
    public HLRBRep_EdgeInterferenceTool(HLRBRep_EdgeInterferenceTool theOther)
    {
        myDS = theOther.myDS;
        inter[0] = theOther.inter[0];
        inter[1] = theOther.inter[1];
        cur = theOther.cur;
    }

    /// <summary>Reads the vertices of the data's current edge.</summary>
    public void LoadEdge()
    {
        double p1, p2;
        float t1, t2;
        NCollection_Array1<HLRBRep_EdgeData> ED = myDS.EDataArray();
        HLRBRep_EdgeData ed = ED[myDS.Edge()];
        ed.Status().Bounds(out p1, out t1, out p2, out t2);
        inter[0].Parameter(p1);
        inter[0].Tolerance(t1);
        inter[0].Index(ed.VSta());
        inter[1].Parameter(p2);
        inter[1].Tolerance(t2);
        inter[1].Index(ed.VEnd());
    }

    public void InitVertices() => cur = 0;

    public bool MoreVertices() => cur < 2;

    public void NextVertex() => cur++;

    public HLRAlgo_Intersection CurrentVertex() => inter[cur];

    public TopAbs_Orientation CurrentOrientation()
    {
        if (cur == 0)
        {
            return TopAbs_FORWARD;
        }
        else
        {
            return TopAbs_REVERSED;
        }
    }

    public double CurrentParameter() => inter[cur].Parameter();

    public bool IsPeriodic() => false;

    /// <summary>The tangent, normal and curvature of the current edge at Param, in the projection plane.</summary>
    public void EdgeGeometry(double Param, out gp_Dir Tgt, out gp_Dir Nrm, out double CrLE)
    {
        myDS.LocalLEGeometry2D(Param, out gp_Dir2d TgLE, out gp_Dir2d NmLE, out CrLE);
        Tgt = new gp_Dir(TgLE.X(), TgLE.Y(), 0);
        Nrm = new gp_Dir(NmLE.X(), NmLE.Y(), 0);
    }

    public double ParameterOfInterference(in HLRAlgo_Interference I) => I.Intersection().Parameter();

    /// <summary>True if the two interferences are at the same vertex.</summary>
    public bool SameInterferences(in HLRAlgo_Interference I1, in HLRAlgo_Interference I2)
    {
        int ind1 = I1.Intersection().Index();
        int ind2 = I2.Intersection().Index();
        if (ind1 != 0 && ind2 != 0)
        {
            return ind1 == ind2;
        }
        return false;
    }

    /// <summary>True if the interference is at the current vertex.</summary>
    public bool SameVertexAndInterference(in HLRAlgo_Interference I)
    {
        if (I.Intersection().Index() == inter[cur].Index())
        {
            return true;
        }
        return I.Intersection().Orientation() == ((cur == 0) ? TopAbs_FORWARD : TopAbs_REVERSED);
    }

    /// <summary>The tangent, normal and curvature of the interference's boundary edge, in the projection plane.</summary>
    public void InterferenceBoundaryGeometry(in HLRAlgo_Interference I, out gp_Dir Tang, out gp_Dir Norm,
                                             out double CrFE)
    {
        I.Boundary().Value2D(out int FE, out double Param);
        myDS.LocalFEGeometry2D(FE, Param, out gp_Dir2d TgFE, out gp_Dir2d NmFE, out CrFE);
        Tang = new gp_Dir(TgFE.X(), TgFE.Y(), 0);
        Norm = new gp_Dir(NmFE.X(), NmFE.Y(), 0);
    }
}
