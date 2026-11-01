// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

public partial class ChFi3d_Builder
{
    protected ChFi3d_Builder(in TopoDS_Shape S, double Ta)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal IChFi3d_Reconstruction Builder() => myCoup;

    public void SetParams(double Tang, double Tesp, double T2d, double TApp3d, double TolApp2d, double Fleche)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void SetContinuity(GeomAbs_Shape InternalContinuity, double AngularTolerance)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public bool IsDone()
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public TopoDS_Shape Shape()
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public int NbFaultyContours()
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public int FaultyContour(int I)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public int NbComputedSurfaces(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public Geom_Surface? ComputedSurface(int IC, int IS)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public int NbFaultyVertices()
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public TopoDS_Vertex FaultyVertex(int IV)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public bool HasResult()
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public TopoDS_Shape BadShape()
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public ChFiDS_ErrorStatus StripeStatus(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    private bool FaceTangency(in TopoDS_Edge E0, in TopoDS_Edge E1, in TopoDS_Vertex V)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal void PerformExtremity(ChFiDS_Spine? Spine)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool PerformElement(ChFiDS_Spine? Spine, double Offset, in TopoDS_Face theFirstFace)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void Remove(in TopoDS_Edge E)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public ChFiDS_Spine? Value(int I)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public int NbElements()
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public int Contains(in TopoDS_Edge E)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public int Contains(in TopoDS_Edge E, ref int IndexInSpine)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public double Length(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public TopoDS_Vertex FirstVertex(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public TopoDS_Vertex LastVertex(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public double Abscissa(int IC, in TopoDS_Vertex V)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public double RelativeAbscissa(int IC, in TopoDS_Vertex V)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public bool Closed(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public bool ClosedAndTangent(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

}
