// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeChamfer.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeChamfer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using static Occt.ChFiDS_ChamfMethod;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// Builds chamfers on the edges of a shell or solid: by one distance (symmetric), two distances or a distance and an
/// angle, each measured on a face adjacent to the edge.
/// </summary>
/// <remarks>
/// A maker over a <see cref="ChFi3d_ChBuilder"/>, as <see cref="BRepFilletAPI_MakeFillet"/>: the caller's shape is
/// never written, the result, the history and the lists hand out frozen shapes, the history answers for the caller's
/// sub-shapes, and the faces come from unit 7R's reconstruction (decision 992).
/// </remarks>
public class BRepFilletAPI_MakeChamfer : BRepFilletAPI_LocalOperation
{
    private readonly ChFi3d_ChBuilder myBuilder;
    private readonly NCollection_Map<Shape> myMap = new(ShapeHasher.Instance);

    //=================================================================================================

    /// <summary>Initializes an algorithm for computing chamfers on the shape <paramref name="S"/>.</summary>
    public BRepFilletAPI_MakeChamfer(in Shape S)
    {
        myBuilder = new ChFi3d_ChBuilder(S);
    }

    //=================================================================================================

    /// <summary>Adds the edge <paramref name="E"/> (and the edges tangent to it) as a contour; the distances are set after.</summary>
    public override void Add(in Edge E)
    {
        myBuilder.Add(E);
    }

    //=================================================================================================

    /// <summary>Adds a symmetric chamfer contour of distance <paramref name="Dis"/> on the edge <paramref name="E"/>.</summary>
    public void Add(double Dis, in Edge E)
    {
        myBuilder.Add(Dis, E);
    }

    //=================================================================================================

    /// <summary>Sets the distance of the symmetric chamfer of the contour <paramref name="IC"/>.</summary>
    public void SetDist(double Dis, int IC, in Face F)
    {
        myBuilder.SetDist(Dis, IC, F);
    }

    //=================================================================================================

    /// <summary>The distance of the symmetric chamfer of the contour <paramref name="IC"/>.</summary>
    public void GetDist(int IC, ref double Dis)
    {
        myBuilder.GetDist(IC, ref Dis);
    }

    //=================================================================================================

    /// <summary>
    /// Adds a chamfer contour by two distances on the edge <paramref name="E"/>: <paramref name="Dis1"/> measured on the
    /// face <paramref name="F"/>, <paramref name="Dis2"/> on the other.
    /// </summary>
    public void Add(double Dis1, double Dis2, in Edge E, in Face F)
    {
        myBuilder.Add(Dis1, Dis2, E, F);
    }

    //=================================================================================================

    /// <summary>Sets the two distances of the contour <paramref name="IC"/>, <paramref name="Dis1"/> on the face <paramref name="F"/>.</summary>
    public void SetDists(double Dis1, double Dis2, int IC, in Face F)
    {
        myBuilder.SetDists(Dis1, Dis2, IC, F);
    }

    //=================================================================================================

    /// <summary>The two distances of the contour <paramref name="IC"/>.</summary>
    public void Dists(int IC, ref double Dis1, ref double Dis2)
    {
        double temp1 = 0.0, temp2 = 0.0;
        myBuilder.Dists(IC, ref temp1, ref temp2);
        Dis1 = temp1;
        Dis2 = temp2;
    }

    //=================================================================================================

    /// <summary>
    /// Adds a chamfer contour by a distance <paramref name="Dis"/> on the face <paramref name="F"/> and an angle
    /// <paramref name="Angle"/> (radians) on the edge <paramref name="E"/>.
    /// </summary>
    public void AddDA(double Dis, double Angle, in Edge E, in Face F)
    {
        myBuilder.AddDA(Dis, Angle, E, F);
    }

    //=================================================================================================

    /// <summary>Sets the distance and the angle of the contour <paramref name="IC"/>, the distance on the face <paramref name="F"/>.</summary>
    public void SetDistAngle(double Dis, double Angle, int IC, in Face F)
    {
        myBuilder.SetDistAngle(Dis, Angle, IC, F);
    }

    //=================================================================================================

    /// <summary>The distance and the angle of the contour <paramref name="IC"/>.</summary>
    public void GetDistAngle(int IC, ref double Dis, ref double Angle)
    {
        myBuilder.GetDistAngle(IC, ref Dis, ref Angle);
    }

    //=================================================================================================

    /// <summary>Sets the mode of the chamfer (classic, or a throat: ChFiDS_ChamfMode).</summary>
    public void SetMode(ChFiDS_ChamfMode theMode)
    {
        myBuilder.SetMode(theMode);
    }

    //=================================================================================================

    /// <summary>True if the chamfer of the contour <paramref name="IC"/> is symmetric (one distance).</summary>
    public bool IsSymetric(int IC)
    {
        ChFiDS_ChamfMethod ChamfMeth = myBuilder.IsChamfer(IC);
        bool ret = false;

        if (ChamfMeth == ChFiDS_Sym)
        {
            ret = true;
        }

        return ret;
    }

    //=================================================================================================

    /// <summary>True if the chamfer of the contour <paramref name="IC"/> is by two distances.</summary>
    public bool IsTwoDistances(int IC)
    {
        ChFiDS_ChamfMethod ChamfMeth = myBuilder.IsChamfer(IC);
        bool ret = false;

        if (ChamfMeth == ChFiDS_TwoDist)
        {
            ret = true;
        }

        return ret;
    }

    //=================================================================================================

    /// <summary>True if the chamfer of the contour <paramref name="IC"/> is by a distance and an angle.</summary>
    public bool IsDistanceAngle(int IC)
    {
        ChFiDS_ChamfMethod ChamfMeth = myBuilder.IsChamfer(IC);
        bool ret = false;

        if (ChamfMeth == ChFiDS_DistAngle)
        {
            ret = true;
        }

        return ret;
    }

    //=================================================================================================

    public override void ResetContour(int IC)
    {
        myBuilder.ResetContour(IC);
    }

    //=================================================================================================

    public override int NbContours()
    {
        return myBuilder.NbElements();
    }

    //=================================================================================================

    public override int Contour(in Edge E)
    {
        return myBuilder.Contains(E);
    }

    //=================================================================================================

    public override int NbEdges(int I)
    {
        ChFiDS_Spine Spine = myBuilder.Value(I)!;
        int n = Spine.NbEdges();
        return n;
    }

    //=================================================================================================

    /// <summary>The edge <paramref name="J"/> of the contour <paramref name="I"/>: the caller's edge.</summary>
    public override Edge Edge(int I, int J)
    {
        ChFiDS_Spine Spine = myBuilder.Value(I)!;
        Edge S = Spine.Edges(J);
        return ShapeCasts.Edge(myBuilder.FromWork(S)); // the caller's (OCCT.NET)
    }

    //=================================================================================================

    public override void Remove(in Edge E)
    {
        myBuilder.Remove(E);
    }

    //=================================================================================================

    public override double Length(int IC)
    {
        return myBuilder.Length(IC);
    }

    //=================================================================================================

    public override Vertex FirstVertex(int IC)
    {
        return myBuilder.FirstVertex(IC);
    }

    //=================================================================================================

    public override Vertex LastVertex(int IC)
    {
        return myBuilder.LastVertex(IC);
    }

    //=================================================================================================

    public override double Abscissa(int IC, in Vertex V)
    {
        return myBuilder.Abscissa(IC, V);
    }

    //=================================================================================================

    public override double RelativeAbscissa(int IC, in Vertex V)
    {
        return myBuilder.RelativeAbscissa(IC, V);
    }

    //=================================================================================================

    public override bool ClosedAndTangent(int IC)
    {
        return myBuilder.ClosedAndTangent(IC);
    }

    //=================================================================================================

    public override bool Closed(int IC)
    {
        return myBuilder.Closed(IC);
    }

    //=================================================================================================

    /// <summary>
    /// OCCT's <c>TopOpeBRepBuild_HBuilder</c>, here the reconstruction's seam (decision 1137): internal, its type
    /// being OCCT.NET's.
    /// </summary>
    internal ChFi3d_Reconstruction Builder()
    {
        return myBuilder.Builder();
    }

    /// <summary>The chamfer builder (OCCT.NET, for ported callers and tests).</summary>
    internal ChFi3d_ChBuilder InternalBuilder() => myBuilder;

    //=================================================================================================

    /// <summary>Builds the chamfers of all the contours; the result is <see cref="BRepBuilderAPI_MakeShape.Shape"/>.</summary>
    public override void Build(ProgressRange theRange = default)
    {
        myBuilder.Compute();
        if (myBuilder.IsDone())
        {
            Done();
            myShape = myBuilder.InternalShape();

            // creation of the Map.
            Explorer ex = new();
            for (ex.Init(myShape, ShapeType.Face); ex.More(); ex.Next())
            {
                myMap.Add(ex.Current());
            }
        }
    }

    //=================================================================================================

    /// <summary>Resets the fields updated by Build, not the contours.</summary>
    public override void Reset()
    {
        NotDone();
        myBuilder.Reset();
        myMap.Clear();
    }

    //=================================================================================================

    //=================================================================================================

    public override void Simulate(int IC)
    {
        myBuilder.Simulate(IC);
    }

    //=================================================================================================

    public override int NbSurf(int IC)
    {
        return myBuilder.NbSurf(IC);
    }

    //=================================================================================================

    public override NCollection_HArray1<ChFiDS_CircSection>? Sect(int IC, int IS)
    {
        return myBuilder.Sect(IC, IS);
    }

    //=================================================================================================

    // The lists hold faces of the result: freeze it with them (decision 414).
    private protected override void BeforePublishingLists()
    {
        if (IsDone())
        {
            myShape.TShape()?.FreezeGraph();
        }
    }

    // Generated(EorV): the faces of the chamfers on the edge or vertex EorV.
    internal override NCollection_List<Shape> InternalGenerated(in Shape EorV)
    {
        return myBuilder.InternalGenerated(EorV);
    }

    //=================================================================================================

    // Modified(F): the pieces of the face F (the union of the states OUT, IN and ON, as OCCT; the reconstruction
    // builds IN only, decision 1110).
    internal override NCollection_List<Shape> InternalModified(in Shape theF)
    {
        Shape F = myBuilder.ToWork(theF); // the working copy's (OCCT.NET)
        myGenerated.Clear();

        if (myBuilder.Builder().IsSplit(F, Out))
        {
            NCollection_List<Shape>.Iterator It = new(myBuilder.Builder().Splits(F, Out));
            for (; It.More(); It.Next())
            {
                myGenerated.Append(It.Value());
            }
        }
        if (myBuilder.Builder().IsSplit(F, In))
        {
            NCollection_List<Shape>.Iterator It = new(myBuilder.Builder().Splits(F, In));
            for (; It.More(); It.Next())
            {
                myGenerated.Append(It.Value());
            }
        }
        if (myBuilder.Builder().IsSplit(F, On))
        {
            NCollection_List<Shape>.Iterator It = new(myBuilder.Builder().Splits(F, On));
            for (; It.More(); It.Next())
            {
                myGenerated.Append(It.Value());
            }
        }
        return myGenerated;
    }

    //=================================================================================================

    /// <summary>True if the face <paramref name="F"/> is neither in the result nor split.</summary>
    public override bool IsDeleted(in Shape F)
    {
        Shape aWork = myBuilder.ToWork(F); // the working copy's (OCCT.NET)
        return !(myMap.Contains(F) || myMap.Contains(aWork) || myBuilder.Builder().IsSplit(aWork, Out)
                 || myBuilder.Builder().IsSplit(aWork, In)
                 || myBuilder.Builder().IsSplit(aWork, On));
    }
}
