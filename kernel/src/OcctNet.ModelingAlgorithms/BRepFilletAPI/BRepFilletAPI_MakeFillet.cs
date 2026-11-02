// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeFillet.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeFillet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Schematiq.Collections;
using static Occt.ChFi3d_FilletShape;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Builds fillets on the broken edges of a shell or solid: initialized with a shape, given contours and radii
/// (constant, linear between two values, by a law or interpolating parameter-radius pairs), built, then consulted.
/// </summary>
/// <remarks>
/// <para>
/// A maker (decisions 21, 414) over a <see cref="ChFi3d_FilBuilder"/>. The caller's shape is never written: the
/// builder copies a frozen shape at the entry and gives back the caller's sub-shapes where the fillets left them
/// unchanged (decision 1136). <see cref="BRepBuilderAPI_MakeShape.Shape"/>, <see cref="BRepBuilderAPI_MakeShape.Generated"/>,
/// <see cref="BRepBuilderAPI_MakeShape.Modified"/>, <see cref="NewFaces"/> and <see cref="BadShape"/> hand out frozen
/// shapes. The history answers for the caller's edges, vertices and faces.
/// </para>
/// <para>
/// The faces come from unit 7R's reconstruction on the Boolean engine (decision 992), not from OCCT's
/// TopOpeBRepBuild_HBuilder: the result is native's geometrically, its faces in another order.
/// </para>
/// </remarks>
public class BRepFilletAPI_MakeFillet : BRepFilletAPI_LocalOperation
{
    private readonly ChFi3d_FilBuilder myBuilder;
    private readonly NCollection_Map<TopoDS_Shape> myMap = new(TopTools_ShapeMapHasher.Instance);

    //=================================================================================================

    /// <summary>
    /// Initializes the computation of the fillets on <paramref name="S"/>. <paramref name="FShape"/> sets the type of
    /// fillet surface: ChFi3d_Rational (classical NURBS representation of circles, the default),
    /// ChFi3d_QuasiAngular (a NURBS representation whose parametrization matches the circle's) or ChFi3d_Polynomial
    /// (a polynomial representation of circles).
    /// </summary>
    public BRepFilletAPI_MakeFillet(in TopoDS_Shape S, ChFi3d_FilletShape FShape = ChFi3d_Rational)
    {
        myBuilder = new ChFi3d_FilBuilder(S, FShape);
    }

    //=================================================================================================

    /// <summary>Sets the tolerances of the computation (OCCT's defaults: 1e-2, 1e-4, 1e-5, 1e-4, 1e-5, 1e-3).</summary>
    public void SetParams(double Tang, double Tesp, double T2d, double TApp3d, double TolApp2d, double Fleche)
    {
        myBuilder.SetParams(Tang, Tesp, T2d, TApp3d, TolApp2d, Fleche);
    }

    //=================================================================================================

    /// <summary>
    /// Changes the continuity of the fillet surfaces (C0, C1 or C2; C1 by default) and the G1 tolerance between the
    /// fillets and their support faces.
    /// </summary>
    public void SetContinuity(GeomAbs_Shape InternalContinuity, double AngularTolerance)
    {
        myBuilder.SetContinuity(InternalContinuity, AngularTolerance);
    }

    //=================================================================================================

    /// <summary>Adds a fillet contour (a contour of tangent edges); the radius must be set after.</summary>
    public override void Add(in TopoDS_Edge E)
    {
        myBuilder.Add(E);
    }

    //=================================================================================================

    /// <summary>Adds a fillet contour with the radius <paramref name="Radius"/>.</summary>
    public void Add(double Radius, in TopoDS_Edge E)
    {
        // myBuilder.Add(Radius,E);
        myBuilder.Add(E);
        int IC = myBuilder.Contains(E, out int IinC);
        if (IC != 0)
        {
            SetRadius(Radius, IC, IinC);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Adds a fillet contour whose radius evolves linearly from <paramref name="R1"/> to <paramref name="R2"/>
    /// between the first and last vertices of the spine.
    /// </summary>
    public void Add(double R1, double R2, in TopoDS_Edge E)
    {
        myBuilder.Add(E);
        int IC = myBuilder.Contains(E, out int IinC);
        if (IC != 0)
        {
            SetRadius(R1, R2, IC, IinC);
        }
    }

    //=================================================================================================

    /// <summary>Adds a fillet contour whose radius evolves by the law <paramref name="L"/>.</summary>
    public void Add(Law_Function L, in TopoDS_Edge E)
    {
        // myBuilder.Add(L,E);
        myBuilder.Add(E);
        int IC = myBuilder.Contains(E, out int IinC);
        if (IC != 0)
        {
            SetRadius(L, IC, IinC);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Adds a fillet contour whose radius interpolates the pairs <paramref name="UandR"/>: X a relative parameter on
    /// the spine in [0, 1], Y the radius there.
    /// </summary>
    public void Add(NCollection_Array1<gp_Pnt2d> UandR, in TopoDS_Edge E)
    {
        myBuilder.Add(E);
        int IC = myBuilder.Contains(E, out int IinC);
        if (IC != 0)
        {
            SetRadius(UandR, IC, IinC);
        }
    }

    //=================================================================================================

    /// <summary>Sets the radius <paramref name="Radius"/> along the contour <paramref name="IC"/>.</summary>
    public void SetRadius(double Radius, int IC, int IinC)
    {
        gp_XY FirstUandR = new(0.0, Radius), LastUandR = new(1.0, Radius);
        myBuilder.SetRadius(FirstUandR, IC, IinC);
        myBuilder.SetRadius(LastUandR, IC, IinC);
    }

    //=================================================================================================

    /// <summary>
    /// Sets a radius evolving linearly from <paramref name="R1"/> to <paramref name="R2"/> between the first and last
    /// vertices of the contour <paramref name="IC"/>.
    /// </summary>
    public void SetRadius(double R1, double R2, int IC, int IinC)
    {
        double r1, r2;

        if (Math.Abs(R1 - R2) < Precision.Confusion())
        {
            r1 = r2 = (R1 + R2) * 0.5;
        }
        else
        {
            r1 = R1;
            r2 = R2;
        }
        gp_XY FirstUandR = new(0.0, r1), LastUandR = new(1.0, r2);
        myBuilder.SetRadius(FirstUandR, IC, IinC);
        myBuilder.SetRadius(LastUandR, IC, IinC);
    }

    //=================================================================================================

    /// <summary>Sets a radius evolving by the law <paramref name="L"/> along the contour <paramref name="IC"/>.</summary>
    public void SetRadius(Law_Function L, int IC, int IinC)
    {
        myBuilder.SetRadius(L, IC, IinC);
    }

    //=================================================================================================

    /// <summary>
    /// Sets a radius interpolating the pairs <paramref name="UandR"/> (X a relative parameter on the contour, Y the
    /// radius) along the contour <paramref name="IC"/>.
    /// </summary>
    public void SetRadius(NCollection_Array1<gp_Pnt2d> UandR, int IC, int IinC)
    {
        if (UandR.Length() == 1)
        {
            SetRadius(UandR[UandR.Lower()].Y(), IC, IinC);
        }
        else if (UandR.Length() == 2)
        {
            SetRadius(UandR[UandR.Lower()].Y(), UandR[UandR.Upper()].Y(), IC, IinC);
        }
        else
        {
            double Uf = UandR[UandR.Lower()].X();
            double Ul = UandR[UandR.Upper()].X();
            for (int i = UandR.Lower(); i <= UandR.Upper(); i++)
            {
                double Ucur = UandR[i].X();
                Ucur = (Ucur - Uf) / (Ul - Uf);
                gp_XY newUandR = new(Ucur, UandR[i].Y());
                myBuilder.SetRadius(newUandR, IC, IinC);
            }
        }
    }

    //=================================================================================================

    /// <summary>Erases the radius of the contour <paramref name="IC"/> (nothing is done if IC is out of bounds).</summary>
    public override void ResetContour(int IC)
    {
        myBuilder.ResetContour(IC);
    }

    //=================================================================================================

    /// <summary>True if the radius along the contour <paramref name="IC"/> is constant (false if IC is out of bounds).</summary>
    public bool IsConstant(int IC)
    {
        return myBuilder.IsConstant(IC);
    }

    //=================================================================================================

    /// <summary>The constant radius of the contour <paramref name="IC"/> (-1 if IC is out of bounds).</summary>
    public double Radius(int IC)
    {
        return myBuilder.Radius(IC);
    }

    //=================================================================================================

    /// <summary>True if the radius along the edge <paramref name="E"/> of the contour <paramref name="IC"/> is constant.</summary>
    public bool IsConstant(int IC, in TopoDS_Edge E)
    {
        return myBuilder.IsConstant(IC, E);
    }

    //=================================================================================================

    /// <summary>The constant radius along the edge <paramref name="E"/> of the contour <paramref name="IC"/> (-1 if none).</summary>
    public double Radius(int IC, in TopoDS_Edge E)
    {
        return myBuilder.Radius(IC, E);
    }

    //=================================================================================================

    /// <summary>Assigns the radius <paramref name="Radius"/> to the edge <paramref name="E"/> of the contour <paramref name="IC"/>.</summary>
    public void SetRadius(double Radius, int IC, in TopoDS_Edge E)
    {
        myBuilder.SetRadius(Radius, IC, E);
    }

    //=================================================================================================

    /// <summary>The bounds of the law on the edge <paramref name="E"/> of the contour <paramref name="IC"/>; false if it has none.</summary>
    public bool GetBounds(int IC, in TopoDS_Edge E, ref double F, ref double L)
    {
        return myBuilder.GetBounds(IC, E, ref F, ref L);
    }

    //=================================================================================================

    /// <summary>The radius law on the edge <paramref name="E"/> of the contour <paramref name="IC"/> (null if none).</summary>
    public Law_Function? GetLaw(int IC, in TopoDS_Edge E)
    {
        return myBuilder.GetLaw(IC, E);
    }

    //=================================================================================================

    /// <summary>Sets the radius law <paramref name="L"/> on the edge <paramref name="E"/> of the contour <paramref name="IC"/>.</summary>
    public void SetLaw(int IC, in TopoDS_Edge E, Law_Function L)
    {
        myBuilder.SetLaw(IC, E, L);
    }

    //=================================================================================================

    /// <summary>Assigns the radius <paramref name="Radius"/> at the vertex <paramref name="V"/> of the contour <paramref name="IC"/>.</summary>
    public void SetRadius(double Radius, int IC, in TopoDS_Vertex V)
    {
        myBuilder.SetRadius(Radius, IC, V);
    }

    //=================================================================================================

    /// <summary>Assigns the type of fillet surface.</summary>
    public void SetFilletShape(ChFi3d_FilletShape FShape)
    {
        myBuilder.SetFilletShape(FShape);
    }

    //=================================================================================================

    /// <summary>The type of fillet surface.</summary>
    public ChFi3d_FilletShape GetFilletShape()
    {
        return myBuilder.GetFilletShape();
    }

    //=================================================================================================

    public override int NbContours()
    {
        return myBuilder.NbElements();
    }

    //=================================================================================================

    public override int Contour(in TopoDS_Edge E)
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
    public override TopoDS_Edge Edge(int I, int J)
    {
        ChFiDS_Spine Spine = myBuilder.Value(I)!;
        TopoDS_Edge S = Spine.Edges(J);
        return TopoDS.Edge(myBuilder.FromWork(S)); // the caller's (OCCT.NET)
    }

    //=================================================================================================

    public override void Remove(in TopoDS_Edge E)
    {
        myBuilder.Remove(E);
    }

    //=================================================================================================

    public override double Length(int IC)
    {
        return myBuilder.Length(IC);
    }

    //=================================================================================================

    public override TopoDS_Vertex FirstVertex(int IC)
    {
        return myBuilder.FirstVertex(IC);
    }

    //=================================================================================================

    public override TopoDS_Vertex LastVertex(int IC)
    {
        return myBuilder.LastVertex(IC);
    }

    //=================================================================================================

    public override double Abscissa(int IC, in TopoDS_Vertex V)
    {
        return myBuilder.Abscissa(IC, V);
    }

    //=================================================================================================

    public override double RelativeAbscissa(int IC, in TopoDS_Vertex V)
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
    /// being OCCT.NET's. <see cref="NbSurfaces"/> and <see cref="NewFaces"/> give what OCCT's callers read from it.
    /// </summary>
    internal ChFi3d_Reconstruction Builder()
    {
        return myBuilder.Builder();
    }

    /// <summary>The fillet builder (OCCT.NET, for ported callers and tests).</summary>
    internal ChFi3d_FilBuilder InternalBuilder() => myBuilder;

    //=================================================================================================

    /// <summary>Builds the fillets of all the contours; the result is <see cref="BRepBuilderAPI_MakeShape.Shape"/>.</summary>
    public override void Build(Message_ProgressRange theRange = default)
    {
        myBuilder.Compute();
        if (myBuilder.IsDone())
        {
            Done();
            myShape = myBuilder.InternalShape();

            // creation of the Map.
            TopExp_Explorer ex = new();
            for (ex.Init(myShape, TopAbs_FACE); ex.More(); ex.Next())
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

    /// <summary>The number of surfaces built (the data structure's).</summary>
    public int NbSurfaces()
    {
        return myBuilder.Builder().DataStructure()!.NbSurfaces();
    }

    //=================================================================================================

    /// <summary>The faces built on the surface <paramref name="I"/> (frozen).</summary>
    public ISchematiqList<TopoDS_Shape> NewFaces(int I)
    {
        BeforePublishingLists();
        return PublishedList(myBuilder.Builder().NewFaces(I));
    }

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

    // Generated(EorV): the faces of the fillets on the edge or vertex EorV.
    internal override NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape EorV)
    {
        return myBuilder.InternalGenerated(EorV);
    }

    //=================================================================================================

    // Modified(F): the pieces of the face F (the union of the states OUT, IN and ON, as OCCT; the reconstruction
    // builds IN only, decision 1110).
    internal override NCollection_List<TopoDS_Shape> InternalModified(in TopoDS_Shape theF)
    {
        TopoDS_Shape F = myBuilder.ToWork(theF); // the working copy's (OCCT.NET)
        myGenerated.Clear();

        if (myBuilder.Builder().IsSplit(F, TopAbs_OUT))
        {
            NCollection_List<TopoDS_Shape>.Iterator It = new(myBuilder.Builder().Splits(F, TopAbs_OUT));
            for (; It.More(); It.Next())
            {
                myGenerated.Append(It.Value());
            }
        }
        if (myBuilder.Builder().IsSplit(F, TopAbs_IN))
        {
            NCollection_List<TopoDS_Shape>.Iterator It = new(myBuilder.Builder().Splits(F, TopAbs_IN));
            for (; It.More(); It.Next())
            {
                myGenerated.Append(It.Value());
            }
        }
        if (myBuilder.Builder().IsSplit(F, TopAbs_ON))
        {
            NCollection_List<TopoDS_Shape>.Iterator It = new(myBuilder.Builder().Splits(F, TopAbs_ON));
            for (; It.More(); It.Next())
            {
                myGenerated.Append(It.Value());
            }
        }
        return myGenerated;
    }

    //=================================================================================================

    /// <summary>True if the face <paramref name="F"/> is neither in the result nor split.</summary>
    public override bool IsDeleted(in TopoDS_Shape F)
    {
        TopoDS_Shape aWork = myBuilder.ToWork(F); // the working copy's (OCCT.NET)
        return !(myMap.Contains(F) || myMap.Contains(aWork) || myBuilder.Builder().IsSplit(aWork, TopAbs_OUT)
                 || myBuilder.Builder().IsSplit(aWork, TopAbs_IN)
                 || myBuilder.Builder().IsSplit(aWork, TopAbs_ON));
    }

    //=================================================================================================

    /// <summary>The number of contours that failed.</summary>
    public int NbFaultyContours()
    {
        return myBuilder.NbFaultyContours();
    }

    //=================================================================================================

    /// <summary>The index of the <paramref name="I"/>-th contour that failed.</summary>
    public int FaultyContour(int I)
    {
        return myBuilder.FaultyContour(I);
    }

    //=================================================================================================

    /// <summary>The number of surfaces computed on the contour <paramref name="IC"/>.</summary>
    public int NbComputedSurfaces(int IC)
    {
        return myBuilder.NbComputedSurfaces(IC);
    }

    //=================================================================================================

    /// <summary>The surface <paramref name="IS"/> computed on the contour <paramref name="IC"/>.</summary>
    public Geom_Surface? ComputedSurface(int IC, int IS)
    {
        return myBuilder.ComputedSurface(IC, IS);
    }

    //=================================================================================================

    /// <summary>The number of vertices where the corner failed.</summary>
    public int NbFaultyVertices()
    {
        return myBuilder.NbFaultyVertices();
    }

    //=================================================================================================

    /// <summary>The <paramref name="IV"/>-th vertex where the corner failed (the caller's).</summary>
    public TopoDS_Vertex FaultyVertex(int IV)
    {
        return myBuilder.FaultyVertex(IV);
    }

    //=================================================================================================

    /// <summary>True if a partial result was built (<see cref="BadShape"/>) though a corner failed.</summary>
    public bool HasResult()
    {
        return myBuilder.HasResult();
    }

    //=================================================================================================

    /// <summary>The partial result (frozen).</summary>
    /// <exception cref="Standard_NoSuchObject">There is none (<see cref="HasResult"/> is false).</exception>
    public TopoDS_Shape BadShape()
    {
        return myBuilder.BadShape();
    }

    //=================================================================================================

    /// <summary>The status of the contour <paramref name="IC"/>.</summary>
    public ChFiDS_ErrorStatus StripeStatus(int IC)
    {
        return myBuilder.StripeStatus(IC);
    }
}
