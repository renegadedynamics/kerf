// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_LocationLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_LocationLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomFill_PipeError;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>A location law on a wire: one GeomFill location law per non-degenerated edge of the path.</summary>
/// <remarks>
/// A working object (decisions 21 and 806): evaluation caches the edges' lengths, <see cref="NbHoles"/> caches the
/// discontinuities, and the transformations change the elementary laws, which are the law's own copies of the
/// caller's law (one per edge, as in OCCT). <see cref="Law"/> returns them, not copies. The path and its edges are the
/// caller's. <see cref="D0"/> hands out a frozen transformed copy of the section; <see cref="PerformVertex"/> a frozen
/// new vertex, and <see cref="InternalPerformVertex"/> the same vertex unfrozen for the sweeps that keep building on it
/// (decision 5862).
/// </remarks>
public abstract class BRepFill_LocationLaw : Standard_Transient
{
    protected TopoDS_Wire myPath = new();
    protected double myTol;
    protected NCollection_HArray1<GeomFill_LocationLaw>? myLaws;
    protected NCollection_HArray1<double>? myLength;
    protected NCollection_HArray1<TopoDS_Shape>? myEdges;
    protected NCollection_HArray1<int>? myDisc;
    // Set by TangentIsMain, NormalIsMain and BiNormalIsMain and never read, as in OCCT.
#pragma warning disable CS0414
    private int myType;
#pragma warning restore CS0414

    //=================================================================================================

    private static double Norm(in gp_Mat M)
    {
        double R, Norme;
        gp_XYZ Coord;
        Coord = M.Row(1);
        Norme = Math.Abs(Coord.X()) + Math.Abs(Coord.Y()) + Math.Abs(Coord.Z());
        Coord = M.Row(2);
        R = Math.Abs(Coord.X()) + Math.Abs(Coord.Y()) + Math.Abs(Coord.Z());
        if (R > Norme)
        {
            Norme = R;
        }
        Coord = M.Row(3);
        R = Math.Abs(Coord.X()) + Math.Abs(Coord.Y()) + Math.Abs(Coord.Z());
        if (R > Norme)
        {
            Norme = R;
        }

        return Norme;
    }

    //=======================================================================
    // function : ToG0
    // purpose  : Calculate transformation T such as T.M2 = M1
    //=======================================================================

    private static void ToG0(in gp_Mat M1, in gp_Mat M2, out gp_Mat T)
    {
        T = M2.Inverted();
        T *= M1;
    }

    //=================================================================================================

    /// <summary>Initialises all the fields; called by the constructors of the derived classes.</summary>
    protected void Init(in TopoDS_Wire Path)

    {
        int NbEdge;
        BRepTools_WireExplorer wexp = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;

        myPath = Path;
        myTol = 1.0e-4;

        for (NbEdge = 0, wexp.Init(myPath); wexp.More(); wexp.Next())
        {
            //    if (! B.Degenerated(wexp.Current())) NbEdge++;
            if (!BRep_Tool.Degenerated(wexp.Current()))
            {
                NbEdge++;
            }
        }

        myLaws = new NCollection_HArray1<GeomFill_LocationLaw>(1, NbEdge);
        myLength = new NCollection_HArray1<double>(1, NbEdge + 1);
        myLength.Init(-1.0);
        myLength.SetValue(1, 0.0);
        myEdges = new NCollection_HArray1<TopoDS_Shape>(1, NbEdge);
        myDisc = null;
        TangentIsMain();
    }

    //=================================================================================================

    /// <summary>The first error status of the elementary laws other than GeomFill_PipeOk, else GeomFill_PipeOk.</summary>
    public GeomFill_PipeError GetStatus()
    {
        int ii, N = myLaws!.Length();
        GeomFill_PipeError Status = GeomFill_PipeOk;
        for (ii = 1; ii <= N && (Status == GeomFill_PipeOk); ii++)
        {
            Status = myLaws.Value(ii).ErrorStatus();
        }
        return Status;
    }

    //=================================================================================================

    /// <summary>To preserve if possible the tangent in transformations (the default mode).</summary>
    protected void TangentIsMain()
    {
        myType = 1;
    }

    //=================================================================================================

    /// <summary>To preserve if possible the normal in transformations.</summary>
    protected void NormalIsMain()
    {
        myType = 2;
    }

    //=================================================================================================

    /// <summary>To preserve if possible the binormal in transformations.</summary>
    protected void BiNormalIsMain()
    {
        myType = 3;
    }

    //=================================================================================================

    /// <summary>
    /// Applies a linear transformation to each elementary law to reduce the discontinuities of the law to one
    /// rotation.
    /// </summary>
    public virtual void TransformInCompatibleLaw(double TolAngular)
    {

        double Angle;
        int ipath;
        gp_Mat Trsf = new(), M1 = new(), M2 = new();
        gp_Vec V = new(), T1 = new(), T2 = new(), N1 = new(), N2 = new();
        gp_XYZ OZ = new(0, 0, 1);

        myLaws!.Value(1).GetDomain(out double First, out double Last);

        for (ipath = 2; ipath <= myLaws.Length(); ipath++)
        {
            myLaws.Value(ipath - 1).D0(Last, ref M1, ref V);
            myLaws.Value(ipath).GetDomain(out First, out Last);
            myLaws.Value(ipath).D0(First, ref M2, ref V);
            T1.SetXYZ(M1.Column(3));
            T2.SetXYZ(M2.Column(3));
            N1.SetXYZ(M1.Column(1));
            N2.SetXYZ(M2.Column(1));
            if (T1.IsParallel(T2, TolAngular) && !T1.IsOpposite(T2, TolAngular))
            { // Correction G0
                ToG0(M1, M2, out Trsf);
            }
            else
            {
                double alpha;
                gp_Vec cross = T1;
                cross.Cross(T2);
                alpha = T2.AngleWithRef(T1, cross);
                gp_Ax1 axe = new(gp.Origin(), new gp_Dir(cross.XYZ()));
                N2.Rotate(axe, alpha);

                Angle = N2.AngleWithRef(N1, T1);
                Trsf.SetRotation(OZ, Angle);
            }
            myLaws.Value(ipath).SetTrsf(Trsf);
        }
    }

    //=================================================================================================

    /// <summary>Applies a linear transformation to each elementary law so that the global law is continuous.</summary>
    public virtual void TransformInG0Law()
    {

        int ipath;
        gp_Mat M1 = new(), M2 = new(); //,Trsf
        gp_Vec V = new();
        myLaws!.Value(1).GetDomain(out double First, out double Last);
        for (ipath = 2; ipath <= myLaws.Length(); ipath++)
        {
            myLaws.Value(ipath - 1).D0(Last, ref M1, ref V);
            myLaws.Value(ipath).GetDomain(out First, out Last);
            myLaws.Value(ipath).D0(First, ref M2, ref V);
            ToG0(M1, M2, out gp_Mat aux);
            myLaws.Value(ipath).SetTrsf(aux);
        }

        // Is the law periodical ?
        if (myPath.Closed())
        {
            myLaws.Value(myLaws.Length()).D0(Last, ref M1, ref V);
            myLaws.Value(1).GetDomain(out First, out Last);
            myLaws.Value(1).D0(First, ref M2, ref V);
        }
    }

    //=======================================================================
    // function : DeleteTransform
    // purpose  : Remove the setting in continuity of law.
    //=======================================================================

    /// <summary>Removes the transformations set by <see cref="TransformInG0Law"/> or TransformInCompatibleLaw.</summary>
    public void DeleteTransform()
    {
        gp_Mat Id = new();
        Id.SetIdentity();
        for (int ii = 1; ii <= myEdges!.Length(); ii++)
        {
            myLaws!.ChangeValue(ii).SetTrsf(Id);
        }
        myDisc = null;
    }

    //=======================================================================
    // function : NbHoles
    // purpose  : Find "Holes"
    //=======================================================================

    /// <summary>The number of vertices of the path where the law isn't continuous (G0) within <paramref name="Tol"/>.</summary>
    public int NbHoles(double Tol = 1.0e-7)
    {
        if (myDisc is null)
        {
            NCollection_Sequence<int> Seq = new();
            int ii, NbDisc;
            for (ii = 2, NbDisc = -1; ii <= myLaws!.Length() + 1; ii++)
            {
                if (IsG1(ii - 1, Tol, 1.0e-12) == -1)
                {
                    Seq.Append(ii);
                }
            }
            NbDisc = Seq.Length();
            if (NbDisc > 0)
            {
                myDisc = new NCollection_HArray1<int>(1, NbDisc);
                for (ii = 1; ii <= NbDisc; ii++)
                {
                    myDisc.SetValue(ii, Seq[ii]);
                }
            }
        }
        if (myDisc is null)
        {
            return 0;
        }
        return myDisc.Length();
    }

    //=================================================================================================

    /// <summary>Fills <paramref name="Disc"/> with the indices found by <see cref="NbHoles"/>.</summary>
    public void Holes(NCollection_Array1<int> Disc)
    {
        if (myDisc is not null)
        {
            for (int ii = 1; ii <= myDisc.Length(); ii++)
            {
                Disc[ii] = myDisc.Value(ii);
            }
        }
    }

    //=================================================================================================

    /// <summary>The number of elementary laws.</summary>
    public int NbLaw() => myLaws!.Length();

    //=================================================================================================

    /// <summary>The elementary law of rank <paramref name="Index"/> (in [1, NbLaw()]): the law's own object.</summary>
    public GeomFill_LocationLaw Law(int Index) => myLaws!.Value(Index);

    //=================================================================================================

    /// <summary>The path.</summary>
    public TopoDS_Wire Wire() => myPath;

    //=================================================================================================

    /// <summary>The edge of rank <paramref name="Index"/> (in [1, NbLaw()]) of the path.</summary>
    public TopoDS_Edge Edge(int Index) => TopoDS.Edge(myEdges!.Value(Index));

    //=================================================================================================

    /// <summary>The vertex of rank <paramref name="Index"/> (in [1, NbLaw() + 1]) of the path.</summary>
    public TopoDS_Vertex Vertex(int Index)
    {
        TopoDS_Edge E;
        TopoDS_Vertex V = new();
        if (Index <= myEdges!.Length())
        {
            E = TopoDS.Edge(myEdges.Value(Index));
            if (E.Orientation() == TopAbs_REVERSED)
            {
                V = TopExp.LastVertex(E);
            }
            else
            {
                V = TopExp.FirstVertex(E);
            }
        }
        else if (Index == myEdges.Length() + 1)
        {
            E = TopoDS.Edge(myEdges.Value(Index - 1));
            if (E.Orientation() == TopAbs_REVERSED)
            {
                V = TopExp.FirstVertex(E);
            }
            else
            {
                V = TopExp.LastVertex(E);
            }
        }
        return V;
    }

    //===================================================================
    // function : PerformVertex
    // purpose  : Calculate a vertex of sweeping from a vertex of section
    //           and the index of the edge in the trajectory
    //===================================================================

    /// <summary>
    /// <paramref name="Input"/> placed by the law at the vertex of rank <paramref name="Index"/> of the path, as a new
    /// frozen vertex. <paramref name="ILoc"/> chooses the law at a discontinuity: -1 the law before the vertex, 1 the
    /// law after it, 0 the average of both (the tolerance then covers half their gap).
    /// </summary>
    public void PerformVertex(int Index, in TopoDS_Vertex Input, double TolMin, out TopoDS_Vertex Output, int ILoc = 0)
    {
        InternalPerformVertex(Index, Input, TolMin, out Output, ILoc);
        Output.TShape()?.FreezeGraph();
    }

    /// <summary><see cref="PerformVertex"/> without freezing the vertex, for the sweeps that build on it.</summary>
    internal void InternalPerformVertex(int Index, in TopoDS_Vertex Input, double TolMin, out TopoDS_Vertex Output,
                                        int ILoc = 0)
    {
        Output = new TopoDS_Vertex();
        BRep_Builder B = new();
        bool IsBary = (ILoc == 0);
        double First, Last;
        gp_Pnt P;
        gp_Vec V1 = new(), V2 = new(); //, V;
        gp_Mat M1 = new(), M2 = new();

        if (Index > 0 && Index < myLaws!.Length())
        {
            if (ILoc <= 0)
            {
                myLaws.Value(Index).GetDomain(out First, out Last);
                myLaws.Value(Index).D0(Last, ref M1, ref V1);
            }

            if (ILoc >= 0)
            {
                myLaws.Value(Index + 1).GetDomain(out First, out Last);
                if (ILoc == 0)
                {
                    myLaws.Value(Index + 1).D0(First, ref M2, ref V2);
                }
                else
                {
                    myLaws.Value(Index + 1).D0(First, ref M1, ref V1);
                }
            }
        }

        if (Index == 0 || Index == myLaws!.Length())
        {
            if (!myPath.Closed() || (IsG1(Index, TolMin) != 1))
            {
                IsBary = false;
                if (Index == 0)
                {
                    myLaws!.Value(1).GetDomain(out First, out Last);
                    myLaws.Value(1).D0(First, ref M1, ref V1);
                }
                else
                {
                    myLaws!.Value(myLaws.Length()).GetDomain(out First, out Last);
                    myLaws.Value(myLaws.Length()).D0(Last, ref M1, ref V1);
                }
            }
            else
            {
                if (ILoc <= 0)
                {
                    myLaws!.Value(myLaws.Length()).GetDomain(out First, out Last);
                    myLaws.Value(myLaws.Length()).D0(Last, ref M1, ref V1);
                }

                if (ILoc >= 0)
                {
                    myLaws!.Value(1).GetDomain(out First, out Last);
                    if (ILoc == 0)
                    {
                        myLaws.Value(1).D0(First, ref M2, ref V2);
                    }
                    else
                    {
                        myLaws.Value(1).D0(First, ref M1, ref V1);
                    }
                }
            }
        }

        P = BRep_Tool.Pnt(Input);

        if (IsBary)
        {
            gp_XYZ P1 = P.XYZ(), P2 = P.XYZ();
            P1 *= M1;
            P1 += V1.XYZ();
            P2 *= M2;
            P2 += V2.XYZ();

            P.ChangeCoord().SetLinearForm(0.5, P1, 0.5, P2);
            P1 -= P2;
            double Tol = P1.Modulus() / 2;
            Tol += TolMin;
            B.MakeVertex(ref Output, P, Tol);
        }
        else
        {
            P.ChangeCoord() *= M1;
            P.ChangeCoord() += V1.XYZ();
            B.MakeVertex(ref Output, P, TolMin);
        }
    }

    //=================================================================================================

    /// <summary>The curvilinear bounds of the law of rank <paramref name="Index"/>.</summary>
    public void CurvilinearBounds(int Index, out double First, out double Last)
    {
        First = myLength!.Value(Index);
        Last = myLength.Value(Index + 1);
        if (Last < 0)
        { // It is required to carry out the calculation
            int ii, NbE = myEdges!.Length();
            double Length;

            for (ii = 1, Length = 0.0; ii <= NbE; ii++)
            {
                myLaws!.Value(ii).GetDomain(out _, out _);
                Length += GCPnts_AbscissaPoint.Length(myLaws.Value(ii).GetCurve()!, myTol);
                myLength.SetValue(ii + 1, Length);
            }

            First = myLength.Value(Index);
            Last = myLength.Value(Index + 1);
        }
    }

    /// <summary>True if the path is closed (by its flag or by its end vertices).</summary>
    public bool IsClosed()
    {
        if (myPath.Closed())
        {
            return true;
        }

        TopExp.Vertices(myPath, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        return (V1.IsSame(V2));
    }

    //=======================================================================
    // function : IsG1
    // purpose  : Evaluate the continuity of the law by a vertex
    //=======================================================================

    /// <summary>
    /// The continuity of the law at the vertex of rank <paramref name="Index"/> between two edges of the path: -1 not
    /// connected, 0 connected (G0), 1 tangent (G1).
    /// </summary>
    public int IsG1(int Index, double SpatialTolerance = 1.0e-7, double AngularTolerance = 1.0e-4)
    {
        gp_Vec V1 = new(), DV1 = new(), V2 = new(), DV2 = new();
        gp_Mat M1 = new(), M2 = new(), DM1 = new(), DM2 = new();
        double First, Last, EpsNul = 1.0e-12;
        double TolEps = SpatialTolerance;
        bool Ok_D1 = false;
        TopoDS_Vertex V;
        TopoDS_Edge E = new();
        NCollection_Array1<gp_Pnt2d> Bid1 = new(1, 1);
        NCollection_Array1<gp_Vec2d> Bid2 = new(1, 1);

        if (Index > 0 && Index < myLaws!.Length())
        {
            myLaws.Value(Index).GetDomain(out First, out Last);
            Ok_D1 = myLaws.Value(Index).D1(Last, ref M1, ref V1, ref DM1, ref DV1, Bid1, Bid2);
            if (!Ok_D1)
            {
                myLaws.Value(Index).D0(Last, ref M1, ref V1);
            }

            myLaws.Value(Index + 1).GetDomain(out First, out Last);
            if (Ok_D1)
            {
                Ok_D1 = myLaws.Value(Index + 1).D1(First, ref M2, ref V2, ref DM2, ref DV2, Bid1, Bid2);
            }
            if (!Ok_D1)
            {
                myLaws.Value(Index + 1).D0(First, ref M2, ref V2);
            }

            E = TopoDS.Edge(myEdges!.Value(Index + 1));
        }
        if (Index == 0 || Index == myLaws!.Length())
        {
            if (!myPath.Closed())
            {
                return -1;
            }
            myLaws!.Value(myLaws.Length()).GetDomain(out First, out Last);
            Ok_D1 = myLaws.Value(myLaws.Length()).D1(Last, ref M1, ref V1, ref DM1, ref DV1, Bid1, Bid2);
            if (!Ok_D1)
            {
                myLaws.Value(myLaws.Length()).D0(Last, ref M1, ref V1);
            }

            myLaws.Value(1).GetDomain(out First, out Last);
            if (Ok_D1)
            {
                // OCCT: the result isn't kept in Ok_D1 here, unlike the branch above
                myLaws.Value(1).D1(First, ref M2, ref V2, ref DM2, ref DV2, Bid1, Bid2);
            }
            if (!Ok_D1)
            {
                myLaws.Value(1).D0(First, ref M2, ref V2);
            }

            E = TopoDS.Edge(myEdges!.Value(1));
        }

        if (E.Orientation() == TopAbs_REVERSED)
        {
            V = TopExp.LastVertex(E);
        }
        else
        {
            V = TopExp.FirstVertex(E);
        }

        TolEps += 2 * BRep_Tool.Tolerance(V);

        bool isG0 = true;
        bool isG1 = true;

        if ((V1 - V2).Magnitude() > TolEps)
        {
            isG0 = false;
        }
        if (Norm(M1 - M2) > SpatialTolerance)
        {
            isG0 = false;
        }

        if (!isG0)
        {
            return -1;
        }
        if (!Ok_D1)
        {
            return 0; // No control of the derivative
        }

        if ((DV1.Magnitude() > EpsNul) && (DV2.Magnitude() > EpsNul)
            && (DV1.Angle(DV2) > AngularTolerance))
        {
            isG1 = false;
        }

        // For the next, the tests are mostly empirical
        double Norm1 = Norm(DM1);
        double Norm2 = Norm(DM2);
        // It two 2 norms are null, it is good
        if ((Norm1 > EpsNul) || (Norm2 > EpsNul))
        {
            // otherwise the normalized matrices are compared
            if ((Norm1 > EpsNul) && (Norm2 > EpsNul))
            {
                DM1 /= Norm1;
                DM2 /= Norm2;
                if (Norm(DM1 - DM2) > AngularTolerance)
                {
                    isG1 = false;
                }
            }
            else
            {
                isG1 = false; // 1 Null the other is not
            }
        }

        if (isG1)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    //=================================================================================================

    /// <summary>
    /// The index of the law and the parameter on it of the curvilinear abscissa <paramref name="Abcissa"/>
    /// (<paramref name="Index"/> 0 when the abscissa is past the path's end; <paramref name="U"/> then unchanged).
    /// </summary>
    public void Parameter(double Abcissa, out int Index, ref double U)
    {
        int iedge, NbE = myEdges!.Length();
        bool Trouve = false;

        // Control that the lengths are calculated
        if (myLength!.Value(NbE + 1) < 0)
        {
            CurvilinearBounds(NbE, out _, out _);
        }

        // Find the interval
        for (iedge = 1; iedge <= NbE && !Trouve;)
        {
            if (myLength.Value(iedge + 1) >= Abcissa)
            {
                Trouve = true;
            }
            else
            {
                iedge++;
            }
        }

        if (Trouve)
        {
            GeomFill_LocationLaw Law = myLaws!.Value(iedge);
            Law.GetDomain(out double f, out double l);

            if (Abcissa == myLength.Value(iedge + 1))
            {
                U = l;
            }
            else if (Abcissa == myLength.Value(iedge))
            {
                U = f;
            }
            else
            {
                GCPnts_AbscissaPoint AbsC = new(myTol,
                                                myLaws.Value(iedge).GetCurve()!,
                                                Abcissa - myLength.Value(iedge),
                                                f);
                U = AbsC.Parameter();
            }
            Index = iedge;
        }
        else
        {
            Index = 0;
        }
    }

    //===================================================================
    // function : D0
    // purpose  : Position of a section, with a given curviline abscissa
    //===================================================================

    /// <summary>
    /// <paramref name="W"/> placed by the law at the curvilinear abscissa <paramref name="Abcissa"/>: a frozen
    /// transformed copy (null past the path's end).
    /// </summary>
    public void D0(double Abcissa, ref TopoDS_Shape W)
    {
        double u = 0;
        gp_Mat M = new();
        gp_Vec V = new();

        Parameter(Abcissa, out int ind, ref u);
        if (ind != 0)
        {
            // Positionement
            myLaws!.Value(ind).D0(u, ref M, ref V);
            gp_Trsf fila = new();
            fila.SetValues(M[1, 1],
                           M[1, 2],
                           M[1, 3],
                           V.X(),
                           M[2, 1],
                           M[2, 2],
                           M[2, 3],
                           V.Y(),
                           M[3, 1],
                           M[3, 2],
                           M[3, 3],
                           V.Z());
            // TopLoc_Location Loc(fila);
            // W.Location(Loc.Multiplied(W.Location()));
            W = new BRepBuilderAPI_Transform(W, fila, true).Shape(); // copy
            ///////////////////////////////////////////
        }
        else
        {
            W.Nullify();
        }
    }

    //=======================================================================
    // function : Abscissa
    // purpose  : Calculate the abscissa of a point
    //=======================================================================

    /// <summary>The curvilinear abscissa of the point at <paramref name="Param"/> on the law of rank <paramref name="Index"/>.</summary>
    public double Abscissa(int Index, double Param)
    {
        double Length = myLength!.Value(Index);
        if (Length < 0)
        {
            // OCCT: takes the edge's last bound, not its first, when the lengths aren't computed yet (occt-issues.md)
            CurvilinearBounds(Index, out _, out Length);
        }

        Length += GCPnts_AbscissaPoint.Length(myLaws!.Value(Index).GetCurve()!,
                                              myLaws.Value(Index).GetCurve()!.FirstParameter(),
                                              Param,
                                              myTol);
        return Length;
    }
}
