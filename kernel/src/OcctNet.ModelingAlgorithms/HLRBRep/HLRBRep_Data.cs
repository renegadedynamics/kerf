// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Data.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Data.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Data.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntRes2d_Position;
using static Occt.IntRes2d_Situation;
using static Occt.IntRes2d_TypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// The data structure of the exact hidden-line removal: the edges and faces of the loaded shapes with their projected
/// geometry, min-max boxes and flags, the projector, and the iterations the hider runs over it (the edges a face may
/// hide, the interferences of an edge with the face's boundary, the classification of a point by the face).
/// </summary>
/// <remarks>
/// <para>
/// A working object (a transient), as OCCT's. Its edge and face data are classes, one instance per index (decision
/// 10602); OCCT's pointers to them, and to their curves, surfaces and min-max boxes, are references to the instances
/// (the min-max boxes are reached through their owners). The projector is kept in a box that the curves and surfaces
/// share (decision 1093). OCCT's file-scope rejection table (<c>TableauRejection</c>, malloc'ed rows) is a private
/// class with growable arrays; OCCT's global intersection counters are static fields with OCCT's behaviour (traces
/// only, plan 2.3, decision 1097).
/// </para>
/// <para>
/// OCCT's <c>TableauRejection::Set</c> tests a row's index before its bound (<c>IndUV[i0][i] != -1 &amp;&amp; i &lt;
/// nbUV[i0]</c>), reading one element past a full row; the port tests the bound first, which gives the same result
/// without the read.
/// </para>
/// </remarks>
public class HLRBRep_Data : Transient
{
    // OCCT's global counters (HLRBRep_Data.cxx), read by the hider's traces (decision 1097).
    internal static int nbOkIntersection;
    internal static int nbPtIntersection;
    internal static int nbSegIntersection;
    internal static int nbClassification;
    internal static int nbCal1Intersection; // pairs of unrejected edges
    internal static int nbCal2Intersection; // true intersections (not vertex)
    internal static int nbCal3Intersection; // Curve-Surface intersections

    private const double CutLar = 2.0e-1;
    private const double CutBig = 1.0e-1;

    //-- voir HLRAlgo.cxx

    private const double DERIVEE_PREMIERE_NULLE = 0.000000000001;

    private static readonly uint[] Mask32 =
    [
        1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536, 131072, 262144,
        524288, 1048576, 2097152, 4194304, 8388608, 16777216, 33554432, 67108864, 134217728, 268435456, 536870912,
        1073741824, 2147483648U
    ];

    private const int SIZEUV = 8;

    private const int THE_MASK = unchecked((int)0x80008000);

    private sealed class TableauRejection
    {
        public double[][]? UV;  //-- UV[i][j]     contient le param (U sur Ci) de l intersection de Ci avec C(IndUV[j])
        public int[][]? IndUV;  //-- IndUV[i][j]  = J0   -> Intersection entre i et J0
        public int[]? nbUV;     //-- nbUV[i][j]   nombre de valeurs pour la ligne i
        public int N;

        public uint[][]? TabBit;
        public int nTabBit;

        public TableauRejection()
        {
            N = 0;
            nTabBit = 0;
            UV = null;
            nbUV = null;
            IndUV = null;
            TabBit = null;
        }

        public void SetDim(int n)
        {
            if (UV is not null)
            {
                Destroy();
            }
            N = n;
            UV = new double[N][];
            IndUV = new int[N][];
            nbUV = new int[N];
            int i;
            for (i = 0; i < N; i++)
            {
                UV[i] = new double[SIZEUV];
            }
            for (i = 0; i < N; i++)
            {
                IndUV[i] = new int[SIZEUV];
                for (int k = 0; k < SIZEUV; k++)
                {
                    IndUV[i][k] = -1;
                }
                nbUV[i] = SIZEUV;
            }
            InitTabBit(n);
        }

        public void Destroy()
        {
            // OCCT_DEBUG statistics left out
            if (N != 0)
            {
                ResetTabBit(N);
                IndUV = null;
                UV = null;
                nbUV = null;
                N = 0;
            }
        }

        public void Set(int i0, int j0, double u)
        {
            int[] nbUV = this.nbUV!;
            int[][] IndUV = this.IndUV!;
            double[][] UV = this.UV!;
            i0--;
            j0--;
            int k = -1;
            int i;
            for (i = 0; k == -1 && i < nbUV![i0]; i++)
            {
                if (IndUV![i0][i] == -1)
                {
                    k = i;
                }
            }
            if (k == -1)
            { //-- on agrandit le tableau
                double[] NvLigneUV = new double[nbUV[i0] + SIZEUV];
                int[] NvLigneInd = new int[nbUV[i0] + SIZEUV];
                for (i = 0; i < nbUV[i0]; i++)
                {
                    NvLigneUV[i] = UV![i0][i];
                    NvLigneInd[i] = IndUV![i0][i];
                }
                k = nbUV[i0];
                nbUV[i0] += SIZEUV;
                UV![i0] = NvLigneUV;
                IndUV![i0] = NvLigneInd;
                for (int kk = k; kk < nbUV[i0]; kk++)
                {
                    IndUV[i0][kk] = -1;
                }
            }
            IndUV![i0][k] = j0;
            UV![i0][k] = u;

            //-- tri par ordre decroissant
            bool TriOk;
            do
            {
                TriOk = true;
                int im1 = 0;
                // OCCT tests IndUV[i0][i] before i < nbUV[i0] (one read past a full row); same result
                for (i = 1; i < nbUV[i0] && IndUV[i0][i] != -1; i++, im1++)
                {
                    if (IndUV[i0][i] > IndUV[i0][im1])
                    {
                        TriOk = false;
                        k = IndUV[i0][i];
                        IndUV[i0][i] = IndUV[i0][im1];
                        IndUV[i0][im1] = k;
                        double t = UV[i0][i];
                        UV[i0][i] = UV[i0][im1];
                        UV[i0][im1] = t;
                    }
                }
            } while (!TriOk);
        }

        public double Get(int i0, int j0)
        {
            i0--;
            j0--;
            //-- for(int i=0; IndUV[i0][i]!=-1 && i<nbUV[i0]; i++) {
            //--   if(IndUV[i0][i]==j0) {
            //--     return(UV[i0][i]);
            //--   }
            //-- }
            //-- ordre decroissant
            int a = 0, b = nbUV![i0] - 1, ab;
            if (IndUV![i0][a] == -1)
            {
                return (RealLast());
            }
            if (IndUV[i0][a] == j0)
            {
                return (UV![i0][a]);
            }
            if (IndUV[i0][b] == j0)
            {
                return (UV![i0][b]);
            }
            while ((IndUV[i0][a] > j0) && (IndUV[i0][b] < j0))
            {
                ab = (a + b) >> 1;
                if (IndUV[i0][ab] < j0)
                {
                    if (b == ab)
                    {
                        return (RealLast());
                    }
                    else
                    {
                        b = ab;
                    }
                }
                else if (IndUV[i0][ab] > j0)
                {
                    if (a == ab)
                    {
                        return (RealLast());
                    }
                    else
                    {
                        a = ab;
                    }
                }
                else
                {
                    return (UV![i0][ab]);
                }
            }

            return (RealLast());
        }

        public void ResetTabBit(int nbedgs)
        {
            if (TabBit is not null)
            {
                TabBit = null;
                nTabBit = 0;
            }
        }

        public void InitTabBit(int nbedgs)
        {
            if (TabBit is not null && nTabBit != 0)
            {
                ResetTabBit(nTabBit);
            }
            TabBit = new uint[nbedgs][];
            nTabBit = nbedgs;
            int n = 1 + (nbedgs >> 5);

            for (int i = 0; i < nbedgs; i++)
            {
                TabBit[i] = new uint[n];
            }
        }

        public void SetNoIntersection(int i0, int i1)
        {
            i0--;
            i1--;
            if (i0 > i1)
            {
                int t = i0;
                i0 = i1;
                i1 = t;
            }
            int c = i1 >> 5;
            int o = i1 & 31;
            TabBit![i0][c] |= Mask32[o];
        }

        public bool NoIntersection(int i0, int i1)
        {
            i0--;
            i1--;
            if (i0 > i1)
            {
                int t = i0;
                i0 = i1;
                i1 = t;
            }
            int c = i1 >> 5;
            int o = i1 & 31;
            if ((TabBit![i0][c] & Mask32[o]) != 0)
            {
                return (true);
            }
            return (false);
        }

        public void SetIntersection(int i0, int i1, in IntRes2d_IntersectionPoint IP)
        {
            IntRes2d_Transition T1 = IP.TransitionOfFirst();
            IntRes2d_Transition T2 = IP.TransitionOfSecond();
            if (T1.PositionOnCurve() == IntRes2d_Middle)
            {
                if (T2.PositionOnCurve() == IntRes2d_Middle)
                {
                    if (T1.TransitionType() == IntRes2d_In || T1.TransitionType() == IntRes2d_Out)
                    {
                        Set(i0, i1, IP.ParamOnFirst());
                        Set(i1, i0, IP.ParamOnSecond());
                    }
                }
            }
        }

        public void GetSingleIntersection(int i0, int i1, out double u, out double v)
        {
            u = Get(i0, i1);
            if (u != RealLast())
            {
                v = Get(i1, i0);
            }
            else
            {
                v = RealLast();
            }
        }
    }

    // The z rejection of OCCT's min-max boxes: true if the encoded boxes A and B are disjoint
    private static bool ZRejected(in HLRAlgo_EdgesBlock.MinMaxIndices A, in HLRAlgo_EdgesBlock.MinMaxIndices B)
    {
        return ((A.Max[0] - B.Min[0]) & THE_MASK) != 0 || ((B.Max[0] - A.Min[0]) & THE_MASK) != 0
               || ((A.Max[1] - B.Min[1]) & THE_MASK) != 0 || ((B.Max[1] - A.Min[1]) & THE_MASK) != 0
               || ((A.Max[2] - B.Min[2]) & THE_MASK) != 0 || ((B.Max[2] - A.Min[2]) & THE_MASK) != 0
               || ((A.Max[3] - B.Min[3]) & THE_MASK) != 0 || ((B.Max[3] - A.Min[3]) & THE_MASK) != 0
               || ((A.Max[4] - B.Min[4]) & THE_MASK) != 0 || ((B.Max[4] - A.Min[4]) & THE_MASK) != 0
               || ((A.Max[5] - B.Min[5]) & THE_MASK) != 0 || ((B.Max[5] - A.Min[5]) & THE_MASK) != 0
               || ((A.Max[6] - B.Min[6]) & THE_MASK) != 0 || ((B.Max[6] - A.Min[6]) & THE_MASK) != 0
               || ((A.Max[7] - B.Min[7]) & THE_MASK) != 0;
    }

    private static void AdjustParameter(HLRBRep_EdgeData E, bool h, ref double p, out float t)
    {
        double p1, p2;
        float t1, t2;
        if (h)
        {
            E.Status().Bounds(out p, out t, out p2, out t2);
            if (E.VerAtSta())
            {
                p = p + (p2 - p) * CutBig;
            }
        }
        else
        {
            E.Status().Bounds(out p1, out t1, out p, out t);
            if (E.VerAtEnd())
            {
                p = p - (p - p1) * CutBig;
            }
        }
    }

    private readonly int myNbVertices;
    private readonly int myNbEdges;
    private readonly int myNbFaces;
    private readonly NCollection_IndexedMap<Shape> myEMap = new(ShapeHasher.Instance);
    private readonly NCollection_IndexedMap<Shape> myFMap = new(ShapeHasher.Instance);
    private readonly NCollection_Array1<HLRBRep_EdgeData> myEData;
    private readonly NCollection_Array1<HLRBRep_FaceData> myFData;
    private readonly NCollection_Array1<int> myEdgeIndices;
    private float myToler;
    private readonly StrongBox<HLRAlgo_Projector> myProj = new(new HLRAlgo_Projector());
    private readonly HLRBRep_CLProps myLLProps;
    private readonly HLRBRep_CLProps myFLProps;
    private readonly HLRBRep_SLProps mySLProps;
    private double myBigSize;
    private readonly HLRBRep_FaceIterator myFaceItr1 = new();
    private readonly HLRBRep_FaceIterator myFaceItr2 = new();
    private int iFace;
    private HLRBRep_FaceData? iFaceData;
    private HLRBRep_Surface? iFaceGeom;
    private HLRAlgo_WiresBlock? iFaceWires; // OCCT's iFaceMinMax points into its min-max
    private SurfaceKind iFaceType;
    private bool iFaceBack;
    private bool iFaceSimp;
    private bool iFaceSmpl;
    private bool iFaceTest;
    private int myHideCount;
    private readonly double[] myDeca = new double[16];
    private readonly double[] mySurD = new double[16];
    private int myCurSortEd;
    private int myNbrSortEd;
    private int myLE;
    private bool myLEOutLine;
    private bool myLEInternal;
    private bool myLEDouble;
    private bool myLEIsoLine;
    private HLRBRep_EdgeData? myLEData;
    private HLRBRep_Curve? myLEGeom;
    private HLRBRep_EdgeData? myLEMinMaxOwner; // OCCT's myLEMinMax points into its min-max
    private CurveKind myLEType;
    private float myLETol;
    private int myFE;
    private Orientation myFEOri;
    private bool myFEOutLine;
    private bool myFEInternal;
    private bool myFEDouble;
    private HLRBRep_EdgeData? myFEData;
    private HLRBRep_Curve? myFEGeom;
    private CurveKind myFEType;
    private float myFETol;
    private readonly HLRBRep_Intersector myIntersector = new();
    private BRepTopAdaptor_TopolTool? myClassifier;
    private bool mySameVertex;
    private bool myIntersected;
    private int myNbPoints;
    private int myNbSegments;
    private int iInterf;
    private HLRAlgo_Interference myIntf;
    private bool myAboveIntf;
    private TableauRejection? myReject;

    private ref HLRAlgo_EdgesBlock.MinMaxIndices iFaceMinMax => ref iFaceWires!.MinMax();

    private ref HLRAlgo_EdgesBlock.MinMaxIndices myLEMinMax => ref myLEMinMaxOwner!.MinMax();

    /// <summary>A data structure for NV vertices, NE edges and NF faces (indexed from 1; index 0 is unused).</summary>
    public HLRBRep_Data(int NV, int NE, int NF)
    {
        myNbVertices = NV;
        myNbEdges = NE;
        myNbFaces = NF;
        myEData = new NCollection_Array1<HLRBRep_EdgeData>(0, NE);
        for (int i = 0; i <= NE; i++)
        {
            myEData[i] = new HLRBRep_EdgeData();
        }
        myFData = new NCollection_Array1<HLRBRep_FaceData>(0, NF);
        for (int i = 0; i <= NF; i++)
        {
            myFData[i] = new HLRBRep_FaceData();
        }
        myEdgeIndices = new NCollection_Array1<int>(0, NE);
        myToler = (float)1e-5;
        myLLProps = new HLRBRep_CLProps(2, Epsilon(1.0));
        myFLProps = new HLRBRep_CLProps(2, Epsilon(1.0));
        mySLProps = new HLRBRep_SLProps(2, Epsilon(1.0));
        myHideCount = 0;
        myReject = new TableauRejection();
        myReject.SetDim(myNbEdges);
    }

    /// <summary>Drops the rejection table (OCCT's destructor calls it).</summary>
    public void Destroy()
    {
        myReject?.Destroy();
        myReject = null;
    }

    /// <summary>Copies the edges and faces of DS here, shifted by dv vertices, de edges and df faces.</summary>
    public void Write(HLRBRep_Data DS, int dv, int de, int df)
    {
        int n1edge = DS.NbEdges();
        int n1face = DS.NbFaces();

        for (int iedge = 1; iedge <= n1edge; iedge++)
        {
            HLRBRep_EdgeData ed = myEData[de + iedge];
            HLRBRep_EdgeData e1 = DS.EDataArray()[iedge];
            ed.Assign(e1);

            if (dv != 0)
            {
                ed.VSta(ed.VSta() + dv);
                ed.VEnd(ed.VEnd() + dv);
            }

            myEMap.Add(DS.EdgeMap().FindKey(iedge));
        }

        for (int iface = 1; iface <= n1face; iface++)
        {
            HLRBRep_FaceData fd = myFData[df + iface];
            HLRBRep_FaceData f1 = DS.FDataArray()[iface];
            fd.Assign(f1);

            if (de != 0)
            {
                // OCCT: the wires block is shared with DS's face, whose edge indices are shifted too
                HLRAlgo_WiresBlock wb = fd.Wires()!;
                int nw = wb.NbWires();

                for (int iw = 1; iw <= nw; iw++)
                {
                    HLRAlgo_EdgesBlock eb = wb.Wire(iw)!;
                    int ne = eb.NbEdges();

                    for (int ie = 1; ie <= ne; ie++)
                    {
                        eb.Edge(ie, eb.Edge(ie) + de);
                    }
                }
            }

            myFMap.Add(DS.FaceMap().FindKey(iface));
        }
    }

    public NCollection_Array1<HLRBRep_EdgeData> EDataArray() => myEData;

    public NCollection_Array1<HLRBRep_FaceData> FDataArray() => myFData;

    public void Tolerance(float tol) => myToler = tol;

    public float Tolerance() => myToler;

    /// <summary>
    /// Sets the projector and computes everything that depends on it: the curves' types and min-max boxes, the edges'
    /// vertical flags, the faces' flags (side, back, hiding, simple, cut), orientations of the outlines, and boxes.
    /// </summary>
    public void Update(in HLRAlgo_Projector P)
    {
        myProj.Value = P;
        Transform3D T = myProj.Value.Transformation();
        int i;
        double tolMinMax = 0;

        HLRAlgo_EdgesBlock.MinMaxIndices FaceMin = new(), FaceMax = new();
        HLRAlgo_EdgesBlock.MinMaxIndices MinMaxFace = new();
        HLRAlgo_EdgesBlock.MinMaxIndices WireMin = new(), WireMax = new(), MinMaxWire = new();
        HLRAlgo_EdgesBlock.MinMaxIndices EdgeMin = new(), EdgeMax = new();
        HLRAlgo_EdgesBlock.MinMaxIndices MinMaxEdge = new();
        Span<double> TotMin = stackalloc double[16];
        Span<double> TotMax = stackalloc double[16];
        HLRAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), TotMin, TotMax);

        // compute the global MinMax
        // *************************
        //  int nbEdge = myEdges.Extent();
        int edge;
        for (edge = 1; edge <= myNbEdges; edge++)
        {
            HLRBRep_EdgeData ed = myEData[edge];
            HLRBRep_Curve EC = ed.ChangeGeometry();
            EC.Projector(myProj);
            double enl = EC.Update(TotMin, TotMax);
            if (enl > tolMinMax)
            {
                tolMinMax = enl;
            }
        }
        HLRAlgo.EnlargeMinMax(tolMinMax, TotMin, TotMax);
        Span<double> d = stackalloc double[16];
        double precad = -global::Kerf.Engine.Foundation.Precision.Infinite();

        for (i = 0; i <= 15; i++)
        {
            d[i] = TotMax[i] - TotMin[i];
            if (precad < d[i])
            {
                precad = d[i];
            }
        }
        myBigSize = precad;
        precad = precad * 0.0005;

        for (i = 0; i <= 15; i++)
        {
            mySurD[i] = 0x00007fff / (d[i] + precad);
        }
        precad = precad * 0.5;

        for (i = 0; i <= 15; i++)
        {
            myDeca[i] = -TotMin[i] + precad;
        }

        double tol;
        bool ver1, ver2;

        // update the edges
        // ****************

        for (edge = 1; edge <= myNbEdges; edge++)
        {

            HLRBRep_EdgeData ed = myEData[edge];
            HLRBRep_Curve EC = ed.ChangeGeometry();
            HLRAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), TotMin, TotMax);
            tolMinMax = EC.UpdateMinMax(TotMin, TotMax);
            tol = (double)(ed.Tolerance());
            ed.Vertical(TotMax[0] - TotMin[0] < tol && TotMax[1] - TotMin[1] < tol && TotMax[2] - TotMin[2] < tol
                        && TotMax[3] - TotMin[3] < tol && TotMax[4] - TotMin[4] < tol && TotMax[5] - TotMin[5] < tol
                        && TotMax[6] - TotMin[6] < tol);
            HLRAlgo.EnlargeMinMax(tolMinMax, TotMin, TotMax);
            // Linux warning :  assignment to `int' from `double'. Cast has been added.
            REJECT1(myDeca, TotMin, TotMax, mySurD, ref EdgeMin, ref EdgeMax);
            HLRAlgo.EncodeMinMax(ref EdgeMin, ref EdgeMax, ref MinMaxEdge);
            ed.UpdateMinMax(MinMaxEdge);
            if (ed.Vertical())
            {
                ver1 = true;
                ver2 = true;
                int vsta = ed.VSta();
                int vend = ed.VEnd();
                bool vout = ed.OutLVSta() || ed.OutLVEnd();
                bool vcut = ed.CutAtSta() || ed.CutAtEnd();

                for (int ebis = 1; ebis <= myNbEdges; ebis++)
                {
                    HLRBRep_EdgeData eb = myEData[ebis];
                    if (vsta == eb.VSta())
                    {
                        eb.VSta(vend);
                        eb.OutLVSta(vout);
                        eb.CutAtSta(vcut);
                    }
                    else if (vsta == eb.VEnd())
                    {
                        eb.VEnd(vend);
                        eb.OutLVEnd(vout);
                        eb.CutAtEnd(vcut);
                    }
                }
            }
            else
            {
                EC.D1(EC.Parameter3d(EC.FirstParameter()), out Point3D Pt, out Vector3D Tg1);
                EC.D1(EC.Parameter3d(EC.LastParameter()), out Pt, out Vector3D Tg2);
                Tg1.Transform(T);
                Tg2.Transform(T);
                if (Abs(Tg1.X()) + Abs(Tg1.Y()) < myToler * 10)
                {
                    ver1 = true;
                }
                else
                {
                    Direction3D Dir1 = new(Tg1);
                    ver1 = Abs(Dir1.X()) + Abs(Dir1.Y()) < myToler * 10;
                }
                if (Abs(Tg2.X()) + Abs(Tg2.Y()) < myToler * 10)
                {
                    ver2 = true;
                }
                else
                {
                    Direction3D Dir2 = new(Tg2);
                    ver2 = Abs(Dir2.X()) + Abs(Dir2.Y()) < myToler * 10;
                }
            }
            ed.VerAtSta(ed.Vertical() || ver1);
            ed.VerAtEnd(ed.Vertical() || ver2);
            ed.AutoIntersectionDone(true);
            ed.Simple(true);
        }

        // update the faces
        // ****************

        for (int face = 1; face <= myNbFaces; face++)
        {

            HLRBRep_FaceData fd = myFData[face];
            HLRBRep_Surface FS = fd.Geometry();
            iFaceGeom = fd.Geometry();
            mySLProps.SetSurface(iFaceGeom);
            FS.Projector(myProj);
            iFaceType = FS.GetType();

            // Is the face cut by an outline

            bool cut = false;
            bool withOutL = false;

            for (myFaceItr1.InitEdge(fd); myFaceItr1.MoreEdge(); myFaceItr1.NextEdge())
            {
                if (myFaceItr1.Internal())
                {
                    withOutL = true;
                    cut = true;
                }
                else if (myFaceItr1.OutLine())
                {
                    withOutL = true;
                    if (myFaceItr1.Double())
                    {
                        cut = true;
                    }
                }
            }
            fd.Cut(cut);
            fd.WithOutL(withOutL);

            // Is the face simple = no auto-hiding
            // not cut and simple surface

            if (!withOutL
                && (iFaceType == Plane || iFaceType == SurfaceKind.Cylinder || iFaceType == SurfaceKind.Cone
                    || iFaceType == SurfaceKind.Sphere || iFaceType == SurfaceKind.Torus))
            {
                fd.Simple(true);
            }
            else
            {
                fd.Simple(false);
            }

            fd.Plane(iFaceType == Plane);
            fd.Cylinder(iFaceType == SurfaceKind.Cylinder);
            fd.Cone(iFaceType == SurfaceKind.Cone);
            fd.Sphere(iFaceType == SurfaceKind.Sphere);
            fd.Torus(iFaceType == SurfaceKind.Torus);
            tol = (double)(fd.Tolerance());
            fd.Side(FS.IsSide(tol, myToler * 10));
            bool inverted = false;
            if (fd.WithOutL() && !fd.Side())
            {
                inverted = OrientOutLine(face, fd);
                OrientOthEdge(face, fd);
            }
            if (fd.Side())
            {
                fd.Hiding(false);
                fd.Back(false);
            }
            else if (!fd.WithOutL())
            {
                double p, pu, pv, r;
                fd.Back(false);
                bool found = false;

                for (myFaceItr1.InitEdge(fd); myFaceItr1.MoreEdge() && !found; myFaceItr1.NextEdge())
                {
                    myFE = myFaceItr1.Edge();
                    myFEOri = myFaceItr1.Orientation();
                    myFEOutLine = myFaceItr1.OutLine();
                    myFEInternal = myFaceItr1.Internal();
                    myFEDouble = myFaceItr1.Double();
                    HLRBRep_EdgeData EDataFE1 = myEData[myFE];
                    if (!myFEDouble && (myFEOri == Forward || myFEOri == Reversed))
                    {
                        myFEGeom = EDataFE1.ChangeGeometry();
                        HLRBRep_Curve EC = EDataFE1.Geometry();
                        p = EC.Parameter3d((EC.LastParameter() + EC.FirstParameter()) / 2);
                        if (HLRBRep_EdgeFaceTool.UVPoint(p, myFEGeom, iFaceGeom, out pu, out pv))
                        {
                            mySLProps.SetParameters(pu, pv);
                            Point3D Pt;
                            Pt = EC.Value3D(p);
                            if (mySLProps.IsNormalDefined())
                            {
                                Vector3D Nm = new(mySLProps.Normal());
                                Pt.Transform(T);
                                Nm.Transform(T);
                                if (myProj.Value.Perspective())
                                {
                                    r = Nm.Z() * myProj.Value.Focus() - (Nm.X() * Pt.X() + Nm.Y() * Pt.Y() + Nm.Z() * Pt.Z());
                                }
                                else
                                {
                                    r = Nm.Z();
                                }
                                if (Abs(r) > myToler * 10)
                                {
                                    fd.Back(r < 0);
                                    found = true;
                                    break;
                                }
                            }
                        }
                    }
                }

                if (!found)
                {
                    fd.Side(true);
                    fd.Hiding(false);
                    fd.Back(false);
                }
                else if (fd.Closed())
                {
                    switch (fd.Orientation())
                    {
                        case Reversed:
                            fd.Hiding(fd.Back());
                            break;
                        case Forward:
                            fd.Hiding(!fd.Back());
                            break;
                        case External:
                            fd.Hiding(true);
                            break;
                        case Internal:
                            fd.Hiding(false);
                            break;
                    }
                }
                else
                {
                    fd.Hiding(true);
                }
            }
            else
            {
                if (inverted)
                {
                    fd.Hiding(false);
                    fd.Back(true);
                }
                else
                {
                    fd.Hiding(true);
                    fd.Back(false);
                }
            }

            bool FirstTime = true;

            for (myFaceItr1.InitEdge(fd); myFaceItr1.MoreEdge(); myFaceItr1.NextEdge())
            {
                myFE = myFaceItr1.Edge();
                HLRBRep_EdgeData EDataFE2 = myEData[myFE];
                if (!fd.Simple())
                {
                    EDataFE2.AutoIntersectionDone(false);
                }
                HLRAlgo.DecodeMinMax(EDataFE2.MinMax(), ref EdgeMin, ref EdgeMax);
                if (myFaceItr1.BeginningOfWire())
                {
                    HLRAlgo.CopyMinMax(ref EdgeMin, ref EdgeMax, ref WireMin, ref WireMax);
                }
                else
                {
                    HLRAlgo.AddMinMax(ref EdgeMin, ref EdgeMax, ref WireMin, ref WireMax);
                }
                if (myFaceItr1.EndOfWire())
                {
                    HLRAlgo.EncodeMinMax(ref WireMin, ref WireMax, ref MinMaxWire);
                    myFaceItr1.Wire().UpdateMinMax(MinMaxWire);
                    if (FirstTime)
                    {
                        FirstTime = false;
                        HLRAlgo.CopyMinMax(ref WireMin, ref WireMax, ref FaceMin, ref FaceMax);
                    }
                    else
                    {
                        HLRAlgo.AddMinMax(ref WireMin, ref WireMax, ref FaceMin, ref FaceMax);
                    }
                }
            }
            HLRAlgo.EncodeMinMax(ref FaceMin, ref FaceMax, ref MinMaxFace);
            fd.Wires()!.UpdateMinMax(MinMaxFace);
            fd.Size(HLRAlgo.SizeBox(ref FaceMin, ref FaceMax));
        }
    }

    /// <summary>The projector (a reference to the one the curves and surfaces share, as OCCT's).</summary>
    public ref HLRAlgo_Projector Projector() => ref myProj.Value;

    public int NbVertices() => myNbVertices;

    public int NbEdges() => myNbEdges;

    public int NbFaces() => myNbFaces;

    public NCollection_IndexedMap<Shape> EdgeMap() => myEMap;

    public NCollection_IndexedMap<Shape> FaceMap() => myFMap;

    /// <summary>Selects the edges e1 to e2 not all hidden whose boxes meet MinMaxTot, for the faces to hide.</summary>
    public void InitBoundSort(in HLRAlgo_EdgesBlock.MinMaxIndices MinMaxTot, int e1, int e2)
    {
        myNbrSortEd = 0;
        HLRAlgo_EdgesBlock.MinMaxIndices MinMaxShap = MinMaxTot;

        for (int e = e1; e <= e2; e++)
        {
            HLRBRep_EdgeData ed = myEData[e];
            if (!ed.Status().AllHidden())
            {
                myLEMinMaxOwner = ed;
                if (!ZRejected(MinMaxShap, myLEMinMax))
                { //- rejection en z
                    myNbrSortEd++;
                    myEdgeIndices[myNbrSortEd] = e;
                }
            }
        }
    }

    /// <summary>Starts the iteration on the edges the face FI may hide; MST caches the faces' classifiers.</summary>
    public void InitEdge(int FI, NCollection_DataMap<Shape, BRepTopAdaptor_Tool> MST)
    {
        myHideCount++;
        myHideCount++;

        iFace = FI;
        iFaceData = myFData[iFace];
        iFaceGeom = iFaceData.Geometry();
        iFaceBack = iFaceData.Back();
        iFaceSimp = iFaceData.Simple();
        iFaceWires = iFaceData.Wires();
        iFaceType = iFaceGeom.GetType();
        iFaceTest = !iFaceSimp;
        mySLProps.SetSurface(iFaceGeom);
        myIntersector.Load(iFaceGeom);

        HLRBRep_Surface p1 = iFaceGeom;
        FaceSurfaceAdaptor bras = p1.Surface();
        Face topodsface = bras.Face();

        if (MST.IsBound(topodsface))
        {
            BRepTopAdaptor_Tool BRT = MST.ChangeFind(topodsface);
            myClassifier = BRT.GetTopolTool();
        }
        else
        {
            BRepTopAdaptor_Tool BRT = new(topodsface, global::Kerf.Engine.Foundation.Precision.PConfusion());
            MST.Bind(topodsface, BRT);
            myClassifier = BRT.GetTopolTool();
        }

        if (iFaceTest)
        {
            iFaceSmpl = !iFaceData.Cut();
            myFaceItr2.InitEdge(iFaceData);
        }
        else
        {

            for (myFaceItr1.InitEdge(iFaceData); myFaceItr1.MoreEdge(); myFaceItr1.NextEdge())
            {
                myFE = myFaceItr1.Edge();                // edges of a simple hiding
                myEData[myFE].HideCount(myHideCount - 1); // face must be jumped.
            }
            myCurSortEd = 1;
        }
        NextEdge(false);
    }

    public bool MoreEdge()
    {
        if (iFaceTest)
        {
            if (myFaceItr2.MoreEdge())
            {                                         // all edges must be tested if
                myLE = myFaceItr2.Edge();             // the face is not a simple
                myLEOutLine = myFaceItr2.OutLine();   // one.
                myLEInternal = myFaceItr2.Internal();
                myLEDouble = myFaceItr2.Double();
                myLEIsoLine = myFaceItr2.IsoLine();
                myLEData = myEData[myLE];
                myLEGeom = myLEData.ChangeGeometry();
                myLEMinMaxOwner = myLEData;
                myLETol = myLEData.Tolerance();
                myLEType = myLEGeom.GetType();
                if (!myLEDouble)
                {
                    myLEData.HideCount(myHideCount - 1);
                }
                return true;
            }
            else
            {
                iFaceTest = false;     // at the end of the test
                iFaceSimp = iFaceSmpl; // we know if it is a simple face
                iFaceData!.Simple(iFaceSimp);
                myCurSortEd = 1;
                NextEdge(false);
            }
        }
        return myCurSortEd <= myNbrSortEd;
    }

    public void NextEdge(bool skip = true)
    {

        if (skip)
        {
            if (iFaceTest)
            {
                myFaceItr2.NextEdge();
            }
            else
            {
                myCurSortEd++;
            }
        }
        if (!MoreEdge())
        {
            return;
        }
        if (iFaceTest)
        {
            myLE = myFaceItr2.Edge();
            myLEOutLine = myFaceItr2.OutLine();
            myLEInternal = myFaceItr2.Internal();
            myLEDouble = myFaceItr2.Double();
            myLEIsoLine = myFaceItr2.IsoLine();
            myLEData = myEData[myLE];
            myLEGeom = myLEData.ChangeGeometry();
            myLEMinMaxOwner = myLEData;
            myLETol = myLEData.Tolerance();
            myLEType = myLEGeom.GetType();
            if (myLEData.Vertical() || (myLEDouble && myLEData.HideCount() == myHideCount - 1))
            {
                NextEdge();
            }
            myLEData.HideCount(myHideCount - 1);
            return;
        }
        else
        {
            myLE = Edge();
            myLEOutLine = false;
            myLEInternal = false;
            myLEDouble = false;
            myLEIsoLine = false;
            myLEData = myEData[myLE];
            myLEGeom = myLEData.ChangeGeometry();
            myLEMinMaxOwner = myLEData;
            myLETol = myLEData.Tolerance();
            myLEType = myLEGeom.GetType();
        }
        if (myLEData.Vertical())
        {
            NextEdge();
            return;
        }
        if (myLEData.HideCount() > myHideCount - 2)
        {
            NextEdge();
            return;
        }
        if (myLEData.Status().AllHidden())
        {
            NextEdge();
            return;
        }
        if (ZRejected(iFaceMinMax, myLEMinMax))
        { //-- rejection en z
            NextEdge();
            return;
        }
        if (iFaceGeom!.IsAbove(iFaceBack, myLEGeom, (double)myLETol))
        {
            NextEdge();
            return;
        }
        return; // edge is OK
    }

    public int Edge()
    {
        if (iFaceTest)
        {
            return myFaceItr2.Edge();
        }
        else
        {
            return myEdgeIndices[myCurSortEd];
        }
    }

    public bool HidingTheFace() => iFaceTest;

    public bool SimpleHidingFace() => iFaceSimp;

    /// <summary>Starts the iteration on the interferences of the current edge with the face's boundary.</summary>
    public void InitInterference()
    {
        myLLProps.SetCurve(myLEGeom!);
        myFaceItr1.InitEdge(iFaceData!);
        myNbPoints = myNbSegments = iInterf = 0;
        NextInterference();
    }

    public bool MoreInterference() => (iInterf <= myNbPoints + 2 * myNbSegments);

    public void NextInterference()
    {
        // are there more intersections on the current edge
        iInterf++;
        //    Standard_Integer miniWire1,miniWire2;
        //    Standard_Integer maxiWire1,maxiWire2,maxiWire3,maxiWire4;

        while (!MoreInterference() && myFaceItr1.MoreEdge())
        {

            // rejection of current wire
            if (myFaceItr1.BeginningOfWire())
            {
                ref HLRAlgo_EdgesBlock.MinMaxIndices MinMaxWire = ref myFaceItr1.Wire().MinMax();
                if (ZRejected(MinMaxWire, myLEMinMax))
                { //-- Rejection en Z
                    myFaceItr1.SkipWire();
                    continue;
                }
            }
            myFE = myFaceItr1.Edge();
            myFEOri = myFaceItr1.Orientation();
            myFEOutLine = myFaceItr1.OutLine();
            myFEInternal = myFaceItr1.Internal();
            myFEDouble = myFaceItr1.Double();
            myFEData = myEData[myFE];
            myFEGeom = myFEData.ChangeGeometry();
            myFETol = myFEData.Tolerance();
            myFEType = myFEGeom.GetType();

            if (myFEOri == Forward || myFEOri == Reversed)
            {
                // Edge from the boundary
                if (!myFEData.Vertical() && (!myFEDouble || myFEOutLine))
                {
                    // not a vertical edge and not a double Edge
                    ref HLRAlgo_EdgesBlock.MinMaxIndices MinMaxFEdg = ref myFEData.MinMax();
                    //-- -----------------------------------------------------------------------
                    //-- Max - Min doit etre positif pour toutes les directions
                    //--
                    //-- Rejection 1   (FEMax-LEMin)& 0x80008000  !=0
                    //--
                    //--                   FE Min ...........  FE Max
                    //--                                                LE Min ....   LE Max
                    //--
                    //-- Rejection 2   (LEMax-FEMin)& 0x80008000  !=0
                    //--                            FE Min ...........  FE Max
                    //--     LE Min ....   LE Max
                    //-- ----------------------------------------------------------------------

                    if (!myReject!.NoIntersection(myLE, myFE))
                    {
                        if (!ZRejected(MinMaxFEdg, myLEMinMax))
                        { //-- Rejection en Z
                            bool rej = false;
                            if (myLE == myFE)
                            { // test if an auto-intersection is not useful
                                if (myLEData!.AutoIntersectionDone())
                                {
                                    myLEData.AutoIntersectionDone(true);
                                    if (myLEData.Simple())
                                    {
                                        rej = true;
                                    }
                                }
                            }

                            if (!rej)
                            {
                                nbCal1Intersection++;
                                bool h1 = false;
                                bool e1 = false;
                                bool h2 = false;
                                bool e2 = false;
                                mySameVertex = false;

                                if (myLE == myFE)
                                {
                                    myIntersected = true;
                                    mySameVertex = false;
                                }
                                else
                                {
                                    myIntersected = true;
                                    if (SameVertex(true, true))
                                    {
                                        mySameVertex = true;
                                        h1 = true;
                                        h2 = true;
                                    }
                                    if (SameVertex(true, false))
                                    {
                                        mySameVertex = true;
                                        h1 = true;
                                        e2 = true;
                                    }
                                    if (SameVertex(false, true))
                                    {
                                        mySameVertex = true;
                                        e1 = true;
                                        h2 = true;
                                    }
                                    if (SameVertex(false, false))
                                    {
                                        mySameVertex = true;
                                        e1 = true;
                                        e2 = true;
                                    }
                                }

                                myNbPoints = myNbSegments = 0;
                                iInterf = 1;

                                if (myIntersected)
                                { // compute real intersection
                                    nbCal2Intersection++;

                                    double da1 = 0;
                                    double db1 = 0;
                                    double da2 = 0;
                                    double db2 = 0;

                                    if (mySameVertex || myLE == myFE)
                                    {
                                        if (h1)
                                        {
                                            da1 = CutLar;
                                        }
                                        if (e1)
                                        {
                                            db1 = CutLar;
                                        }
                                        if (h2)
                                        {
                                            da2 = CutLar;
                                        }
                                        if (e2)
                                        {
                                            db2 = CutLar;
                                        }
                                    }
                                    int NoInter = 0;
                                    if (myLE == myFE)
                                    {
                                        myIntersector.Perform(myLEData!, da1, db1);
                                    }
                                    else
                                    {
                                        myReject.GetSingleIntersection(myLE, myFE, out double su, out double sv);
                                        if (su != RealLast())
                                        {
                                            myIntersector.SimulateOnePoint(myLEData!, su, myFEData, sv);
                                            //-- printf("\n Single Point  : %d %d   %g %g\n",myLE,myFE,su,sv);
                                        }
                                        else
                                        {
                                            myIntersector.Perform(myLE, myLEData!, da1, db1, myFE, myFEData, da2, db2,
                                                                  mySameVertex);
                                            if (myIntersector.IsDone())
                                            {
                                                if (myIntersector.NbPoints() == 1 && myIntersector.NbSegments() == 0)
                                                {
                                                    myReject.SetIntersection(myLE, myFE, myIntersector.Point(1));
                                                }
                                            }
                                        }
                                        NoInter = 0;
                                    }
                                    if (NoInter != 0)
                                    {
                                        myNbPoints = myNbSegments = 0;
                                    }
                                    else
                                    {
                                        if (myIntersector.IsDone())
                                        {
                                            myNbPoints = myIntersector.NbPoints();
                                            myNbSegments = myIntersector.NbSegments();
                                            if ((myNbSegments + myNbPoints) > 0)
                                            {
                                                nbOkIntersection++;
                                            }
                                            else
                                            {
                                                myReject.SetNoIntersection(myLE, myFE);
                                            }
                                        }
                                        else
                                        {
                                            myNbPoints = myNbSegments = 0;
                                            // OCCT_DEBUG trace left out
                                        }
                                    }
                                }
                                nbPtIntersection += myNbPoints;
                                nbSegIntersection += myNbSegments;
                            }
                        }
                        else
                        {
                        }
                    }
                    else
                    {
                    }
                }
            }
            // next edge in face
            myFaceItr1.NextEdge();
        }
    }

    /// <summary>
    /// Builds the current interference; true if it is rejected (above the face, or not crossing it as the hiding
    /// needs).
    /// </summary>
    public bool RejectedInterference()
    {
        if (iInterf <= myNbPoints)
        {
            return RejectedPoint(myIntersector.Point(iInterf), External, 0);
        }
        else
        {
            int n = iInterf - myNbPoints;
            bool firstPoint = (n & 1) != 0;
            int nseg = n >> 1;
            if (firstPoint)
            {
                nseg++;
            }
            double pf = myLEGeom!.Parameter3d(myIntersector.Segment(nseg).FirstPoint().ParamOnFirst());
            double pl = myLEGeom.Parameter3d(myIntersector.Segment(nseg).LastPoint().ParamOnFirst());
            if (pf > pl)
            {
                firstPoint = !firstPoint;
            }

            if (firstPoint)
            {
                bool ret1 = RejectedPoint(myIntersector.Segment(nseg).FirstPoint(), Forward, nseg);
                return (ret1);
            }
            else
            {
                bool ret2 = RejectedPoint(myIntersector.Segment(nseg).LastPoint(), Reversed, -nseg);
                return (ret2);
            }
        }
    }

    /// <summary>True if the last interference built is above the face.</summary>
    public bool AboveInterference() => myAboveIntf;

    /// <summary>The current interference (a reference, as OCCT's).</summary>
    public ref HLRAlgo_Interference Interference() => ref myIntf;

    /// <summary>The tangent, normal and curvature of the current edge at Param, in the projection plane.</summary>
    /// <exception cref="FailureException">The tangent is undefined.</exception>
    public void LocalLEGeometry2D(double Param, out Direction2D Tg, out Direction2D Nm, out double Cu)
    {
        myLLProps.SetParameter(Param);
        if (!myLLProps.IsTangentDefined())
        {
            throw new FailureException("HLRBRep_Data::LocalGeometry2D");
        }
        myLLProps.Tangent(out Tg);
        Cu = myLLProps.Curvature();
        if (Cu > Epsilon(1.0) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(Cu))
        {
            myLLProps.Normal(out Nm);
        }
        else
        {
            Nm = new Direction2D(-Tg.Y(), Tg.X());
        }
    }

    /// <summary>The tangent, normal and curvature of the edge FE at Param, in the projection plane.</summary>
    /// <exception cref="FailureException">The tangent is undefined.</exception>
    public void LocalFEGeometry2D(int FE, double Param, out Direction2D Tg, out Direction2D Nm, out double Cu)
    {
        HLRBRep_Curve aCurve = myEData[FE].ChangeGeometry();
        myFLProps.SetCurve(aCurve);
        myFLProps.SetParameter(Param);
        if (!myFLProps.IsTangentDefined())
        {
            throw new FailureException("HLRBRep_Data::LocalGeometry2D");
        }
        myFLProps.Tangent(out Tg);
        Cu = myFLProps.Curvature();
        if (Cu > Epsilon(1.0) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(Cu))
        {
            myFLProps.Normal(out Nm);
        }
        else
        {
            Nm = new Direction2D(-Tg.Y(), Tg.X());
        }
    }

    /// <summary>The 3D states of the current edge before and after the point p1 where it is on the face boundary.</summary>
    public void EdgeState(double p1, double p2, out Containment stbef, out Containment staft)
    {
        // compute the state of The Edge near the Intersection
        // this method should give the states before and after
        // it should get the parameters on the surface

        double pu, pv;
        if (HLRBRep_EdgeFaceTool.UVPoint(p2, myFEGeom!, iFaceGeom!, out pu, out pv))
        {
            mySLProps.SetParameters(pu, pv);
            if (mySLProps.IsNormalDefined())
            {
                Direction3D NrmFace = mySLProps.Normal();

                myLEGeom!.D1(p1, out Point3D Pbid, out Vector3D TngEdge);

                Transform3D TI = myProj.Value.InvertedTransformation();
                Direction3D V;
                if (myProj.Value.Perspective())
                {
                    Point2D P2d = new();
                    myProj.Value.Project(Pbid, ref P2d);
                    V = new Direction3D(P2d.X(), P2d.Y(), -myProj.Value.Focus());
                }
                else
                {
                    V = new Direction3D(Direction3D.D.NZ);
                }
                V.Transform(TI);
                if (NrmFace.Dot(V) > 0.0)
                {
                    NrmFace.Reverse();
                }

                double scal = (TngEdge.SquareMagnitude() > 1.0e-10) ? NrmFace.Dot(new Direction3D(TngEdge)) : 0.0;

                if (scal > myToler * 10)
                {
                    stbef = In;
                    staft = Out;
                }
                else if (scal < -myToler * 10)
                {
                    stbef = Out;
                    staft = In;
                }
                else
                {
                    stbef = On;
                    staft = On;
                }
            }
            else
            {
                stbef = Out;
                staft = Out;
                // OCCT_DEBUG trace left out
            }
        }
        else
        {
            stbef = Out;
            staft = Out;
            // OCCT_DEBUG trace left out
        }
    }

    /// <summary>True if the edge ED is an edge of the face hiding it (its hiding count).</summary>
    public bool EdgeOfTheHidingFace(int E, HLRBRep_EdgeData ED) => ED.HideCount() == myHideCount - 1;

    /// <summary>The number of times the start of the edge is hidden by the face, from its interferences IL.</summary>
    public int HidingStartLevel(int E, HLRBRep_EdgeData ED, NCollection_List<HLRAlgo_Interference> IL)
    {
        bool Loop;
        NCollection_List<HLRAlgo_Interference>.Iterator It = new();
        HLRBRep_Curve EC = ED.Geometry();
        double sta = EC.Parameter3d(EC.FirstParameter());
        double end = EC.Parameter3d(EC.LastParameter());
        double tolpar = (end - sta) * 0.01;
        double param;
        Loop = true;
        It.Initialize(IL);

        while (It.More() && Loop)
        {
            param = It.Value().Intersection().Parameter();
            if (param > end)
            {
                Loop = false;
            }
            else
            {
                if (Abs(param - sta) > Abs(param - end))
                {
                    end = param;
                }
                else
                {
                    sta = param;
                }
            }
            It.Next();
        }
        param = 0.5 * (sta + end);
        int level = 0;
        /*TopAbs_State st = */
        Classify(E, ED, true, ref level, param);
        Loop = true;
        It.Initialize(IL);

        while (It.More() && Loop)
        {
            HLRAlgo_Interference Int = It.Value();
            double p = Int.Intersection().Parameter();
            if (p < param - tolpar)
            {
                switch (Int.Transition())
                {

                    case Forward:
                        level -= Int.Intersection().Level();
                        break;
                    case Reversed:
                        level += Int.Intersection().Level();
                        break;
                    case External:
                    case Internal:
                    default:
                        break;
                }
            }
            else if (p > param + tolpar)
            {
                Loop = false;
            }
            else
            {
                // OCCT_DEBUG trace left out
            }
            It.Next();
        }
        return level;
    }

    /// <summary>The state of the edge ED against the current face (IN if hidden by it).</summary>
    public Containment Compare(int E, HLRBRep_EdgeData ED)
    {
        int level = 0;
        double parbid = 0.0;
        return Classify(E, ED, false, ref level, parbid);
    }

    /// <summary>Orients the outlines of the face FD as the projection sees them; true if the face is inverted.</summary>
    private bool OrientOutLine(int I, HLRBRep_FaceData FD)
    {
        _ = I; // avoid compiler warning

        HLRAlgo_WiresBlock wb = FD.Wires()!;
        int nw = wb.NbWires();
        int iw1, ie1, ne1;
        Transform3D T = myProj.Value.Transformation();
        Transform3D TI = myProj.Value.InvertedTransformation();
        bool inverted = false;
        bool FirstInversion = true;

        for (iw1 = 1; iw1 <= nw; iw1++)
        {
            HLRAlgo_EdgesBlock eb1 = wb.Wire(iw1)!;
            ne1 = eb1.NbEdges();

            for (ie1 = 1; ie1 <= ne1; ie1++)
            {
                myFE = eb1.Edge(ie1);
                HLRBRep_EdgeData ed1 = myEData[myFE];
                if (eb1.Double(ie1) || eb1.IsoLine(ie1) || ed1.Vertical())
                {
                    ed1.Used(true);
                }
                else
                {
                    ed1.Used(false);
                }
                if ((eb1.OutLine(ie1) || eb1.Internal(ie1)) && !ed1.Vertical())
                {
                    double p, pu, pv, r;
                    myFEGeom = ed1.ChangeGeometry();
                    HLRBRep_Curve EC = ed1.Geometry();
                    int vsta = ed1.VSta();
                    int vend = ed1.VEnd();
                    if (vsta == 0 && vend == 0)
                    {
                        p = 0;
                    }
                    else if (vsta == 0)
                    {
                        p = EC.Parameter3d(EC.LastParameter());
                    }
                    else if (vend == 0)
                    {
                        p = EC.Parameter3d(EC.FirstParameter());
                    }
                    else
                    {
                        p = EC.Parameter3d((EC.LastParameter() + EC.FirstParameter()) / 2);
                    }
                    if (HLRBRep_EdgeFaceTool.UVPoint(p, myFEGeom, iFaceGeom!, out pu, out pv))
                    {
                        mySLProps.SetParameters(pu, pv);
                        EC.D1(p, out Point3D Pt, out Vector3D Tg);
                        Direction3D V;
                        if (myProj.Value.Perspective())
                        {
                            Point2D P2d = new();
                            myProj.Value.Project(Pt, ref P2d);
                            V = new Direction3D(P2d.X(), P2d.Y(), -myProj.Value.Focus());
                        }
                        else
                        {
                            V = new Direction3D(Direction3D.D.NZ);
                        }
                        V.Transform(TI);
                        if (mySLProps.IsNormalDefined())
                        {
                            double curv = HLRBRep_EdgeFaceTool.CurvatureValue(iFaceGeom!, pu, pv, V);
                            Vector3D Nm = new(mySLProps.Normal());
                            if (curv == 0)
                            {
                                // OCCT_DEBUG trace left out
                            }
                            if (curv > 0)
                            {
                                Nm.Reverse();
                            }
                            Tg.Transform(T);
                            Pt.Transform(T);
                            Nm.Transform(T);
                            Nm.Cross(Tg);
                            if (Tg.Magnitude() < Constants.Resolution())
                            {
                                // OCCT_DEBUG trace left out
                            }
                            if (myProj.Value.Perspective())
                            {
                                r = Nm.Z() * myProj.Value.Focus() - (Nm.X() * Pt.X() + Nm.Y() * Pt.Y() + Nm.Z() * Pt.Z());
                            }
                            else
                            {
                                r = Nm.Z();
                            }
                            myFEOri = (r > 0) ? Forward : Reversed;
                            if (!FD.Cut() && FD.Closed() && FirstInversion)
                            {
                                if ((eb1.Orientation(ie1) == myFEOri) != (FD.Orientation() == Forward))
                                {
                                    FirstInversion = false;
                                    inverted = true;
                                }
                            }
                            eb1.Orientation(ie1, myFEOri);
                        }
                    }
                    else
                    {
                        // OCCT_DEBUG trace left out
                    }
                    ed1.Used(true);
                }
            }
        }
        return inverted;
    }

    /// <summary>Orients the other edges of the face FD as the projection sees them.</summary>
    private void OrientOthEdge(int I, HLRBRep_FaceData FD)
    {
        double p, pu, pv, r;
        HLRAlgo_WiresBlock wb = FD.Wires()!;
        int nw = wb.NbWires();
        int iw1, ie1, ne1;
        Transform3D T = myProj.Value.Transformation();

        for (iw1 = 1; iw1 <= nw; iw1++)
        {
            HLRAlgo_EdgesBlock eb1 = wb.Wire(iw1)!;
            ne1 = eb1.NbEdges();

            for (ie1 = 1; ie1 <= ne1; ie1++)
            {
                myFE = eb1.Edge(ie1);
                myFEOri = eb1.Orientation(ie1);
                HLRBRep_EdgeData ed1 = myEData[myFE];

                if (!ed1.Used())
                {
                    ed1.Used(true);
                    myFEGeom = ed1.ChangeGeometry();
                    HLRBRep_Curve EC = ed1.Geometry();
                    p = EC.Parameter3d((EC.LastParameter() + EC.FirstParameter()) / 2);
                    if (HLRBRep_EdgeFaceTool.UVPoint(p, myFEGeom, iFaceGeom!, out pu, out pv))
                    {
                        Point3D Pt = EC.Value3D(p);
                        mySLProps.SetParameters(pu, pv);
                        if (mySLProps.IsNormalDefined())
                        {
                            Vector3D Nm = new(mySLProps.Normal());
                            Pt.Transform(T);
                            Nm.Transform(T);
                            if (myProj.Value.Perspective())
                            {
                                r = Nm.Z() * myProj.Value.Focus() - (Nm.X() * Pt.X() + Nm.Y() * Pt.Y() + Nm.Z() * Pt.Z());
                            }
                            else
                            {
                                r = Nm.Z();
                            }
                            if (r < 0)
                            {
                                myFEOri = global::Kerf.Engine.Topology.TopAbs.Reverse(myFEOri);
                                eb1.Orientation(ie1, myFEOri);
                            }
                        }
                    }
                    // OCCT_DEBUG trace left out
                    _ = I; // avoid compiler warning
                }
            }
        }
    }

    private static void REJECT1(double[] theDeca, ReadOnlySpan<double> theTotMin, ReadOnlySpan<double> theTotMax,
                                double[] theSurD, ref HLRAlgo_EdgesBlock.MinMaxIndices theVertMin,
                                ref HLRAlgo_EdgesBlock.MinMaxIndices theVertMax)
    {
        theVertMin.Min[0] = (int)((theDeca[0] + theTotMin[0]) * theSurD[0]);
        theVertMax.Min[0] = (int)((theDeca[0] + theTotMax[0]) * theSurD[0]);
        theVertMin.Min[1] = (int)((theDeca[1] + theTotMin[1]) * theSurD[1]);
        theVertMax.Min[1] = (int)((theDeca[1] + theTotMax[1]) * theSurD[1]);
        theVertMin.Min[2] = (int)((theDeca[2] + theTotMin[2]) * theSurD[2]);
        theVertMax.Min[2] = (int)((theDeca[2] + theTotMax[2]) * theSurD[2]);
        theVertMin.Min[3] = (int)((theDeca[3] + theTotMin[3]) * theSurD[3]);
        theVertMax.Min[3] = (int)((theDeca[3] + theTotMax[3]) * theSurD[3]);
        theVertMin.Min[4] = (int)((theDeca[4] + theTotMin[4]) * theSurD[4]);
        theVertMax.Min[4] = (int)((theDeca[4] + theTotMax[4]) * theSurD[4]);
        theVertMin.Min[5] = (int)((theDeca[5] + theTotMin[5]) * theSurD[5]);
        theVertMax.Min[5] = (int)((theDeca[5] + theTotMax[5]) * theSurD[5]);
        theVertMin.Min[6] = (int)((theDeca[6] + theTotMin[6]) * theSurD[6]);
        theVertMax.Min[6] = (int)((theDeca[6] + theTotMax[6]) * theSurD[6]);
        theVertMin.Min[7] = (int)((theDeca[7] + theTotMin[7]) * theSurD[7]);
        theVertMax.Min[7] = (int)((theDeca[7] + theTotMax[7]) * theSurD[7]);
        theVertMin.Max[0] = (int)((theDeca[8] + theTotMin[8]) * theSurD[8]);
        theVertMax.Max[0] = (int)((theDeca[8] + theTotMax[8]) * theSurD[8]);
        theVertMin.Max[1] = (int)((theDeca[9] + theTotMin[9]) * theSurD[9]);
        theVertMax.Max[1] = (int)((theDeca[9] + theTotMax[9]) * theSurD[9]);
        theVertMin.Max[2] = (int)((theDeca[10] + theTotMin[10]) * theSurD[10]);
        theVertMax.Max[2] = (int)((theDeca[10] + theTotMax[10]) * theSurD[10]);
        theVertMin.Max[3] = (int)((theDeca[11] + theTotMin[11]) * theSurD[11]);
        theVertMax.Max[3] = (int)((theDeca[11] + theTotMax[11]) * theSurD[11]);
        theVertMin.Max[4] = (int)((theDeca[12] + theTotMin[12]) * theSurD[12]);
        theVertMax.Max[4] = (int)((theDeca[12] + theTotMax[12]) * theSurD[12]);
        theVertMin.Max[5] = (int)((theDeca[13] + theTotMin[13]) * theSurD[13]);
        theVertMax.Max[5] = (int)((theDeca[13] + theTotMax[13]) * theSurD[13]);
        theVertMin.Max[6] = (int)((theDeca[14] + theTotMin[14]) * theSurD[14]);
        theVertMax.Max[6] = (int)((theDeca[14] + theTotMax[14]) * theSurD[14]);
        theVertMin.Max[7] = (int)((theDeca[15] + theTotMin[15]) * theSurD[15]);
        theVertMax.Max[7] = (int)((theDeca[15] + theTotMax[15]) * theSurD[15]);
    }

    /// <summary>
    /// The state of the edge ED against the current face: classifies a point of the edge (the parameter param with
    /// LevelFlag, counting in Level how many times the face covers it) by shooting a line of sight at the face.
    /// </summary>
    public Containment Classify(int E, HLRBRep_EdgeData ED, bool LevelFlag, ref int Level, double param)
    {
        _ = E; // avoid compiler warning

        nbClassification++;
        HLRAlgo_EdgesBlock.MinMaxIndices VertMin = new(), VertMax = new(), MinMaxVert = new();
        Span<double> TotMin = stackalloc double[16];
        Span<double> TotMax = stackalloc double[16];

        int i;
        Level = 0;
        Containment state = Out;
        //  Standard_Boolean rej = Standard_False;
        HLRBRep_Curve EC = ED.Geometry();
        double sta, xsta, ysta, zsta, end, xend, yend, zend;
        double tol = (double)(ED.Tolerance());

        if (LevelFlag)
        {
            sta = param;
            myProj.Value.Project(EC.Value3D(sta), out xsta, out ysta, out zsta);

            HLRAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), TotMin, TotMax);
            HLRAlgo.UpdateMinMax(xsta, ysta, zsta, TotMin, TotMax);
            HLRAlgo.EnlargeMinMax(tol, TotMin, TotMax);
            REJECT1(myDeca, TotMin, TotMax, mySurD, ref VertMin, ref VertMax);

            HLRAlgo.EncodeMinMax(ref VertMin, ref VertMax, ref MinMaxVert);
            if (ZRejected(iFaceMinMax, MinMaxVert))
            { //-- Rejection en Z
                return state;
            }
        }
        else
        {
            sta = EC.Parameter3d(EC.FirstParameter());
            myProj.Value.Project(EC.Value3D(sta), out xsta, out ysta, out zsta);

            HLRAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), TotMin, TotMax);
            HLRAlgo.UpdateMinMax(xsta, ysta, zsta, TotMin, TotMax);
            HLRAlgo.EnlargeMinMax(tol, TotMin, TotMax);
            REJECT1(myDeca, TotMin, TotMax, mySurD, ref VertMin, ref VertMax);

            HLRAlgo.EncodeMinMax(ref VertMin, ref VertMax, ref MinMaxVert);
            if (ZRejected(iFaceMinMax, MinMaxVert))
            { //-- Rejection en Z
                return state;
            }
            end = EC.Parameter3d(EC.LastParameter());
            myProj.Value.Project(EC.Value3D(end), out xend, out yend, out zend);

            HLRAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), TotMin, TotMax);
            HLRAlgo.UpdateMinMax(xend, yend, zend, TotMin, TotMax);
            HLRAlgo.EnlargeMinMax(tol, TotMin, TotMax);
            REJECT1(myDeca, TotMin, TotMax, mySurD, ref VertMin, ref VertMax);

            HLRAlgo.EncodeMinMax(ref VertMin, ref VertMax, ref MinMaxVert);
            if (ZRejected(iFaceMinMax, MinMaxVert))
            { //-- Rejection en Z
                return state;
            }
            sta = 0.4 * sta + 0.6 * end; // dangerous if it is the middle
            myProj.Value.Project(EC.Value3D(sta), out xsta, out ysta, out zsta);

            HLRAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), TotMin, TotMax);
            HLRAlgo.UpdateMinMax(xsta, ysta, zsta, TotMin, TotMax);
            HLRAlgo.EnlargeMinMax(tol, TotMin, TotMax);
            REJECT1(myDeca, TotMin, TotMax, mySurD, ref VertMin, ref VertMax);

            HLRAlgo.EncodeMinMax(ref VertMin, ref VertMax, ref MinMaxVert);
            // OCCT's commented-out OCCT_DEBUG dump is left out
            if (ZRejected(iFaceMinMax, MinMaxVert))
            { //-- Rejection en Z
                return state;
            }
        }

        nbCal3Intersection++;
        Point3D PLim;
        Point2D Psta;
        Psta = EC.Value(sta);
        PLim = EC.Value3D(sta);

        // OCCT's debug dump of the point (a function-scope static flag never set) is left out

        Line3D L = myProj.Value.Shoot(Psta.X(), Psta.Y());
        double wLim = global::Kerf.Engine.Geometry.ElCLib.Parameter(L, PLim);
        myIntersector.Perform(L, wLim);
        if (myIntersector.IsDone())
        {
            int nbPoints = myIntersector.NbPoints();
            if (nbPoints > 0)
            {
                double TolZ = myBigSize * 0.000001;
                if (iFaceTest)
                {
                    if (!myLEOutLine && !myLEInternal)
                    {
                        TolZ = myBigSize * 0.001;
                    }
                    else
                    {
                        TolZ = myBigSize * 0.01;
                    }
                }
                wLim -= TolZ;
                double PeriodU, PeriodV, UMin = 0.0, UMax = 0.0, VMin = 0.0, VMax = 0.0;
                if (iFaceGeom!.IsUPeriodic())
                {
                    PeriodU = iFaceGeom.UPeriod();
                    UMin = iFaceGeom.FirstUParameter();
                    UMax = iFaceGeom.LastUParameter();
                }
                else
                {
                    PeriodU = 0.0;
                }
                if (iFaceGeom.IsVPeriodic())
                {
                    PeriodV = iFaceGeom.VPeriod();
                    VMin = iFaceGeom.FirstVParameter();
                    VMax = iFaceGeom.LastVParameter();
                }
                else
                {
                    PeriodV = 0;
                }
                Point3D PInter;
                double u, v, w;
                IntCurveSurface_TransitionOnCurve Tr;

                for (i = 1; i <= nbPoints; i++)
                {
                    myIntersector.CSPoint(i).Values(out PInter, out u, out v, out w, out Tr);
                    if (w < wLim)
                    {
                        double aDummyShift;
                        if (PeriodU > 0.0)
                        {
                            GeomInt.AdjustPeriodic(u, UMin, UMax, PeriodU, out u, out aDummyShift);
                        }
                        if (PeriodV > 0.0)
                        {
                            GeomInt.AdjustPeriodic(v, VMin, VMax, PeriodV, out v, out aDummyShift);
                        }

                        Point2D pnt2d = new(u, v);
                        if (myClassifier!.Classify(pnt2d, global::Kerf.Engine.Foundation.Precision.PConfusion()) != Out)
                        {
                            state = In;
                            Level++;
                            if (!LevelFlag)
                            {
                                return state;
                            }
                        }
                    }
                }
            }
        }
        return state;
    }

    /// <summary>The state of the edge ED between p1 and p2 by Nbp points: OUT if one of them is rejected by the box.</summary>
    public Containment SimplClassify(int E, HLRBRep_EdgeData ED, int Nbp, double p1, double p2)
    {
        nbClassification++;
        HLRAlgo_EdgesBlock.MinMaxIndices VertMin = new(), VertMax = new(), MinMaxVert = new();
        Span<double> TotMin = stackalloc double[16];
        Span<double> TotMax = stackalloc double[16];

        int i;
        Containment state = In;
        //  Standard_Boolean rej = Standard_False;
        HLRBRep_Curve EC = ED.Geometry();
        double sta, xsta, ysta, zsta, dp;
        double tol = (double)(ED.Tolerance());

        dp = (p2 - p1) / (Nbp + 1);

        for (sta = p1 + dp, i = 1; i <= Nbp; ++i, sta += dp)
        {
            myProj.Value.Project(EC.Value3D(sta), out xsta, out ysta, out zsta);

            HLRAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), TotMin, TotMax);
            HLRAlgo.UpdateMinMax(xsta, ysta, zsta, TotMin, TotMax);
            HLRAlgo.EnlargeMinMax(tol, TotMin, TotMax);
            REJECT1(myDeca, TotMin, TotMax, mySurD, ref VertMin, ref VertMax);

            HLRAlgo.EncodeMinMax(ref VertMin, ref VertMax, ref MinMaxVert);
            if (ZRejected(iFaceMinMax, MinMaxVert))
            { //-- Rejection en Z
                return Out;
            }
        }
        return state;
    }

    /// <summary>Builds the interference of the intersection point PInter; true if it is rejected.</summary>
    private bool RejectedPoint(in IntRes2d_IntersectionPoint PInter, Orientation BoundOri, int NumSeg)
    {
        int Ind = 0;
        int decal;
        double p1, p2, dz;
        float t1, t2;
        Containment st;
        Orientation Orie = Forward;
        Orientation Or2 = Internal;
        bool inverted = false;
        IntRes2d_Transition Tr1;
        IntRes2d_Transition Tr2;
        double TolZ = myBigSize * 0.00001;

        p1 = myLEGeom!.Parameter3d(PInter.ParamOnFirst());
        p2 = myFEGeom!.Parameter3d(PInter.ParamOnSecond());
        dz = myLEGeom.Z(p1) - myFEGeom.Z(p2);

        if (myLE == myFE)
        { // auto intersection can be inverted
            if (dz >= TolZ)
            {
                inverted = true;
                double p = p1;
                p1 = p2;
                p2 = p;
                dz = -dz;
            }
        }

        if (dz >= TolZ)
        {
            myAboveIntf = true;
            return true;
        }
        myAboveIntf = false;
        st = (dz <= -TolZ) ? In : On;

        if (inverted)
        {
            Tr1 = PInter.TransitionOfSecond();
            Tr2 = PInter.TransitionOfFirst();
        }
        else
        {
            Tr1 = PInter.TransitionOfFirst();
            Tr2 = PInter.TransitionOfSecond();
        }

        if (iFaceTest)
        {
            if (myLE == myFE)
            {
                if (st == In)
                {
                    myLEData!.Simple(false);
                }
            }
            else
            {
                if (mySameVertex)
                {
                    if ((st == On) || (Tr1.PositionOnCurve() != IntRes2d_Middle)
                        || (Tr2.PositionOnCurve() != IntRes2d_Middle))
                    {
                        return true;
                    }
                }
            }
            if (st == In)
            {
                iFaceSmpl = false;
            }
        }

        switch (Tr1.TransitionType())
        { // compute the transition
            case IntRes2d_In:
                Orie = (myFEOri == Reversed ? Reversed : Forward);
                break;
            case IntRes2d_Out:
                Orie = (myFEOri == Reversed ? Forward : Reversed);
                break;
            case IntRes2d_Touch:
                switch (Tr1.Situation())
                {
                    case IntRes2d_Inside:
                        Orie = (myFEOri == Reversed ? External : Internal);
                        break;
                    case IntRes2d_Outside:
                        Orie = (myFEOri == Reversed ? Internal : External);
                        break;
                    case IntRes2d_Unknown:
                        return true;
                }
                break;
            case IntRes2d_Undecided:
                return true;
        }

        if (iFaceBack)
        {
            Orie = global::Kerf.Engine.Topology.TopAbs.Complement(Orie); // change the transition
        }
        Orientation Ori = Forward;
        switch (Tr1.PositionOnCurve())
        {
            case IntRes2d_Head:
                Ori = Forward;
                break;
            case IntRes2d_Middle:
                Ori = Internal;
                break;
            case IntRes2d_End:
                Ori = Reversed;
                break;
        }

        if (st != Out)
        {
            if (Tr2.PositionOnCurve() != IntRes2d_Middle)
            { // correction de la transition  sur myFE
                if (mySameVertex)
                {
                    return true; // si intersection a une extremite verticale !
                }

                bool douteux = false;
                double psav = p2;
                Point2D Ptsav;
                Vector2D Tgsav = new(), Nmsav;
                if (Tr2.PositionOnCurve() == IntRes2d_Head)
                {
                    Ind = myFEData!.VSta();
                    Or2 = Forward;
                    AdjustParameter(myFEData, true, ref p2, out t2);
                    if (myFEData.VerAtSta())
                    {
                        douteux = true;
                        myFEGeom.D2(psav, out Ptsav, out Tgsav, out Nmsav);
                        if (Tgsav.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
                        {
                            Tgsav = Nmsav;
                        }
                    }
                }
                else
                {
                    Ind = myFEData!.VEnd();
                    Or2 = Reversed;
                    AdjustParameter(myFEData, false, ref p2, out t2);
                    if (myFEData.VerAtEnd())
                    {
                        douteux = true;
                        myFEGeom.D2(psav, out Ptsav, out Tgsav, out Nmsav);
                        if (Tgsav.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
                        {
                            Tgsav = Nmsav;
                        }
                    }
                }
                myFEGeom.D1(p2, out Ptsav, out Vector2D TgFE);
                if (douteux)
                {
                    if (TgFE.XY().Dot(Tgsav.XY()) < 0.0)
                    {
                        if (Orie == Forward)
                        {
                            Orie = Reversed;
                        }
                        else if (Orie == Reversed)
                        {
                            Orie = Forward;
                        }
                    }
                }
                myIntf.ChangeBoundary().Set2D(myFE, p2);
            }
            if (Ori != Internal)
            {                           // correction de la transition  sur myLE
                bool douteux = false; // si intersection a une extremite verticale !
                double psav = p1;
                Point2D Ptsav;
                Vector2D Tgsav = new(), Nmsav;
                if (Ori == Forward)
                {
                    AdjustParameter(myLEData!, true, ref p1, out t1);
                    if (myLEData!.VerAtSta())
                    {
                        douteux = true;
                        myLEGeom.D2(psav, out Ptsav, out Tgsav, out Nmsav);
                        if (Tgsav.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
                        {
                            Tgsav = Nmsav;
                        }
                    }
                }
                else
                {
                    AdjustParameter(myLEData!, false, ref p1, out t1);
                    if (myLEData!.VerAtEnd())
                    {
                        douteux = true;
                        myLEGeom.D2(psav, out Ptsav, out Tgsav, out Nmsav);
                        if (Tgsav.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
                        {
                            Tgsav = Nmsav;
                        }
                    }
                }
                if (douteux)
                {
                    myLEGeom.D1(p1, out Ptsav, out Vector2D TgLE);
                    if (TgLE.XY().Dot(Tgsav.XY()) < 0.0)
                    {
                        if (Orie == Forward)
                        {
                            Orie = Reversed;
                        }
                        else if (Orie == Reversed)
                        {
                            Orie = Forward;
                        }
                    }
                }
            }
            if (st == On)
            {
                EdgeState(p1, p2, out Containment stbef, out Containment staft);
                myIntf.ChangeBoundary().SetState3D(stbef, staft);
            }
        }

        if (myFEInternal)
        {
            decal = 2;
        }
        else
        {
            decal = 1;
            if (st == In && Ori == Forward && Orie == Forward)
            {
                decal = 0;
            }
        }
        ref HLRAlgo_Intersection inter = ref myIntf.ChangeIntersection();
        inter.Orientation(Ori);
        inter.Level(decal);
        inter.SegIndex(NumSeg);
        inter.Index(Ind);
        inter.Parameter(p1);
        inter.Tolerance(myLETol);
        inter.State(st);
        myIntf.Orientation(Or2);
        myIntf.Transition(Orie);
        myIntf.BoundaryTransition(BoundOri);
        myIntf.ChangeBoundary().Set2D(myFE, p2);
        return false;
    }

    /// <summary>True if the vertex at the head (h1) or end of the current edge is the head (h2) or end of the boundary edge.</summary>
    private bool SameVertex(bool h1, bool h2)
    {
        int v1, v2;
        if (h1)
        {
            v1 = myLEData!.VSta();
        }
        else
        {
            v1 = myLEData!.VEnd();
        }
        if (h2)
        {
            v2 = myFEData!.VSta();
        }
        else
        {
            v2 = myFEData!.VEnd();
        }
        bool SameV = v1 == v2;
        if (SameV)
        {
            myIntersected = true; // compute the intersections
            if ((myLEType == Line || myLEType == Circle || myLEType == Ellipse)
                && (myFEType == Line || myFEType == Circle || myFEType == Ellipse))
            {
                myIntersected = false; // no other intersection
            }

            bool otherCase = true;

            if ((h1 && myLEData.OutLVSta()) || (!h1 && myLEData.OutLVEnd()))
            {
                if (iFaceTest || myLEInternal)
                {
                    otherCase = false;
                }
            }
            else if (iFaceTest)
            {
                otherCase = false;
            }

            if (otherCase)
            {
                if ((h1 && myLEData.CutAtSta()) || (!h1 && myLEData.CutAtEnd()))
                {
                    myIntersected = false; // two connected OutLines do not
                }                          // intersect themselves.
            }
        }
        return SameV;
    }

    /// <summary>True if the current face is periodic with bounds spanning more than two periods.</summary>
    public bool IsBadFace()
    {
        if (iFaceGeom is not null)
        {
            // check for garbage data - if periodic then bounds must not exceed period
            HLRBRep_Surface pGeom = iFaceGeom;
            if (pGeom.IsUPeriodic())
            {
                double aPeriod = pGeom.UPeriod();
                double aMin = pGeom.FirstUParameter();
                double aMax = pGeom.LastUParameter();
                if (aPeriod * 2 < aMax - aMin)
                {
                    return true;
                }
            }
            if (pGeom.IsVPeriodic())
            {
                double aPeriod = pGeom.VPeriod();
                double aMin = pGeom.FirstVParameter();
                double aMax = pGeom.LastVParameter();
                if (aPeriod * 2 < aMax - aMin)
                {
                    return true;
                }
            }
        }
        return false;
    }
}
