// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Data.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Data.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Data.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Runtime.CompilerServices;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Engine.Intersection.TwoD.Situation;
using static Kerf.Engine.Intersection.TwoD.TypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.HiddenLines;

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
public class HiddenLineData : Transient
{
    // OCCT's global counters (HLRBRep_Data.cxx) are left out: only OCCT_DEBUG traces read them, and nothing here did
    // (decision 1097; unit 10I)

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

    private const int Sizeuv = 8;

    private const int THE_MASK = unchecked((int)0x80008000);

    private sealed class TableauRejection
    {
        public double[][]? UV;  //-- UV[i][j]     contient le param (U sur Ci) de l intersection de Ci avec C(IndUV[j])
        public int[][]? IndUV;  //-- IndUV[i][j]  = J0   -> Intersection entre i et J0
        public int[]? nbUV;     //-- nbUV[i][j]   nombre de valeurs pour la ligne i
        public int N;

        public uint[][]? TabBit;
        public int NTabBit;

        public TableauRejection()
        {
            N = 0;
            NTabBit = 0;
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
                UV[i] = new double[Sizeuv];
            }
            for (i = 0; i < N; i++)
            {
                IndUV[i] = new int[Sizeuv];
                for (int k = 0; k < Sizeuv; k++)
                {
                    IndUV[i][k] = -1;
                }
                nbUV[i] = Sizeuv;
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
            int[][] indUV = this.IndUV!;
            double[][] uv = this.UV!;
            i0--;
            j0--;
            int k = -1;
            int i;
            for (i = 0; k == -1 && i < nbUV![i0]; i++)
            {
                if (indUV![i0][i] == -1)
                {
                    k = i;
                }
            }
            if (k == -1)
            { //-- on agrandit le tableau
                double[] nvLigneUV = new double[nbUV[i0] + Sizeuv];
                int[] nvLigneInd = new int[nbUV[i0] + Sizeuv];
                for (i = 0; i < nbUV[i0]; i++)
                {
                    nvLigneUV[i] = uv![i0][i];
                    nvLigneInd[i] = indUV![i0][i];
                }
                k = nbUV[i0];
                nbUV[i0] += Sizeuv;
                uv![i0] = nvLigneUV;
                indUV![i0] = nvLigneInd;
                for (int kk = k; kk < nbUV[i0]; kk++)
                {
                    indUV[i0][kk] = -1;
                }
            }
            indUV![i0][k] = j0;
            uv![i0][k] = u;

            //-- tri par ordre decroissant
            bool triOk;
            do
            {
                triOk = true;
                int im1 = 0;
                // OCCT tests IndUV[i0][i] before i < nbUV[i0] (one read past a full row); same result
                for (i = 1; i < nbUV[i0] && indUV[i0][i] != -1; i++, im1++)
                {
                    if (indUV[i0][i] > indUV[i0][im1])
                    {
                        triOk = false;
                        k = indUV[i0][i];
                        indUV[i0][i] = indUV[i0][im1];
                        indUV[i0][im1] = k;
                        double t = uv[i0][i];
                        uv[i0][i] = uv[i0][im1];
                        uv[i0][im1] = t;
                    }
                }
            } while (!triOk);
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
                NTabBit = 0;
            }
        }

        public void InitTabBit(int nbedgs)
        {
            if (TabBit is not null && NTabBit != 0)
            {
                ResetTabBit(NTabBit);
            }
            TabBit = new uint[nbedgs][];
            NTabBit = nbedgs;
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

        public void SetIntersection(int i0, int i1, in IntersectionPoint2D ip)
        {
            Transition t1 = ip.TransitionOfFirst();
            Transition t2 = ip.TransitionOfSecond();
            if (t1.PositionOnCurve() == Position2D.Middle)
            {
                if (t2.PositionOnCurve() == Position2D.Middle)
                {
                    if (t1.TransitionType() == TypeTrans.In || t1.TransitionType() == TypeTrans.Out)
                    {
                        Set(i0, i1, ip.ParamOnFirst());
                        Set(i1, i0, ip.ParamOnSecond());
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
    private static bool ZRejected(in EdgesBlock.MinMaxIndices a, in EdgesBlock.MinMaxIndices b)
    {
        return ((a.Max[0] - b.Min[0]) & THE_MASK) != 0 || ((b.Max[0] - a.Min[0]) & THE_MASK) != 0
               || ((a.Max[1] - b.Min[1]) & THE_MASK) != 0 || ((b.Max[1] - a.Min[1]) & THE_MASK) != 0
               || ((a.Max[2] - b.Min[2]) & THE_MASK) != 0 || ((b.Max[2] - a.Min[2]) & THE_MASK) != 0
               || ((a.Max[3] - b.Min[3]) & THE_MASK) != 0 || ((b.Max[3] - a.Min[3]) & THE_MASK) != 0
               || ((a.Max[4] - b.Min[4]) & THE_MASK) != 0 || ((b.Max[4] - a.Min[4]) & THE_MASK) != 0
               || ((a.Max[5] - b.Min[5]) & THE_MASK) != 0 || ((b.Max[5] - a.Min[5]) & THE_MASK) != 0
               || ((a.Max[6] - b.Min[6]) & THE_MASK) != 0 || ((b.Max[6] - a.Min[6]) & THE_MASK) != 0
               || ((a.Max[7] - b.Min[7]) & THE_MASK) != 0;
    }

    private static void AdjustParameter(HiddenLineEdgeData e, bool h, ref double p, out float t)
    {
        double p1, p2;
        float t1, t2;
        if (h)
        {
            e.Status().Bounds(out p, out t, out p2, out t2);
            if (e.VerAtSta())
            {
                p = p + (p2 - p) * CutBig;
            }
        }
        else
        {
            e.Status().Bounds(out p1, out t1, out p, out t);
            if (e.VerAtEnd())
            {
                p = p - (p - p1) * CutBig;
            }
        }
    }

    private readonly int _nbVertices;
    private readonly int _nbEdges;
    private readonly int _nbFaces;
    private readonly NCollection_IndexedMap<Shape> _eMap = new(ShapeHasher.Instance);
    private readonly NCollection_IndexedMap<Shape> _fMap = new(ShapeHasher.Instance);
    private readonly NCollection_Array1<HiddenLineEdgeData> _eData;
    private readonly NCollection_Array1<HiddenLineFaceData> _fData;
    private readonly NCollection_Array1<int> _edgeIndices;
    private float _toler;
    private readonly StrongBox<AlgorithmProjector> _proj = new(new AlgorithmProjector());
    private readonly HiddenLineCLProps _llProps;
    private readonly HiddenLineCLProps _flProps;
    private readonly HiddenLineSLProps _slProps;
    private double _bigSize;
    private readonly FaceIterator _faceItr1 = new();
    private readonly FaceIterator _faceItr2 = new();
    private int _iFace;
    private HiddenLineFaceData? _iFaceData;
    private HlrSurface? _iFaceGeom;
    private WiresBlock? _iFaceWires; // OCCT's IFaceMinMax points into its min-max
    private SurfaceKind _iFaceType;
    private bool _iFaceBack;
    private bool _iFaceSimp;
    private bool _iFaceSmpl;
    private bool _iFaceTest;
    private int _hideCount;
    private readonly double[] _deca = new double[16];
    private readonly double[] _surD = new double[16];
    private int _curSortEd;
    private int _nbrSortEd;
    private int _le;
    private bool _leOutLine;
    private bool _leInternal;
    private bool _leDouble;
    private bool _leIsoLine;
    private HiddenLineEdgeData? _leData;
    private HiddenLineCurve? _leGeom;
    private HiddenLineEdgeData? _leMinMaxOwner; // OCCT's LEMinMax points into its min-max
    private CurveKind _leType;
    private float _leTol;
    private int _fe;
    private Orientation _feOri;
    private bool _feOutLine;
    private bool _feInternal;
    private bool _feDouble;
    private HiddenLineEdgeData? _feData;
    private HiddenLineCurve? _feGeom;
    private CurveKind _feType;
    private float _feTol;
    private readonly HiddenLineIntersector _intersector = new();
    private BrepTopolTool? _classifier;
    private bool _sameVertex;
    private bool _intersected;
    private int _nbPoints;
    private int _nbSegments;
    private int _iInterf;
    private Interference _intf;
    private bool _aboveIntf;
    private TableauRejection? _reject;

    private ref EdgesBlock.MinMaxIndices IFaceMinMax => ref _iFaceWires!.MinMax();

    private ref EdgesBlock.MinMaxIndices LEMinMax => ref _leMinMaxOwner!.MinMax();

    /// <summary>A data structure for NV vertices, NE edges and NF faces (indexed from 1; index 0 is unused).</summary>
    public HiddenLineData(int nv, int ne, int nf)
    {
        _nbVertices = nv;
        _nbEdges = ne;
        _nbFaces = nf;
        _eData = new NCollection_Array1<HiddenLineEdgeData>(0, ne);
        for (int i = 0; i <= ne; i++)
        {
            _eData[i] = new HiddenLineEdgeData();
        }
        _fData = new NCollection_Array1<HiddenLineFaceData>(0, nf);
        for (int i = 0; i <= nf; i++)
        {
            _fData[i] = new HiddenLineFaceData();
        }
        _edgeIndices = new NCollection_Array1<int>(0, ne);
        _toler = (float)1e-5;
        _llProps = new HiddenLineCLProps(2, Epsilon(1.0));
        _flProps = new HiddenLineCLProps(2, Epsilon(1.0));
        _slProps = new HiddenLineSLProps(2, Epsilon(1.0));
        _hideCount = 0;
        _reject = new TableauRejection();
        _reject.SetDim(_nbEdges);
    }

    /// <summary>Drops the rejection table (OCCT's destructor calls it).</summary>
    public void Destroy()
    {
        _reject?.Destroy();
        _reject = null;
    }

    /// <summary>Copies the edges and faces of DS here, shifted by dv vertices, de edges and df faces.</summary>
    public void Write(HiddenLineData ds, int dv, int de, int df)
    {
        int n1edge = ds.EdgeCount;
        int n1face = ds.FaceCount;

        for (int iedge = 1; iedge <= n1edge; iedge++)
        {
            HiddenLineEdgeData ed = _eData[de + iedge];
            HiddenLineEdgeData e1 = ds.EDataArray()[iedge];
            ed.Assign(e1);

            if (dv != 0)
            {
                ed.VSta(ed.VSta() + dv);
                ed.VEnd(ed.VEnd() + dv);
            }

            _eMap.Add(ds.EdgeMap().FindKey(iedge));
        }

        for (int iface = 1; iface <= n1face; iface++)
        {
            HiddenLineFaceData fd = _fData[df + iface];
            HiddenLineFaceData f1 = ds.FDataArray()[iface];
            fd.Assign(f1);

            if (de != 0)
            {
                // OCCT: the wires block is shared with DS's face, whose edge indices are shifted too
                WiresBlock wb = fd.Wires()!;
                int nw = wb.WireCount;

                for (int iw = 1; iw <= nw; iw++)
                {
                    EdgesBlock eb = wb.Wire(iw)!;
                    int ne = eb.EdgeCount;

                    for (int ie = 1; ie <= ne; ie++)
                    {
                        eb.Edge(ie, eb.Edge(ie) + de);
                    }
                }
            }

            _fMap.Add(ds.FaceMap().FindKey(iface));
        }
    }

    public NCollection_Array1<HiddenLineEdgeData> EDataArray() => _eData;

    public NCollection_Array1<HiddenLineFaceData> FDataArray() => _fData;

    public void Tolerance(float tol) => _toler = tol;

    public float Tolerance() => _toler;

    /// <summary>
    /// Sets the projector and computes everything that depends on it: the curves' types and min-max boxes, the edges'
    /// vertical flags, the faces' flags (side, back, hiding, simple, cut), orientations of the outlines, and boxes.
    /// </summary>
    public void Update(in AlgorithmProjector P)
    {
        _proj.Value = P;
        Transform3D t = _proj.Value.Transformation();
        int i;
        double tolMinMax = 0;

        EdgesBlock.MinMaxIndices faceMin = new(), faceMax = new();
        EdgesBlock.MinMaxIndices minMaxFace = new();
        EdgesBlock.MinMaxIndices wireMin = new(), wireMax = new(), minMaxWire = new();
        EdgesBlock.MinMaxIndices edgeMin = new(), edgeMax = new();
        EdgesBlock.MinMaxIndices minMaxEdge = new();
        Span<double> totMin = stackalloc double[16];
        Span<double> totMax = stackalloc double[16];
        HlrAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), totMin, totMax);

        // compute the global MinMax
        // *************************
        //  int nbEdge = myEdges.Extent();
        int edge;
        for (edge = 1; edge <= _nbEdges; edge++)
        {
            HiddenLineEdgeData ed = _eData[edge];
            HiddenLineCurve ec = ed.ChangeGeometry();
            ec.Projector(_proj);
            double enl = ec.Update(totMin, totMax);
            if (enl > tolMinMax)
            {
                tolMinMax = enl;
            }
        }
        HlrAlgo.EnlargeMinMax(tolMinMax, totMin, totMax);
        Span<double> d = stackalloc double[16];
        double precad = -global::Kerf.Engine.Foundation.Precision.Infinite();

        for (i = 0; i <= 15; i++)
        {
            d[i] = totMax[i] - totMin[i];
            if (precad < d[i])
            {
                precad = d[i];
            }
        }
        _bigSize = precad;
        precad = precad * 0.0005;

        for (i = 0; i <= 15; i++)
        {
            _surD[i] = 0x00007fff / (d[i] + precad);
        }
        precad = precad * 0.5;

        for (i = 0; i <= 15; i++)
        {
            _deca[i] = -totMin[i] + precad;
        }

        double tol;
        bool ver1, ver2;

        // update the edges
        // ****************

        for (edge = 1; edge <= _nbEdges; edge++)
        {

            HiddenLineEdgeData ed = _eData[edge];
            HiddenLineCurve ec = ed.ChangeGeometry();
            HlrAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), totMin, totMax);
            tolMinMax = ec.UpdateMinMax(totMin, totMax);
            tol = (double)(ed.Tolerance());
            ed.Vertical(totMax[0] - totMin[0] < tol && totMax[1] - totMin[1] < tol && totMax[2] - totMin[2] < tol
                        && totMax[3] - totMin[3] < tol && totMax[4] - totMin[4] < tol && totMax[5] - totMin[5] < tol
                        && totMax[6] - totMin[6] < tol);
            HlrAlgo.EnlargeMinMax(tolMinMax, totMin, totMax);
            // Linux warning :  assignment to `int' from `double'. Cast has been added.
            REJECT1(_deca, totMin, totMax, _surD, ref edgeMin, ref edgeMax);
            HlrAlgo.EncodeMinMax(ref edgeMin, ref edgeMax, ref minMaxEdge);
            ed.UpdateMinMax(minMaxEdge);
            if (ed.Vertical())
            {
                ver1 = true;
                ver2 = true;
                int vsta = ed.VSta();
                int vend = ed.VEnd();
                bool vout = ed.OutLVSta() || ed.OutLVEnd();
                bool vcut = ed.CutAtSta() || ed.CutAtEnd();

                for (int ebis = 1; ebis <= _nbEdges; ebis++)
                {
                    HiddenLineEdgeData eb = _eData[ebis];
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
                ec.D1(ec.Parameter3d(ec.FirstParameter()), out Point3D pt, out Vector3D tg1);
                ec.D1(ec.Parameter3d(ec.LastParameter()), out pt, out Vector3D tg2);
                tg1.Transform(t);
                tg2.Transform(t);
                if (Abs(tg1.X()) + Abs(tg1.Y()) < _toler * 10)
                {
                    ver1 = true;
                }
                else
                {
                    Direction3D dir1 = new(tg1);
                    ver1 = Abs(dir1.X()) + Abs(dir1.Y()) < _toler * 10;
                }
                if (Abs(tg2.X()) + Abs(tg2.Y()) < _toler * 10)
                {
                    ver2 = true;
                }
                else
                {
                    Direction3D dir2 = new(tg2);
                    ver2 = Abs(dir2.X()) + Abs(dir2.Y()) < _toler * 10;
                }
            }
            ed.VerAtSta(ed.Vertical() || ver1);
            ed.VerAtEnd(ed.Vertical() || ver2);
            ed.AutoIntersectionDone(true);
            ed.Simple(true);
        }

        // update the faces
        // ****************

        for (int face = 1; face <= _nbFaces; face++)
        {

            HiddenLineFaceData fd = _fData[face];
            HlrSurface fs = fd.Geometry();
            _iFaceGeom = fd.Geometry();
            _slProps.SetSurface(_iFaceGeom);
            fs.Projector(_proj);
            _iFaceType = fs.GetType();

            // Is the face cut by an outline

            bool cut = false;
            bool withOutL = false;

            for (_faceItr1.InitEdge(fd); _faceItr1.MoreEdge(); _faceItr1.NextEdge())
            {
                if (_faceItr1.Internal())
                {
                    withOutL = true;
                    cut = true;
                }
                else if (_faceItr1.OutLine())
                {
                    withOutL = true;
                    if (_faceItr1.Double())
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
                && (_iFaceType == Plane || _iFaceType == SurfaceKind.Cylinder || _iFaceType == SurfaceKind.Cone
                    || _iFaceType == SurfaceKind.Sphere || _iFaceType == SurfaceKind.Torus))
            {
                fd.Simple(true);
            }
            else
            {
                fd.Simple(false);
            }

            fd.Plane(_iFaceType == Plane);
            fd.Cylinder(_iFaceType == SurfaceKind.Cylinder);
            fd.Cone(_iFaceType == SurfaceKind.Cone);
            fd.Sphere(_iFaceType == SurfaceKind.Sphere);
            fd.Torus(_iFaceType == SurfaceKind.Torus);
            tol = (double)(fd.Tolerance());
            fd.Side(fs.IsSide(tol, _toler * 10));
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

                for (_faceItr1.InitEdge(fd); _faceItr1.MoreEdge() && !found; _faceItr1.NextEdge())
                {
                    _fe = _faceItr1.Edge();
                    _feOri = _faceItr1.Orientation();
                    _feOutLine = _faceItr1.OutLine();
                    _feInternal = _faceItr1.Internal();
                    _feDouble = _faceItr1.Double();
                    HiddenLineEdgeData eDataFE1 = _eData[_fe];
                    if (!_feDouble && (_feOri == Forward || _feOri == Reversed))
                    {
                        _feGeom = eDataFE1.ChangeGeometry();
                        HiddenLineCurve ec = eDataFE1.Geometry();
                        p = ec.Parameter3d((ec.LastParameter() + ec.FirstParameter()) / 2);
                        if (EdgeFaceTool.UVPoint(p, _feGeom, _iFaceGeom, out pu, out pv))
                        {
                            _slProps.SetParameters(pu, pv);
                            Point3D pt;
                            pt = ec.Value3D(p);
                            if (_slProps.IsNormalDefined)
                            {
                                Vector3D nm = new(_slProps.Normal());
                                pt.Transform(t);
                                nm.Transform(t);
                                if (_proj.Value.Perspective())
                                {
                                    r = nm.Z() * _proj.Value.Focus() - (nm.X() * pt.X() + nm.Y() * pt.Y() + nm.Z() * pt.Z());
                                }
                                else
                                {
                                    r = nm.Z();
                                }
                                if (Abs(r) > _toler * 10)
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

            bool firstTime = true;

            for (_faceItr1.InitEdge(fd); _faceItr1.MoreEdge(); _faceItr1.NextEdge())
            {
                _fe = _faceItr1.Edge();
                HiddenLineEdgeData eDataFE2 = _eData[_fe];
                if (!fd.Simple())
                {
                    eDataFE2.AutoIntersectionDone(false);
                }
                HlrAlgo.DecodeMinMax(eDataFE2.MinMax(), ref edgeMin, ref edgeMax);
                if (_faceItr1.BeginningOfWire())
                {
                    HlrAlgo.CopyMinMax(ref edgeMin, ref edgeMax, ref wireMin, ref wireMax);
                }
                else
                {
                    HlrAlgo.AddMinMax(ref edgeMin, ref edgeMax, ref wireMin, ref wireMax);
                }
                if (_faceItr1.EndOfWire())
                {
                    HlrAlgo.EncodeMinMax(ref wireMin, ref wireMax, ref minMaxWire);
                    _faceItr1.Wire().UpdateMinMax(minMaxWire);
                    if (firstTime)
                    {
                        firstTime = false;
                        HlrAlgo.CopyMinMax(ref wireMin, ref wireMax, ref faceMin, ref faceMax);
                    }
                    else
                    {
                        HlrAlgo.AddMinMax(ref wireMin, ref wireMax, ref faceMin, ref faceMax);
                    }
                }
            }
            HlrAlgo.EncodeMinMax(ref faceMin, ref faceMax, ref minMaxFace);
            fd.Wires()!.UpdateMinMax(minMaxFace);
            fd.Size(HlrAlgo.SizeBox(ref faceMin, ref faceMax));
        }
    }

    /// <summary>The projector (a reference to the one the curves and surfaces share, as OCCT's).</summary>
    public ref AlgorithmProjector Projector() => ref _proj.Value;

    public int VertexCount => _nbVertices;

    public int EdgeCount => _nbEdges;

    public int FaceCount => _nbFaces;

    public NCollection_IndexedMap<Shape> EdgeMap() => _eMap;

    public NCollection_IndexedMap<Shape> FaceMap() => _fMap;

    /// <summary>Selects the edges e1 to e2 not all hidden whose boxes meet MinMaxTot, for the faces to hide.</summary>
    public void InitBoundSort(in EdgesBlock.MinMaxIndices minMaxTot, int e1, int e2)
    {
        _nbrSortEd = 0;
        EdgesBlock.MinMaxIndices minMaxShap = minMaxTot;

        for (int e = e1; e <= e2; e++)
        {
            HiddenLineEdgeData ed = _eData[e];
            if (!ed.Status().AllHidden())
            {
                _leMinMaxOwner = ed;
                if (!ZRejected(minMaxShap, LEMinMax))
                { //- rejection en z
                    _nbrSortEd++;
                    _edgeIndices[_nbrSortEd] = e;
                }
            }
        }
    }

    /// <summary>Starts the iteration on the edges the face FI may hide; MST caches the faces' classifiers.</summary>
    public void InitEdge(int fi, NCollection_DataMap<Shape, AdaptorTool> mst)
    {
        _hideCount++;
        _hideCount++;

        _iFace = fi;
        _iFaceData = _fData[_iFace];
        _iFaceGeom = _iFaceData.Geometry();
        _iFaceBack = _iFaceData.Back();
        _iFaceSimp = _iFaceData.Simple();
        _iFaceWires = _iFaceData.Wires();
        _iFaceType = _iFaceGeom.GetType();
        _iFaceTest = !_iFaceSimp;
        _slProps.SetSurface(_iFaceGeom);
        _intersector.Load(_iFaceGeom);

        HlrSurface p1 = _iFaceGeom;
        FaceSurfaceAdaptor bras = p1.Surface();
        Face topodsface = bras.Face();

        if (mst.IsBound(topodsface))
        {
            AdaptorTool brt = mst.ChangeFind(topodsface);
            _classifier = brt.GetTopolTool();
        }
        else
        {
            AdaptorTool brt = new(topodsface, global::Kerf.Engine.Foundation.Precision.PConfusion());
            mst.Bind(topodsface, brt);
            _classifier = brt.GetTopolTool();
        }

        if (_iFaceTest)
        {
            _iFaceSmpl = !_iFaceData.Cut();
            _faceItr2.InitEdge(_iFaceData);
        }
        else
        {

            for (_faceItr1.InitEdge(_iFaceData); _faceItr1.MoreEdge(); _faceItr1.NextEdge())
            {
                _fe = _faceItr1.Edge();                // edges of a simple hiding
                _eData[_fe].HideCount(_hideCount - 1); // face must be jumped.
            }
            _curSortEd = 1;
        }
        NextEdge(false);
    }

    public bool MoreEdge()
    {
        if (_iFaceTest)
        {
            if (_faceItr2.MoreEdge())
            {                                         // all edges must be tested if
                _le = _faceItr2.Edge();             // the face is not a simple
                _leOutLine = _faceItr2.OutLine();   // one.
                _leInternal = _faceItr2.Internal();
                _leDouble = _faceItr2.Double();
                _leIsoLine = _faceItr2.IsoLine();
                _leData = _eData[_le];
                _leGeom = _leData.ChangeGeometry();
                _leMinMaxOwner = _leData;
                _leTol = _leData.Tolerance();
                _leType = _leGeom.GetType();
                if (!_leDouble)
                {
                    _leData.HideCount(_hideCount - 1);
                }
                return true;
            }
            else
            {
                _iFaceTest = false;     // at the end of the test
                _iFaceSimp = _iFaceSmpl; // we know if it is a simple face
                _iFaceData!.Simple(_iFaceSimp);
                _curSortEd = 1;
                NextEdge(false);
            }
        }
        return _curSortEd <= _nbrSortEd;
    }

    public void NextEdge(bool skip = true)
    {

        if (skip)
        {
            if (_iFaceTest)
            {
                _faceItr2.NextEdge();
            }
            else
            {
                _curSortEd++;
            }
        }
        if (!MoreEdge())
        {
            return;
        }
        if (_iFaceTest)
        {
            _le = _faceItr2.Edge();
            _leOutLine = _faceItr2.OutLine();
            _leInternal = _faceItr2.Internal();
            _leDouble = _faceItr2.Double();
            _leIsoLine = _faceItr2.IsoLine();
            _leData = _eData[_le];
            _leGeom = _leData.ChangeGeometry();
            _leMinMaxOwner = _leData;
            _leTol = _leData.Tolerance();
            _leType = _leGeom.GetType();
            if (_leData.Vertical() || (_leDouble && _leData.HideCount() == _hideCount - 1))
            {
                NextEdge();
            }
            _leData.HideCount(_hideCount - 1);
            return;
        }
        else
        {
            _le = Edge();
            _leOutLine = false;
            _leInternal = false;
            _leDouble = false;
            _leIsoLine = false;
            _leData = _eData[_le];
            _leGeom = _leData.ChangeGeometry();
            _leMinMaxOwner = _leData;
            _leTol = _leData.Tolerance();
            _leType = _leGeom.GetType();
        }
        if (_leData.Vertical())
        {
            NextEdge();
            return;
        }
        if (_leData.HideCount() > _hideCount - 2)
        {
            NextEdge();
            return;
        }
        if (_leData.Status().AllHidden())
        {
            NextEdge();
            return;
        }
        if (ZRejected(IFaceMinMax, LEMinMax))
        { //-- rejection en z
            NextEdge();
            return;
        }
        if (_iFaceGeom!.IsAbove(_iFaceBack, _leGeom, (double)_leTol))
        {
            NextEdge();
            return;
        }
        return; // edge is OK
    }

    public int Edge()
    {
        if (_iFaceTest)
        {
            return _faceItr2.Edge();
        }
        else
        {
            return _edgeIndices[_curSortEd];
        }
    }

    public bool HidingTheFace() => _iFaceTest;

    public bool SimpleHidingFace() => _iFaceSimp;

    /// <summary>Starts the iteration on the interferences of the current edge with the face's boundary.</summary>
    public void InitInterference()
    {
        _llProps.SetCurve(_leGeom!);
        _faceItr1.InitEdge(_iFaceData!);
        _nbPoints = _nbSegments = _iInterf = 0;
        NextInterference();
    }

    public bool MoreInterference() => (_iInterf <= _nbPoints + 2 * _nbSegments);

    public void NextInterference()
    {
        // are there more intersections on the current edge
        _iInterf++;
        //    Standard_Integer miniWire1,miniWire2;
        //    Standard_Integer maxiWire1,maxiWire2,maxiWire3,maxiWire4;

        while (!MoreInterference() && _faceItr1.MoreEdge())
        {

            // rejection of current wire
            if (_faceItr1.BeginningOfWire())
            {
                ref EdgesBlock.MinMaxIndices minMaxWire = ref _faceItr1.Wire().MinMax();
                if (ZRejected(minMaxWire, LEMinMax))
                { //-- Rejection en Z
                    _faceItr1.SkipWire();
                    continue;
                }
            }
            _fe = _faceItr1.Edge();
            _feOri = _faceItr1.Orientation();
            _feOutLine = _faceItr1.OutLine();
            _feInternal = _faceItr1.Internal();
            _feDouble = _faceItr1.Double();
            _feData = _eData[_fe];
            _feGeom = _feData.ChangeGeometry();
            _feTol = _feData.Tolerance();
            _feType = _feGeom.GetType();

            if (_feOri == Forward || _feOri == Reversed)
            {
                // Edge from the boundary
                if (!_feData.Vertical() && (!_feDouble || _feOutLine))
                {
                    // not a vertical edge and not a double Edge
                    ref EdgesBlock.MinMaxIndices minMaxFEdg = ref _feData.MinMax();
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

                    if (!_reject!.NoIntersection(_le, _fe))
                    {
                        if (!ZRejected(minMaxFEdg, LEMinMax))
                        { //-- Rejection en Z
                            bool rej = false;
                            if (_le == _fe)
                            { // test if an auto-intersection is not useful
                                if (_leData!.AutoIntersectionDone())
                                {
                                    _leData.AutoIntersectionDone(true);
                                    if (_leData.Simple())
                                    {
                                        rej = true;
                                    }
                                }
                            }

                            if (!rej)
                            {
                                bool h1 = false;
                                bool e1 = false;
                                bool h2 = false;
                                bool e2 = false;
                                _sameVertex = false;

                                if (_le == _fe)
                                {
                                    _intersected = true;
                                    _sameVertex = false;
                                }
                                else
                                {
                                    _intersected = true;
                                    if (SameVertex(true, true))
                                    {
                                        _sameVertex = true;
                                        h1 = true;
                                        h2 = true;
                                    }
                                    if (SameVertex(true, false))
                                    {
                                        _sameVertex = true;
                                        h1 = true;
                                        e2 = true;
                                    }
                                    if (SameVertex(false, true))
                                    {
                                        _sameVertex = true;
                                        e1 = true;
                                        h2 = true;
                                    }
                                    if (SameVertex(false, false))
                                    {
                                        _sameVertex = true;
                                        e1 = true;
                                        e2 = true;
                                    }
                                }

                                _nbPoints = _nbSegments = 0;
                                _iInterf = 1;

                                if (_intersected)
                                { // compute real intersection

                                    double da1 = 0;
                                    double db1 = 0;
                                    double da2 = 0;
                                    double db2 = 0;

                                    if (_sameVertex || _le == _fe)
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
                                    int noInter = 0;
                                    if (_le == _fe)
                                    {
                                        _intersector.Perform(_leData!, da1, db1);
                                    }
                                    else
                                    {
                                        _reject.GetSingleIntersection(_le, _fe, out double su, out double sv);
                                        if (su != RealLast())
                                        {
                                            _intersector.SimulateOnePoint(_leData!, su, _feData, sv);
                                            //-- printf("\n Single Point  : %d %d   %g %g\n",myLE,myFE,su,sv);
                                        }
                                        else
                                        {
                                            _intersector.Perform(_le, _leData!, da1, db1, _fe, _feData, da2, db2,
                                                                  _sameVertex);
                                            if (_intersector.IsDone())
                                            {
                                                if (_intersector.NbPoints() == 1 && _intersector.NbSegments() == 0)
                                                {
                                                    _reject.SetIntersection(_le, _fe, _intersector.Point(1));
                                                }
                                            }
                                        }
                                        noInter = 0;
                                    }
                                    if (noInter != 0)
                                    {
                                        _nbPoints = _nbSegments = 0;
                                    }
                                    else
                                    {
                                        if (_intersector.IsDone())
                                        {
                                            _nbPoints = _intersector.NbPoints();
                                            _nbSegments = _intersector.NbSegments();
                                            if ((_nbSegments + _nbPoints) == 0)
                                            {
                                                _reject.SetNoIntersection(_le, _fe);
                                            }
                                        }
                                        else
                                        {
                                            _nbPoints = _nbSegments = 0;
                                            // OCCT_DEBUG trace left out
                                        }
                                    }
                                }
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
            _faceItr1.NextEdge();
        }
    }

    /// <summary>
    /// Builds the current interference; true if it is rejected (above the face, or not crossing it as the hiding
    /// needs).
    /// </summary>
    public bool RejectedInterference()
    {
        if (_iInterf <= _nbPoints)
        {
            return RejectedPoint(_intersector.Point(_iInterf), External, 0);
        }
        else
        {
            int n = _iInterf - _nbPoints;
            bool firstPoint = (n & 1) != 0;
            int nseg = n >> 1;
            if (firstPoint)
            {
                nseg++;
            }
            double pf = _leGeom!.Parameter3d(_intersector.Segment(nseg).FirstPoint().ParamOnFirst());
            double pl = _leGeom.Parameter3d(_intersector.Segment(nseg).LastPoint().ParamOnFirst());
            if (pf > pl)
            {
                firstPoint = !firstPoint;
            }

            if (firstPoint)
            {
                bool ret1 = RejectedPoint(_intersector.Segment(nseg).FirstPoint(), Forward, nseg);
                return (ret1);
            }
            else
            {
                bool ret2 = RejectedPoint(_intersector.Segment(nseg).LastPoint(), Reversed, -nseg);
                return (ret2);
            }
        }
    }

    /// <summary>True if the last interference built is above the face.</summary>
    public bool AboveInterference() => _aboveIntf;

    /// <summary>The current interference (a reference, as OCCT's).</summary>
    public ref Interference Interference() => ref _intf;

    /// <summary>The tangent, normal and curvature of the current edge at Param, in the projection plane.</summary>
    /// <exception cref="FailureException">The tangent is undefined.</exception>
    public void LocalLEGeometry2D(double param, out Direction2D tg, out Direction2D nm, out double cu)
    {
        _llProps.SetParameter(param);
        if (!_llProps.IsTangentDefined())
        {
            throw new FailureException("HLRBRep_Data::LocalGeometry2D");
        }
        _llProps.Tangent(out tg);
        cu = _llProps.Curvature();
        if (cu > Epsilon(1.0) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(cu))
        {
            _llProps.Normal(out nm);
        }
        else
        {
            nm = new Direction2D(-tg.Y(), tg.X());
        }
    }

    /// <summary>The tangent, normal and curvature of the edge FE at Param, in the projection plane.</summary>
    /// <exception cref="FailureException">The tangent is undefined.</exception>
    public void LocalFEGeometry2D(int fe, double param, out Direction2D tg, out Direction2D nm, out double cu)
    {
        HiddenLineCurve curve = _eData[fe].ChangeGeometry();
        _flProps.SetCurve(curve);
        _flProps.SetParameter(param);
        if (!_flProps.IsTangentDefined())
        {
            throw new FailureException("HLRBRep_Data::LocalGeometry2D");
        }
        _flProps.Tangent(out tg);
        cu = _flProps.Curvature();
        if (cu > Epsilon(1.0) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(cu))
        {
            _flProps.Normal(out nm);
        }
        else
        {
            nm = new Direction2D(-tg.Y(), tg.X());
        }
    }

    /// <summary>The 3D states of the current edge before and after the point p1 where it is on the face boundary.</summary>
    public void EdgeState(double p1, double p2, out Containment stbef, out Containment staft)
    {
        // compute the state of The Edge near the Intersection
        // this method should give the states before and after
        // it should get the parameters on the surface

        double pu, pv;
        if (EdgeFaceTool.UVPoint(p2, _feGeom!, _iFaceGeom!, out pu, out pv))
        {
            _slProps.SetParameters(pu, pv);
            if (_slProps.IsNormalDefined)
            {
                Direction3D nrmFace = _slProps.Normal();

                _leGeom!.D1(p1, out Point3D pbid, out Vector3D tngEdge);

                Transform3D ti = _proj.Value.InvertedTransformation();
                Direction3D v;
                if (_proj.Value.Perspective())
                {
                    Point2D p2d = new();
                    _proj.Value.Project(pbid, ref p2d);
                    v = new Direction3D(p2d.X(), p2d.Y(), -_proj.Value.Focus());
                }
                else
                {
                    v = new Direction3D(Direction3D.D.NZ);
                }
                v.Transform(ti);
                if (nrmFace.Dot(v) > 0.0)
                {
                    nrmFace.Reverse();
                }

                double scal = (tngEdge.SquareMagnitude() > 1.0e-10) ? nrmFace.Dot(new Direction3D(tngEdge)) : 0.0;

                if (scal > _toler * 10)
                {
                    stbef = global::Kerf.Topology.Containment.In;
                    staft = global::Kerf.Topology.Containment.Out;
                }
                else if (scal < -_toler * 10)
                {
                    stbef = global::Kerf.Topology.Containment.Out;
                    staft = global::Kerf.Topology.Containment.In;
                }
                else
                {
                    stbef = On;
                    staft = On;
                }
            }
            else
            {
                stbef = global::Kerf.Topology.Containment.Out;
                staft = global::Kerf.Topology.Containment.Out;
                // OCCT_DEBUG trace left out
            }
        }
        else
        {
            stbef = global::Kerf.Topology.Containment.Out;
            staft = global::Kerf.Topology.Containment.Out;
            // OCCT_DEBUG trace left out
        }
    }

    /// <summary>True if the edge ED is an edge of the face hiding it (its hiding count).</summary>
    public bool EdgeOfTheHidingFace(int e, HiddenLineEdgeData ed) => ed.HideCount() == _hideCount - 1;

    /// <summary>The number of times the start of the edge is hidden by the face, from its interferences IL.</summary>
    public int HidingStartLevel(int e, HiddenLineEdgeData ed, NCollection_List<Interference> il)
    {
        bool loop;
        NCollection_List<Interference>.Iterator it = new();
        HiddenLineCurve ec = ed.Geometry();
        double sta = ec.Parameter3d(ec.FirstParameter());
        double end = ec.Parameter3d(ec.LastParameter());
        double tolpar = (end - sta) * 0.01;
        double param;
        loop = true;
        it.Initialize(il);

        while (it.More() && loop)
        {
            param = it.Value().Intersection().Parameter();
            if (param > end)
            {
                loop = false;
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
            it.Next();
        }
        param = 0.5 * (sta + end);
        int level = 0;
        /*TopAbs_State st = */
        Classify(e, ed, true, ref level, param);
        loop = true;
        it.Initialize(il);

        while (it.More() && loop)
        {
            Interference Int = it.Value();
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
                loop = false;
            }
            else
            {
                // OCCT_DEBUG trace left out
            }
            it.Next();
        }
        return level;
    }

    /// <summary>The state of the edge ED against the current face (IN if hidden by it).</summary>
    public Containment Compare(int e, HiddenLineEdgeData ed)
    {
        int level = 0;
        double parbid = 0.0;
        return Classify(e, ed, false, ref level, parbid);
    }

    /// <summary>Orients the outlines of the face FD as the projection sees them; true if the face is inverted.</summary>
    private bool OrientOutLine(int i, HiddenLineFaceData fd)
    {
        _ = i; // avoid compiler warning

        WiresBlock wb = fd.Wires()!;
        int nw = wb.WireCount;
        int iw1, ie1, ne1;
        Transform3D t = _proj.Value.Transformation();
        Transform3D ti = _proj.Value.InvertedTransformation();
        bool inverted = false;
        bool firstInversion = true;

        for (iw1 = 1; iw1 <= nw; iw1++)
        {
            EdgesBlock eb1 = wb.Wire(iw1)!;
            ne1 = eb1.EdgeCount;

            for (ie1 = 1; ie1 <= ne1; ie1++)
            {
                _fe = eb1.Edge(ie1);
                HiddenLineEdgeData ed1 = _eData[_fe];
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
                    _feGeom = ed1.ChangeGeometry();
                    HiddenLineCurve ec = ed1.Geometry();
                    int vsta = ed1.VSta();
                    int vend = ed1.VEnd();
                    if (vsta == 0 && vend == 0)
                    {
                        p = 0;
                    }
                    else if (vsta == 0)
                    {
                        p = ec.Parameter3d(ec.LastParameter());
                    }
                    else if (vend == 0)
                    {
                        p = ec.Parameter3d(ec.FirstParameter());
                    }
                    else
                    {
                        p = ec.Parameter3d((ec.LastParameter() + ec.FirstParameter()) / 2);
                    }
                    if (EdgeFaceTool.UVPoint(p, _feGeom, _iFaceGeom!, out pu, out pv))
                    {
                        _slProps.SetParameters(pu, pv);
                        ec.D1(p, out Point3D pt, out Vector3D tg);
                        Direction3D v;
                        if (_proj.Value.Perspective())
                        {
                            Point2D p2d = new();
                            _proj.Value.Project(pt, ref p2d);
                            v = new Direction3D(p2d.X(), p2d.Y(), -_proj.Value.Focus());
                        }
                        else
                        {
                            v = new Direction3D(Direction3D.D.NZ);
                        }
                        v.Transform(ti);
                        if (_slProps.IsNormalDefined)
                        {
                            double curv = EdgeFaceTool.CurvatureValue(_iFaceGeom!, pu, pv, v);
                            Vector3D nm = new(_slProps.Normal());
                            if (curv == 0)
                            {
                                // OCCT_DEBUG trace left out
                            }
                            if (curv > 0)
                            {
                                nm.Reverse();
                            }
                            tg.Transform(t);
                            pt.Transform(t);
                            nm.Transform(t);
                            nm.Cross(tg);
                            if (tg.Magnitude() < Constants.Resolution())
                            {
                                // OCCT_DEBUG trace left out
                            }
                            if (_proj.Value.Perspective())
                            {
                                r = nm.Z() * _proj.Value.Focus() - (nm.X() * pt.X() + nm.Y() * pt.Y() + nm.Z() * pt.Z());
                            }
                            else
                            {
                                r = nm.Z();
                            }
                            _feOri = (r > 0) ? Forward : Reversed;
                            if (!fd.Cut() && fd.Closed() && firstInversion)
                            {
                                if ((eb1.Orientation(ie1) == _feOri) != (fd.Orientation() == Forward))
                                {
                                    firstInversion = false;
                                    inverted = true;
                                }
                            }
                            eb1.Orientation(ie1, _feOri);
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
    private void OrientOthEdge(int i, HiddenLineFaceData fd)
    {
        double p, pu, pv, r;
        WiresBlock wb = fd.Wires()!;
        int nw = wb.WireCount;
        int iw1, ie1, ne1;
        Transform3D t = _proj.Value.Transformation();

        for (iw1 = 1; iw1 <= nw; iw1++)
        {
            EdgesBlock eb1 = wb.Wire(iw1)!;
            ne1 = eb1.EdgeCount;

            for (ie1 = 1; ie1 <= ne1; ie1++)
            {
                _fe = eb1.Edge(ie1);
                _feOri = eb1.Orientation(ie1);
                HiddenLineEdgeData ed1 = _eData[_fe];

                if (!ed1.Used())
                {
                    ed1.Used(true);
                    _feGeom = ed1.ChangeGeometry();
                    HiddenLineCurve ec = ed1.Geometry();
                    p = ec.Parameter3d((ec.LastParameter() + ec.FirstParameter()) / 2);
                    if (EdgeFaceTool.UVPoint(p, _feGeom, _iFaceGeom!, out pu, out pv))
                    {
                        Point3D pt = ec.Value3D(p);
                        _slProps.SetParameters(pu, pv);
                        if (_slProps.IsNormalDefined)
                        {
                            Vector3D nm = new(_slProps.Normal());
                            pt.Transform(t);
                            nm.Transform(t);
                            if (_proj.Value.Perspective())
                            {
                                r = nm.Z() * _proj.Value.Focus() - (nm.X() * pt.X() + nm.Y() * pt.Y() + nm.Z() * pt.Z());
                            }
                            else
                            {
                                r = nm.Z();
                            }
                            if (r < 0)
                            {
                                _feOri = global::Kerf.Engine.Topology.TopAbs.Reverse(_feOri);
                                eb1.Orientation(ie1, _feOri);
                            }
                        }
                    }
                    // OCCT_DEBUG trace left out
                    _ = i; // avoid compiler warning
                }
            }
        }
    }

    private static void REJECT1(double[] deca, ReadOnlySpan<double> totMin, ReadOnlySpan<double> totMax,
                                double[] surD, ref EdgesBlock.MinMaxIndices vertMin,
                                ref EdgesBlock.MinMaxIndices vertMax)
    {
        vertMin.Min[0] = (int)((deca[0] + totMin[0]) * surD[0]);
        vertMax.Min[0] = (int)((deca[0] + totMax[0]) * surD[0]);
        vertMin.Min[1] = (int)((deca[1] + totMin[1]) * surD[1]);
        vertMax.Min[1] = (int)((deca[1] + totMax[1]) * surD[1]);
        vertMin.Min[2] = (int)((deca[2] + totMin[2]) * surD[2]);
        vertMax.Min[2] = (int)((deca[2] + totMax[2]) * surD[2]);
        vertMin.Min[3] = (int)((deca[3] + totMin[3]) * surD[3]);
        vertMax.Min[3] = (int)((deca[3] + totMax[3]) * surD[3]);
        vertMin.Min[4] = (int)((deca[4] + totMin[4]) * surD[4]);
        vertMax.Min[4] = (int)((deca[4] + totMax[4]) * surD[4]);
        vertMin.Min[5] = (int)((deca[5] + totMin[5]) * surD[5]);
        vertMax.Min[5] = (int)((deca[5] + totMax[5]) * surD[5]);
        vertMin.Min[6] = (int)((deca[6] + totMin[6]) * surD[6]);
        vertMax.Min[6] = (int)((deca[6] + totMax[6]) * surD[6]);
        vertMin.Min[7] = (int)((deca[7] + totMin[7]) * surD[7]);
        vertMax.Min[7] = (int)((deca[7] + totMax[7]) * surD[7]);
        vertMin.Max[0] = (int)((deca[8] + totMin[8]) * surD[8]);
        vertMax.Max[0] = (int)((deca[8] + totMax[8]) * surD[8]);
        vertMin.Max[1] = (int)((deca[9] + totMin[9]) * surD[9]);
        vertMax.Max[1] = (int)((deca[9] + totMax[9]) * surD[9]);
        vertMin.Max[2] = (int)((deca[10] + totMin[10]) * surD[10]);
        vertMax.Max[2] = (int)((deca[10] + totMax[10]) * surD[10]);
        vertMin.Max[3] = (int)((deca[11] + totMin[11]) * surD[11]);
        vertMax.Max[3] = (int)((deca[11] + totMax[11]) * surD[11]);
        vertMin.Max[4] = (int)((deca[12] + totMin[12]) * surD[12]);
        vertMax.Max[4] = (int)((deca[12] + totMax[12]) * surD[12]);
        vertMin.Max[5] = (int)((deca[13] + totMin[13]) * surD[13]);
        vertMax.Max[5] = (int)((deca[13] + totMax[13]) * surD[13]);
        vertMin.Max[6] = (int)((deca[14] + totMin[14]) * surD[14]);
        vertMax.Max[6] = (int)((deca[14] + totMax[14]) * surD[14]);
        vertMin.Max[7] = (int)((deca[15] + totMin[15]) * surD[15]);
        vertMax.Max[7] = (int)((deca[15] + totMax[15]) * surD[15]);
    }

    /// <summary>
    /// The state of the edge ED against the current face: classifies a point of the edge (the parameter param with
    /// LevelFlag, counting in Level how many times the face covers it) by shooting a line of sight at the face.
    /// </summary>
    public Containment Classify(int e, HiddenLineEdgeData ed, bool levelFlag, ref int level, double param)
    {
        _ = e; // avoid compiler warning

        EdgesBlock.MinMaxIndices vertMin = new(), vertMax = new(), minMaxVert = new();
        Span<double> totMin = stackalloc double[16];
        Span<double> totMax = stackalloc double[16];

        int i;
        level = 0;
        Containment state = global::Kerf.Topology.Containment.Out;
        //  Standard_Boolean rej = Standard_False;
        HiddenLineCurve ec = ed.Geometry();
        double sta, xsta, ysta, zsta, end, xend, yend, zend;
        double tol = (double)(ed.Tolerance());

        if (levelFlag)
        {
            sta = param;
            _proj.Value.Project(ec.Value3D(sta), out xsta, out ysta, out zsta);

            HlrAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), totMin, totMax);
            HlrAlgo.UpdateMinMax(xsta, ysta, zsta, totMin, totMax);
            HlrAlgo.EnlargeMinMax(tol, totMin, totMax);
            REJECT1(_deca, totMin, totMax, _surD, ref vertMin, ref vertMax);

            HlrAlgo.EncodeMinMax(ref vertMin, ref vertMax, ref minMaxVert);
            if (ZRejected(IFaceMinMax, minMaxVert))
            { //-- Rejection en Z
                return state;
            }
        }
        else
        {
            sta = ec.Parameter3d(ec.FirstParameter());
            _proj.Value.Project(ec.Value3D(sta), out xsta, out ysta, out zsta);

            HlrAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), totMin, totMax);
            HlrAlgo.UpdateMinMax(xsta, ysta, zsta, totMin, totMax);
            HlrAlgo.EnlargeMinMax(tol, totMin, totMax);
            REJECT1(_deca, totMin, totMax, _surD, ref vertMin, ref vertMax);

            HlrAlgo.EncodeMinMax(ref vertMin, ref vertMax, ref minMaxVert);
            if (ZRejected(IFaceMinMax, minMaxVert))
            { //-- Rejection en Z
                return state;
            }
            end = ec.Parameter3d(ec.LastParameter());
            _proj.Value.Project(ec.Value3D(end), out xend, out yend, out zend);

            HlrAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), totMin, totMax);
            HlrAlgo.UpdateMinMax(xend, yend, zend, totMin, totMax);
            HlrAlgo.EnlargeMinMax(tol, totMin, totMax);
            REJECT1(_deca, totMin, totMax, _surD, ref vertMin, ref vertMax);

            HlrAlgo.EncodeMinMax(ref vertMin, ref vertMax, ref minMaxVert);
            if (ZRejected(IFaceMinMax, minMaxVert))
            { //-- Rejection en Z
                return state;
            }
            sta = 0.4 * sta + 0.6 * end; // dangerous if it is the middle
            _proj.Value.Project(ec.Value3D(sta), out xsta, out ysta, out zsta);

            HlrAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), totMin, totMax);
            HlrAlgo.UpdateMinMax(xsta, ysta, zsta, totMin, totMax);
            HlrAlgo.EnlargeMinMax(tol, totMin, totMax);
            REJECT1(_deca, totMin, totMax, _surD, ref vertMin, ref vertMax);

            HlrAlgo.EncodeMinMax(ref vertMin, ref vertMax, ref minMaxVert);
            // OCCT's commented-out OCCT_DEBUG dump is left out
            if (ZRejected(IFaceMinMax, minMaxVert))
            { //-- Rejection en Z
                return state;
            }
        }

        Point3D pLim;
        Point2D psta;
        psta = ec.Value(sta);
        pLim = ec.Value3D(sta);

        // OCCT's debug dump of the point (a function-scope static flag never set) is left out

        Line3D l = _proj.Value.Shoot(psta.X(), psta.Y());
        double wLim = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, pLim);
        _intersector.Perform(l, wLim);
        if (_intersector.IsDone())
        {
            int nbPoints = _intersector.NbPoints();
            if (nbPoints > 0)
            {
                double tolZ = _bigSize * 0.000001;
                if (_iFaceTest)
                {
                    if (!_leOutLine && !_leInternal)
                    {
                        tolZ = _bigSize * 0.001;
                    }
                    else
                    {
                        tolZ = _bigSize * 0.01;
                    }
                }
                wLim -= tolZ;
                double periodU, periodV, uMin = 0.0, uMax = 0.0, vMin = 0.0, vMax = 0.0;
                if (_iFaceGeom!.IsUPeriodic)
                {
                    periodU = _iFaceGeom.UPeriod();
                    uMin = _iFaceGeom.FirstUParameter();
                    uMax = _iFaceGeom.LastUParameter();
                }
                else
                {
                    periodU = 0.0;
                }
                if (_iFaceGeom.IsVPeriodic)
                {
                    periodV = _iFaceGeom.VPeriod();
                    vMin = _iFaceGeom.FirstVParameter();
                    vMax = _iFaceGeom.LastVParameter();
                }
                else
                {
                    periodV = 0;
                }
                Point3D pInter;
                double u, v, w;
                TransitionOnCurve tr;

                for (i = 1; i <= nbPoints; i++)
                {
                    _intersector.CSPoint(i).Values(out pInter, out u, out v, out w, out tr);
                    if (w < wLim)
                    {
                        double dummyShift;
                        if (periodU > 0.0)
                        {
                            global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(u, uMin, uMax, periodU, out u, out dummyShift);
                        }
                        if (periodV > 0.0)
                        {
                            global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(v, vMin, vMax, periodV, out v, out dummyShift);
                        }

                        Point2D pnt2d = new(u, v);
                        if (_classifier!.Classify(pnt2d, global::Kerf.Engine.Foundation.Precision.PConfusion()) != global::Kerf.Topology.Containment.Out)
                        {
                            state = global::Kerf.Topology.Containment.In;
                            level++;
                            if (!levelFlag)
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
    public Containment SimplClassify(int e, HiddenLineEdgeData ed, int nbp, double p1, double p2)
    {
        EdgesBlock.MinMaxIndices vertMin = new(), vertMax = new(), minMaxVert = new();
        Span<double> totMin = stackalloc double[16];
        Span<double> totMax = stackalloc double[16];

        int i;
        Containment state = global::Kerf.Topology.Containment.In;
        //  Standard_Boolean rej = Standard_False;
        HiddenLineCurve ec = ed.Geometry();
        double sta, xsta, ysta, zsta, dp;
        double tol = (double)(ed.Tolerance());

        dp = (p2 - p1) / (nbp + 1);

        for (sta = p1 + dp, i = 1; i <= nbp; ++i, sta += dp)
        {
            _proj.Value.Project(ec.Value3D(sta), out xsta, out ysta, out zsta);

            HlrAlgo.InitMinMax(global::Kerf.Engine.Foundation.Precision.Infinite(), totMin, totMax);
            HlrAlgo.UpdateMinMax(xsta, ysta, zsta, totMin, totMax);
            HlrAlgo.EnlargeMinMax(tol, totMin, totMax);
            REJECT1(_deca, totMin, totMax, _surD, ref vertMin, ref vertMax);

            HlrAlgo.EncodeMinMax(ref vertMin, ref vertMax, ref minMaxVert);
            if (ZRejected(IFaceMinMax, minMaxVert))
            { //-- Rejection en Z
                return global::Kerf.Topology.Containment.Out;
            }
        }
        return state;
    }

    /// <summary>Builds the interference of the intersection point PInter; true if it is rejected.</summary>
    private bool RejectedPoint(in IntersectionPoint2D pInter, Orientation boundOri, int numSeg)
    {
        int ind = 0;
        int decal;
        double p1, p2, dz;
        float t1, t2;
        Containment st;
        Orientation orie = Forward;
        Orientation or2 = Internal;
        bool inverted = false;
        Transition tr1;
        Transition tr2;
        double tolZ = _bigSize * 0.00001;

        p1 = _leGeom!.Parameter3d(pInter.ParamOnFirst());
        p2 = _feGeom!.Parameter3d(pInter.ParamOnSecond());
        dz = _leGeom.Z(p1) - _feGeom.Z(p2);

        if (_le == _fe)
        { // auto intersection can be inverted
            if (dz >= tolZ)
            {
                inverted = true;
                double p = p1;
                p1 = p2;
                p2 = p;
                dz = -dz;
            }
        }

        if (dz >= tolZ)
        {
            _aboveIntf = true;
            return true;
        }
        _aboveIntf = false;
        st = (dz <= -tolZ) ? global::Kerf.Topology.Containment.In : On;

        if (inverted)
        {
            tr1 = pInter.TransitionOfSecond();
            tr2 = pInter.TransitionOfFirst();
        }
        else
        {
            tr1 = pInter.TransitionOfFirst();
            tr2 = pInter.TransitionOfSecond();
        }

        if (_iFaceTest)
        {
            if (_le == _fe)
            {
                if (st == global::Kerf.Topology.Containment.In)
                {
                    _leData!.Simple(false);
                }
            }
            else
            {
                if (_sameVertex)
                {
                    if ((st == On) || (tr1.PositionOnCurve() != Position2D.Middle)
                        || (tr2.PositionOnCurve() != Position2D.Middle))
                    {
                        return true;
                    }
                }
            }
            if (st == global::Kerf.Topology.Containment.In)
            {
                _iFaceSmpl = false;
            }
        }

        switch (tr1.TransitionType())
        { // compute the transition
            case TypeTrans.In:
                orie = (_feOri == Reversed ? Reversed : Forward);
                break;
            case TypeTrans.Out:
                orie = (_feOri == Reversed ? Forward : Reversed);
                break;
            case TypeTrans.Touch:
                switch (tr1.Situation())
                {
                    case Situation.Inside:
                        orie = (_feOri == Reversed ? External : Internal);
                        break;
                    case Situation.Outside:
                        orie = (_feOri == Reversed ? Internal : External);
                        break;
                    case Situation.Unknown:
                        return true;
                }
                break;
            case TypeTrans.Undecided:
                return true;
        }

        if (_iFaceBack)
        {
            orie = global::Kerf.Engine.Topology.TopAbs.Complement(orie); // change the transition
        }
        Orientation ori = Forward;
        switch (tr1.PositionOnCurve())
        {
            case Position2D.Head:
                ori = Forward;
                break;
            case Position2D.Middle:
                ori = Internal;
                break;
            case Position2D.End:
                ori = Reversed;
                break;
        }

        if (st != global::Kerf.Topology.Containment.Out)
        {
            if (tr2.PositionOnCurve() != Position2D.Middle)
            { // correction de la transition  sur myFE
                if (_sameVertex)
                {
                    return true; // si intersection a une extremite verticale !
                }

                bool douteux = false;
                double psav = p2;
                Point2D ptsav;
                Vector2D tgsav = new(), nmsav;
                if (tr2.PositionOnCurve() == Position2D.Head)
                {
                    ind = _feData!.VSta();
                    or2 = Forward;
                    AdjustParameter(_feData, true, ref p2, out t2);
                    if (_feData.VerAtSta())
                    {
                        douteux = true;
                        _feGeom.D2(psav, out ptsav, out tgsav, out nmsav);
                        if (tgsav.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
                        {
                            tgsav = nmsav;
                        }
                    }
                }
                else
                {
                    ind = _feData!.VEnd();
                    or2 = Reversed;
                    AdjustParameter(_feData, false, ref p2, out t2);
                    if (_feData.VerAtEnd())
                    {
                        douteux = true;
                        _feGeom.D2(psav, out ptsav, out tgsav, out nmsav);
                        if (tgsav.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
                        {
                            tgsav = nmsav;
                        }
                    }
                }
                _feGeom.D1(p2, out ptsav, out Vector2D tgFE);
                if (douteux)
                {
                    if (tgFE.XY().Dot(tgsav.XY()) < 0.0)
                    {
                        if (orie == Forward)
                        {
                            orie = Reversed;
                        }
                        else if (orie == Reversed)
                        {
                            orie = Forward;
                        }
                    }
                }
                _intf.ChangeBoundary().Set2D(_fe, p2);
            }
            if (ori != Internal)
            {                           // correction de la transition  sur myLE
                bool douteux = false; // si intersection a une extremite verticale !
                double psav = p1;
                Point2D ptsav;
                Vector2D tgsav = new(), nmsav;
                if (ori == Forward)
                {
                    AdjustParameter(_leData!, true, ref p1, out t1);
                    if (_leData!.VerAtSta())
                    {
                        douteux = true;
                        _leGeom.D2(psav, out ptsav, out tgsav, out nmsav);
                        if (tgsav.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
                        {
                            tgsav = nmsav;
                        }
                    }
                }
                else
                {
                    AdjustParameter(_leData!, false, ref p1, out t1);
                    if (_leData!.VerAtEnd())
                    {
                        douteux = true;
                        _leGeom.D2(psav, out ptsav, out tgsav, out nmsav);
                        if (tgsav.SquareMagnitude() <= DERIVEE_PREMIERE_NULLE)
                        {
                            tgsav = nmsav;
                        }
                    }
                }
                if (douteux)
                {
                    _leGeom.D1(p1, out ptsav, out Vector2D tgLE);
                    if (tgLE.XY().Dot(tgsav.XY()) < 0.0)
                    {
                        if (orie == Forward)
                        {
                            orie = Reversed;
                        }
                        else if (orie == Reversed)
                        {
                            orie = Forward;
                        }
                    }
                }
            }
            if (st == On)
            {
                EdgeState(p1, p2, out Containment stbef, out Containment staft);
                _intf.ChangeBoundary().SetState3D(stbef, staft);
            }
        }

        if (_feInternal)
        {
            decal = 2;
        }
        else
        {
            decal = 1;
            if (st == global::Kerf.Topology.Containment.In && ori == Forward && orie == Forward)
            {
                decal = 0;
            }
        }
        ref AlgoIntersection inter = ref _intf.ChangeIntersection();
        inter.Orientation(ori);
        inter.Level(decal);
        inter.SegIndex(numSeg);
        inter.Index(ind);
        inter.Parameter(p1);
        inter.Tolerance(_leTol);
        inter.State(st);
        _intf.Orientation(or2);
        _intf.Transition(orie);
        _intf.BoundaryTransition(boundOri);
        _intf.ChangeBoundary().Set2D(_fe, p2);
        return false;
    }

    /// <summary>True if the vertex at the head (h1) or end of the current edge is the head (h2) or end of the boundary edge.</summary>
    private bool SameVertex(bool h1, bool h2)
    {
        int v1, v2;
        if (h1)
        {
            v1 = _leData!.VSta();
        }
        else
        {
            v1 = _leData!.VEnd();
        }
        if (h2)
        {
            v2 = _feData!.VSta();
        }
        else
        {
            v2 = _feData!.VEnd();
        }
        bool sameV = v1 == v2;
        if (sameV)
        {
            _intersected = true; // compute the intersections
            if ((_leType == Line || _leType == Circle || _leType == Ellipse)
                && (_feType == Line || _feType == Circle || _feType == Ellipse))
            {
                _intersected = false; // no other intersection
            }

            bool otherCase = true;

            if ((h1 && _leData.OutLVSta()) || (!h1 && _leData.OutLVEnd()))
            {
                if (_iFaceTest || _leInternal)
                {
                    otherCase = false;
                }
            }
            else if (_iFaceTest)
            {
                otherCase = false;
            }

            if (otherCase)
            {
                if ((h1 && _leData.CutAtSta()) || (!h1 && _leData.CutAtEnd()))
                {
                    _intersected = false; // two connected OutLines do not
                }                          // intersect themselves.
            }
        }
        return sameV;
    }

    /// <summary>True if the current face is periodic with bounds spanning more than two periods.</summary>
    public bool IsBadFace()
    {
        if (_iFaceGeom is not null)
        {
            // check for garbage data - if periodic then bounds must not exceed period
            HlrSurface pGeom = _iFaceGeom;
            if (pGeom.IsUPeriodic)
            {
                double period = pGeom.UPeriod();
                double min = pGeom.FirstUParameter();
                double max = pGeom.LastUParameter();
                if (period * 2 < max - min)
                {
                    return true;
                }
            }
            if (pGeom.IsVPeriodic)
            {
                double period = pGeom.VPeriod();
                double min = pGeom.FirstVParameter();
                double max = pGeom.LastVParameter();
                if (period * 2 < max - min)
                {
                    return true;
                }
            }
        }
        return false;
    }
}
