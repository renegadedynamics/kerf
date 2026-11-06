// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenExtPS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenExtPS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Extrema.ElementType;
using static Kerf.Engine.Extrema.ExtAlgo;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// All the extrema of the distance between a point and a surface, found by sampling: starting from the extrema of the
/// distance to a grid of points (a mesh of nodes, edges and faces, or a tree of spheres), each is refined on the
/// surface with math_FunctionSetRoot.
/// </summary>
/// <remarks>
/// The sampling must be fine enough that if there are N extrema on the surface, there are N extrema on the grid. The
/// surface adaptor is kept, as OCCT keeps a pointer: it must outlive the algorithm. The grid (or the tree) is built
/// once and reused for the next points, until <see cref="Initialize(SurfaceAdaptorBase, int, int, double, double)"/>
/// or a change of algorithm.
/// </remarks>
internal class GenExtPS
{
    // The selectors of the sphere tree (Extrema_GenExtPS.cxx). OCCT keeps references to the sphere array and to the
    // caller's solution; the port keeps the array and the solution itself, which Sphere() returns.
    private abstract class Bnd_SphereUBTreeSelector : SpatialTree<int, BoundingSphere>.Selector
    {
        protected Point3D _xyz;
        protected readonly NCollection_HArray1<BoundingSphere> _sphereArray;
        protected BoundingSphere _sol;

        protected Bnd_SphereUBTreeSelector(NCollection_HArray1<BoundingSphere> sphereArray, in BoundingSphere sol)
        {
            _xyz = new Point3D(0, 0, 0);
            _sphereArray = sphereArray;
            _sol = sol;
        }

        public void DefineCheckPoint(in Point3D xyz) => _xyz = xyz;

        public BoundingSphere Sphere() => _sol;
    }

    private sealed class Bnd_SphereUBTreeSelectorMin : Bnd_SphereUBTreeSelector
    {
        private double _minDist;

        public Bnd_SphereUBTreeSelectorMin(NCollection_HArray1<BoundingSphere> sphereArray, in BoundingSphere sol)
            : base(sphereArray, sol)
        {
            _minDist = RealLast();
        }

        public void SetMinDist(double minDist) => _minDist = minDist;

        public double MinDist() => _minDist;

        public override bool Reject(in BoundingSphere bnd)
        {
            // myMinDist is decreased each time a nearer object is found
            return bnd.IsOut(_xyz.XYZ(), ref _minDist);
        }

        public override bool Accept(in int ind)
        {
            BoundingSphere sph = _sphereArray.Value(ind);
            double curDist;

            //    if ( (aCurDist = aSph.SquareDistance(myXYZ.XYZ())) < mySol.SquareDistance(myXYZ.XYZ()) )
            if ((curDist = sph.Distance(_xyz.XYZ())) < _sol.Distance(_xyz.XYZ()))
            {
                _sol = sph;
                if (curDist < _minDist)
                {
                    _minDist = curDist;
                }

                return true;
            }

            return false;
        }
    }

    private sealed class Bnd_SphereUBTreeSelectorMax : Bnd_SphereUBTreeSelector
    {
        private double _maxDist;

        public Bnd_SphereUBTreeSelectorMax(NCollection_HArray1<BoundingSphere> sphereArray, in BoundingSphere sol)
            : base(sphereArray, sol)
        {
            // OCCT: Reject reads this as a maximal distance (Bnd_Sphere::IsOut), so with 0 every sphere is rejected
            // and the search for the maximum starts from the first grid node
            _maxDist = 0;
        }

        public void SetMaxDist(double maxDist) => _maxDist = maxDist;

        public double MaxDist() => _maxDist;

        public override bool Reject(in BoundingSphere bnd)
        {
            // myMaxDist is decreased each time a nearer object is found
            return bnd.IsOut(_xyz.XYZ(), ref _maxDist);
        }

        public override bool Accept(in int ind)
        {
            BoundingSphere sph = _sphereArray.Value(ind);
            double curDist;

            //    if ( (aCurDist = aSph.SquareDistance(myXYZ.XYZ())) > mySol.SquareDistance(myXYZ.XYZ()) )
            if ((curDist = sph.Distance(_xyz.XYZ())) > _sol.Distance(_xyz.XYZ()))
            {
                _sol = sph;
                if (curDist > _maxDist)
                {
                    _maxDist = curDist;
                }

                return true;
            }

            return false;
        }
    }

    private bool _done;
    private bool _init;
    private double _myumin;
    private double _myusup;
    private double _myvmin;
    private double _myvsup;
    private int _myusample;
    private int _myvsample;
    private double _mytolu;
    private double _mytolv;

    private readonly NCollection_Array2<POnSurfParams> _points = new();
    private SpatialTree<int, BoundingSphere>? _sphereUBTree;
    private NCollection_HArray1<BoundingSphere>? _sphereArray;
    private readonly FuncPSNorm _f;
    private SurfaceAdaptorBase? _s;
    private ExtFlag _flag;
    private ExtAlgo _algo;
    private NCollection_HArray1<double>? _uParams;
    private NCollection_HArray1<double>? _vParams;
    private readonly NCollection_Array2<POnSurfParams> _facePntParams = new();
    private readonly NCollection_Array2<POnSurfParams> _uEdgePntParams = new();
    private readonly NCollection_Array2<POnSurfParams> _vEdgePntParams = new();
    private POnSurfParams _gridParam = new();

    /*-----------------------------------------------------------------------------
    Function:
       Find all extremum distances between point P and surface
      S using sampling (NbU,NbV).

    Method:
       The algorithm bases on the hypothesis that sampling is precise enough,
      if there exist N extreme distances between the point and the surface,
      so there also exist N extrema between the point and the grid.
      So, the algorithm consists in starting from extrema of the grid to find the
      extrema of the surface.
      The extrema are calculated by the algorithm math_FunctionSetRoot with the
      following arguments:
      - F: Extrema_FuncExtPS created from P and S,
      - UV: math_Vector the components which of are parameters of the extremum on the
        grid,
      - Tol: std::min(TolU,TolV), (Prov.:math_FunctionSetRoot does not authorize a vector)
      - UVinf: math_Vector the components which of are lower limits of u and v,
      - UVsup: math_Vector the components which of are upper limits of u and v.
    -----------------------------------------------------------------------------*/

    /// <summary>An empty algorithm (<see cref="Initialize(SurfaceAdaptorBase, int, int, double, double)"/> sets the surface).</summary>
    public GenExtPS()
    {
        _myumin = 0.0;
        _myusup = 0.0;
        _myvmin = 0.0;
        _myvsup = 0.0;
        _myusample = 0;
        _myvsample = 0;
        _mytolu = 0.0;
        _mytolv = 0.0;
        _s = null;
        _f = new FuncPSNorm();
        _done = false;
        _init = false;
        _flag = ExtFlag_MINMAX;
        _algo = ExtAlgo_Grad;
    }

    /// <summary>
    /// Computes all the extrema of the distance between P and S on S's whole range, starting from a sampling NbU x NbV;
    /// TolU and TolV stop the iterations.
    /// </summary>
    public GenExtPS(in Point3D p, SurfaceAdaptorBase s, int nbU, int nbV, double tolU, double tolV,
        ExtFlag f = ExtFlag_MINMAX, ExtAlgo a = ExtAlgo_Grad)
    {
        _f = new FuncPSNorm(p, s);
        _flag = f;
        _algo = a;
        Initialize(s, nbU, nbV, tolU, tolV);
        Perform(p);
    }

    /// <summary>
    /// Computes all the extrema of the distance between P and S on [Umin, Usup] x [Vmin, Vsup], starting from a
    /// sampling NbU x NbV; TolU and TolV stop the iterations.
    /// </summary>
    public GenExtPS(in Point3D p, SurfaceAdaptorBase s, int nbU, int nbV, double umin, double usup, double vmin,
        double vsup, double tolU, double tolV, ExtFlag f = ExtFlag_MINMAX,
        ExtAlgo a = ExtAlgo_Grad)
    {
        _f = new FuncPSNorm(p, s);
        _flag = f;
        _algo = a;
        Initialize(s, nbU, nbV, umin, usup, vmin, vsup, tolU, tolV);
        Perform(p);
    }

    public void Initialize(SurfaceAdaptorBase s, int nbU, int nbV, double tolU, double tolV)
    {
        _myumin = s.FirstUParameter();
        _myusup = s.LastUParameter();
        _myvmin = s.FirstVParameter();
        _myvsup = s.LastVParameter();
        Initialize(s, nbU, nbV, _myumin, _myusup, _myvmin, _myvsup, tolU, tolV);
    }

    public void Initialize(SurfaceAdaptorBase s, int nbU, int nbV, double umin, double usup, double vmin, double vsup,
        double tolU, double tolV)
    {
        _s = s;
        _myusample = nbU;
        _myvsample = nbV;
        _mytolu = tolU;
        _mytolv = tolV;
        _myumin = umin;
        _myusup = usup;
        _myvmin = vmin;
        _myvsup = vsup;

        if ((_myusample < 2) || (_myvsample < 2))
        {
            throw new OutOfRangeException();
        }

        _f.Initialize(s);

        _sphereUBTree = null;
        _uParams = null;
        _vParams = null;
        _init = false;
    }

    private static void FillParams(NCollection_Array1<double> knots, int degree, double parMin,
        double parMax, ref NCollection_HArray1<double>? theParams, int sample)
    {
        List<double> aParams = new();
        int i = 1;
        double prevPar = parMin;
        aParams.Add(prevPar);
        // calculation the array of parametric points depending on the knots array variation and degree of
        // given surface
        for (; i < knots.Length() && knots[i] < (parMax - global::Kerf.Engine.Foundation.Precision.PConfusion()); i++)
        {
            if (knots[i + 1] < parMin + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                continue;
            }

            double step = (knots[i + 1] - knots[i]) / Max(degree, 2);
            int k = 1;
            for (; k <= degree; k++)
            {
                double par = knots[i] + k * step;
                if (par > parMax - global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    break;
                }
                if (par > prevPar + global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    aParams.Add(par);
                    prevPar = par;
                }
            }
        }
        aParams.Add(parMax);
        int nbPar = aParams.Count;
        // in case of an insufficient number of points the grid will be built later
        if (nbPar < sample)
        {
            return;
        }
        theParams = new NCollection_HArray1<double>(1, nbPar);
        for (i = 0; i < nbPar; i++)
        {
            theParams.SetValue(i + 1, aParams[i]);
        }
    }

    // Selection of points to build the grid, depending on the type of surface.
    private void GetGridPoints(SurfaceAdaptorBase surf)
    {
        // creation parametric points for BSpline and Bezier surfaces
        // with taking into account of Degree and NbKnots of BSpline or Bezier geometry
        if (surf.GetType() == SurfaceKind.OffsetSurface)
        {
            GetGridPoints(surf.BasisSurface());
        }
        // parametric points for BSpline surfaces
        else if (surf.GetType() == SurfaceKind.BSplineSurface)
        {
            global::Kerf.Geometry.BSplineSurface? bspl = surf.BSpline();
            if (bspl is not null)
            {
                NCollection_Array1<double> uKnots = bspl.InternalUKnots();
                NCollection_Array1<double> vKnots = bspl.InternalVKnots();
                FillParams(uKnots, bspl.UDegree, _myumin, _myusup, ref _uParams, _myusample);
                FillParams(vKnots, bspl.VDegree, _myvmin, _myvsup, ref _vParams, _myvsample);
            }
        }
        // calculation parametric points for Bezier surfaces
        else if (surf.GetType() == SurfaceKind.BezierSurface)
        {
            global::Kerf.Geometry.BezierSurface? bezier = surf.Bezier();
            if (bezier is null)
            {
                return;
            }

            NCollection_Array1<double> uKnots = new(1, 2);
            NCollection_Array1<double> vKnots = new(1, 2);
            bezier.ParameterBounds(out uKnots[1], out uKnots[2], out vKnots[1], out vKnots[2]);
            FillParams(uKnots, bezier.UDegree, _myumin, _myusup, ref _uParams, _myusample);
            FillParams(vKnots, bezier.VDegree, _myvmin, _myvsup, ref _vParams, _myvsample);
        }
        // creation points for surfaces based on BSpline or Bezier curves
        else if (surf.GetType() == SurfaceOfRevolution || surf.GetType() == SurfaceOfExtrusion)
        {
            NCollection_HArray1<double>? arrKnots = null;
            int degree = 0;
            if (surf.BasisCurve().GetType() == BSplineCurve)
            {
                BSplineCurve3D? bspl = surf.BasisCurve().BSpline();
                if (bspl is not null)
                {
                    arrKnots = new NCollection_HArray1<double>(bspl.InternalKnots());
                    degree = bspl.Degree;
                }
            }
            if (surf.BasisCurve().GetType() == BezierCurve)
            {
                BezierCurve3D? bez = surf.BasisCurve().Bezier();
                if (bez is not null)
                {
                    arrKnots = new NCollection_HArray1<double>(1, 2);
                    arrKnots.SetValue(1, bez.FirstParameter);
                    arrKnots.SetValue(2, bez.LastParameter);
                    degree = bez.Degree;
                }
            }
            if (arrKnots is null)
            {
                return;
            }
            if (surf.GetType() == SurfaceOfRevolution)
            {
                FillParams(arrKnots.Array1(), degree, _myvmin, _myvsup, ref _vParams, _myvsample);
            }
            else
            {
                FillParams(arrKnots.Array1(), degree, _myumin, _myusup, ref _uParams, _myusample);
            }
        }
        // update the number of points in sample
        if (_uParams is not null)
        {
            _myusample = _uParams.Length();
        }
        if (_vParams is not null)
        {
            _myvsample = _vParams.Length();
        }
    }

    // Computes the point on surface parameters on an edge. If it coincides with theParam0 or theParam1, that one is
    // returned. (OCCT returns a reference to theParam0, theParam1 or myGridParam; the callers copy it.)
    private POnSurfParams ComputeEdgeParameters(bool isUEdge, in POnSurfParams param0,
        in POnSurfParams param1, in Point3D point, double diffTol)
    {
        double sqrDist01 = param0.Value().SquareDistanceTo(param1.Value());

        if (sqrDist01 <= diffTol)
        {
            // The points are confused. Get the first point and change its type.
            return param0;
        }
        else
        {
            double diffDist = Math.Abs(param0.GetSqrDistance() - param1.GetSqrDistance());

            if (diffDist >= sqrDist01 - diffTol)
            {
                // The shortest distance is one of the nodes.
                if (param0.GetSqrDistance() > param1.GetSqrDistance())
                {
                    // The shortest distance is the point 1.
                    return param1;
                }
                else
                {
                    // The shortest distance is the point 0.
                    return param0;
                }
            }
            else
            {
                // The shortest distance is inside the edge.
                Xyz poP = point.XYZ().Subtracted(param0.Value().XYZ());
                Xyz poP1 = param1.Value().XYZ().Subtracted(param0.Value().XYZ());
                double ratio = poP.Dot(poP1) / sqrDist01;
                Span<double> u = stackalloc double[2];
                Span<double> v = stackalloc double[2];

                param0.Parameter(out u[0], out v[0]);
                param1.Parameter(out u[1], out v[1]);

                double uPar = u[0];
                double vPar = v[0];

                if (isUEdge)
                {
                    uPar += ratio * (u[1] - u[0]);
                }
                else
                {
                    vPar += ratio * (v[1] - v[0]);
                }

                _gridParam.SetParameters(uPar, vPar, _s!.Value(uPar, vPar));
                Span<int> indices = stackalloc int[2];

                param0.GetIndices(out indices[0], out indices[1]);
                _gridParam.SetElementType(isUEdge ? UIsoEdge : VIsoEdge);
                _gridParam.SetSqrDistance(point.SquareDistanceTo(_gridParam.Value()));
                _gridParam.SetIndices(indices[0], indices[1]);
                return _gridParam;
            }
        }
    }

    // Creation of the grid of parametric points.
    private void BuildGrid(in Point3D point)
    {
        // if grid was already built skip its creation
        if (!_init)
        {
            // build parametric grid in case of a complex surface geometry (BSpline and Bezier surfaces)
            GetGridPoints(_s!);

            // build grid in other cases
            if (_uParams is null)
            {
                double pasU = _myusup - _myumin;
                double u0 = pasU / _myusample / 100.0;
                pasU = (pasU - u0) / (_myusample - 1);
                u0 = u0 / 2.0 + _myumin;
                _uParams = new NCollection_HArray1<double>(1, _myusample);
                double U = u0;
                for (int noU = 1; noU <= _myusample; noU++, U += pasU)
                {
                    _uParams.SetValue(noU, U);
                }
            }

            if (_vParams is null)
            {
                double pasV = _myvsup - _myvmin;
                double v0 = pasV / _myvsample / 100.0;
                pasV = (pasV - v0) / (_myvsample - 1);
                v0 = v0 / 2.0 + _myvmin;

                _vParams = new NCollection_HArray1<double>(1, _myvsample);
                double V = v0;
                for (int noV = 1; noV <= _myvsample; noV++, V += pasV)
                {
                    _vParams.SetValue(noV, V);
                }
            }

            // If flag was changed and extrema not reinitialized Extrema would fail
            _points.Resize(0, _myusample + 1, 0, _myvsample + 1, false);

            SurfaceGrid gridEval = new(_s!);

            NCollection_Array2<Point3D> gridPoints = gridEval.EvaluateGrid(_uParams.Array1(), _vParams.Array1());

            if (gridPoints.IsEmpty())
            {
                return;
            }

            for (int noU = 1; noU <= _myusample; noU++)
            {
                for (int noV = 1; noV <= _myvsample; noV++)
                {
                    Point3D p1 = gridPoints.Value(noU, noV);
                    POnSurfParams param = new(_uParams.Value(noU), _vParams.Value(noV), p1);
                    param.SetElementType(Node);
                    param.SetIndices(noU, noV);
                    _points.SetValue(noU, noV, param);
                }
            }

            _facePntParams.Resize(0, _myusample, 0, _myvsample, false);
            _uEdgePntParams.Resize(1, _myusample - 1, 1, _myvsample, false);
            _vEdgePntParams.Resize(1, _myusample, 1, _myvsample - 1, false);

            // Fill boundary with negative square distance.
            // It is used for computation of Maximum.
            for (int noV = 0; noV <= _myvsample + 1; noV++)
            {
                _points.ChangeValue(0, noV).SetSqrDistance(-1.0);
                _points.ChangeValue(_myusample + 1, noV).SetSqrDistance(-1.0);
            }

            for (int noU = 1; noU <= _myusample; noU++)
            {
                _points.ChangeValue(noU, 0).SetSqrDistance(-1.0);
                _points.ChangeValue(noU, _myvsample + 1).SetSqrDistance(-1.0);
            }

            _init = true;
        }

        // Compute distances to mesh.
        // Step 1. Compute distances to nodes.
        for (int noU = 1; noU <= _myusample; noU++)
        {
            for (int noV = 1; noV <= _myvsample; noV++)
            {
                ref POnSurfParams param = ref _points.ChangeValue(noU, noV);

                param.SetSqrDistance(point.SquareDistanceTo(param.Value()));
            }
        }

        // For search of minimum compute distances to mesh.
        if (_flag == ExtFlag_MIN || _flag == ExtFlag_MINMAX)
        {
            // This is the tolerance of difference of squared values.
            // No need to set it too small.
            double diffTol = _mytolu + _mytolv;

            // Step 2. Compute distances to edges.
            // Assume UEdge(i, j) = { Point(i, j); Point(i + 1, j    ) }
            // Assume VEdge(i, j) = { Point(i, j); Point(i,     j + 1) }
            for (int noU = 1; noU <= _myusample; noU++)
            {
                for (int noV = 1; noV <= _myvsample; noV++)
                {
                    POnSurfParams param0 = _points.Value(noU, noV);

                    if (noU < _myusample)
                    {
                        // Compute parameters to UEdge.
                        POnSurfParams param1 = _points.Value(noU + 1, noV);
                        POnSurfParams edgeParam =
                            ComputeEdgeParameters(true, param0, param1, point, diffTol);

                        _uEdgePntParams.SetValue(noU, noV, edgeParam);
                    }

                    if (noV < _myvsample)
                    {
                        // Compute parameters to VEdge.
                        POnSurfParams param1 = _points.Value(noU, noV + 1);
                        POnSurfParams edgeParam =
                            ComputeEdgeParameters(false, param0, param1, point, diffTol);

                        _vEdgePntParams.SetValue(noU, noV, edgeParam);
                    }
                }
            }

            // Step 3. Compute distances to faces.
            // Assume myFacePntParams(i, j) =
            // { Point(i, j); Point(i + 1, j); Point(i + 1, j + 1); Point(i, j + 1) }
            //   Or
            // { UEdge(i, j); VEdge(i + 1, j); UEdge(i, j + 1); VEdge(i, j) }
            double sqrDist01;
            double diffDist;
            bool isOut;
            Span<double> u = stackalloc double[2]; // declared in the loop in OCCT
            Span<double> v = stackalloc double[2];

            for (int noU = 1; noU < _myusample; noU++)
            {
                for (int noV = 1; noV < _myvsample; noV++)
                {
                    POnSurfParams ue0 = _uEdgePntParams.Value(noU, noV);
                    POnSurfParams ue1 = _uEdgePntParams.Value(noU, noV + 1);
                    POnSurfParams ve0 = _vEdgePntParams.Value(noU, noV);
                    POnSurfParams ve1 = _vEdgePntParams.Value(noU + 1, noV);

                    sqrDist01 = ue0.Value().SquareDistanceTo(ue1.Value());
                    diffDist = Math.Abs(ue0.GetSqrDistance() - ue1.GetSqrDistance());
                    isOut = false;

                    if (diffDist >= sqrDist01 - diffTol)
                    {
                        // The projection is outside the face.
                        isOut = true;
                    }
                    else
                    {
                        sqrDist01 = ve0.Value().SquareDistanceTo(ve1.Value());
                        diffDist = Math.Abs(ve0.GetSqrDistance() - ve1.GetSqrDistance());

                        if (diffDist >= sqrDist01 - diffTol)
                        {
                            // The projection is outside the face.
                            isOut = true;
                        }
                    }

                    if (isOut)
                    {
                        // Get the closest point on an edge.
                        POnSurfParams ueMin = ue0.GetSqrDistance() < ue1.GetSqrDistance() ? ue0 : ue1;
                        POnSurfParams veMin = ve0.GetSqrDistance() < ve1.GetSqrDistance() ? ve0 : ve1;
                        POnSurfParams eMin =
                            ueMin.GetSqrDistance() < veMin.GetSqrDistance() ? ueMin : veMin;

                        _facePntParams.SetValue(noU, noV, eMin);
                    }
                    else
                    {
                        // Find closest point inside the face.
                        double uPar;
                        double vPar;

                        // Compute U parameter.
                        ue0.Parameter(out u[0], out v[0]);
                        ue1.Parameter(out u[1], out v[1]);
                        uPar = 0.5 * (u[0] + u[1]);
                        // Compute V parameter.
                        ve0.Parameter(out u[0], out v[0]);
                        ve1.Parameter(out u[1], out v[1]);
                        vPar = 0.5 * (v[0] + v[1]);

                        POnSurfParams param = new(uPar, vPar, _s!.Value(uPar, vPar));

                        param.SetElementType(Face);
                        param.SetSqrDistance(point.SquareDistanceTo(param.Value()));
                        param.SetIndices(noU, noV);
                        _facePntParams.SetValue(noU, noV, param);
                    }
                }
            }

            // Fill boundary with RealLast square distance.
            for (int noV = 0; noV <= _myvsample; noV++)
            {
                _facePntParams.ChangeValue(0, noV).SetSqrDistance(RealLast());
                _facePntParams.ChangeValue(_myusample, noV).SetSqrDistance(RealLast());
            }

            for (int noU = 1; noU < _myusample; noU++)
            {
                _facePntParams.ChangeValue(noU, 0).SetSqrDistance(RealLast());
                _facePntParams.ChangeValue(noU, _myvsample).SetSqrDistance(RealLast());
            }
        }
    }

    private static double LengthOfIso(SurfaceAdaptorBase s, IsoType iso, double par1, double par2,
        int nbPnts, double par)
    {
        double len = 0.0;
        int i;
        double dPar = (par2 - par1) / (nbPnts - 1);
        Point3D p1, p2;
        double aPar = par1 + dPar;
        if (iso == IsoU)
        {
            p1 = s.Value(par, par1);
        }
        else
        {
            p1 = s.Value(par1, par);
        }

        for (i = 2; i <= nbPnts; ++i)
        {
            if (iso == IsoU)
            {
                p2 = s.Value(par, aPar);
            }
            else
            {
                p2 = s.Value(aPar, par);
            }
            len += p1.DistanceTo(p2);
            p1 = p2;
            aPar += dPar;
        }
        return len;
    }

    private static void CorrectNbSamples(SurfaceAdaptorBase s, double u1, double u2, ref int nbU,
        double v1, double v2, ref int nbV)
    {
        double minLen = 1.0e-3;
        int nbp = Min(23, nbV);
        double lenU1 = LengthOfIso(s, IsoU, v1, v2, nbp, u1);
        if (lenU1 <= minLen)
        {
            double l = LengthOfIso(s, IsoU, v1, v2, nbp, .7 * u1 + 0.3 * u2);
            lenU1 = Max(l, lenU1);
        }
        double lenU2 = LengthOfIso(s, IsoU, v1, v2, nbp, u2);
        if (lenU2 <= minLen)
        {
            double l = LengthOfIso(s, IsoU, v1, v2, nbp, .3 * u1 + 0.7 * u2);
            lenU2 = Max(l, lenU2);
        }
        nbp = Min(23, nbV);
        double lenV1 = LengthOfIso(s, IsoV, u1, u2, nbp, v1);
        if (lenV1 <= minLen)
        {
            double l = LengthOfIso(s, IsoV, u1, u2, nbp, .7 * v1 + 0.3 * v2);
            lenV1 = Max(l, lenV1);
        }
        double lenV2 = LengthOfIso(s, IsoV, u1, u2, nbp, v2);
        if (lenV2 <= minLen)
        {
            double l = LengthOfIso(s, IsoV, u1, u2, nbp, .3 * v1 + 0.7 * v2);
            lenV2 = Max(l, lenV2);
        }
        //
        double stepV1 = lenU1 / nbV;
        double stepV2 = lenU2 / nbV;
        double stepU1 = lenV1 / nbU;
        double stepU2 = lenV2 / nbU;

        double maxStepV = Max(stepV1, stepV2);
        double maxStepU = Max(stepU1, stepU2);
        //
        double ratio = maxStepV / maxStepU;
        if (ratio > 10.0)
        {
            int mult = RealToInt(Math.Log(ratio));
            if (mult > 1)
            {
                nbV *= mult;
            }
        }
        else if (ratio < 0.1)
        {
            int mult = RealToInt(-Math.Log(ratio));
            if (mult > 1)
            {
                nbV *= mult; // OCCT: multiplies NbV again where NbU was presumably meant
            }
        }
    }

    private void BuildTree()
    {
        // if tree already exists, assume it is already correctly filled
        if (_sphereUBTree is not null)
        {
            return;
        }

        if (_s!.GetType() == SurfaceKind.BSplineSurface)
        {
            global::Kerf.Geometry.BSplineSurface bspl = _s.BSpline();
            int uValue = bspl.UDegree * bspl.UKnotCount;
            int vValue = bspl.VDegree * bspl.VKnotCount;
            // 300 is value, which is used for singular points (see Extrema_ExtPS.cxx::Initialize(...))
            if (uValue > _myusample)
            {
                _myusample = Min(uValue, 300);
            }
            if (vValue > _myvsample)
            {
                _myvsample = Min(vValue, 300);
            }
        }
        //
        CorrectNbSamples(_s, _myumin, _myusup, ref _myusample, _myvmin, _myvsup, ref _myvsample);
        //
        double pasU = _myusup - _myumin;
        double pasV = _myvsup - _myvmin;
        double u0 = pasU / _myusample / 100.0;
        double v0 = pasV / _myvsample / 100.0;
        pasU = (pasU - u0) / (_myusample - 1);
        pasV = (pasV - v0) / (_myvsample - 1);
        u0 = u0 / 2.0 + _myumin;
        v0 = v0 / 2.0 + _myvmin;

        // build grid of parametric points
        _uParams = new NCollection_HArray1<double>(1, _myusample);
        _vParams = new NCollection_HArray1<double>(1, _myvsample);
        int noU, noV;
        double u = u0, v = v0;
        for (noU = 1; noU <= _myusample; noU++, u += pasU)
        {
            _uParams.SetValue(noU, u);
        }
        for (noV = 1; noV <= _myvsample; noV++, v += pasV)
        {
            _vParams.SetValue(noV, v);
        }

        // Build UB-tree with surface points for fast proximity search.
        // Use optimized grid evaluator with span-based caching for B-spline surfaces.
        _sphereUBTree = new SpatialTree<int, BoundingSphere>();
        SpatialTreeFiller<int, BoundingSphere> filler = new(_sphereUBTree);
        int i = 0;

        _sphereArray = new NCollection_HArray1<BoundingSphere>(0, _myusample * _myvsample);

        // Use unified grid evaluator for all surface types (optimized for BSpline, Bezier, etc.)
        SurfaceGrid gridEval = new(_s);

        NCollection_Array2<Point3D> gridPoints = gridEval.EvaluateGrid(_uParams.Array1(), _vParams.Array1());

        for (noU = 1; noU <= _myusample; noU++)
        {
            for (noV = 1; noV <= _myvsample; noV++)
            {
                Point3D p1 = gridPoints.Value(noU, noV);
                BoundingSphere sph = new(p1.XYZ(), 0 /*mytolu < mytolv ? mytolu : mytolv*/, noU, noV);
                filler.Add(i, sph);
                _sphereArray.SetValue(i, sph);
                i++;
            }
        }
        filler.Fill();
    }

    private void FindSolution(in Point3D p, in POnSurfParams theParams)
    {
        _ = p;
        math_Vector tol = new(1, 2);
        tol[1] = _mytolu;
        tol[2] = _mytolv;

        math_Vector uv = new(1, 2);
        theParams.Parameter(out uv[1], out uv[2]);

        math_Vector uVinf = new(1, 2), uVsup = new(1, 2);
        uVinf[1] = _myumin;
        uVinf[2] = _myvmin;
        uVsup[1] = _myusup;
        uVsup[2] = _myvsup;

        FunctionSetRoot s = new(_f, tol);
        s.Perform(_f, uv, uVinf, uVsup);

        _done = true;
    }

    public void SetFlag(ExtFlag f) => _flag = f;

    public void SetAlgo(ExtAlgo a)
    {
        if (_algo != a)
        {
            _init = false;
        }
        _algo = a;
    }

    /// <summary>Computes the extrema for the point P. Throws if the surface has not been initialized.</summary>
    public void Perform(in Point3D p)
    {
        _done = false;
        _f.SetPoint(p);

        if (_algo == ExtAlgo_Grad)
        {
            BuildGrid(p);
            int noU, noV;

            if (_flag == ExtFlag_MIN || _flag == ExtFlag_MINMAX)
            {
                ElementType elemType;
                int iU;
                int iV;
                int iU2;
                int iV2;
                bool isMin;
                int i;

                for (noU = 1; noU < _myusample; noU++)
                {
                    for (noV = 1; noV < _myvsample; noV++)
                    {
                        POnSurfParams param = _facePntParams.Value(noU, noV);

                        isMin = false;
                        elemType = param.GetElementType();

                        if (elemType == Face)
                        {
                            isMin = true;
                        }
                        else
                        {
                            // Check if it is a boundary edge or corner vertex.
                            param.GetIndices(out iU, out iV);

                            if (elemType == UIsoEdge)
                            {
                                isMin = (iV == 1 || iV == _myvsample);
                            }
                            else if (elemType == VIsoEdge)
                            {
                                isMin = (iU == 1 || iU == _myusample);
                            }
                            else if (elemType == Node)
                            {
                                isMin = (iU == 1 || iU == _myusample) && (iV == 1 || iV == _myvsample);
                            }

                            if (!isMin)
                            {
                                // This is a middle element.
                                if (elemType == UIsoEdge
                                    || (elemType == Node && (iU == 1 || iU == _myusample)))
                                {
                                    // Check the down face.
                                    POnSurfParams downParam = _facePntParams.Value(noU, noV - 1);

                                    if (downParam.GetElementType() == elemType)
                                    {
                                        downParam.GetIndices(out iU2, out iV2);
                                        isMin = (iU == iU2 && iV == iV2);
                                    }
                                }
                                else if (elemType == VIsoEdge
                                         || (elemType == Node && (iV == 1 || iV == _myvsample)))
                                {
                                    // Check the right face.
                                    POnSurfParams rightParam = _facePntParams.Value(noU - 1, noV);

                                    if (rightParam.GetElementType() == elemType)
                                    {
                                        rightParam.GetIndices(out iU2, out iV2);
                                        isMin = (iU == iU2 && iV == iV2);
                                    }
                                }
                                else if (iU == noU && iV == noV)
                                {
                                    // Check the lower-left node. For this purpose it is necessary
                                    // to check lower-left, lower and left faces.
                                    isMin = true;

                                    Span<POnSurfParams> otherParam =
                                    [
                                        _facePntParams.Value(noU, noV - 1), // Down
                                        _facePntParams.Value(noU - 1, noV - 1), // Lower-left
                                        _facePntParams.Value(noU - 1, noV), // Left
                                    ];

                                    for (i = 0; i < 3 && isMin; i++)
                                    {
                                        if (otherParam[i].GetElementType() == Node)
                                        {
                                            otherParam[i].GetIndices(out iU2, out iV2);
                                            isMin = (iU == iU2 && iV == iV2);
                                        }
                                        else
                                        {
                                            isMin = false;
                                        }
                                    }
                                }
                            }
                        }

                        if (isMin)
                        {
                            FindSolution(p, param);
                        }
                    }
                }
            }

            if (_flag == ExtFlag_MAX || _flag == ExtFlag_MINMAX)
            {
                double dist;

                for (noU = 1; noU <= _myusample; noU++)
                {
                    for (noV = 1; noV <= _myvsample; noV++)
                    {
                        POnSurfParams paramMain = _points.Value(noU, noV);
                        POnSurfParams param1 = _points.Value(noU - 1, noV - 1);
                        POnSurfParams param2 = _points.Value(noU - 1, noV);
                        POnSurfParams param3 = _points.Value(noU - 1, noV + 1);
                        POnSurfParams param4 = _points.Value(noU, noV - 1);
                        POnSurfParams param5 = _points.Value(noU, noV + 1);
                        POnSurfParams param6 = _points.Value(noU + 1, noV - 1);
                        POnSurfParams param7 = _points.Value(noU + 1, noV);
                        POnSurfParams param8 = _points.Value(noU + 1, noV + 1);

                        dist = paramMain.GetSqrDistance();

                        if ((param1.GetSqrDistance() <= dist) && (param2.GetSqrDistance() <= dist)
                            && (param3.GetSqrDistance() <= dist) && (param4.GetSqrDistance() <= dist)
                            && (param5.GetSqrDistance() <= dist) && (param6.GetSqrDistance() <= dist)
                            && (param7.GetSqrDistance() <= dist) && (param8.GetSqrDistance() <= dist))
                        {
                            // Find maximum.
                            FindSolution(p, _points.Value(noU, noV));
                        }
                    }
                }
            }
        }
        else
        {
            BuildTree();
            if (_flag == ExtFlag_MIN || _flag == ExtFlag_MINMAX)
            {
                BoundingSphere sol = _sphereArray!.Value(0);
                Bnd_SphereUBTreeSelectorMin selector = new(_sphereArray, sol);
                // aSelector.SetMaxDist( RealLast() );
                selector.DefineCheckPoint(p);
                _sphereUBTree!.Select(selector);
                // TODO: check if no solution in binary tree
                BoundingSphere sph = selector.Sphere();
                double u = _uParams!.Value(sph.U());
                double v = _vParams!.Value(sph.V());
                POnSurfParams aParams = new(u, v, _s!.Value(u, v));

                aParams.SetSqrDistance(p.SquareDistanceTo(aParams.Value()));
                aParams.SetIndices(sph.U(), sph.V());
                FindSolution(p, aParams);
            }
            if (_flag == ExtFlag_MAX || _flag == ExtFlag_MINMAX)
            {
                BoundingSphere sol = _sphereArray!.Value(0);
                Bnd_SphereUBTreeSelectorMax selector = new(_sphereArray, sol);
                // aSelector.SetMaxDist( RealLast() );
                selector.DefineCheckPoint(p);
                _sphereUBTree!.Select(selector);
                // TODO: check if no solution in binary tree
                BoundingSphere sph = selector.Sphere();
                double u = _uParams!.Value(sph.U());
                double v = _vParams!.Value(sph.V());
                POnSurfParams aParams = new(u, v, _s!.Value(u, v));
                aParams.SetSqrDistance(p.SquareDistanceTo(aParams.Value()));
                aParams.SetIndices(sph.U(), sph.V());

                FindSolution(p, aParams);
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>The number of extremum distances.</summary>
    public int ExtCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _f.ExtCount;
        }
    }

    /// <summary>The Nth square distance.</summary>
    public double SquareDistance(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }

        return _f.SquareDistance(n);
    }

    /// <summary>The point of the Nth distance.</summary>
    public POnSurf Point(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }

        return _f.Point(n);
    }
}
