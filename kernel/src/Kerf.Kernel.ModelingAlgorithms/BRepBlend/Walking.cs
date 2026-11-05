// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Walking.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Walking.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Extrema;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Blend.BlendStatus;
using static Kerf.Engine.Geometry.NormalStatus;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// Walks a blend between two surfaces along a guide: from a first section, solves the blend function at steps of
/// the guide's parameter, checks the deflection, and stops on the surfaces' restrictions (the "recadrage", solving
/// the inverse function on the arc), building a <see cref="Line"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21), the walker of ChFi3d_Builder's <c>ComputeData</c>. The order of its calls to the
/// function (<c>Set</c>, the root finder, <c>IsSolution</c>, the tangents) is OCCT's line for line, which the
/// functions' caches depend on (decisions 1037 and 1043). OCCT's <c>OCCT_DEBUG</c> drawing of the sections
/// (<c>Drawsect</c>) and its traces are left out.
/// </remarks>
internal class Walking
{
    private const double CosRef3D_File = 0.88; // OCCT's file-static CosRef3D (InternalPerform)

    private BlendPoint _previousP = new();
    private Line? _line;
    private readonly math_Vector _sol;
    private readonly NCollection_Sequence<BlendPoint> _jalons = new();
    private readonly SurfaceAdaptorBase _surf1;
    private readonly SurfaceAdaptorBase _surf2;
    private TopolTool _domain1;
    private TopolTool _domain2;
    private TopolTool _recdomain1;
    private TopolTool _recdomain2;
    private ElSpine _hguide;
    private bool _toCorrectOnRst1;
    private bool _toCorrectOnRst2;
    private double _correctedParam;
    private double _tolpoint3d;
    private double _tolgui;
    private double _pasmax;
    private double _fleche;
    private double _param;
    private double _sens;
    private bool _done;
#pragma warning disable CS0414 // set and never read, as in OCCT
    private bool _rebrou;
#pragma warning restore CS0414
    private bool _iscomplete;
    private bool _comptra;
    private bool _clasonS1;
    private bool _clasonS2;
    private bool _check2d;
    private bool _check;
    private bool _twistflag1;
    private bool _twistflag2;

    public Walking(SurfaceAdaptorBase surf1, SurfaceAdaptorBase surf2, TopolTool domain1,
                             TopolTool domain2, ElSpine hGuide)
    {
        _sol = new math_Vector(1, 4);
        _surf1 = surf1;
        _surf2 = surf2;
        _toCorrectOnRst1 = false;
        _toCorrectOnRst2 = false;
        _done = false;
        _clasonS1 = true;
        _clasonS2 = true;
        _check2d = true;
        _check = true;
        _twistflag1 = false;
        _twistflag2 = false;

        _domain1 = domain1;
        _domain2 = domain2;
        _recdomain1 = domain1;
        _recdomain2 = domain2;
        _hguide = hGuide;
    }

    /// <summary>The domains on which the walking stops (the restrictions), when they differ from the classification's.</summary>
    public void SetDomainsToRecadre(TopolTool domain1, TopolTool domain2)
    {
        _recdomain1 = domain1;
        _recdomain2 = domain2;
    }

    /// <summary>Adds a singular point (a "jalon") the walking must pass, in the order of the guide's parameter.</summary>
    public void AddSingularPoint(in BlendPoint p)
    {
        if (_jalons.Length() == 0)
        {
            _jalons.Append(p);
        }
        else
        {
            int ii, jj;
            double tp = p.Parameter(), ti = _jalons.First().Parameter();
            for (jj = 1, ii = 1; ii <= _jalons.Length() && tp > ti; ii++)
            {
                jj = ii;
                ti = _jalons.Value(jj).Parameter();
            }
            if (tp > ti)
            {
                _jalons.InsertAfter(jj, p);
            }
            else
            {
                _jalons.InsertBefore(jj, p);
            }
        }
    }

    /// <summary>
    /// Walks from the guide's parameter <paramref name="pdep"/> towards <paramref name="pmax"/>, starting from the
    /// solution <paramref name="parDep"/> (solved again first when <paramref name="appro"/>).
    /// </summary>
    public void Perform(BlendFunction func, FuncInv funcInv, double pdep, double pmax, double maxStep,
                        double tol3d, double tolGuide, math_Vector parDep, double fleche, bool appro = false)
    {
        _done = false;
        _iscomplete = false;
        _comptra = false;
        bool doextremities = true;
        if (_line is null)
        {
            _line = new Line();
        }
        else
        {
            _line.Clear();
            doextremities = false;
        }
        _tolpoint3d = tol3d;
        _tolgui = Math.Abs(tolGuide);
        _fleche = Math.Abs(fleche);
        _rebrou = false;
        _pasmax = Math.Abs(maxStep);
        if (pmax - pdep >= 0.0)
        {
            _sens = 1.0;
        }
        else
        {
            _sens = -1.0;
        }

        BlendStatus state;

        _param = pdep;
        func.Set(_param);

        if (appro)
        {
            Containment situ1, situ2;
            math_Vector tolerance = new(1, 4), infbound = new(1, 4), supbound = new(1, 4);
            func.GetTolerance(tolerance, _tolpoint3d);
            func.GetBounds(infbound, supbound);
            FunctionSetRoot rsnld = new(func, tolerance, 30);

            rsnld.Perform(func, parDep, infbound, supbound);

            if (!rsnld.IsDone)
            {
                return;
            }
            rsnld.Root(_sol);

            if (_clasonS1)
            {
                situ1 = _domain1.Classify(new Point2D(_sol[1], _sol[2]), Min(tolerance[1], tolerance[2]), false);
            }
            else
            {
                situ1 = global::Kerf.Topology.Containment.In;
            }
            if (_clasonS2)
            {
                situ2 = _domain2.Classify(new Point2D(_sol[3], _sol[4]), Min(tolerance[3], tolerance[4]), false);
            }
            else
            {
                situ2 = global::Kerf.Topology.Containment.In;
            }

            if (situ1 != global::Kerf.Topology.Containment.In || situ2 != global::Kerf.Topology.Containment.In)
            {
                return;
            }
        }
        else
        {
            _sol.Initialized(parDep);
        }

        state = TestArret(func, BlendStatus.OK, false);
        if (state != BlendStatus.OK)
        {
            return;
        }
        // OCCT_DEBUG Drawsect left out.
        // Update the line.
        // Correct first parameter if needed
        if (_toCorrectOnRst1 || _toCorrectOnRst2)
        {
            _previousP.SetParameter(_correctedParam);
        }
        _line.Append(_previousP);

        if (doextremities)
        {
            // OCCT: four arguments make the curve overload (P, W, Param, Tol): U is u, the guide's parameter v
            Extremity ptf1 = new(_previousP.PointOnS1(), _sol[1], _sol[2], _tolpoint3d);
            Extremity ptf2 = new(_previousP.PointOnS2(), _sol[3], _sol[4], _tolpoint3d);
            if (!_previousP.IsTangencyPoint)
            {
                ptf1.SetTangent(_previousP.TangentOnS1());
                ptf2.SetTangent(_previousP.TangentOnS2());
            }

            if (_sens > 0.0)
            {
                _line.SetStartPoints(ptf1, ptf2);
            }
            else
            {
                _line.SetEndPoints(ptf1, ptf2);
            }
        }

        InternalPerform(func, funcInv, pmax);

        _done = true;
    }

    /// <summary>
    /// Solves the first section at <paramref name="pdep"/> from <paramref name="parDep"/> (which receives the
    /// solution) and classifies its points: true if both are inside their faces.
    /// </summary>
    public bool PerformFirstSection(BlendFunction func, double pdep, math_Vector parDep, double tol3d,
                                    double tolGuide, out Containment pos1, out Containment pos2)
    {
        _iscomplete = false;
        _comptra = false;
        _line = new Line();
        _tolpoint3d = tol3d;
        _tolgui = Math.Abs(tolGuide);

        pos1 = pos2 = Unknown;

        _param = pdep;
        func.Set(_param);

        math_Vector tolerance = new(1, 4), infbound = new(1, 4), supbound = new(1, 4);
        func.GetTolerance(tolerance, _tolpoint3d);
        func.GetBounds(infbound, supbound);
        FunctionSetRoot rsnld = new(func, tolerance, 30);

        rsnld.Perform(func, parDep, infbound, supbound);

        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(_sol);
        parDep.Initialized(_sol);
        pos1 = _domain1.Classify(new Point2D(_sol[1], _sol[2]), Min(tolerance[1], tolerance[2]), false);
        pos2 = _domain2.Classify(new Point2D(_sol[3], _sol[4]), Min(tolerance[3], tolerance[4]), false);
        if (pos1 != global::Kerf.Topology.Containment.In || pos2 != global::Kerf.Topology.Containment.In)
        {
            return false;
        }

        TestArret(func, BlendStatus.OK, false);
        // OCCT_DEBUG Drawsect left out.
        return true;
    }

    /// <summary>
    /// Solves a first section at <paramref name="pdep"/> and moves it onto the restrictions (of the first surface if
    /// <paramref name="recOnS1"/>, of the second if <paramref name="recOnS2"/>): <paramref name="psol"/> and
    /// <paramref name="parSol"/> receive the section on the restriction.
    /// </summary>
    public bool PerformFirstSection(BlendFunction func, FuncInv funcInv, double pdep, double pmax,
                                    math_Vector parDep, double tol3d, double tolGuide, bool recOnS1, bool recOnS2,
                                    ref double psol, math_Vector parSol)
    {
        _iscomplete = false;
        _comptra = false;
        _line = new Line();

        double w1, w2, extrapol;
        bool recad1, recad2;

        _tolpoint3d = tol3d;
        _tolgui = Math.Abs(tolGuide);
        if (pmax - pdep >= 0.0)
        {
            _sens = 1.0;
        }
        else
        {
            _sens = -1.0;
        }
        extrapol = Math.Abs(pmax - pdep) / 50.0; // 2%

        BlendStatus state;

        _param = pdep;
        func.Set(_param);

        math_Vector tolerance = new(1, 4), infbound = new(1, 4), supbound = new(1, 4);
        math_Vector solrst1 = new(1, 4), solrst2 = new(1, 4);
        Extremity ext1 = new(), ext2 = new();
        int index1 = 0, index2 = 0, nbarc;
        bool isvtx1 = false, isvtx2 = false;
        HVertex? vtx1 = null, vtx2 = null;
        Point2D p2d;
        double correctedU = 0.0, correctedV = 0.0;
        Point3D correctedPnt = new();

        func.GetTolerance(tolerance, _tolpoint3d);
        func.GetBounds(infbound, supbound);
        FunctionSetRoot rsnld = new(func, tolerance, 30);

        rsnld.Perform(func, parDep, infbound, supbound);

        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(_sol);

        w1 = w2 = pmax;

        recad1 = recOnS1 && Recadre(funcInv, true, _sol, solrst1, out index1, out isvtx1, ref vtx1, extrapol);
        if (recad1)
        {
            w1 = solrst1[2];
        }

        recad2 = recOnS2 && Recadre(funcInv, false, _sol, solrst2, out index2, out isvtx2, ref vtx2, extrapol);
        if (recad2)
        {
            w2 = solrst2[2];
        }

        if (!recad1 && !recad2)
        {
            return false;
        }

        if (recad1 && recad2)
        {
            if (Math.Abs(w1 - w2) <= _tolgui)
            {
                // the solution is on 1 and 2 at once
                state = BlendStatus.OnRst12;
                _param = w1;
                parSol[1] = solrst2[3];
                parSol[2] = solrst2[4];
                parSol[3] = solrst1[3];
                parSol[4] = solrst1[4];
            }
            else if (_sens * (w2 - w1) < 0.0)
            { // keep the greater
                // solution on 1
                state = BlendStatus.OnRst1;
                _param = w1;

                _recdomain1.Init();
                nbarc = 1;
                while (nbarc < index1)
                {
                    nbarc++;
                    _recdomain1.Next();
                }
                p2d = HCurve2dTool.Value(_recdomain1.Value(), solrst1[1]);
                parSol[1] = p2d.X;
                parSol[2] = p2d.Y;
                parSol[3] = solrst1[3];
                parSol[4] = solrst1[4];
            }
            else
            {
                // solution on 2
                state = BlendStatus.OnRst2;
                _param = w2;

                _recdomain2.Init();
                nbarc = 1;
                while (nbarc < index2)
                {
                    nbarc++;
                    _recdomain2.Next();
                }
                p2d = HCurve2dTool.Value(_recdomain2.Value(), solrst2[1]);
                parSol[1] = solrst2[3];
                parSol[2] = solrst2[4];
                parSol[3] = p2d.X;
                parSol[4] = p2d.Y;
            }
        }
        else if (recad1)
        {
            // solution on 1
            state = BlendStatus.OnRst1;
            _param = w1;
            _recdomain1.Init();
            nbarc = 1;
            while (nbarc < index1)
            {
                nbarc++;
                _recdomain1.Next();
            }
            p2d = HCurve2dTool.Value(_recdomain1.Value(), solrst1[1]);
            parSol[1] = p2d.X;
            parSol[2] = p2d.Y;
            parSol[3] = solrst1[3];
            parSol[4] = solrst1[4];
            Point3D thePntOnRst = HSurfaceTool.Value(_surf1, parSol[1], parSol[2]);
            if (CorrectExtremityOnOneRst(1, parSol[3], parSol[4], _param, thePntOnRst, ref correctedU, ref correctedV,
                                         ref correctedPnt, ref _correctedParam))
            {
                _toCorrectOnRst1 = true;
            }
        }
        else
        { // if (recad2) {
            // solution on 2
            state = BlendStatus.OnRst2;
            _param = w2;

            _recdomain2.Init();
            nbarc = 1;
            while (nbarc < index2)
            {
                nbarc++;
                _recdomain2.Next();
            }
            p2d = HCurve2dTool.Value(_recdomain2.Value(), solrst2[1]);
            parSol[1] = solrst2[3];
            parSol[2] = solrst2[4];
            parSol[3] = p2d.X;
            parSol[4] = p2d.Y;
            Point3D thePntOnRst = HSurfaceTool.Value(_surf2, parSol[3], parSol[4]);
            if (CorrectExtremityOnOneRst(2, parSol[1], parSol[2], _param, thePntOnRst, ref correctedU, ref correctedV,
                                         ref correctedPnt, ref _correctedParam))
            {
                _toCorrectOnRst2 = true;
            }
        }

        psol = _param;
        _sol.Initialized(parSol);
        func.Set(_param);
        state = TestArret(func, state, false);
        switch (state)
        {
            case BlendStatus.OnRst1:
            {
                // OCCT_DEBUG Drawsect left out.
                MakeExtremity(ref ext1, true, index1, solrst1[1], isvtx1, vtx1);
                if (_toCorrectOnRst1)
                {
                    ext2.SetValue(correctedPnt, correctedU, correctedV, _tolpoint3d); // OCCT: the curve overload
                }
                else
                {
                    ext2.SetValue(_previousP.PointOnS2(), _sol[3], _sol[4], _tolpoint3d);
                }
            }
            break;

            case BlendStatus.OnRst2:
            {
                // OCCT_DEBUG Drawsect left out.
                if (_toCorrectOnRst2)
                {
                    ext1.SetValue(correctedPnt, correctedU, correctedV, _tolpoint3d); // OCCT: the curve overload
                }
                else
                {
                    ext1.SetValue(_previousP.PointOnS1(), _sol[1], _sol[2], _tolpoint3d);
                }
                MakeExtremity(ref ext2, false, index2, solrst2[1], isvtx2, vtx2);
            }
            break;

            case BlendStatus.OnRst12:
            {
                // OCCT_DEBUG Drawsect left out.
                MakeExtremity(ref ext1, true, index1, solrst1[1], isvtx1, vtx1);
                MakeExtremity(ref ext2, false, index2, solrst2[1], isvtx2, vtx2);
            }
            break;
            default:
            {
                throw new FailureException("BRepBlend_Walking::PerformFirstSection : echec");
            }
        }
        if (_sens < 0.0)
        {
            _line.SetEndPoints(ext1, ext2);
        }
        else
        {
            _line.SetStartPoints(ext1, ext2);
        }
        return true;
    }

    /// <summary>Continues the walking up to the guide's parameter <paramref name="p"/>, beyond the line's ends.</summary>
    public bool Continu(BlendFunction func, FuncInv funcInv, double p)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        BlendPoint firstBP = _line!.Point(1);
        BlendPoint lastBP = _line.Point(_line.NbPoints());

        if (p < firstBP.Parameter())
        {
            _sens = -1.0;
            _previousP = firstBP;
        }
        else if (p > lastBP.Parameter())
        {
            _sens = 1.0;
            _previousP = lastBP;
        }

        _param = _previousP.Parameter();
        _previousP.ParametersOnS1(out _sol[1], out _sol[2]);
        _previousP.ParametersOnS2(out _sol[3], out _sol[4]);

        InternalPerform(func, funcInv, p);
        return true;
    }

    /// <summary>
    /// Continues the walking up to <paramref name="p"/> past the restriction of one surface (the first if
    /// <paramref name="onS1"/>), without classifying on it; false, and the line restored, if it does not reach the
    /// other surface's restriction.
    /// </summary>
    public bool Continu(BlendFunction func, FuncInv funcInv, double p, bool onS1)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        Extremity ext1, ext2;
        if (_sens < 0.0)
        {
            ext1 = _line!.StartPointOnFirst();
            ext2 = _line.StartPointOnSecond();
            if ((onS1 && ext1.PointOnRstCount == 0) || (!onS1 && ext2.PointOnRstCount == 0))
            {
                return false;
            }
            _previousP = _line.Point(1);
        }
        else
        {
            ext1 = _line!.EndPointOnFirst();
            ext2 = _line.EndPointOnSecond();
            if ((onS1 && ext1.PointOnRstCount == 0) || (!onS1 && ext2.PointOnRstCount == 0))
            {
                return false;
            }
            _previousP = _line.Point(_line.NbPoints());
        }

        int length = _line.NbPoints();
        _param = _previousP.Parameter();
        _previousP.ParametersOnS1(out _sol[1], out _sol[2]);
        _previousP.ParametersOnS2(out _sol[3], out _sol[4]);

        if (onS1)
        {
            _clasonS1 = false;
        }
        else
        {
            _clasonS2 = false;
        }

        InternalPerform(func, funcInv, p);

        _clasonS1 = true;
        _clasonS2 = true;

        int newlength = _line.NbPoints();
        if (_sens < 0.0)
        {
            if ((onS1 && _line.StartPointOnSecond().PointOnRstCount == 0)
                || (!onS1 && _line.StartPointOnFirst().PointOnRstCount == 0))
            {
                _line.Remove(1, newlength - length);
                _line.SetStartPoints(ext1, ext2);
                return false;
            }
        }
        else
        {
            if ((onS1 && _line.EndPointOnSecond().PointOnRstCount == 0)
                || (!onS1 && _line.EndPointOnFirst().PointOnRstCount == 0))
            {
                _line.Remove(length, newlength);
                _line.SetEndPoints(ext1, ext2);
                return false;
            }
        }
        return true;
    }

    /// <summary>Walks the other way from the line's start, down to <paramref name="pmin"/>.</summary>
    public bool Complete(BlendFunction func, FuncInv funcInv, double pmin)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_iscomplete)
        {
            return true;
        }

        if (_sens > 0.0)
        {
            _previousP = _line!.Point(1);
        }
        else
        {
            _previousP = _line!.Point(_line.NbPoints());
        }

        _sens = -_sens;

        _param = _previousP.Parameter();
        _previousP.ParametersOnS1(out _sol[1], out _sol[2]);
        _previousP.ParametersOnS2(out _sol[3], out _sol[4]);

        InternalPerform(func, funcInv, pmin);

        _iscomplete = true;
        return true;
    }

    public void ClassificationOnS1(bool c)
    {
        _clasonS1 = c;
    }

    public void ClassificationOnS2(bool c)
    {
        _clasonS2 = c;
    }

    public void Check2d(bool c)
    {
        _check2d = c;
    }

    public void Check(bool c)
    {
        _check = c;
    }

    public bool TwistOnS1() => _twistflag1;

    public bool TwistOnS2() => _twistflag2;

    public bool IsDone => _done;

    public Line Line()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _line!;
    }

    // Is the given point a solution? If so, check the deflection on surf1 and surf2; if that is OK, classify the
    // points on surf1 and surf2: both inside, Blend_OK; one inside, solve the inverse problem on that restriction;
    // otherwise on the surface where the point is the furthest. If the deflection is not OK, or the point is not a
    // solution, Blend_StepTooLarge.
    private BlendStatus TestArret(BlendFunction function, BlendStatus state, bool testDefl, bool testSolu = true,
                                   bool testLengthStep = false)
    {
        Point3D pt1, pt2;
        Vector3D v1 = new(), v2 = new();
        Vector3D tgp1, tgp2, nor1, nor2;
        Vector2D v12d, v22d;
        BlendStatus state1, state2;
        WalkingTypeTrans tras1, tras2;
        BlendPoint curpoint = new();
        bool loctwist1 = false, loctwist2 = false;
        double tolsolu = _tolpoint3d;

        if (!testSolu)
        {
            tolsolu *= 1000; // it must always be good
        }
        if (function.IsSolution(_sol, tolsolu))
        {
            bool curpointistangent = function.IsTangencyPoint;
            pt1 = function.PointOnS1();
            pt2 = function.PointOnS2();
            if (curpointistangent)
            {
                curpoint.SetValue(pt1, pt2, _param, _sol[1], _sol[2], _sol[3], _sol[4]);
            }
            else
            {
                v1 = function.TangentOnS1();
                v2 = function.TangentOnS2();
                v12d = function.Tangent2dOnS1();
                v22d = function.Tangent2dOnS2();
                curpoint.SetValue(pt1, pt2, _param, _sol[1], _sol[2], _sol[3], _sol[4], v1, v2, v12d, v22d);
                if (function.TwistOnS1())
                {
                    loctwist1 = true;
                }
                if (function.TwistOnS2())
                {
                    loctwist2 = true;
                }
            }

            if (testDefl && _check)
            {
                // Check the deflection on each surface and on the guide.
                state1 = CheckDeflection(true, curpoint);
                state2 = CheckDeflection(false, curpoint);
            }
            else
            {
                state1 = BlendStatus.OK;
                state2 = BlendStatus.OK;
                if (testLengthStep)
                {
                    // Only check that the step is not too large (bad extensions).
                    double curparamu, curparamv, prevparamu, prevparamv;
                    math_Vector inf = new(1, 4), sup = new(1, 4);
                    function.GetBounds(inf, sup);
                    sup -= inf;
                    sup *= 0.05; // max step: 5% of the domain

                    curpoint.ParametersOnS1(out curparamu, out curparamv);
                    _previousP.ParametersOnS1(out prevparamu, out prevparamv);
                    if (Math.Abs(curparamu - prevparamu) > sup[1])
                    {
                        state1 = BlendStatus.StepTooLarge;
                    }
                    if (Math.Abs(curparamv - prevparamv) > sup[2])
                    {
                        state1 = BlendStatus.StepTooLarge;
                    }
                    curpoint.ParametersOnS2(out curparamu, out curparamv);
                    _previousP.ParametersOnS2(out prevparamu, out prevparamv);
                    if (Math.Abs(curparamu - prevparamu) > sup[3])
                    {
                        state2 = BlendStatus.StepTooLarge;
                    }
                    if (Math.Abs(curparamv - prevparamv) > sup[4])
                    {
                        state2 = BlendStatus.StepTooLarge;
                    }
                }
            }

            if (state1 == BlendStatus.Backward)
            {
                state1 = BlendStatus.StepTooLarge;
                _rebrou = true;
            }

            if (state2 == BlendStatus.Backward)
            {
                state2 = BlendStatus.StepTooLarge;
                _rebrou = true;
            }

            if (state1 == BlendStatus.StepTooLarge || state2 == BlendStatus.StepTooLarge)
            {
                return BlendStatus.StepTooLarge;
            }

            // Only here can the twist be decided, the rejections being done (BUC60322).
            if (loctwist1)
            {
                _twistflag1 = true;
            }
            if (loctwist2)
            {
                _twistflag2 = true;
            }

            if (!_comptra && !curpointistangent)
            {
                function.Tangent(_sol[1], _sol[2], _sol[3], _sol[4], out tgp1, out tgp2, out nor1, out nor2);
                nor1 = nor1.Normalized();
                nor2 = nor2.Normalized();
                double testra = tgp1.Dot(nor1.Crossed(v1));
                if (Math.Abs(testra) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    tras1 = WalkingTypeTrans.In;
                    if ((testra > 0.0 && !loctwist1) || (testra < 0.0 && loctwist1))
                    {
                        tras1 = WalkingTypeTrans.Out;
                    }

                    testra = tgp2.Dot(nor2.Crossed(v2));
                    if (Math.Abs(testra) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        tras2 = WalkingTypeTrans.Out;
                        if ((testra > 0.0 && !loctwist2) || (testra < 0.0 && loctwist2))
                        {
                            tras2 = WalkingTypeTrans.In;
                        }
                        _comptra = true;
                        _line!.Set(tras1, tras2);
                    }
                }
            }

            if (state1 == BlendStatus.OK || state2 == BlendStatus.OK)
            {
                _previousP = curpoint;
                return state;
            }

            if (state1 == BlendStatus.StepTooSmall && state2 == BlendStatus.StepTooSmall)
            {
                _previousP = curpoint;
                if (state == BlendStatus.OK)
                {
                    return BlendStatus.StepTooSmall;
                }
                else
                {
                    return state;
                }
            }

            if (state == BlendStatus.OK)
            {
                return BlendStatus.SamePoints;
            }
            else
            {
                return state;
            }
        }
        else
        {
            return BlendStatus.StepTooLarge;
        }
    }

    private BlendStatus CheckDeflection(bool onFirst, in BlendPoint curPoint)
    {
        // set by tests in U4, corresponds to 11.478 degrees
        const double cosRef3D = 0.98;

        const double cosRef2D = 0.88; // corresponds to 25 degrees

        double norme, cosi, cosi2;
        double prevNorme = 0.0;
        double flecheCourante;
        double du, dv, duv;
        double tolu, tolv;

        Point3D psurf;
        Vector3D tgsurf = new();
        Vector2D tgonsurf = new();
        double curparamu, curparamv;
        bool curpointistangent = curPoint.IsTangencyPoint;

        Point3D prevP;
        Vector3D prevTg = new();
        Vector2D previousd2d = new();
        double prevparamu, prevparamv;
        bool prevpointistangent = _previousP.IsTangencyPoint;

        if (onFirst)
        {
            psurf = curPoint.PointOnS1();
            if (!curpointistangent)
            {
                tgsurf = curPoint.TangentOnS1();
            }
            prevP = _previousP.PointOnS1();
            if (!prevpointistangent)
            {
                prevTg = _previousP.TangentOnS1();
            }
            tolu = HSurfaceTool.UResolution(_surf1, _tolpoint3d);
            tolv = HSurfaceTool.VResolution(_surf1, _tolpoint3d);
        }
        else
        {
            psurf = curPoint.PointOnS2();
            if (!curpointistangent)
            {
                tgsurf = curPoint.TangentOnS2();
            }
            prevP = _previousP.PointOnS2();
            if (!prevpointistangent)
            {
                prevTg = _previousP.TangentOnS2();
            }
            tolu = HSurfaceTool.UResolution(_surf2, _tolpoint3d);
            tolv = HSurfaceTool.VResolution(_surf2, _tolpoint3d);
        }

        Vector3D corde = new(prevP, psurf);
        norme = corde.SquareMagnitude;
        if (!prevpointistangent)
        {
            prevNorme = prevTg.SquareMagnitude;
        }

        double toler3d = 0.01 * _tolpoint3d;
        if (norme <= toler3d * toler3d)
        {
            // the same point may have to be forced
            return BlendStatus.SamePoints;
        }
        if (!prevpointistangent)
        {
            if (prevNorme <= toler3d * toler3d)
            {
                return BlendStatus.SamePoints;
            }
            cosi = _sens * corde * prevTg;
            if (cosi < 0.0)
            { // 3d angle > pi/2: going back
                return BlendStatus.Backward;
            }

            cosi2 = cosi * cosi / prevNorme / norme;
            if (cosi2 < cosRef3D)
            {
                return BlendStatus.StepTooLarge;
            }
        }

        if (!curpointistangent)
        {
            cosi = _sens * corde * tgsurf;
            cosi2 = cosi * cosi / tgsurf.SquareMagnitude / norme;
            if (cosi2 < cosRef3D || cosi < 0.0)
            {
                return BlendStatus.StepTooLarge;
            }
        }

        if (_check2d)
        {
            if (onFirst)
            {
                curPoint.ParametersOnS1(out curparamu, out curparamv);
                if (!curpointistangent)
                {
                    tgonsurf = curPoint.Tangent2dOnS1();
                }
                _previousP.ParametersOnS1(out prevparamu, out prevparamv);
                if (!prevpointistangent)
                {
                    previousd2d = _previousP.Tangent2dOnS1();
                }
            }
            else
            {
                curPoint.ParametersOnS2(out curparamu, out curparamv);
                if (!curpointistangent)
                {
                    tgonsurf = curPoint.Tangent2dOnS2();
                }
                _previousP.ParametersOnS2(out prevparamu, out prevparamv);
                if (!prevpointistangent)
                {
                    previousd2d = _previousP.Tangent2dOnS2();
                }
            }

            du = curparamu - prevparamu;
            dv = curparamv - prevparamv;
            duv = du * du + dv * dv;
            if (Math.Abs(du) < tolu && Math.Abs(dv) < tolv)
            {
                // the same point may have to be forced
                return BlendStatus.SamePoints; // same 2d point
            }
            if (!prevpointistangent)
            {
                if (Math.Abs(previousd2d.X) < tolu && Math.Abs(previousd2d.Y) < tolv)
                {
                    return BlendStatus.SamePoints; // same 2d point
                }
                cosi = _sens * (du * previousd2d.X + dv * previousd2d.Y);
                if (cosi < 0)
                {
                    return BlendStatus.Backward;
                }
            }
            if (!curpointistangent)
            {
                cosi = _sens * (du * tgonsurf.X + dv * tgonsurf.Y) / tgonsurf.Magnitude;
                cosi2 = cosi * cosi / duv;
                if (cosi2 < cosRef2D || cosi < 0.0)
                {
                    return BlendStatus.StepTooLarge;
                }
            }
        }
        if (!curpointistangent && !prevpointistangent)
        {
            // Estimate of the current deflection
            flecheCourante = (prevTg.Normalized().XYZ() - tgsurf.Normalized().XYZ()).SquareModulus() * norme / 64.0;

            if (flecheCourante <= 0.25 * _fleche * _fleche)
            {
                return BlendStatus.StepTooSmall;
            }
            if (flecheCourante > _fleche * _fleche)
            {
                // step too large
                return BlendStatus.StepTooLarge;
            }
        }
        return BlendStatus.OK;
    }

    private int ArcToRecadre(bool onFirst, math_Vector sol, int prevIndex, out Point2D lastpt2d,
                             out Point2D pt2d, ref double ponarc)
    {
        int indexSol = 0, nbarc = 0;
        bool ok = false;
        bool byinter = (_line!.NbPoints() != 0), okinter = false;
        double distmin = RealLast();
        double uprev = 0.0, vprev = 0.0, prm = 0.0, dist = 0.0;
        TopolTool iter;

        if (onFirst)
        {
            if (byinter)
            {
                _previousP.ParametersOnS1(out uprev, out vprev);
            }
            pt2d = new Point2D(sol[1], sol[2]);
            iter = _recdomain1;
        }
        else
        {
            if (byinter)
            {
                _previousP.ParametersOnS2(out uprev, out vprev);
            }
            pt2d = new Point2D(sol[3], sol[4]);
            iter = _recdomain2;
        }
        lastpt2d = new Point2D(uprev, vprev);
        iter.Init();
        while (iter.More())
        {
            nbarc++;
            ok = false;
            if (onFirst)
            {
                if (byinter)
                {
                    ok = okinter = BlendTool.Inters(pt2d, lastpt2d, _surf1, iter.Value(), ref prm, ref dist);
                }
                if (!ok)
                {
                    ok = BlendTool.Project(pt2d, _surf1, iter.Value(), out prm, out dist);
                }
            }
            else
            {
                if (byinter)
                {
                    ok = okinter = BlendTool.Inters(pt2d, lastpt2d, _surf2, iter.Value(), ref prm, ref dist);
                }
                if (!ok)
                {
                    ok = BlendTool.Project(pt2d, _surf2, iter.Value(), out prm, out dist);
                }
            }
            if (ok && (nbarc != prevIndex))
            {
                if (dist < distmin || okinter)
                {
                    distmin = dist;
                    ponarc = prm;
                    indexSol = nbarc;
                    if (okinter && (prevIndex == 0))
                    {
                        break;
                    }
                }
            }
            iter.Next();
        }
        return indexSol;
    }

    private bool Recadre(FuncInv funcInv, bool onFirst, math_Vector sol, math_Vector solrst,
                         out int indexsol, out bool isVtx, ref HVertex? vtx, double extrap = 0.0)
    {
        bool jalons_Trouve = false;
        bool recadre = true, ok;
        bool byinter = (_line!.NbPoints() != 0);
        int leJalon = 0;

        int nbarc;
        double dist, prm, pmin = 0.0, vtol;
        Point2D pt2d, lastpt2d;

        math_Vector toler = new(1, 4), infb = new(1, 4), supb = new(1, 4);

        Curve2DAdaptorBase thecur;
        TopolTool iter;

        if (onFirst)
        {
            iter = _recdomain1;
        }
        else
        {
            iter = _recdomain2;
        }

        indexsol = ArcToRecadre(onFirst, sol, 0, out lastpt2d, out pt2d, ref pmin);
        isVtx = false;
        if (indexsol == 0)
        {
            return false;
        }

        iter.Init();
        nbarc = 1;
        while (nbarc < indexsol)
        {
            nbarc++;
            iter.Next();
        }

        Curve2DAdaptorBase thearc = iter.Value();

        if (onFirst)
        {
            thecur = BlendTool.CurveOnSurf(thearc, _surf1);
        }
        else
        {
            thecur = BlendTool.CurveOnSurf(thearc, _surf2);
        }

        // The problem to solve
        funcInv.Set(onFirst, thecur);
        funcInv.GetBounds(infb, supb);
        infb[2] -= extrap;
        supb[2] += extrap;

        funcInv.GetTolerance(toler, 0.1 * _tolpoint3d); // better keep some margin
        FunctionSetRoot rsnld = new(funcInv, toler, 35);
        toler *= 10; // but make the tests correctly

        // Computation of a starting point
        double ufirst, ulast;
        BlendTool.Bounds(thecur, out ufirst, out ulast);
        // To help finding singular corners, the parameter may be moved
        if (Math.Abs(pmin - ufirst) < Math.Abs(ulast - ufirst) / 1000)
        {
            pmin = ufirst;
        }
        if (Math.Abs(pmin - ulast) < Math.Abs(ulast - ufirst) / 1000)
        {
            pmin = ulast;
        }

        if (byinter)
        {
            double lastParam = _previousP.Parameter();
            // Check that the recadrage is not a jalon
            if (_jalons.Length() != 0)
            {
                double t1, t2, t;
                bool cherche = true;
                int ii;
                if (lastParam < _param)
                {
                    t1 = lastParam;
                    t2 = _param;
                }
                else
                {
                    t1 = _param;
                    t2 = lastParam;
                }
                for (ii = 1; ii <= _jalons.Length() && cherche; ii++)
                {
                    t = _jalons.Value(ii).Parameter();
                    if (((t1 < t) && (t2 > t)) || (t == _param))
                    {
                        jalons_Trouve = true;
                        leJalon = ii;
                        cherche = false; // only works if both are left at once
                    }
                    else
                    {
                        cherche = t < t2; // stop if t >= t2
                    }
                }
            }
            if (!jalons_Trouve)
            {
                // Initialisation by interpolation
                double lambda, u, v;
                Point2D pnt, pnt1, pnt2;
                thecur.D0(pmin, out pnt);
                if (onFirst)
                {
                    _previousP.ParametersOnS2(out u, out v);
                    pnt1 = new Point2D(u, v);
                    pnt2 = new Point2D(sol[3], sol[4]);
                }
                else
                {
                    _previousP.ParametersOnS1(out u, out v);
                    pnt1 = new Point2D(u, v);
                    pnt2 = new Point2D(sol[1], sol[2]);
                }

                lambda = pnt.DistanceTo(lastpt2d);
                if (lambda > 1.0e-12)
                {
                    lambda /= pnt.DistanceTo(lastpt2d) + pnt.DistanceTo(pt2d);
                }
                else
                {
                    lambda = 0;
                }
                solrst[1] = pmin;
                solrst[2] = (1 - lambda) * lastParam + lambda * _param;
                solrst[3] = (1 - lambda) * pnt1.X + lambda * pnt2.X;
                solrst[4] = (1 - lambda) * pnt1.Y + lambda * pnt2.Y;
            }
        }
        else
        { // otherwise start from the last computed point
            solrst[1] = pmin;
            solrst[2] = _param;
            if (onFirst)
            {
                solrst[3] = sol[3];
                solrst[4] = sol[4];
            }
            else
            {
                solrst[3] = sol[1];
                solrst[4] = sol[2];
            }
        }

        if (jalons_Trouve)
        { // take the jalon
            BlendPoint monJalon;
            bool periodic;
            double uperiod = 0, vperiod = 0;
            Point2D pnt;
            double distaux;
            monJalon = _jalons.Value(leJalon);
            solrst[2] = monJalon.Parameter();
            if (onFirst)
            {
                monJalon.ParametersOnS2(out solrst[3], out solrst[4]);
                periodic = (_surf2.IsUPeriodic || _surf2.IsVPeriodic);
            }
            else
            {
                monJalon.ParametersOnS1(out solrst[3], out solrst[4]);
                periodic = (_surf1.IsUPeriodic || _surf1.IsVPeriodic);
            }

            // Possible recadrage for the periodic case
            if (periodic)
            {
                SurfaceAdaptorBase surf;
                if (onFirst)
                {
                    surf = _surf2;
                }
                else
                {
                    surf = _surf1;
                }

                lastpt2d = thecur.Value(pmin);

                if (surf.IsUPeriodic)
                {
                    uperiod = surf.UPeriod();
                    if (solrst[3] - lastpt2d.X > uperiod * 0.6)
                    {
                        solrst[3] -= uperiod;
                    }
                    if (solrst[3] - lastpt2d.X < -uperiod * 0.6)
                    {
                        solrst[3] += uperiod;
                    }
                }
                if (surf.IsVPeriodic)
                {
                    vperiod = surf.VPeriod();
                    if (solrst[4] - lastpt2d.Y > vperiod * 0.6)
                    {
                        solrst[4] -= vperiod;
                    }
                    if (solrst[4] - lastpt2d.Y < -vperiod * 0.6)
                    {
                        solrst[4] += vperiod;
                    }
                }
            }

            // For the parameter on the arc, project...
            pt2d = new Point2D(solrst[3], solrst[4]);
            pnt = thecur.Value(ufirst);
            dist = pt2d.DistanceTo(pnt);
            solrst[1] = ufirst;
            pnt = thecur.Value(ulast);
            distaux = pt2d.DistanceTo(pnt);
            if (distaux < dist)
            {
                solrst[1] = ulast;
                dist = distaux;
            }

            if (dist > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                prm = pmin;
                if (onFirst)
                {
                    ok = BlendTool.Project(pt2d, _surf1, thearc, out prm, out distaux);
                }
                else
                {
                    ok = BlendTool.Project(pt2d, _surf2, thearc, out prm, out distaux);
                }
                if (ok && (pt2d.DistanceTo(thecur.Value(prm)) < dist))
                {
                    solrst[1] = prm;
                }
                else
                {
                    solrst[1] = pmin;
                }
            }
            // Check the jalon
            jalons_Trouve = (funcInv.IsSolution(solrst, _tolpoint3d));
        }

        if (!jalons_Trouve)
        {
            // Solve...
            rsnld.Perform(funcInv, solrst, infb, supb);
            if (!rsnld.IsDone)
            {
                recadre = false;
            }
            else
            {
                rsnld.Root(solrst);
                recadre = funcInv.IsSolution(solrst, _tolpoint3d);
            }
        }

        // On failure, see whether another arc fits (exits near a vertex)
        dist = (ulast - ufirst) / 100;
        if ((!recadre) && ((Math.Abs(pmin - ulast) < dist) || (Math.Abs(pmin - ufirst) < dist)))
        {
            indexsol = ArcToRecadre(onFirst, sol, indexsol, out lastpt2d, out pt2d, ref pmin);
            if (indexsol == 0)
            {
                return false;
            }

            iter.Init();
            nbarc = 1;
            while (nbarc < indexsol)
            {
                nbarc++;
                iter.Next();
            }
            thearc = iter.Value();

            if (onFirst)
            {
                thecur = BlendTool.CurveOnSurf(thearc, _surf1);
            }
            else
            {
                thecur = BlendTool.CurveOnSurf(thearc, _surf2);
            }
            solrst[1] = pmin;
            // The problem to solve
            funcInv.Set(onFirst, thecur);
            funcInv.GetBounds(infb, supb);
            funcInv.GetTolerance(toler, 0.1 * _tolpoint3d); // better keep some margin
            FunctionSetRoot aRsnld = new(funcInv, toler, 35);
            toler *= 10; // but make the tests correctly
            // Solve...
            aRsnld.Perform(funcInv, solrst, infb, supb);

            if (!aRsnld.IsDone)
            {
                recadre = false;
            }
            else
            {
                aRsnld.Root(solrst);
                recadre = funcInv.IsSolution(solrst, _tolpoint3d);
            }
        }

        if (recadre)
        {
            // Topological classification
            if (onFirst)
            {
                thecur = BlendTool.CurveOnSurf(thearc, _surf1);
            }
            else
            {
                thecur = BlendTool.CurveOnSurf(thearc, _surf2);
            }
            BlendTool.Bounds(thecur, out ufirst, out ulast);

            iter.Initialize(thearc);
            iter.InitVertexIterator();
            isVtx = !iter.MoreVertex();
            while (!isVtx)
            {
                vtx = iter.Vertex();
                vtol = 0.4 * Math.Abs(ulast - ufirst); // an upper bound of the tolerance
                if (vtol > Max(BlendTool.Tolerance(vtx, thearc), toler[1]))
                {
                    vtol = Max(BlendTool.Tolerance(vtx, thearc), toler[1]);
                }
                if (Math.Abs(BlendTool.Parameter(vtx, thearc) - solrst[1]) <= vtol)
                {
                    isVtx = true; // in the vertex's ball, or the vertex in the recadrage's "ball"
                }
                else
                {
                    iter.NextVertex();
                    isVtx = !iter.MoreVertex();
                }
            }
            if (!iter.MoreVertex())
            {
                isVtx = false;
            }
            return true;
        }
        return false;
    }

    private void Transition(bool onFirst, Curve2DAdaptorBase a, double param, ref WalkingTransition tLine,
                            ref WalkingTransition tArc)
    {
        bool computetranstionaveclacorde = false;
        Vector3D tgline;
        BlendPoint prevprev = new();

        if (_previousP.IsTangencyPoint)
        {
            if (_line!.NbPoints() < 2)
            {
                return;
            }
            computetranstionaveclacorde = true;
            if (_sens < 0)
            {
                prevprev = _line.Point(2);
            }
            else
            {
                prevprev = _line.Point(_line.NbPoints() - 1);
            }
        }
        Point2D p2d;
        Vector2D dp2d;

        Point3D pbid;
        Vector3D d1u, d1v, normale = new(), tgrst = new();
        Direction3D thenormal = new();
        NormalStatus stat;

        HCurve2dTool.D1(a, param, out p2d, out dp2d);
        if (onFirst)
        {
            HSurfaceTool.D1(_surf1, p2d.X, p2d.Y, out pbid, out d1u, out d1v);
            if (!computetranstionaveclacorde)
            {
                tgline = _previousP.TangentOnS1();
            }
            else
            {
                tgline = new Vector3D(prevprev.PointOnS1(), _previousP.PointOnS1());
            }
        }
        else
        {
            HSurfaceTool.D1(_surf2, p2d.X, p2d.Y, out pbid, out d1u, out d1v);
            if (!computetranstionaveclacorde)
            {
                tgline = _previousP.TangentOnS2();
            }
            else
            {
                tgline = new Vector3D(prevprev.PointOnS2(), _previousP.PointOnS2());
            }
        }

        tgrst = Vector3D.LinearForm(dp2d.X, d1u, dp2d.Y, d1v);

        global::Kerf.Engine.Geometry.CSLib.Normal(d1u, d1v, 1.0e-9, out stat, ref thenormal);
        if (stat == Defined)
        {
            normale = normale.WithXYZ(thenormal.XYZ());
        }
        else
        {
            SurfaceAdaptorBase surf;
            if (onFirst)
            {
                surf = _surf1;
            }
            else
            {
                surf = _surf2;
            }
            int iu, iv;
            NCollection_Array2<Vector3D> der = new(0, 2, 0, 2);
            HSurfaceTool.D2(surf, p2d.X, p2d.Y, out pbid, out der[1, 0], out der[0, 1], out der[2, 0],
                                      out der[0, 2], out der[1, 1]);
            der[2, 1] = HSurfaceTool.DN(surf, p2d.X, p2d.Y, 2, 1);
            der[1, 2] = HSurfaceTool.DN(surf, p2d.X, p2d.Y, 1, 2);
            der[2, 2] = HSurfaceTool.DN(surf, p2d.X, p2d.Y, 2, 2);
            global::Kerf.Engine.Geometry.CSLib.Normal(2, der, 1.0e-9, p2d.X, p2d.Y, HSurfaceTool.FirstUParameter(surf),
                         HSurfaceTool.LastUParameter(surf), HSurfaceTool.FirstVParameter(surf),
                         HSurfaceTool.LastVParameter(surf), out stat, ref thenormal, out iu, out iv);
            normale = normale.WithXYZ(thenormal.XYZ());
            // OCCT_DEBUG trace left out.
        }

        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(tgline, tgrst, new Direction3D(normale), ref tLine, ref tArc);
    }

    private void MakeExtremity(ref Extremity extrem, bool onFirst, int index, double param, bool isVtx,
                               HVertex? vtx)
    {
        WalkingTransition tline = new(), tarc = new();
        int nbarc;
        TopolTool iter;

        if (onFirst)
        {
            extrem.SetValue(_previousP.PointOnS1(), _sol[1], _sol[2], _previousP.Parameter(), _tolpoint3d);
            if (!_previousP.IsTangencyPoint)
            {
                extrem.SetTangent(_previousP.TangentOnS1());
            }
            iter = _recdomain1;
        }
        else
        {
            extrem.SetValue(_previousP.PointOnS2(), _sol[3], _sol[4], _previousP.Parameter(), _tolpoint3d);
            if (!_previousP.IsTangencyPoint)
            {
                extrem.SetTangent(_previousP.TangentOnS2());
            }
            iter = _recdomain2;
        }

        iter.Init();
        nbarc = 1;

        while (nbarc < index)
        {
            nbarc++;
            iter.Next();
        }

        Transition(onFirst, iter.Value(), param, ref tline, ref tarc);
        extrem.AddArc(iter.Value(), param, tline, tarc);
        if (isVtx)
        {
            extrem.SetVertex(vtx!);
        }
    }

    private void MakeSingularExtremity(ref Extremity extrem, bool onFirst, HVertex vtx)
    {
        WalkingTransition tline = new(), tarc = new();
        TopolTool iter;
        double prm;

        if (onFirst)
        {
            iter = _recdomain1;
            if (!_previousP.IsTangencyPoint)
            {
                extrem.SetTangent(_previousP.TangentOnS1());
            }
        }
        else
        {
            if (!_previousP.IsTangencyPoint)
            {
                extrem.SetTangent(_previousP.TangentOnS2());
            }
            iter = _recdomain2;
        }

        iter.Init();
        extrem.SetVertex(vtx);
        while (iter.More())
        {
            Curve2DAdaptorBase arc = iter.Value();
            iter.Initialize(arc);
            iter.InitVertexIterator();
            while (iter.MoreVertex())
            {
                if (iter.Identical(vtx, iter.Vertex()))
                {
                    prm = BlendTool.Parameter(vtx, arc);
                    Transition(onFirst, arc, prm, ref tline, ref tarc);
                    extrem.AddArc(arc, prm, tline, tarc);
                }
                iter.NextVertex();
            }
            iter.Next();
        }
    }

    private static void RecadreIfPeriodic(ref double newU, ref double newV, double oldU, double oldV, double uPeriod,
                                          double vPeriod)
    {
        if (uPeriod > 0.0)
        {
            double sign = (newU < oldU) ? 1 : -1;
            while (Math.Abs(newU - oldU) > uPeriod / 2)
            {
                newU += sign * uPeriod;
            }
        }
        if (vPeriod > 0.0)
        {
            double sign = (newV < oldV) ? 1 : -1;
            while (Math.Abs(newV - oldV) > vPeriod / 2)
            {
                newV += sign * vPeriod;
            }
        }
    }

    private static void Evalpinit(math_Vector parinit, in BlendPoint previousP, double parprec, double param,
                                  math_Vector infbound, math_Vector supbound, bool classonS1, bool classonS2)
    {
        if (previousP.IsTangencyPoint)
        {
            previousP.ParametersOnS1(out parinit[1], out parinit[2]);
            previousP.ParametersOnS2(out parinit[3], out parinit[4]);
        }
        else
        {
            double u1, v1, u2, v2;
            double du1, dv1, du2, dv2;
            bool inside = true;
            previousP.ParametersOnS1(out u1, out v1);
            previousP.ParametersOnS2(out u2, out v2);
            previousP.Tangent2dOnS1().Coord(out du1, out dv1);
            previousP.Tangent2dOnS2().Coord(out du2, out dv2);
            double step = param - parprec;
            u1 += step * du1;
            v1 += step * dv1;
            if (classonS1)
            {
                if ((u1 < infbound[1]) || (u1 > supbound[1]))
                {
                    inside = false;
                }
                if ((v1 < infbound[2]) || (v1 > supbound[2]))
                {
                    inside = false;
                }
            }
            u2 += step * du2;
            v2 += step * dv2;
            if (classonS2)
            {
                if ((u2 < infbound[3]) || (u2 > supbound[3]))
                {
                    inside = false;
                }
                if ((v2 < infbound[4]) || (v2 > supbound[4]))
                {
                    inside = false;
                }
            }

            if (inside)
            {
                parinit[1] = u1;
                parinit[2] = v1;
                parinit[3] = u2;
                parinit[4] = v2;
            }
            else
            { // don't play the smart one
                previousP.ParametersOnS1(out parinit[1], out parinit[2]);
                previousP.ParametersOnS2(out parinit[3], out parinit[4]);
            }
        }
    }

    private void InternalPerform(BlendFunction func, FuncInv funcInv, double bound)
    {
        double cosi = 0.0, cosi2 = 0.0;

        double stepw = _pasmax;
        int nbp = _line!.NbPoints();
        if (nbp >= 2)
        { // Take the last step back if it is not too small.
            if (_sens < 0.0)
            {
                stepw = (_line.Point(2).Parameter() - _line.Point(1).Parameter());
            }
            else
            {
                stepw = (_line.Point(nbp).Parameter() - _line.Point(nbp - 1).Parameter());
            }
            stepw = Max(stepw, 100.0 * _tolgui);
        }
        double parprec = _param;
        Vector3D tgOnGuide, prevTgOnGuide;
        Point3D ptOnGuide;
        _hguide.D1(parprec, out ptOnGuide, out tgOnGuide);
        prevTgOnGuide = tgOnGuide;

        if (_sens * (parprec - bound) >= -_tolgui)
        {
            return;
        }
        BlendStatus state = BlendStatus.OnRst12;
        Containment situ1 = global::Kerf.Topology.Containment.In, situ2 = global::Kerf.Topology.Containment.In;
        double w1, w2;
        int index1 = 0, index2 = 0, nbarc;
        bool arrive, recad1, recad2, control;
        bool isvtx1 = false, isvtx2 = false, echecrecad;
        Point2D p2d;
        math_Vector tolerance = new(1, 4), infbound = new(1, 4), supbound = new(1, 4), parinit = new(1, 4);
        math_Vector solrst1 = new(1, 4), solrst2 = new(1, 4);
        HVertex? vtx1 = null, vtx2 = null;
        Extremity ext1 = new(), ext2 = new();

        func.GetTolerance(tolerance, _tolpoint3d);
        func.GetBounds(infbound, supbound);

        FunctionSetRoot rsnld = new(func, tolerance, 30);
        parinit.Initialized(_sol);

        arrive = false;
        _param = parprec + _sens * stepw;
        if (_sens * (_param - bound) > 0.0)
        {
            stepw = _sens * (bound - parprec) * 0.5;
            _param = parprec + _sens * stepw;
        }

        Evalpinit(parinit, _previousP, parprec, _param, infbound, supbound, _clasonS1, _clasonS2);

        while (!arrive)
        {
            _hguide.D1(_param, out ptOnGuide, out tgOnGuide);
            // Check deflection on guide
            cosi = prevTgOnGuide * tgOnGuide;
            if (cosi < Constants.Resolution())
            { // angle>=pi/2 or null magnitude
                cosi2 = 0.0;
            }
            else
            {
                cosi2 = cosi * cosi / prevTgOnGuide.SquareMagnitude / tgOnGuide.SquareMagnitude;
            }
            if (cosi2 < CosRef3D_File) // angle 3d too great
            {
                state = BlendStatus.StepTooLarge;
                stepw = stepw / 2.0;
                _param = parprec + _sens * stepw; // no risk of going past Bound.
                if (Math.Abs(stepw) < _tolgui)
                {
                    ext1.SetValue(_previousP.PointOnS1(), _sol[1], _sol[2], _previousP.Parameter(), _tolpoint3d);
                    ext2.SetValue(_previousP.PointOnS2(), _sol[3], _sol[4], _previousP.Parameter(), _tolpoint3d);
                    if (!_previousP.IsTangencyPoint)
                    {
                        ext1.SetTangent(_previousP.TangentOnS1());
                        ext2.SetTangent(_previousP.TangentOnS2());
                    }
                    arrive = true;
                }
                continue;
            }
            prevTgOnGuide = tgOnGuide;
            //////////////////////////

            bool bonpoint = true;
            func.Set(_param);
            rsnld.Perform(func, parinit, infbound, supbound);

            if (!rsnld.IsDone)
            {
                state = BlendStatus.StepTooLarge;
                bonpoint = false;
            }
            else
            {
                rsnld.Root(_sol);

                if (_clasonS1)
                {
                    situ1 = _domain1.Classify(new Point2D(_sol[1], _sol[2]), Min(tolerance[1], tolerance[2]), false);
                }
                else
                {
                    situ1 = global::Kerf.Topology.Containment.In;
                }
                if (_clasonS2)
                {
                    situ2 = _domain2.Classify(new Point2D(_sol[3], _sol[4]), Min(tolerance[3], tolerance[4]), false);
                }
                else
                {
                    situ2 = global::Kerf.Topology.Containment.In;
                }
            }
            if (bonpoint && _line.NbPoints() == 1 && (situ1 != global::Kerf.Topology.Containment.In || situ2 != global::Kerf.Topology.Containment.In))
            {
                state = BlendStatus.StepTooLarge;
                bonpoint = false;
            }
            if (bonpoint)
            {
                w1 = w2 = bound;
                recad1 = false;
                recad2 = false;
                echecrecad = false;
                control = false;

                if (situ1 == global::Kerf.Topology.Containment.Out || situ1 == On)
                {
                    // inverse problem on surf1
                    // If the recadrage goes in the direction of the progression within a tolerance, the wrong
                    // solution was taken.
                    recad1 = Recadre(funcInv, true, _sol, solrst1, out index1, out isvtx1, ref vtx1);

                    if (recad1)
                    {
                        double wtemp;
                        wtemp = solrst1[2];
                        if ((_param - wtemp) / _sens >= -10 * _tolgui)
                        {
                            w1 = solrst1[2];
                            control = true;
                        }
                        else
                        {
                            echecrecad = true;
                            recad1 = false;
                            state = BlendStatus.StepTooLarge;
                            bonpoint = false;
                            stepw = stepw / 2.0;
                        }
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }
                if (situ2 == global::Kerf.Topology.Containment.Out || situ2 == On)
                {
                    // inverse problem on surf2
                    recad2 = Recadre(funcInv, false, _sol, solrst2, out index2, out isvtx2, ref vtx2);

                    if (recad2)
                    {
                        double wtemp;
                        wtemp = solrst2[2];
                        if ((_param - wtemp) / _sens >= -10 * _tolgui)
                        {
                            w2 = solrst2[2];
                            control = true;
                        }
                        else
                        {
                            echecrecad = true;
                            recad2 = false;
                            state = BlendStatus.StepTooLarge;
                            bonpoint = false;
                            stepw = stepw / 2.0;
                        }
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }

                // What must be checked
                if (recad1 && recad2)
                {
                    if (Math.Abs(w1 - w2) <= 10 * _tolgui)
                    {
                        // no need to check the recadrages, the check could fail (cf model blend10). The tolerance
                        // is large so that the next walking can put a few sections...
                        control = false;
                    }
                    else if (_sens * (w1 - w2) < 0.0)
                    {
                        // solution on 1?
                        recad2 = false;
                    }
                    else
                    {
                        // solution on 2?
                        recad1 = false;
                    }
                }

                // Effective check of the recadrages
                if (control)
                {
                    Containment situ;
                    if (recad1 && _clasonS2)
                    {
                        situ = _recdomain2.Classify(new Point2D(solrst1[3], solrst1[4]),
                                                   Min(tolerance[3], tolerance[4]));
                        if (situ == global::Kerf.Topology.Containment.Out)
                        {
                            recad1 = false;
                            echecrecad = true;
                        }
                    }
                    else if (recad2 && _clasonS1)
                    {
                        situ = _recdomain1.Classify(new Point2D(solrst2[3], solrst2[4]),
                                                   Min(tolerance[1], tolerance[1]));
                        if (situ == global::Kerf.Topology.Containment.Out)
                        {
                            recad2 = false;
                            echecrecad = true;
                        }
                    }
                }

                if (recad1 || recad2)
                {
                    echecrecad = false;
                }

                if (!echecrecad)
                {
                    if (recad1 && recad2)
                    {
                        // solution on 1 and 2 at once
                        // Go through the arcs, to avoid problems with periodic surfaces.
                        state = BlendStatus.OnRst12;
                        _param = (w1 + w2) / 2;
                        Point3D pnt1, pnt2;
                        p2d = HCurve2dTool.Value(_recdomain1.Value(), solrst1[1]);
                        _sol[1] = p2d.X;
                        _sol[2] = p2d.Y;
                        pnt1 = HSurfaceTool.Value(_surf1, _sol[1], _sol[2]);
                        p2d = HCurve2dTool.Value(_recdomain2.Value(), solrst2[1]);
                        _sol[3] = p2d.X;
                        _sol[4] = p2d.Y;
                        pnt2 = HSurfaceTool.Value(_surf2, _sol[3], _sol[4]);
                        const double tolProd = 1.0e-5;
                        double[] savedParams = new double[2];
                        bool[] sameDirs = { false, false };
                        ElSpine theElSpine = _hguide;
                        savedParams[0] = theElSpine.GetSavedFirstParameter();
                        savedParams[1] = theElSpine.GetSavedLastParameter();
                        for (int ind = 0; ind < 2; ind++)
                        {
                            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(savedParams[ind]))
                            {
                                // Check the original first and last parameters of guide curve for equality to
                                // found parameter <param>: check equality of tangent to guide curve and normal to
                                // plane built on 3 points: point on guide curve and points on restrictions of
                                // adjacent surfaces.
                                Point3D pnt0;
                                Vector3D dir0;
                                _hguide.D1(savedParams[ind], out pnt0, out dir0);
                                double length = dir0.Magnitude;
                                if (length <= Constants.Resolution())
                                {
                                    continue;
                                }
                                dir0 /= length;
                                MakePln planeBuilder = new(pnt0, pnt1, pnt2);
                                if (!planeBuilder.IsDone)
                                {
                                    continue;
                                }
                                Plane3D thePlane = planeBuilder.Value();
                                Direction3D dirPlane = thePlane.Axis.Direction;
                                Vector3D theProd = dir0 ^ new Vector3D(dirPlane);
                                double prodMod = theProd.Magnitude;
                                if (prodMod <= tolProd)
                                {
                                    sameDirs[ind] = true;
                                }
                            }
                        }
                        double theParam = global::Kerf.Engine.Foundation.Precision.Infinite();
                        // Choose the closest parameter
                        if (sameDirs[0] && sameDirs[1])
                        {
                            theParam = (Math.Abs(_param - savedParams[0]) < Math.Abs(_param - savedParams[1]))
                                         ? savedParams[0]
                                         : savedParams[1];
                        }
                        else if (sameDirs[0])
                        {
                            theParam = savedParams[0];
                        }
                        else if (sameDirs[1])
                        {
                            theParam = savedParams[1];
                        }

                        double newU = 0.0, newV = 0.0, newParam = 0.0;
                        Point3D newPnt = new();
                        bool corrected = CorrectExtremityOnOneRst(1, _sol[3], _sol[4], _param, pnt1, ref newU, ref newV,
                                                                  ref newPnt, ref newParam);
                        if (corrected)
                        {
                            if (Math.Abs(_param - newParam) < Math.Abs(_param - theParam))
                            {
                                theParam = newParam;
                            }
                        }

                        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(theParam))
                        {
                            _param = theParam;
                        }
                    }
                    else if (recad1)
                    {
                        // solution on 1
                        state = BlendStatus.OnRst1;
                        _param = w1;
                        _recdomain1.Init();
                        nbarc = 1;
                        while (nbarc < index1)
                        {
                            nbarc++;
                            _recdomain1.Next();
                        }
                        p2d = HCurve2dTool.Value(_recdomain1.Value(), solrst1[1]);
                        _sol[1] = p2d.X;
                        _sol[2] = p2d.Y;
                        _sol[3] = solrst1[3];
                        _sol[4] = solrst1[4];
                        Point3D thePntOnRst = HSurfaceTool.Value(_surf1, _sol[1], _sol[2]);
                        double newU = 0.0, newV = 0.0, newParam = 0.0;
                        Point3D newPnt = new();
                        bool corrected = CorrectExtremityOnOneRst(1, _sol[3], _sol[4], _param, thePntOnRst, ref newU,
                                                                  ref newV, ref newPnt, ref newParam);
                        if (corrected)
                        {
                            _param = newParam;
                            _sol[3] = newU;
                            _sol[4] = newV;
                        }
                    }
                    else if (recad2)
                    {
                        // solution on 2
                        state = BlendStatus.OnRst2;
                        _param = w2;

                        _recdomain2.Init();
                        nbarc = 1;
                        while (nbarc < index2)
                        {
                            nbarc++;
                            _recdomain2.Next();
                        }
                        p2d = HCurve2dTool.Value(_recdomain2.Value(), solrst2[1]);
                        _sol[1] = solrst2[3];
                        _sol[2] = solrst2[4];
                        _sol[3] = p2d.X;
                        _sol[4] = p2d.Y;
                        Point3D thePntOnRst = HSurfaceTool.Value(_surf2, _sol[3], _sol[4]);
                        double newU = 0.0, newV = 0.0, newParam = 0.0;
                        Point3D newPnt = new();
                        bool corrected = CorrectExtremityOnOneRst(2, _sol[1], _sol[2], _param, thePntOnRst, ref newU,
                                                                  ref newV, ref newPnt, ref newParam);
                        if (corrected)
                        {
                            _param = newParam;
                            _sol[1] = newU;
                            _sol[2] = newV;
                        }
                    }
                    else
                    {
                        state = BlendStatus.OK;
                    }

                    bool testdefl = true;
                    // OCCT_DEBUG: testdefl = !Blend_GetcontextNOTESTDEFL(), left out.
                    if (recad1 || recad2)
                    {
                        func.Set(_param);
                        // Better an unorthodox step than no recadrage!! PMN
                        state = TestArret(func, state, (testdefl && (Math.Abs(stepw) > 3 * _tolgui)), false, true);
                    }
                    else
                    {
                        state = TestArret(func, state, testdefl);
                    }
                }
                else
                {
                    // Or the max step is badly set. Divide.
                    if (stepw > 2 * _tolgui)
                    {
                        state = BlendStatus.StepTooLarge;
                        // Otherwise the recadrage failed. Leave with SamePoints
                    }
                    else
                    {
                        state = BlendStatus.SamePoints;
                    }
                }
            }

            // OCCT_DEBUG Drawsect left out.
            switch (state)
            {
                case BlendStatus.OK:
                {
                    // Update the line.
                    if (_sens > 0.0)
                    {
                        _line.Append(_previousP);
                    }
                    else
                    {
                        _line.Prepend(_previousP);
                    }

                    parprec = _param;

                    if (_param == bound)
                    {
                        arrive = true;
                        ext1.SetValue(_previousP.PointOnS1(), _sol[1], _sol[2], _previousP.Parameter(), _tolpoint3d);
                        ext2.SetValue(_previousP.PointOnS2(), _sol[3], _sol[4], _previousP.Parameter(), _tolpoint3d);
                        if (!_previousP.IsTangencyPoint)
                        {
                            ext1.SetTangent(_previousP.TangentOnS1());
                            ext2.SetTangent(_previousP.TangentOnS2());
                        }

                        // Indicate the end on Bound.
                    }
                    else
                    {
                        _param = _param + _sens * stepw;
                        if (_sens * (_param - bound) > -_tolgui)
                        {
                            _param = bound;
                        }
                    }
                    Evalpinit(parinit, _previousP, parprec, _param, infbound, supbound, _clasonS1, _clasonS2);
                }
                break;

                case BlendStatus.StepTooLarge:
                {
                    stepw = stepw / 2.0;
                    if (Math.Abs(stepw) < _tolgui)
                    {
                        ext1.SetValue(_previousP.PointOnS1(), _sol[1], _sol[2], _previousP.Parameter(), _tolpoint3d);
                        ext2.SetValue(_previousP.PointOnS2(), _sol[3], _sol[4], _previousP.Parameter(), _tolpoint3d);
                        if (!_previousP.IsTangencyPoint)
                        {
                            ext1.SetTangent(_previousP.TangentOnS1());
                            ext2.SetTangent(_previousP.TangentOnS2());
                        }
                        arrive = true;
                    }
                    else
                    {
                        _param = parprec + _sens * stepw; // no risk of going past Bound.
                        Evalpinit(parinit, _previousP, parprec, _param, infbound, supbound, _clasonS1, _clasonS2);
                    }
                }
                break;

                case BlendStatus.StepTooSmall:
                {
                    // Update the line.
                    if (_sens > 0.0)
                    {
                        _line.Append(_previousP);
                    }
                    else
                    {
                        _line.Prepend(_previousP);
                    }

                    parprec = _param;

                    stepw = Min(1.5 * stepw, _pasmax);
                    if (_param == bound)
                    {
                        arrive = true;
                        ext1.SetValue(_previousP.PointOnS1(), _sol[1], _sol[2], _previousP.Parameter(), _tolpoint3d);
                        ext2.SetValue(_previousP.PointOnS2(), _sol[3], _sol[4], _previousP.Parameter(), _tolpoint3d);
                        if (!_previousP.IsTangencyPoint)
                        {
                            ext1.SetTangent(_previousP.TangentOnS1());
                            ext2.SetTangent(_previousP.TangentOnS2());
                        }
                        // Indicate the end on Bound.
                    }
                    else
                    {
                        _param = _param + _sens * stepw;
                        if (_sens * (_param - bound) > -_tolgui)
                        {
                            _param = bound;
                        }
                    }
                    Evalpinit(parinit, _previousP, parprec, _param, infbound, supbound, _clasonS1, _clasonS2);
                }
                break;

                case BlendStatus.OnRst1:
                {
                    if (_sens > 0.0)
                    {
                        _line.Append(_previousP);
                    }
                    else
                    {
                        _line.Prepend(_previousP);
                    }

                    MakeExtremity(ref ext1, true, index1, solrst1[1], isvtx1, vtx1);
                    // Guard the singular case where one of the recadrages failed
                    if (_previousP.PointOnS1().IsEqual(_previousP.PointOnS2(), 2.0 * _tolpoint3d))
                    {
                        ext2.SetValue(_previousP.PointOnS1(), _sol[3], _sol[4], _tolpoint3d); // OCCT: the curve overload
                        if (isvtx1)
                        {
                            MakeSingularExtremity(ref ext2, false, vtx1!);
                        }
                    }
                    else
                    {
                        ext2.SetValue(_previousP.PointOnS2(), _sol[3], _sol[4], _previousP.Parameter(), _tolpoint3d);
                    }
                    arrive = true;
                }
                break;

                case BlendStatus.OnRst2:
                {
                    if (_sens > 0.0)
                    {
                        _line.Append(_previousP);
                    }
                    else
                    {
                        _line.Prepend(_previousP);
                    }

                    // Guard the singular case where one of the recadrages failed
                    if (_previousP.PointOnS1().IsEqual(_previousP.PointOnS2(), 2.0 * _tolpoint3d))
                    {
                        ext1.SetValue(_previousP.PointOnS2(), _sol[1], _sol[2], _tolpoint3d); // OCCT: the curve overload
                        if (isvtx2)
                        {
                            MakeSingularExtremity(ref ext1, true, vtx2!);
                        }
                    }
                    else
                    {
                        ext1.SetValue(_previousP.PointOnS1(), _sol[1], _sol[2], _previousP.Parameter(), _tolpoint3d);
                    }
                    MakeExtremity(ref ext2, false, index2, solrst2[1], isvtx2, vtx2);
                    arrive = true;
                }
                break;

                case BlendStatus.OnRst12:
                {
                    if (_sens > 0.0)
                    {
                        _line.Append(_previousP);
                    }
                    else
                    {
                        _line.Prepend(_previousP);
                    }

                    if ((isvtx1 != isvtx2) && (_previousP.PointOnS1().IsEqual(_previousP.PointOnS2(), 2.0 * _tolpoint3d)))
                    {
                        // Guard the singular case where only one recadrage is recognised as a vertex.
                        if (isvtx1)
                        {
                            isvtx2 = true;
                            vtx2 = vtx1;
                        }
                        else
                        {
                            isvtx1 = true;
                            vtx1 = vtx2;
                        }
                    }

                    MakeExtremity(ref ext1, true, index1, solrst1[1], isvtx1, vtx1);
                    MakeExtremity(ref ext2, false, index2, solrst2[1], isvtx2, vtx2);
                    arrive = true;
                }
                break;

                case BlendStatus.SamePoints:
                {
                    // Stop
                    ext1.SetValue(_previousP.PointOnS1(), _sol[1], _sol[2], _previousP.Parameter(), _tolpoint3d);
                    ext2.SetValue(_previousP.PointOnS2(), _sol[3], _sol[4], _previousP.Parameter(), _tolpoint3d);
                    if (!_previousP.IsTangencyPoint)
                    {
                        ext1.SetTangent(_previousP.TangentOnS1());
                        ext2.SetTangent(_previousP.TangentOnS2());
                    }
                    arrive = true;
                }
                break;
                default:
                    break;
            }
            if (arrive)
            {
                if (_sens > 0.0)
                {
                    _line.SetEndPoints(ext1, ext2);
                }
                else
                {
                    _line.SetStartPoints(ext1, ext2);
                }
            }
        }
    }

    // NewU, NewV and NewPoint are written only on success; NewParam as soon as the guide is projected (OCCT's
    // references), so they are ref parameters.
    private bool CorrectExtremityOnOneRst(int indexOfRst, double u, double v, double param,
                                          in Point3D pntOnRst, ref double newU, ref double newV,
                                          ref Point3D newPoint, ref double newParam)
    {
        const double tolAng = 0.001; // bug OCC25701

        ElSpine theElSpine = _hguide;
        if (theElSpine.VertexCount == 0)
        {
            return false;
        }

        TopolTool domainOfRst = (indexOfRst == 1) ? _recdomain1 : _recdomain2;
        SurfaceAdaptorBase surfOfRst = (indexOfRst == 1) ? _surf1 : _surf2;
        SurfaceAdaptorBase anotherSurf = (indexOfRst == 1) ? _surf2 : _surf1;

        // Correct point on surface 2
        // First we find right <param>
        Span<double> ends = stackalloc double[2];
        ends[0] = HCurve2dTool.FirstParameter(domainOfRst.Value());
        ends[1] = HCurve2dTool.LastParameter(domainOfRst.Value());
        double globalMinSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
        double paramOnGuide = 0;
        Point3D pointOnGuide = new();
        for (int k = 0; k < 2; k++)
        {
            Point2D p2dOnEnd = HCurve2dTool.Value(domainOfRst.Value(), ends[k]);
            Point3D pntOnEnd = HSurfaceTool.Value(surfOfRst, p2dOnEnd.X, p2dOnEnd.Y);
            CurvePointExtrema projoncurv = new(pntOnEnd, theElSpine);
            if (!projoncurv.IsDone)
            {
                continue;
            }
            double minSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
            int imin = 0;
            for (int ind = 1; ind <= projoncurv.ExtCount; ind++)
            {
                double sqDist = projoncurv.SquareDistance(ind);
                if (sqDist < minSqDist)
                {
                    minSqDist = sqDist;
                    imin = ind;
                }
            }
            if (minSqDist < globalMinSqDist)
            {
                globalMinSqDist = minSqDist;
                paramOnGuide = projoncurv.Point(imin).Parameter();
                pointOnGuide = projoncurv.Point(imin).Value();
            }
        }
        newParam = paramOnGuide;
        if (_hguide.IsPeriodic)
        {
            double period = _hguide.Period();
            double sign = (newParam < param) ? 1 : -1;
            while (Math.Abs(newParam - param) > period / 2)
            {
                newParam += sign * period;
            }
        }

        // Second we find right point and tangent on guide
        globalMinSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
        Axis3D theAx1 = new();
        for (int ind = 1; ind <= theElSpine.VertexCount; ind++)
        {
            Axis3D ax1 = theElSpine.VertexWithTangent(ind);
            Point3D pnt = ax1.Location;
            double sqDist = pointOnGuide.SquareDistanceTo(pnt);
            if (sqDist < globalMinSqDist)
            {
                globalMinSqDist = sqDist;
                theAx1 = ax1;
            }
        }
        Point3D pnt0 = theAx1.Location;
        Direction3D dir0 = theAx1.Direction;
        // Check new point: is it real solution?
        Point3D oldPonGuide = _hguide.Value(param);
        Point3D pntOnSurf2 = HSurfaceTool.Value(anotherSurf, u, v); // old point
        MakePln planeBuilder = new(pntOnRst, oldPonGuide, pntOnSurf2);
        if (!planeBuilder.IsDone)
        {
            return false;
        }
        Plane3D oldPlane = planeBuilder.Value();
        Direction3D oldDir = oldPlane.Axis.Direction;
        double angle = oldDir.AngleTo(dir0);
        if (angle > M_PI / 2)
        {
            angle = M_PI - angle;
        }
        if (angle > tolAng)
        {
            return false;
        }
        ///////////////////////////////////////
        // Project the point(theU,theV) on the plane(Pnt0,Dir0)
        Vector3D vec = new(pnt0, pntOnSurf2);
        Vector3D translation = new((vec.XYZ() * dir0.XYZ()) * dir0.XYZ());
        Point3D pntOnPlane = pntOnSurf2.Translated(-translation);

        // Check new point again: does point on restriction belong to the plane?
        planeBuilder = new MakePln(pntOnRst, pnt0, pntOnPlane);
        if (!planeBuilder.IsDone)
        {
            return false;
        }
        Plane3D newPlane = planeBuilder.Value();
        Direction3D dirOfNewPlane = newPlane.Axis.Direction;
        angle = dir0.AngleTo(dirOfNewPlane);
        if (angle > M_PI / 2)
        {
            angle = M_PI - angle;
        }
        if (angle > tolAng)
        {
            return false;
        }
        ////////////////////////////////////////////////////////////////////////

        // Project the point <PntOnPlane> on the surface 2
        ExtPS projonsurf = new(pntOnPlane, anotherSurf, global::Kerf.Engine.Foundation.Precision.PConfusion(), global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                       ExtFlag_MIN);
        if (projonsurf.IsDone)
        {
            double minSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
            int imin = 0;
            for (int ind = 1; ind <= projonsurf.ExtCount; ind++)
            {
                double sqDist = projonsurf.SquareDistance(ind);
                if (sqDist < minSqDist)
                {
                    minSqDist = sqDist;
                    imin = ind;
                }
            }
            if (imin != 0)
            {
                POnSurf newPOnSurf2 = projonsurf.Point(imin);
                newPoint = newPOnSurf2.Value();
                newPOnSurf2.Parameter(out newU, out newV);
                double uperiod = (anotherSurf.IsUPeriodic) ? anotherSurf.UPeriod() : 0.0;
                double vperiod = (anotherSurf.IsVPeriodic) ? anotherSurf.VPeriod() : 0.0;
                RecadreIfPeriodic(ref newU, ref newV, u, v, uperiod, vperiod);
                return true;
            }
        }

        return false;
    }
}
