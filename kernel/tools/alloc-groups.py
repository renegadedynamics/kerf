# Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
#
# What allocation is made of (unit 11Q, docs/performance.md): sorts every allocation tick of a trace into immutability's
# own costs, temporaries, results and other, and works out the sceptic's upper bound for immutability. A tick takes the first
# group that matches. Take the trace as tools/profile-report.cs says, then:
#   dotnet run kernel/tools/alloc-report.cs -- bento.nettrace 3500 ticks ticks.tsv
#   python kernel/tools/alloc-groups.py ticks.tsv [rows of types in each group]
import collections,re,sys
A_stack=[('overlay',r'WorkingOverlay\.|FinishOverlay'),('freeze',r'FreezeGraph|Freeze$'),('reshape',r'ReShapeBuilder\.|Topology\.ReShape\.'),
 ('with/immutable collections',r'<Clone>\$|System\.Collections\.Immutable|ToImmutable'),('placement',r'Kerf\.Topology\.Placement\.|SListOfItemLocation')]
A_stack=[(k,re.compile(p)) for k,p in A_stack]
Tshape=re.compile(r'^Kerf\.Engine\.Topology\.(Brep\.)?(T(Wire|Shell|Solid|Compound|CompSolid|Edge|Face|Vertex)|Brep(TEdge|TFace|TVertex)|.*Representation.*|Curve(OnSurface|On2Surfaces|OnClosedSurface|3D|Polygon).*|Point(On|Rep).*|PolygonOn.*|Polygon3D.*)')
Geom=re.compile(r'^Kerf\.Geometry\.(\w*(Curve[23]D|Surface))$')
TempCtx=re.compile(r'Tool\.(CurveOnPlane|CurveOnSurface|Curve$|Surface$)|\.Bounds\.|Adaptor|Projection\.|Extrema\.|Healing\.Analysis|Healing\.Checks|Classification\.|IsTangentFaces|\.AddUVBounds|Measurement\.|Approx')
Mesh=re.compile(r'MeshExtraction|Kerf\.Meshing\.ShapeMesh\.|Kerf\.Meshing\.Mesher\.Read|CommitSurfaceTriangulation|Meshing\.Triangulation\.|ArrayOfNodes\.Allocate|PolygonOnTriangulation\.\.ctor|Kerf\.Modeling\.ShapeMesh\+<>c\.')
Copyish=re.compile(r'\.(Copy|Transformed|Reversed|Mirrored|Rotated|Scaled|Translated|Moved|Located|Oriented|EmptyCopy|Reverse)$')
BuilderKeep=re.compile(r'(Topology\.Builder|BrepBuilder|Brep\.Builder)\.|Topology\.Builder')
def sub_temp(t,fr):
    l=fr[0]
    if re.search(r'DisplayClass|^Func|^Action|Func`|Action`',t): return 'closures, delegates'
    if re.search(r'Iterator|Explorer|Enumerator',t): return 'iterators, explorers'
    if re.search(r'Adaptor|CurveOnSurface|TransformedCurve|TransformedSurface|Projector|ProjectOnPlane|ProjectedCurve|BSplineCurve2dBounds|CurveBounds|Curve2dBounds',t): return 'adaptors, projectors'
    if re.match(r'^Kerf\.Geometry\.(Line|Plane|Circle|Cylinder|Torus|Sphere|Cone|Ellipse|Hyperbola|Parabola)[23]?D?$',t): return 'boxed elementary geometry values'
    if re.search(r'VectorBase|Storage|Matrix|DoubleTab|NCollection_Array2|Kerf\.Engine\.Numerics',t): return 'math objects'
    if t.endswith('[]') or re.search(r'^System\.Collections|NCollection|Node|Entry|ListNode|TreeNode|Bucket|Tables|PackedMap|Segment',t):
        if any('Meshing.' in f for f in fr): return 'collections and arrays: mesher'
        return 'collections and arrays: other'
    return 'helper objects'
TT=collections.defaultdict(collections.Counter);G=collections.Counter();S=collections.Counter();N=0;Ticks=0
sceptic=collections.Counter();U=collections.Counter();unsc=0
tick_list=[]
for line in open(sys.argv[1],encoding='utf8'):
    b,n,t,st=line.rstrip('\n').split('\t');b=int(b);n=int(n);fr=st.split('|');N+=b;Ticks+=n
    a=None
    for k,p in A_stack:
        if any(p.search(f) for f in fr): a=k;break
    firstkerf=next((f for f in fr if f.startswith('Kerf.') and 'Collections' not in f),'')
    if a:
        g,s='A immutability',a
    elif re.match(r'Kerf\.Engine\.Topology\.(Brep\.Brep)?T\w+\.\.ctor',firstkerf):
        g,s='R results',"topology and geometry built (kept, or a step's result)"
    elif Mesh.search(st):
        g,s='R results','meshes'
    elif (Tshape.match(t) or Geom.match(t) or re.search(r'Kerf\.Topology\.Shape\[\]|List.1\[Kerf.Topology.Shape\]|NCollection_List.1\[Kerf.Topology.Shape\]',t) and 'Topology.Builder.Add' in st[:300]) :
        if TempCtx.search(st.split('|Kerf.Modeling')[0]) and Geom.match(t):
            g,s='T temporaries','geometry made to be read once (adaptors, projections, boxes)'
        else:
            g,s='R results','topology and geometry built (kept, or a step\'s result)'
    elif re.search(r'BSplineCurve|BSplineSurface|Kerf\.Geometry\.',firstkerf) and not re.match(r'Kerf.Geometry.\w+$',t) and not TempCtx.search(st.split('|Kerf.Modeling')[0]):
        g,s='R results','topology and geometry built (kept, or a step\'s result)'
    elif not firstkerf or re.search(r'^(System\.Threading|System\.String|System\.Char|System\.Object$|TwoElement|ThreeElement|.*AsyncLocal|.*ExecutionContext|Replica|.*ThreadPool|.*Task)',t) or re.search(r'^System.Threading',fr[0]) :
        g,s='O other','runtime, tasks, threads, strings'
    else:
        g,s='T temporaries',sub_temp(t,fr)
    G[g]+=b;S[(g,s)]+=b;TT[s][t[:70]]+=b
    # the sceptic's bound
    bound=[]
    if a: bound.append('A: the group above')
    else:
        if any(Copyish.search(f) for f in fr[:6]): bound.append('copies, reversals, transforms of anything')
        if s=='boxed elementary geometry values': bound.append('boxed immutable values')
        if s=='meshes': bound.append('mesh extraction')
        if re.match(r'Kerf\.(Modeling|Topology|Booleans|Fillets|Healing|Offsets|Meshing|Geometry)\.',firstkerf) or 'ModelHistory' in st: bound.append('anything allocated directly by the public layer')
        if s.startswith('topology and geometry'): bound.append('every shape and curve built')
    sceptic[bound[0] if bound else 'none']+=b
print(f"total {N/1048576:.0f} MB, {Ticks} ticks")
for g,v in sorted(G.items()): print(f"{g}: {100*v/N:.2f}%")
for (g,s),v in sorted(S.items(),key=lambda x:(x[0][0],-x[1])): print(f"  {g[0]} {100*v/N:6.2f}%  {s}")
print('--- sceptic')
for k,v in sceptic.most_common():
    print(f"{100*v/N:6.2f} {k}")


if len(sys.argv)>2:
    for k,c in TT.items():
        print('---',k)
        for t,v in c.most_common(int(sys.argv[2])): print(f"{100*v/N:6.2f} {t}")
